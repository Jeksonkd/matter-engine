#pragma once

#include "p2d/Collision.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace p2d {

// Uniform spatial hash grid broadphase.
//
// History: this engine used sweep-and-prune before this. SAP is excellent
// for spread-out scenes, but its cost is driven by how many bodies share an
// x-range at any point in the sweep -- and a genuinely dense cluster (many
// bodies resting in a small area, which is exactly what "drop N bodies and
// let them settle" produces) makes that active-set large regardless of how
// good the rest of the algorithm is. Stress-testing at 10,000 clustered
// bodies showed per-step cost growing unboundedly over time (tens of ms and
// climbing) as more bodies piled up, even though nothing crashed or leaked.
//
// A grid fixes this *if* sized correctly: cells sized to roughly a body's
// own extent mean a dense pile of similarly-sized bodies only ever has a
// small, bounded number of bodies per cell, so cost stays close to O(n)
// even as the pile grows. (An earlier attempt at a grid in this codebase
// got this wrong in two ways: it re-picked the cell size every step with no
// hysteresis, causing the map's key set to grow without bound as the same
// world position hashed to a different cell each step; and it deduplicated
// cross-cell pairs with a hash *set*, which heap-allocates a node per
// candidate pair. Both are avoided below.)
//
// Each body is inserted into exactly one cell (the one containing its AABB
// center) UNLESS its AABB is larger than a cell, in which case it goes into
// a small "oversized" list tested against everything instead -- this keeps
// a handful of huge static bodies (e.g. the ground) from being inserted
// into, and bloating, many cells. Same-cell pairs use the obvious i<j test;
// cross-cell pairs use a "half neighborhood" trick (4 of the 8 neighbor
// directions, one from each opposite pair) so every pair is generated
// exactly once with no separate deduplication step needed at all.
//
// Bucket contents (not the map's node allocations) are cleared every call
// and reused: profiling at 10,000 bodies showed a *fully* torn-down-and-
// rebuilt map (unconditional cells_.clear() every step) spending a large
// chunk of total time in node allocation/deallocation churn -- clearing
// only avoids that while keeping capacity, which is safe as long as the
// cell size is stable, since a stable size means a given world position
// always hashes to the same key (so key set stays bounded by the area
// actually visited, not ever-growing). If the cell size changes by more
// than a small tolerance, a full teardown *is* needed (old keys become
// meaningless under a new size) -- see setCellSize().
class SpatialHashGrid {
public:
    template <typename Fn>
    void computePairs(const std::vector<AABB>& aabbs, float cellSize, Fn&& fn) {
        setCellSize(cellSize);
        oversized_.clear();

        for (size_t i = 0; i < aabbs.size(); ++i) {
            const AABB& box = aabbs[i];
            float w = box.max.x - box.min.x;
            float h = box.max.y - box.min.y;
            if (w > cellSize || h > cellSize) {
                oversized_.push_back(static_cast<int>(i));
            } else {
                Vec2 center = (box.min + box.max) * 0.5f;
                int cx = cellCoord(center.x, cellSize);
                int cy = cellCoord(center.y, cellSize);
                cells_[cellKey(cx, cy)].push_back(static_cast<int>(i));
            }
        }

        // Oversized-vs-oversized: expected to stay tiny (a handful of large
        // static bodies), so a plain nested loop is fine.
        for (size_t i = 0; i < oversized_.size(); ++i) {
            for (size_t j = i + 1; j < oversized_.size(); ++j) {
                emit(oversized_[i], oversized_[j], fn);
            }
        }
        // Oversized-vs-everything-in-the-grid.
        if (!oversized_.empty()) {
            for (auto& cellEntry : cells_) {
                for (int gridIdx : cellEntry.second) {
                    for (int bigIdx : oversized_) emit(gridIdx, bigIdx, fn);
                }
            }
        }

        // Grid-vs-grid: same-cell pairs, plus a fixed half-neighborhood so
        // each cross-cell pair is found exactly once.
        static constexpr int kNeighborOffsets[4][2] = {{1, 0}, {1, 1}, {0, 1}, {-1, 1}};

        for (auto& cellEntry : cells_) {
            const std::vector<int>& bucket = cellEntry.second;
            for (size_t i = 0; i < bucket.size(); ++i) {
                for (size_t j = i + 1; j < bucket.size(); ++j) emit(bucket[i], bucket[j], fn);
            }

            int cx = static_cast<int>(cellEntry.first >> 32);
            int cy = static_cast<int>(static_cast<uint32_t>(cellEntry.first));
            for (const auto& offset : kNeighborOffsets) {
                auto it = cells_.find(cellKey(cx + offset[0], cy + offset[1]));
                if (it == cells_.end()) continue;
                for (int a : bucket) {
                    for (int b : it->second) emit(a, b, fn);
                }
            }
        }

        // Prune cells not touched this step (their bucket is still empty
        // from setCellSize()'s clear). Without this, the map's key set
        // grows to cover every distinct cell ever visited across the whole
        // simulation instead of just the currently-occupied ones -- fine
        // for a pile that settles in place, but unbounded for anything
        // that wanders over a long-running session (found via a bacteria-
        // simulation regression test going from ~0.5s to 14s: population
        // stayed ~140 the whole time, but cells_ silently accumulated many
        // thousands of stale empty entries as bodies moved/reproduced
        // across the dish over 40 simulated seconds, and both pair-
        // generation loops above iterate every entry in the map regardless
        // of whether its bucket is empty).
        for (auto it = cells_.begin(); it != cells_.end();) {
            if (it->second.empty()) {
                it = cells_.erase(it);
            } else {
                ++it;
            }
        }
    }

private:
    std::unordered_map<int64_t, std::vector<int>> cells_;
    std::vector<int> oversized_;
    float cellSize_ = 0.0f; // 0 = not yet initialized

    // Full teardown only when the size actually changes meaningfully (old
    // keys would be wrong under a new size); otherwise just empty each
    // bucket's contents and keep its allocation, which is the common case
    // every step once a scene's average body size has stabilized.
    void setCellSize(float cellSize) {
        bool changed = cellSize_ <= 0.0f || std::fabs(cellSize - cellSize_) > cellSize_ * 0.1f;
        cellSize_ = cellSize;
        if (changed) {
            cells_.clear();
        } else {
            for (auto& entry : cells_) entry.second.clear();
        }
    }

    template <typename Fn>
    static void emit(int a, int b, Fn&& fn) {
        if (a == b) return;
        fn(std::min(a, b), std::max(a, b));
    }

    static int cellCoord(float v, float cellSize) { return static_cast<int>(std::floor(v / cellSize)); }
    static int64_t cellKey(int cx, int cy) {
        return (static_cast<int64_t>(cx) << 32) | static_cast<uint32_t>(cy);
    }
};

} // namespace p2d
