# Matter Engine

A small 2D rigid-body physics engine written in C++17, with Lua scripting
embedded via [sol2](https://github.com/ThePhD/sol2), and a visual editor
built with [SFML](https://www.sfml-dev.org/) + [Dear ImGui](https://github.com/ocornut/imgui).

## Features

- **Engine core** (`engine/`): circles and convex polygons, SAT-based
  narrowphase collision (circle-circle, circle-polygon, polygon-polygon with
  contact clipping), restitution and Coulomb friction. No external physics
  dependency — it's a from-scratch implementation, similar in scope to
  box2d-lite. The solver and broadphase (below) are closer to Box2D's actual
  architecture than a first-pass implementation:
  - **Spatial hash grid broadphase** instead of an O(n²) all-pairs scan —
    verified to find exactly the same candidate pairs as brute force
    (`tests/broadphase_test.cpp`). See "Handling massive object counts"
    below for the full story: this replaced a sweep-and-prune implementation
    that degraded badly under dense clustering, and along the way surfaced
    (and fixed) two real bugs of its own.
  - **Sleeping bodies**: bodies at rest for a short while stop being
    integrated/solved entirely until disturbed (by a force/impulse, a
    script write, or another body touching them) — a large performance win
    for any scene with things at rest, which is most scenes eventually.
  - **Warm-started sequential-impulse solver** with a fixed geometric
    tangent and accumulated, clamped impulses (matching Box2D's actual
    algorithm), plus a *separate*, non-accumulated positional-correction
    pass. Positional (Baumgarte) correction folded into the same
    accumulator that gets warm-started is a real, easy-to-hit bug: it
    compounds frame over frame and slowly pumps energy into resting
    stacks. `tests/stacking_test.cpp` settles a 6-box stack and checks it
    actually comes (and stays) to rest rather than sinking or drifting.
  - **Adaptive substepping**: a step is automatically subdivided when a
    body would otherwise move more than ~half its own size in one step,
    mitigating tunneling for fast-moving bodies (`tests/tunneling_test.cpp`
    fires a bullet through a thin wall both with and without this enabled
    to prove it's actually doing something).
- **Scripting** (`scripting/`): attach a `.lua` file to any body. Scripts get
  `on_start(body)` / `on_update(body, dt)` hooks, can read/write the body's
  transform, velocity, material and dynamics properties (see below), apply
  forces/impulses, and spawn new bodies via the bound `world` global. Each
  attached script runs in its own sandboxed Lua environment.
- **Per-body properties**: beyond the transform/velocity, every body has
  bounciness (restitution), friction, density, linear/angular damping
  (drag), a gravity scale multiplier, a fixed-rotation lock, and a sensor
  flag (detects overlap without physically colliding, for trigger zones) —
  all editable live in the Inspector or from Lua, and all covered by
  `tests/properties_test.cpp`.
- **Per-body viewport color**: the Inspector's Appearance section has a
  color picker for each body, independent of its `BodyType` or whether it's
  currently asleep. A fresh body still starts out colored by type (gray
  Static, orange Kinematic, blue Dynamic — the same defaults as before) so
  nothing looks different out of the box, but from then on it's just an
  ordinary property: it survives Play/Stop/Reset and Ctrl+C/Ctrl+V, and
  won't silently change on its own the way it used to (the color used to be
  *recomputed* from type + sleep state every frame, so a body visibly
  dimmed the moment it fell asleep with no way to pin a specific look — the
  Inspector's Awake/Asleep text indicator already covers that status
  separately).
- **Show circle direction line**: Settings has a checkbox for the small
  radial line marking a circle's current rotation (otherwise a circle looks
  identical at every angle). On by default; off is useful for scenes with
  many overlapping circles where it's just clutter.
- **Built-in code editor**: a Lua-syntax-highlighted, line-numbered,
  scrollable editor (via [ImGuiColorTextEdit](https://github.com/BalazsJako/ImGuiColorTextEdit))
  for editing scripts without leaving the app. Closed by default, and docked
  into the *same tab well* as the Viewport, so opening it replaces the
  viewport view rather than sitting alongside it (same idea as Godot's
  2D/Script view tabs) — Play/Pause/Step still work normally while it's open.
  Edits **autosave** ~2 seconds after you stop typing (debounced off the
  editor's own change events, reset on every keystroke) in addition to the
  manual **Save** button.
- **Visual editor** (`app/`): a Godot-like docked layout — a toolbar strip
  along the top, a Scene/Hierarchy panel and a FileSystem panel on the left,
  an Inspector on the right, a Console at the bottom, and a viewport in the
  middle that renders the physics world into its own panel (so it can be
  resized/undocked like any other editor panel). Drag panels around and
  rearrange them like any docking editor; the layout is rebuilt to this
  default on first launch. Styled with a dark, rounded, blue-violet-accented
  theme (`app/src/Theme.cpp`) rather than default ImGui grey.
  **Edit mode vs Play mode**, same as Godot: while stopped you can freely
  drag *any* body (static walls, kinematic platforms, dynamic props) to
  place it exactly where you want; press **Play** and the scene simulates
  live (dragging a dynamic body now pulls it with a spring instead of
  teleporting it); press **Stop** and the scene snaps back to exactly how
  you arranged it before pressing Play — spawns/deletes/edits made while
  playing are discarded, just like leaving Godot's play mode.
  **Reset Scene** goes further than Stop: it restores the scene to how it
  looked the last time the project was in genuine edit mode — *not* just
  paused mid-session. Pressing Play (or Step) marks the checkpoint frozen
  until the next Stop/Reset, so anything spawned or changed during a play
  session — including bodies a scripted UI button spawns while
  playing — is discarded by Reset Scene even if you paused first before
  clicking it. **Settings > Keep objects spawned during Play after
  Reset** opts back into the old behavior (checkpoint stays continuous
  through pause too, so Reset just reflects the current scene).
- ~~Optimization Mode~~ (removed): this used to be a single toolbar
  checkbox trading physical realism for raw speed scene-wide. It's gone now
  that the same trade-off is available per-body via **Matter Kind**
  (Inspector dropdown: Rigidbody/Matter/OptiMatter) — see
  "`Body::matterKind`" below — which is strictly more useful: only the
  bodies that actually need to be cheap pay the accuracy cost, instead of
  every body in the scene at once.
- **Viewport manipulation tools**: beyond Select/Move, the toolbar has
  **Resize** (click a body, drag to resize its radius/half-extents,
  recomputing mass), **Rotate** (click a body, drag around it to spin it
  to face the mouse), and **Box Select** (drag a rubber-band rectangle to
  select every body inside it at once — dragging any one member then moves
  the whole group together, and the Inspector gets a "Delete All Selected"
  button while a group is active).
- **One unified Spawn tool, any shape**: rather than separate "Spawn
  Circle"/"Spawn Box" tools, a single **Spawn** tool has a **Shape**
  dropdown — Circle, Box, Triangle, Pentagon, or Hexagon. (Soft bodies and
  UI Elements are created differently — see below — since neither is "an
  ordinary rigid shape placed at a clicked point".) The polygon shapes are built with
  `ShapeData::MakePolygon` (regular N-gons), which needed zero engine
  changes: the SAT narrowphase already handles polygon-polygon collision
  for any convex shape, not just boxes — Box was just the only polygon the
  editor ever *constructed*.
- **Soft bodies**: the engine was rigid-body-only by original scope (a
  from-scratch deformable-body solver — FEM or position-based dynamics —
  is a substantially different kind of engine). `p2d::SoftBody`
  (`engine/include/p2d/SoftBody.hpp`) adds a lightweight mass-spring
  alternative instead: a soft body is a group of ordinary small circle
  Bodies (so it collides with everything else via the *exact same*
  broadphase/solver, for free), held together by Hooke's-law springs.
  Three shapes ship: a hollow **Ring** (perimeter + skip-one cross-brace
  springs), a filled **Jelly** blob (a Ring plus a center hub particle
  spoked to every ring particle, so it resists collapsing/holds its volume
  much better than a hollow ring), and a **Cloth** grid (structural +
  shear springs, with its top row optionally pinned as Static bodies so it
  hangs and drapes instead of just falling). This trades away what a
  "real" solver gives you — exact volume preservation, no
  self-intersection — for reusing the whole existing rigid-body pipeline
  unchanged; see "Making the soft body actually settle" below for a real
  bug this surfaced and how it was fixed. **Script-only, not in the Spawn
  tool**: `soft_body.create_ring(x,y,radius,segments)`, `.create_jelly(...)`,
  and `.create_cloth(x,y,cols,rows,spacing,pin_top)` (see Scripting API
  below) still spawn all three exactly as before — only the toolbar's
  Spawn tool no longer offers them as a Shape option.
- **Spring joints — tie any two existing objects together**: `p2d::SpringJoint`
  (`engine/include/p2d/SpringJoint.hpp`) generalizes the same spring math a
  Cloth/Jelly uses between its own particles to *any* two bodies already in
  the scene — any shape, any BodyType, neither one replaced or otherwise
  modified. The Inspector's **Connections** section lets you pick another
  body and connect them (rest length = their current distance apart) with
  Stiffness/Damping sliders, and lists/removes existing connections on the
  selected body; Box Select's **Connect Selected with Springs** wires a
  whole selected group together the same way (a chain plus skip-one
  cross-braces, mirroring `makeSoftBodyRing`'s bracing). This is the
  general primitive; Ring/Jelly/Cloth are convenience presets built from
  the same idea on auto-generated particles rather than existing objects.
  Persists through Play/Stop/Reset by body name, the same way softBodies_
  does (see `EditorApp::connectWithSpring()`/`registerSoftBody()`). For a
  *rigid* connection instead (exact distance/pivot/weld/slide axis, not
  springy) see the `DistanceJoint`/`RevoluteJoint`/`WeldJoint`/
  `PrismaticJoint` family below (script-only for now, no Inspector/Box
  Select support yet).
- **Scriptable UI, on the viewport too**: `on_gui` controls render both in
  the dedicated **Script UI** panel *and* as an opaque HUD overlay (see
  Create/"Opaque by default" above for the one exception, Panel) directly
  on top of the **Viewport** — the same `ui.*` calls just run
  twice, once per ImGui window, so a click in either place is independent
  and works the same. The overlay only draws while the Viewport tab is
  actually the visible one (it shares a dock node/tab bar with **Code
  Editor** — see `viewportTabVisible_`); a body of the fix here was that
  `ImGui::Begin("Viewport")`'s return value (false when Code Editor is the
  active tab instead) was previously ignored, so the overlay kept drawing
  at the Viewport's last-known screen position on top of the Code Editor.
- **Project manager**: the app opens into a Godot-like project browser
  listing every project under `~/Physics2DProjects/` — **New Project**
  scaffolds a new one (an empty `scripts/` folder, nothing seeded into it),
  double-click or **Open** loads it into the editor, and **Delete** removes
  one (with a confirmation). The **Projects** button in the editor's
  toolbar saves nothing extra (everything's already on disk) and just
  returns you to this list. First-ever launch auto-seeds one "MyProject" so
  the list isn't empty.
- **Project folder**: each project gets its own `scripts/` folder, separate
  from this repo's own example scripts (see "Example scripts in `scripts/`"
  below — those stay as source/documentation, they're no longer copied
  anywhere). The **FileSystem** panel browses the *open* project's folder
  like Godot's FileSystem dock — right-click (or the toolbar buttons) to
  create new folders/scripts, delete files, and drag any `.lua` file
  straight onto the Inspector's *Script path* field to attach it. Click a
  file or folder to select it, then **Ctrl+C**/**Ctrl+V**/**Delete** copy,
  paste (into the selected folder, or next to the selected file, or the
  project root if nothing's selected), and remove it — mirrored by
  Copy/Paste/Delete buttons above the tree for mouse-only use. A brand new
  project's scene is a floor, two walls, 5 balls already dropped in
  (`ball_1`-`ball_5`), and a "Spawn Ball" button that adds more, one per
  click, at a random position/size/velocity — its script
  (`scripts/spawn_balls.lua`) is written into the new project's `scripts/`
  folder the first time it's created, a real editable file from the start
  rather than something baked into the editor binary.
- **Autosave and reload**: every project persists itself to
  `<project>/scene.json` every few seconds while editing, and loads it back
  the next time that project is opened — bodies, their properties, attached
  scripts, soft bodies, and spring connections are all exactly where you
  left them, instead of starting over from the blank sandbox each time. See
  "Project persistence" below for how the encode/decode logic is kept
  independently (and headlessly) testable.
- **Documentation, in the Project Manager**: a **Documentation** button on
  the project browser opens an in-app quick reference (editor controls,
  viewport tools, FileSystem shortcuts, and the full scripting API) so you
  don't have to leave the app to remember a Lua binding's exact name.
- **Scriptable UI**: a script's `on_gui(body)` hook runs once per rendered
  frame and can call `ui.button(...)`, `ui.text(...)`, etc. — bound
  directly onto the editor's ImGui panel for that frame, so `if
  ui.button("Spawn Cube") then ... end` reads exactly like calling ImGui
  from C++ would. Controls from every attached script appear together in
  the **Script UI** panel (tabbed with the Inspector). `ui.is_item_hovered()`/
  `is_item_clicked()`/`is_item_active()` give generic interaction detection
  on whatever widget was drawn last, for cases `ui.button()`'s own return
  value doesn't cover (e.g. detecting a click on `ui.text(...)`). See
  `scripts/spawn_cube_button.lua` for the runnable example.
- **Create (Hierarchy panel) — one button, any object, at (0, 0) or as a
  UI Element**: a single **Kind** dropdown covers ordinary physics shapes
  (**Circle**/**Box**/**Triangle**/**Pentagon**/**Hexagon**) AND UI kinds
  (**Button**/**Text**/**Checkbox**/**Slider**/**Input Field**/**Panel**) —
  **Create** makes one, and which of those two very different things it
  makes depends entirely on the Kind picked. A physics shape spawns at
  world **(0, 0)** as an ordinary body (**not** a UI Element) — no viewport
  click needed, unlike the Spawn tool. A UI kind creates a UI Element with
  **no script attached at all** (see below) — it renders itself immediately,
  usable right away. A **"..."** button next to Create opens a small popup
  with **Matter Kind** (Rigidbody/Matter/OptiMatter), **Body Type**
  (Static/Kinematic/Dynamic), and **Quantity** (1, 5, 10, 25, 50, 100, 500,
  or 1000) — all three only apply to physics shapes (a UI kind is never
  batched or given a Matter/Body Type). A batch spawns spread across a
  small grid centered on the origin, not stacked exactly on top of itself.
- **UI Elements render themselves now — no script required**: a UI Element
  created via Create is just a Static, sensor host body with `isUiElement`
  and a **Kind** set — `EditorApp::drawNativeUiElement()` draws its widget
  directly in C++ (the exact same behavior the generated Lua templates
  below give a *scripted* one), reading/writing `body.ui_text`/`ui_value`/
  `ui_min`/`ui_max`/`texture_path` exactly the same way. **Input Field** is
  a text box bound to `ui_text` (typing into the live widget and editing
  the Inspector's field land in the same place); **Panel** is a background/
  decoration rectangle, sized via the Inspector's **Panel Size** field (or
  an Appearance texture instead of a flat color fill) — the deliberate
  exception to the point below. You can still attach a script to any UI
  Element afterward (Inspector > Script path) for custom behavior beyond
  the default widget — a script, if present, always takes over from the
  native rendering (see "Marked" below); this is purely the no-script
  default, not a restriction.
- **Opaque by default; Panel is the deliberate exception**: every UI
  Element's floating window over the Viewport used to be translucent
  (60% background alpha) regardless of kind. That's now reserved for
  **Panel** specifically (it's meant as a background/decoration sitting
  behind other Elements you drag over it) — every other kind is fully
  opaque, so a button or slider doesn't look like a ghost floating over the
  scene.
- **FileSystem's New Script popup — the separate, script-first path**: for
  when you *do* want a generated template to build custom behavior on top
  of from the start (rather than attaching a script to an already-working
  native element later). Its own **Kind** dropdown is unchanged — **Plain
  Script** (an ordinary `on_start`/`on_update` stub, no host body, the
  default) or **Button**/**Text**/**Checkbox**/**Slider** (writes that
  Kind's `on_gui` template AND creates a live host body already running
  it, selected immediately) — deliberately not offering Input Field/Panel,
  since those two have no reason to *need* a script (there's no `ui.*`
  binding for either; native rendering is the only way they exist).
- **Marked, so a plain script can become one too**: every script
  `uiElementTemplateScript()` generates (via the FileSystem popup above)
  starts with a machine-readable `-- @p2d_ui_kind: <kind>` marker line
  (distinct from the human-readable comment below it). The FileSystem panel
  reads it back to badge a matching entry as `name.lua [UI: Button]` at a
  glance, and — more usefully — attaching a marked script to *any* body
  (drag onto the Inspector's Script path field, the Script Library
  dropdown, or Attach/Reload) auto-tags that body as a UI Element of the
  matching Kind, showing the same UI Text/Value/Min/Max fields it would
  have gotten from `createUiElement()`, even though it was never created
  through that flow — and, since a script always wins over native rendering
  when both are present, this is also how you'd give a Create-made native
  element real custom behavior later. Attaching a plain (unmarked) script
  never clears an already-configured UI Element's settings.
- **Texture and Hide Text, on UI Elements**: the Inspector's Appearance >
  Texture field (see above) works on a UI Element's host body exactly like
  any other body — set it and the widget (native or scripted) switches
  from a plain button/checkbox/slider to drawing that image instead (via
  `drawNativeUiElement()`'s `ImGui::Image`/`ImageButton` calls, or a
  scripted element's `ui.image`/`ui.image_button` — see Scripting API
  below). Once an icon speaks for itself, a **Hide Text** checkbox appears
  right under UI Text (for Button/Checkbox/Slider — not offered for
  Text/Input Field/Panel, none of which have a separate label) to drop the
  label and leave just the widget; the underlying ImGui widget keeps a
  stable, hidden id either way so it still works correctly with the label
  gone.
- **Upload Texture, and image files stand out in FileSystem**: since OS
  drag-and-drop (below) isn't available/reliable on every desktop
  environment, the FileSystem panel's **Upload Texture** button (also on a
  folder's right-click menu) copies an image file from anywhere on disk
  into the project via a typed path — the same collision-avoiding copy
  `importDroppedFiles()` already used for OS drops, just reachable without
  depending on XDND working at all. Image file entries in the tree are also
  rendered in a distinct color so they're visually easy to pick out from
  scripts at a glance (Dear ImGui has no italic font loaded in this build,
  so a color tint is the practical stand-in for that).
- **Matter — its own object type, not a rigidbody**: `p2d::Matter` is a
  genuinely separate simulation primitive from `p2d::Body`, not a cheaper
  rigidbody preset — see "Matter: an object type genuinely separate from
  Body" below for the full design and why. Script-only for now
  (`world:create_matter(x, y, radius, MatterKind.Matter)`); it renders in
  the viewport but has no Inspector/Spawn-tool/persistence support yet.
- **`Body::matterKind` — the same Matter/OptiMatter fidelity dial, on full
  rigidbodies**: an ordinary `Body` (any shape, rotation, scripts, UI
  Elements — everything Body already does) can opt into the same
  substep/sleep/correction handling as `Matter`/`OptiMatter` via an
  Inspector dropdown or `body.matter_kind` — see "`Body::matterKind`" below.
  Full editor support (Inspector, persistence, scripting) from day one,
  unlike `p2d::Matter` above. `MatterKind::Matter` is genuinely MORE
  realistic than the Rigidbody default (tighter sleep/correction/substep
  handling), not just a label — see below for what that means concretely.
- **Default Matter Kind (Settings)**: which `MatterKind` the Spawn tool
  gives every newly placed body — Settings > Spawning > "Default Matter
  Kind". `MatterKind::OptiMatter` by default; only affects bodies created
  *after* changing it, not anything already in the scene (change an
  existing body's own kind from its Inspector instead).
- **Raycasting and point queries**: `World::raycastClosest()`/`raycastAll()`
  cast a ray against every Body and Matter object (closest hit, or every
  hit sorted nearest-first); `World::queryPoint()` finds whatever's at a
  point. All three respect collision filtering (below) via an optional
  `mask` parameter. See "Physics engine capabilities" below.
- **Collision filtering**: `Body`/`Matter` both have `collisionCategory`/
  `collisionMask` (Box2D-style bitmasks, default "collides with everything")
  — checked before any narrowphase work runs at all, so a filtered-out pair
  costs nothing beyond the bitwise check.
- **Rigid joints**: `DistanceJoint`/`RevoluteJoint`/`WeldJoint`/
  `PrismaticJoint` — genuinely bilateral (equality) constraints solved by
  the same sequential-impulse solver contacts use, unlike `SpringJoint`
  (which is deliberately springy/force-based). A rope that holds its exact
  length, a door that swings on a real hinge, a fixed weld, a piston that
  only slides along one axis.
- **Continuous collision detection (opt-in)**: `World::enableCcd` (off by
  default) adds a genuine swept time-of-impact check for fast-moving
  circles, on top of adaptive substepping — see "Physics engine
  capabilities" below for what it does and doesn't cover.

## Handling massive object counts

**No crash, no leak.** Stress-tested at 10,000 bodies and at
60,000-bodies-created-over-a-session (continuously spawning and destroying
around a steady population of 500, simulating long-running churn): no crash
at any scale tried, and RSS memory stays flat after an initial ramp-up even
after 60,000 total creations — `World::createBody`/`removeBody` don't leak.
Object pooling (reusing body slots instead of allocating/freeing each one)
would still shave allocator overhead in high-churn scenes, but isn't needed
for correctness here — there's no leak to fix.

**Broadphase**: a spatial hash grid (the 2D-friendly equivalent of a
quadtree/octree) instead of an O(n²) all-pairs scan, verified to find
exactly the same candidate pairs as brute force (`tests/broadphase_test.cpp`).
Per-step cost stays flat rather than growing, both for a large contained
dense cluster and for a smaller population that keeps moving around over a
long run.

**Sleeping** (see Features above) is implemented and works, but a very
deep, densely-packed pile can take a long time to fully quiesce — low-level
jitter propagating through a large contact network before everything drops
below the sleep threshold is a real property of impulse-based solvers
generally (not unique to this one), not a bug to chase further.

**Mesh simplification** doesn't apply here: the engine only ever supports
circles and convex polygons — there's no high-poly mesh path to begin with,
so there's nothing to swap for a simpler collision shape.

## Physics engine capabilities: raycasting, filtering, joints, CCD

Asked directly "what does every great physics engine have" and told to just
build it — four genuinely new engine-level capabilities, on top of
everything above, each scoped deliberately rather than left silently
incomplete.

**Raycasting/queries** (`Collision.hpp`'s `raycastCircle()`/`raycastPolygon()`,
`World::raycastClosest()`/`raycastAll()`/`queryPoint()`): the circle cast is
the standard quadratic-formula solve; the polygon cast is Box2D's own
slab-clipping algorithm (successively narrowing an accepted `[lower, upper]`
fraction range against each edge's half-plane) ported directly rather than
reinvented, since it's exactly the right tool and well-proven. A ray
starting *inside* a shape never reports a hit for it — a raycast answers
"the first surface this ray reaches from outside," not an
already-overlapping query. `raycastClosest`/`raycastAll`/`queryPoint` all
take an optional `mask`, checked against each candidate's own
`collisionCategory` the same way ordinary filtering is.

**Collision filtering** (`Body`/`Matter`'s `collisionCategory`/
`collisionMask`): Box2D's exact category/mask scheme — two things collide
only if *each* side's category is present in the *other's* mask,
`(a.category & b.mask) != 0 && (b.category & a.mask) != 0`. Checked in
`World::broadAndNarrowPhase()` before any narrowphase geometry test runs at
all (a filtered-out pair costs one bitwise check, nothing more), and
identically in the CCD sweep below. Defaults (category bit 0, mask
all-ones) mean "collides with everything," so nothing changes until a
script/the Inspector actually sets one.

**Rigid joints** (`Joint.hpp`; solved in `World.cpp`'s `solveJointVelocities()`/
`correctJointPositions()`, mirroring the exact three-phase structure
(init effective-mass terms once → warm-start → iterate) `solveVelocities()`
already uses for contacts): `DistanceJoint` (exact distance, 1 constrained
DOF), `RevoluteJoint` (shared pivot point, 2 DOF, rotation free — a hinge),
`WeldJoint` (Revolute's point constraint *plus* a fixed relative angle, 3
DOF total), `PrismaticJoint` (slides along one axis, 2 DOF constrained, 1
free). Weld and Prismatic solve their two constraints as independent
sequential passes rather than one coupled system — a deliberate
simplification that still converges to a correctly rigid result at rest,
just not via the exact transient path a fully-coupled solve would take.
Position correction (not just velocity) is critical here in a way it isn't
for contacts: a joint's anchor is generally offset from its body's center,
so correcting position without *also* correcting rotation can't converge to
zero error, it just asymptotes at some nonzero residual — caught empirically
by `tests/joint_test.cpp` (a persistent ~0.22m pin error that never shrank,
traced to exactly this) before being fixed. The fix uncovered a second,
subtler issue: `correctPositions()`'s contacts use a *damped* (20%-per-step)
Baumgarte-style correction on purpose (a big one-shot correction on deep
penetration looks like an explosion — see "Fixing the spawn explosion"
below) — copying that same damping onto joints seemed like the safe,
consistent choice, but it's actually wrong for them: a joint's correction is
a *linearization* (exact only for infinitesimal position/rotation changes),
and repeatedly applying a *weakened* version of it doesn't shrink the
residual to zero, it converges geometrically to a **different, still-wrong**
fixed point instead (confirmed with gravity/velocity zeroed out entirely, so
only the position-correction recursion itself was in play: it plateaued at
a stable, nonzero pin error with 20% damping, and converged to ~0 in one
step at full strength). Box2D's real joint solver applies the full
correction, no damping factor — matching that (not the contact convention)
was the actual fix, verified with a throwaway diagnostic before landing in
`correctJointPositions()`.

**Continuous collision detection** (`World::enableCcd`, `applyCcd()`): after
`integrateVelocities()` moves a Dynamic circle Body or a Matter object, if
it moved more than `ccdMinMovementFraction * radius` this substep, its
straight-line path is swept (the same `raycastCircle`/`raycastPolygon`
above, with the mover's own radius added to the target's — or, for
polygons, used as `raycastPolygon`'s inflation parameter) against every
other Body/Matter; a hit pulls it back to just short of the first impact
point, so the *next* step's ordinary discrete narrowphase finds a small,
sane overlap to resolve normally instead of having already passed clean
through. Three deliberate scope boundaries, not bugs:
- **Circle movers only.** A swept polygon-vs-polygon time-of-impact test is
  a substantially larger undertaking, and a fast circle (bullet, particle,
  ball) is the overwhelmingly common tunneling case in practice.
- **Off by default, and exempt for `MatterKind::OptiMatter`** even when on.
  This check has no broadphase acceleration of its own — it sweeps against
  *every* other Body/Matter, unconditionally — so enabling it on a scene
  with many simultaneously-fast-moving circles is a real O(n)-per-mover
  cost, not a free upgrade. First shipped unconditional and default-on,
  which regressed `tests/broadphase_test.cpp`'s 2000-body performance
  budget roughly 5x (a genuinely fast-falling swarm, briefly, before
  settling) — turning it opt-in and OptiMatter-exempt was the fix, matching
  the existing "OptiMatter skips forced substeps" precedent exactly.
- **The swept-against target is treated as stationary for the sweep's
  duration**, not itself continuously moving — correct for a fast mover vs.
  anything slow/static (the common case), an approximation for two things
  both moving very fast at once.

**Not done: multithreading.** Deliberately not attempted this pass. It was
the lowest-value, highest-risk item on the original list — correctness
here is instead verified almost entirely through headless, deterministic
tests, and introducing concurrency (even scoped to just narrowphase contact
generation, which is embarrassingly parallel) trades that determinism for
race conditions that don't reproduce reliably and are much harder to catch
with the same testing approach that caught the two real bugs above. Worth
doing as a *separate*, carefully-tested pass if body counts genuinely
demand it — not bundled into this one under time pressure.

## Matter: an object type genuinely separate from Body

This started as a per-object performance dial: a `MatterKind` flag directly
on `Body`, so a single rigidbody could opt into cheaper treatment (fewer
substeps, eager sleep, rougher position correction) without turning down
quality for the whole scene. Asked directly whether that made "Matter" a
rigidbody, the honest answer was yes — it was still a full `Body` underneath
the whole time, just with a flag. Making it *genuinely* separate meant
picking one of two real designs: keep today's exact rotating-rigidbody
physics and just wrap it in its own class (cosmetic — the simulation
wouldn't actually differ), or give it real point-mass semantics: no
rotation, no torque, no moment of inertia at all. The second one is what
got built.

**`p2d::Matter`** is a point-mass: `position`, `velocity`, `mass` — nothing
else describing orientation, because there's no rotation to have one.
That's a real, structural difference from `p2d::Body` (which always carries
`rotation`/`angularVelocity`/`torque`/`invInertia`), not a cheaper preset of
the same thing. Deliberate scope decisions for this first version, all
differences from Body rather than bugs:

- **Always a circle.** A shape's corners only mean something once it can be
  oriented; something that never rotates has no reason to be anything but a
  circle. This also means Matter-vs-Body collision reuses the exact same
  circle-circle/circle-polygon math `Body`'s own circle shapes already use
  (`circleVsCircleRaw`/`circleVsPolygonRaw` in `Collision.cpp`, refactored
  to take a bare position+radius instead of a `Body` so both share it) —
  not a parallel reimplementation.
- **Always simulated.** No Static/Kinematic equivalent — an object that
  never moves has little reason to exist. Every `Matter` behaves like an
  ordinary Dynamic `Body` would.
- **No scripts, UI Elements, or spring joints (yet).** `Body`'s scripting/
  UI-Element/joint machinery is all keyed by `Body*`; extending it to
  `Matter` too is a reasonable future step, just not this one.

**It still collides with everything.** A `Matter` object generates real
contacts against other `Matter` *and* against ordinary `Body` rigidbodies
(walls, crates, ...) through `World`'s existing broadphase — one combined
AABB list (bodies then matter) through a single `SpatialHashGrid` pass finds
Body-Body, Matter-Matter, and Matter-Body pairs together, rather than three
separate grid builds. The solver math genuinely differs by pair kind: a
Matter-Matter contact has no moment-arm term on *either* side (neither
rotates, so a contact point's velocity IS just the object's velocity); a
Matter-Body contact keeps the Body's usual `r × impulse` rotational response
but drops it entirely for the Matter side — which means **a Matter object
can still torque a Body it hits off-center, it just never spins itself in
response** (`tests/matter_test.cpp` fires one at a box's edge and checks the
box's `angularVelocity` becomes nonzero from exactly that).

**`MatterKind` on `Matter`** works exactly as described above: OptiMatter
never forces extra substeps (`World::computeSubstepCount()`), always sleeps
at least as eagerly as `optiMatterLinearSleepThreshold`/
`optiMatterTimeToSleep`, and gets the larger `optiMatterMaxLinearCorrection`
cap on any contact it's part of — all verified in `tests/matter_test.cpp`
(an OptiMatter object sleeps sooner than an identically-dropped Matter
one; an OptiMatter bullet tunnels through a wall that catches an otherwise-
identical Matter one, since it doesn't force the substeps that would've
caught it). `MatterKind::Matter` is `Matter`'s own default and pulls the
opposite direction from OptiMatter — see `matter*` fields below, shared
with `Body::matterKind`.

### `Body::matterKind` — the same fidelity dial, back on Body too

Once `Matter` existed as its own real object type, the next ask was for something
that behaves like Matter/OptiMatter but keeps *every* rigidbody capability
— rotation, arbitrary shapes, scripting, UI-Element hosting. Duplicating
all of that a second time inside `Matter` would mean two parallel,
independently-maintained implementations of the same rigidbody machinery.
Instead, `MatterKind` (now `Rigidbody`/`Matter`/`OptiMatter`, shared between
both) went back onto `Body` itself as `Body::matterKind`, exactly as it was
in the very first version of this feature — a plain fidelity flag on an
otherwise completely ordinary rigidbody, so it's automatically compatible
with everything Body already does (rotation/torque/inertia, circle or any
polygon shape, `scriptPath`, `isUiElement`, Inspector editing, scene
persistence) with zero duplicate code.

- `MatterKind::Rigidbody` (default): an ordinary rigidbody, entirely
  unaffected by any of the handling below.
- `MatterKind::Matter`: **more realistic than Rigidbody**, not just a label
  — pulls every dial the opposite direction from OptiMatter: a tighter
  sleep threshold and longer `timeToSleep` (`matterLinearSleepThreshold`/
  `matterAngularSleepThreshold`/`matterTimeToSleep`, so it keeps simulating
  instead of settling early), a smaller, more gradual position-correction
  cap (`matterMaxLinearCorrection`), and a tighter continuous-displacement
  fraction (`matterContinuousDisplacementFraction`) that forces MORE
  substeps than a Rigidbody moving at the same speed would — genuinely
  higher fidelity, at real extra cost, matching the original "Matter will
  be realistic and optimized" framing.
- `MatterKind::OptiMatter`: same three effects as an OptiMatter-kind
  `Matter` object — exempt from forced extra substeps, looser sleep thresholds
  (now including an angular one, `optiMatterAngularSleepThreshold`, since a
  Body — unlike Matter — actually has angular velocity to check), and the
  larger `optiMatterMaxLinearCorrection` cap.

Set via the Inspector's "Matter Kind" dropdown (right under a body's Type),
`body.matter_kind` from Lua, or **Settings > Spawning > Default Matter
Kind**, which sets what the Spawn tool gives every newly placed body
(`MatterKind::OptiMatter` by default — existing bodies are never
retroactively changed by it). Verified in `tests/body_matter_kind_test.cpp`:
an OptiMatter body still gains angular velocity from an off-center hit
(proves rotation isn't disabled — this is a solver knob, not a
stripped-down object); an OptiMatter body sleeps sooner than an
identically-dropped Rigidbody, which itself sleeps sooner than an
identically-dropped Matter one; an OptiMatter bullet tunnels through a thin
wall that catches an otherwise-identical Rigidbody; an OptiMatter body
resolves a deep overlap by a visibly larger step than Rigidbody, which
resolves a visibly larger step than Matter; and a Matter body needs
strictly more substeps (checked directly via `World::onPreSubstep`'s
per-substep callback count) than an identically-moving Rigidbody.

`p2d::Matter` (the standalone object type above) is unaffected by any of
this and remains the lighter-weight option when full rigidbody features
genuinely aren't needed — `Body::matterKind` is for when they are.

**Script-only for now.** `world:create_matter(x, y, radius, kind)` spawns
one (`MatterKind.Matter`/`MatterKind.OptiMatter`), and the `Matter` usertype
exposes `.position`/`.velocity`/`.restitution`/`.friction`/
`.linear_damping`/`.gravity_scale`/`.matter_kind`/`.radius`/`.texture_path`/
`.density`/`.mass` (read)/`.is_awake` (read), plus `:apply_force(v)`/
`:apply_impulse(v)`/`:wake()`/`:set_velocity(x,y)`/`:set_position(x,y)` —
deliberately a smaller surface than `Body`'s (no `.rotation`, no
`.angular_velocity`, no `.type`, no `:apply_torque`, since none of those
mean anything for something that never rotates). It renders in the
viewport (`Renderer::drawMatterParticle` — considerably simpler than a
Body's circle case, since there's no rotation to keep a texture stable
against, so a plain `sf::CircleShape` suffices, no manual triangulation
needed), but has no Inspector editing, Spawn tool placement, Hierarchy
listing, or scene persistence yet — creating and tuning one is scripting-
only until that editor-side work happens.

## Making the soft body actually settle

Adding `p2d::SoftBody` (see Features above) surfaced a real, subtle
integration bug — worth documenting the same way the broadphase work above
is, since it's the same kind of lesson: stress-test rather than reason
abstractly, and believe the numbers over the intuition.

The first version applied every spring's Hooke's-law force once, right
before calling `World::step(dt)`, the same way any one-off `applyForce()`
call is normally used. `tests/soft_body_test.cpp` dropped a 10-particle
ring onto a floor and checked it settled — instead, particles were still
moving at 10-20 m/s after 10 simulated seconds, and the ring's centroid
drifted several meters sideways with no lateral force ever applied to
explain it.

The cause: `World::step(dt)` doesn't always run the physics at `dt` — it
calls `computeSubstepCount(dt)` and, for fast-moving bodies, subdivides
into several smaller **substeps** internally (that's the adaptive
substepping anti-tunneling feature). `integrateForces(subDt)` consumes each
body's accumulated `force` and zeroes it — on the *first* substep only.
A spring force set once before `step()` was therefore only ever actually
applied on that first substep; every later substep saw stale geometry (the
positions hadn't been re-read) and zero additional force. The faster
things moved, the more substeps triggered, and the more of the "continuous"
spring force went missing relative to what the current geometry called
for — a feedback loop with no clean equilibrium.

The fix: `World` grew a new `onPreSubstep` hook, called at the start of
*every* internal substep (before `integrateForces`), so a host can
recompute a continuous force from current positions/velocities at exactly
the resolution the engine is actually integrating at. `SoftBody`'s spring
forces are registered there instead of called once before `step()`.

That fix alone didn't fully explain the numbers, though — isolating the
question further with a throwaway 2-particle spring pair (no gravity, no
floor, no other bodies) in `/tmp`, even *with* the `onPreSubstep` fix
in place, showed the same failure to settle at first. Turned out to be a
second, independent issue: the damping term (`F = stiffness * stretch +
damping * closingVelocity`, integrated forward via `applyForce`) has its
own, much tighter numerical stability bound than the stiffness term does —
empirically, `damping * dt / mass` needs to stay well under 2 for this
explicit discretization, where the stiffness term's bound is closer to 4.
The original defaults (`stiffness=220, damping=3, particleRadius=0.12`)
crossed both bounds at once for a particle carrying multiple springs;
tuning to `stiffness=50, damping=2, particleRadius=0.15` (see
`SoftBody.hpp`'s doc comment for the full reasoning) keeps comfortable
margin on both, and `soft_body_test.cpp` now asserts real settling
(bounded shape, low velocity after 10 simulated seconds) rather than just
"doesn't crash" — the previous failure mode wouldn't have been caught by a
NaN/crash check alone, since the ring's *shape* stayed intact throughout;
only its energy never dissipated.

Adding Jelly (a hub particle spoked to every ring particle, at up to
`segments` springs on one particle — worse fan-in than the ring's own
worst case) and Cloth (interior particles carrying 4 structural + 4 shear
springs at once) meant re-checking this margin rather than assuming it
still held: both were verified the same way, via a throwaway `/tmp`
diagnostic dropping/hanging one and watching centroid drift and settled
velocity over 10-15 simulated seconds, before writing them into
`SoftBody.cpp` and `soft_body_test.cpp` for real. Both settle comfortably
within the same `stiffness=50, damping=2` budget (the jelly hub just gets
1.5x the particle radius, for extra inertia against its own higher
fan-in) — worth re-verifying rather than assuming, since "one more spring
per particle" is exactly the kind of change the earlier bug hid inside.

## Fixing the "spawn explosion"

A reported bug: spawning something (via a script, or `world:create_*`)
inside/overlapping an existing body made everything fly apart violently —
only in full-accuracy mode, not with Optimization Mode on (a scene-wide
toolbar toggle this project had at the time, since removed in favor of the
per-body Matter Kind dial — see "`Body::matterKind`" below), which was the
clue.

`World::correctPositions()` resolves contact penetration directly (a
position nudge, not a velocity change — see its comment) by a fixed 20%
of the *remaining* penetration each step, which sounds bounded — 20% of
anything converges. But nothing capped the *absolute* amount: 20% of a
huge initial overlap (two bodies spawned dead-center on each other) is
still huge, and since `correctPositions()` runs once per internal substep
(see `World::step()`'s adaptive substepping), full-accuracy mode's up to
8 substeps per rendered frame could resolve most of a large overlap
within a single visible frame — reads exactly like an explosion.
Optimization Mode's `maxSubsteps = 1` masked this by spreading the same
correction over many more (individually still-large) rendered frames
instead of compounding it into one.

The fix mirrors Box2D's actual `b2_maxLinearCorrection`: clamp the
*absolute* amount resolved per contact per step (`World::maxLinearCorrection`,
0.2m default), on top of the existing 20%-of-remaining-penetration
fraction. `tests/spawn_overlap_test.cpp` drops two same-center, radius-2.0
circles (maximal overlap for that shape) and checks no single step
resolves more than a small bound of the separation — confirmed to
actually catch the regression by temporarily reverting the clamp and
re-running: the largest single-step jump went from 0.2m (fixed) to 0.799m
(reverted, i.e. resolving essentially the *entire* required separation in
one step) for the exact same scenario.

## Fixing the mass-overlap explosion (a deeper bug behind the same symptom)

The fix above wasn't the whole story. A follow-up report: spawning ~1000
circles directly into an already-settled ~1000-circle pile still sent
everything flying — not a slow, one-step snap this time, but velocities
that kept *climbing* over several seconds, unbounded.

Isolating it (`/tmp` diagnostics, not the shipped tests) found this had
nothing to do with the fix above: even just ~1000 circles falling and
settling into a dense pile on their own (no second spawn at all) showed
the same runaway growth once restitution was nonzero. The tell: it got
**worse** with **more** `velocityIterations` (8 → 30 → 100 pushed the
pile's resting height from 23 up to 636). More iterations making a bug
worse, not better, means it isn't a convergence problem — a correctly-
converging solver approaches the right answer faster with more iterations,
it doesn't diverge harder. Confirmed with `restitution = 0`: the exact
same scene settled perfectly regardless of iteration count. The bug was
specifically in how restitution and warm-starting interact.

`World::solveVelocities()` computes each contact's restitution bias
(`pc.velocityBias`) from the two bodies' current relative velocity, then
applies that contact's warm-started impulse — and the *original* code did
both, contact by contact, in the same loop. For a handful of simultaneous
contacts (an ordinary stack) that's harmless: by the time contact 2 reads
velocities, contact 1's warm-start perturbation is negligible. But for a
body touching many neighbors at once (exactly what "spawn 1000 into a
settled 1000" produces), later contacts in the list computed their bias
from velocities *already* perturbed by earlier contacts' warm-start
impulses in the same pass — contaminated relative velocity → a spurious
restitution bias → a bigger warm-started impulse carried into next step →
more contamination. A real feedback loop, not noise.

The fix: split the single interleaved loop into two passes — first
compute every contact's `velocityBias` from a consistent, unperturbed
velocity snapshot, *then* apply all warm-started impulses in a second
pass. Verified against the existing suite before trusting it: simply
*removing* warm-starting entirely also "fixes" the explosion, but was
rejected because it breaks `stacking_test.cpp` (warm-starting is what
makes ordinary box-stacking stable in the first place) — the two-pass
split fixes the explosion *and* keeps stacking passing.
`tests/mass_overlap_test.cpp` settles ~1000 circles, spawns another ~1000
directly on top, and asserts speed decays back down within 20 simulated
seconds rather than still climbing — confirmed to catch the regression by
reverting to the interleaved version and re-running (speed still elevated
at 3.63 m/s 20s later, vs. 1.29 m/s with the fix, on the same seeded
scene).

## Fixing the never-settling friction jitter

A reported bug: place two boxes, one flush on top of the other, and the
stack *looks* at rest, but its friction never actually settles — the
contact's tangent impulse keeps flipping between a tiny positive and
negative value in periodic bursts, forever, instead of converging once.

Two separate things were going on here, found by hooking `World::onContact`
and logging `tangentImpulse`/`isAwake`/`sleepTime` together over a full run.

**The recurring trigger:** the two boxes' sleep timers (`Body::sleepTime`)
are independent, and cross `World::timeToSleep` a step or two apart —
routine floating-point noise, not a sign either one is actually still
moving. Whichever crosses first goes to sleep, but `World`'s contact
generation unconditionally wakes any sleeping body touched by one that
isn't *also* asleep (`isEffectivelyStatic()` needs *both* sides asleep to
skip re-testing a pair) — so it's immediately woken right back up next
step, with its velocity hard-reset to zero and then one step of ungoverned
gravity before the solver catches it again. That hard reset perturbs the
warm-started contact impulses, and since the same desync recurs every time
either body's timer next crosses the threshold, this repeated forever: the
stack never reached a lasting sleep at all. The fix generalizes what every
mainstream engine actually does — bodies that touch each other sleep and
wake as one *island*, not independently. `World::updateSleepState()` is now
two passes: first each awake dynamic body's own timer updates from its own
velocity as before; then, for every current contact between two awake
dynamic bodies, if either side isn't sleep-ready yet, neither is (a cheap
single-step approximation of full connected-component propagation — an
ordinary pair or short stack converges within the same step; a long chain
may take a few extra steps, which only delays sleep slightly and is far
cheaper than iterating contacts to a fixed point every step).

**The visible symptom, on top of that:** each forced re-wake's hard reset
is exactly the kind of transient that can flip polygon-polygon SAT's
reference-face tie-break. For a flush contact, both candidate separations
sit right at 0, so ordinary noise can flip which face wins the tie from one
step to the next — and since the fixed geometric tangent the friction term
uses is derived from the contact normal, that flip alternates the sign of
the tangent impulse it's warm-started against. `generateContact()` now
takes an optional previous-step normal hint (threaded through from
`World`'s existing warm-start bookkeeping — no new state, just reusing what
was already being carried over) and only consults it to break an otherwise
ambiguous tie, so a resolved tie stays resolved the same way it was last
step instead of chasing sub-epsilon noise.

Fixing just the sleep desync (the actual root cause of "forever") already
stops the perpetual pattern, since the whole trigger stops recurring; the
SAT tie-break hysteresis on top of it shortens and shrinks whatever
transient a *genuine* wake (a new object landing on the stack, a script
push, etc.) still causes. `tests/friction_jitter_test.cpp` drops two flush
boxes on a floor, runs 5 simulated seconds, and asserts both reach a
lasting sleep (the bug's boxes never did) and that the tangent impulse's
sign stays stable through the back half of the pre-sleep window (a burst of
flips right after initial impact is normal damped convergence, not the
bug — what mattered was that a *quiet* back half exists at all).

## Project persistence: autosave and reload

Every project now saves itself: while in edit mode, `EditorApp::update()`
autosaves the current checkpoint (the same data `captureSnapshot()` already
builds continuously, see Features above) to `<project>/scene.json` every
few seconds, and the next time that project is opened, it's loaded back
instead of rebuilding the hardcoded blank sandbox — bodies, their
properties, attached scripts, soft bodies, and spring connections are
exactly where they were left.

The encode/decode logic (`app/src/ScenePersistence.{hpp,cpp}`)
deliberately has zero dependency on SFML/ImGui/`ScriptEngine`/`EditorApp`
— it operates on a plain `p2d::World` plus the `SoftBody`/`SpringJoint`
lists, with script attachment injected as a callback rather than a direct
`ScriptEngine` call. That's what makes `tests/scene_persistence_test.cpp`
possible at all: everywhere else, this codebase's own stated limitation is
that `EditorApp`/UI-layer work can only be checked by compilation and code
review, never headlessly, because it's tightly coupled to a live SFML/
ImGui context this sandbox doesn't have. Structuring the *serialization*
as an `EditorApp`-independent module sidesteps that entirely for this one
feature, and it's exactly the kind of feature (JSON schema, reconstruction
order, name-based re-linking) worth having real regression coverage for
rather than trusting on inspection alone.

**Closing the save-timing gap:** the periodic autosave alone had a real
hole — attach a script (or make any other edit) and immediately click
**Projects** or close the window, and `main.cpp` destroys the `EditorApp`
right then, via `unique_ptr::reset()` or just program exit. If that
happened to land inside the few-second window before the next autosave
tick, the edit was silently never written to disk. `EditorApp` now has an
explicit destructor that forces one final `saveSceneToDisk()` (only if not
mid-Play, matching every other checkpoint/persistence rule in this app —
see "Fixing the mass-overlap explosion" and Reset Scene above for the same
"only the edit-mode state persists" principle applied elsewhere), so
leaving a project — by any path — always saves whatever was last edited,
not whatever the timer happened to catch.

## Building

Requires CMake >= 3.16, a C++17 compiler (tested with GCC 15), Ninja (or any
CMake generator), and an internet connection the first time you configure —
dependencies (SFML, Dear ImGui, ImGui-SFML, Lua 5.4, sol2) are fetched via
`FetchContent` and built from source, nothing needs to be installed system-wide.
On Linux you'll need the usual X11/OpenGL/udev/freetype development headers
(`libx11-dev`, `libxrandr-dev`, `libxcursor-dev`, `libgl1-mesa-dev`,
`libudev-dev`, `libfreetype-dev` on Debian/Ubuntu; equivalent `-devel`
packages elsewhere) — these are what SFML's window/graphics modules build
against.

```sh
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

This builds the visual editor plus fifteen headless test targets:

- `build/app/p2d_editor` — the visual editor
- `build/tests/p2d_smoke_test` — headless numerical checks on the physics
  engine (resting contacts, no tunneling/explosions across all shape pairs)
- `build/tests/p2d_properties_test` — headless checks on damping, gravity
  scale, fixed rotation, and sensors
- `build/tests/p2d_broadphase_test` — spatial hash grid vs. brute-force
  equivalence, a contained dense-cluster stability check, and a
  wandering-population stability check (see "Handling massive object
  counts" below for what these guard against)
- `build/tests/p2d_sleeping_test` — bodies actually sleep, stay put while
  asleep, and wake from a force or an incoming collision
- `build/tests/p2d_stacking_test` — a settling box stack comes to (and
  stays at) rest without sinking, sliding, or gaining energy
- `build/tests/p2d_tunneling_test` — adaptive substepping actually prevents
  a fast bullet from tunneling through a thin wall
- `build/tests/p2d_soft_body_test` — a dropped soft-body ring settles
  (bounded shape, low velocity) rather than gaining energy indefinitely
  (see "Making the soft body actually settle" below)
- `build/tests/p2d_spawn_overlap_test` — spawning a body directly on top of
  another resolves the overlap gradually, not in one explosive step (see
  "Fixing the spawn explosion" below)
- `build/tests/p2d_spring_joint_test` — headless checks on `p2d::SpringJoint`
  connecting two arbitrary existing bodies
- `build/tests/p2d_mass_overlap_test` — spawning ~1000 circles onto an
  already-settled ~1000-circle pile decays back to rest instead of
  climbing in velocity forever (see "Fixing the mass-overlap explosion"
  below)
- `build/tests/p2d_friction_jitter_test` — two boxes stacked flush on top of
  each other reach a lasting sleep, and their contact's friction impulse
  settles rather than jittering in small bursts forever (see "Fixing the
  never-settling friction jitter" below)
- `build/tests/p2d_scene_persistence_test` — headless checks on the
  scene.json encode/decode logic (see "Project persistence" below)
- `build/tests/p2d_script_test` — headless checks on the Lua binding layer
  (including the newer property bindings)
- `build/tests/p2d_bacteria_test` — headless checks that run the default
  bacteria scene's script for 40 simulated seconds (reproduction, aging,
  death, and a full `reset()` afterwards) — this is the exact scenario that
  used to crash before the reset/reentrancy fixes described below
- `build/tests/p2d_project_paths_test` — headless checks on the project-folder
  scaffolding logic (creates `scripts/`, seeds bundled examples, never
  clobbers an existing project on re-run)

Run all tests with `ctest --test-dir build` or invoke the binaries directly.

Set `-DP2D_BUILD_APP=OFF` to skip the SFML/ImGui dependency chain entirely
(e.g. for a headless CI environment) and only build the engine, scripting,
and tests.

> **Note on this sandbox:** the environment this project was built in has no
> display server (no X11, no Wayland compositor reachable, no Xvfb), so the
> editor binary compiles and links cleanly but can't actually be shown or
> click-tested here — running it here fails immediately with `Failed to open
> X11 display`, not a bug in the code (confirmed with gdb: SFML's
> `RenderTexture` eagerly creates a shared GL context on construction, before
> `.create()` is ever called, which is what actually needs the display). The
> physics, scripting, and project-scaffolding layers were instead verified
> headlessly via the test binaries above — anything that touches SFML/ImGui
> rendering (the docked layout, drag-and-drop, the theme, the code editor
> panel) could only be reasoned about, not run. One dependency needed a
> source patch to build at all here (`cmake/patch_texteditor.cmake`; see its
> comment for why) — that much is build-verified, but the code editor's
> actual on-screen behavior (highlighting, line numbers, tabbing with the
> Viewport) is not. Build and run `p2d_editor` on your own machine to use
> and check the visual interface.

### A note on OS drag-and-drop

Dragging a file in from the desktop file manager (see the FileSystem panel
above) is not an `sf::Event` — SFML has no `FileDropped` event of its own,
even in 2.6, the version this project uses. `app/src/FileDropWatcher.{hpp,cpp}`
implements XDND (the X11 drag-and-drop protocol) directly: it opens its own
Xlib connection to the editor window's native handle (X11 allows more than
one client to select events on the same window, so this never competes with
or interferes with SFML's own event pump), advertises `XdndAware` on that
window, and handles the `XdndEnter`/`XdndPosition`/`XdndDrop` handshake,
converting the `XdndSelection` to `text/uri-list` to get the dropped files'
local paths. It's Linux/X11-only, matching this project's own build/run
instructions (a harmless no-op stub everywhere else) — and, like the rest of
the visual editor, this is exactly the kind of interactive-gesture feature
that can't be driven or observed at all in this sandbox (there's no drag
source to drop *from* even if a display were reachable). It compiles
cleanly and follows the documented protocol precisely, but has not been
confirmed against a real drag from a real file manager — try dragging an
image onto the FileSystem panel on your own machine to check it. If it
doesn't work reliably on your desktop (Wayland/XWayland compositors in
particular can be inconsistent with legacy XDND), the FileSystem panel's
**Upload Texture** button sidesteps this whole mechanism entirely — it
copies an image into the project from a typed path instead, using the same
`importImageFile()` copy logic `importDroppedFiles()` calls, just reachable
without depending on the OS-level drag protocol at all.

## Running the editor

```sh
./build/app/p2d_editor
```

It opens into the **project manager**: pick an existing project (double-click
or **Open**) or click **New Project** to create one; a **Documentation**
button here opens an in-app quick reference covering all of this. Every new
project starts with a small starter scene: a floor, two walls, 5 balls
already dropped in, and a "Spawn Ball" button wired to a real script
(`spawn_balls.lua`, written into that project's own `scripts/` folder the
first time it's created) — something to press Play on immediately rather
than a completely empty scene. The other example scripts described later in
this document (bacteria, bouncy ball, spawner, ...) live only in this repo's
own `scripts/` folder as runnable reference material/test fixtures; copy one
into a project's FileSystem panel yourself if you want to explore it.

- **Projects** — top-left of the toolbar; returns to the project manager
  (nothing to save first, the project's already on disk).
- **Play / Pause / Step / Reset Scene** — next in the toolbar. **Reset
  Scene** restores the scene to how it looked the last time the project was
  in genuine edit mode — not a hard rebuild once you've played at least
  once, and *not* satisfied by merely pausing (see Features above); the
  **Settings** button next to it opens a small window with a "Keep objects
  spawned during Play after Reset" checkbox if you'd rather Reset preserve
  whatever's currently in the scene, a "Show circle direction line" toggle,
  and a **Background** section — a Color picker (drawn first, always
  visible), a Texture path field (type/paste, or drag an image in from the
  FileSystem panel) with a Clear button, and a **Tiled** checkbox (on:
  repeats every couple world units and pans/zooms with the scene; off:
  stretched to exactly cover the current view). Both the color and texture
  persist with the rest of the scene (see "Project persistence" below). The
  toolbar also shows whether you're **Editing** or **Playing** and a live
  body count.
- **Gravity Y** — slider in the toolbar. **Time Scale** has both a slider
  and a numeric input field next to it for typing an exact value (0 freezes
  the simulation without leaving Play mode; values above 1 fast-forward it).
- **Tool: Select / Move** (default) — click a body to select it (shown in
  Hierarchy + Inspector), then drag to move it. While **Editing** this
  teleports the body exactly where your mouse is (use it to place things
  precisely, like arranging nodes in Godot); while **Playing** the same drag
  instead pulls the body with a spring so you can poke a live simulation.
  This works on *any* body, including static walls — pause and drag the dish
  walls around if you want a different container shape. If a **Box Select**
  group is active, dragging any one member moves the whole group.
  **Ctrl+C**/**Ctrl+V** copy/paste the selected body (or the whole Box
  Select group, preserving its layout) — pasted copies land offset from
  the original, cascading further out with each repeated Ctrl+V so they
  don't stack exactly on top of each other. Gated on the Viewport being
  focused *or* hovered (see the FileSystem panel below for why hover is
  included, not just focus).
- **Tool: Spawn** — a **Shape** dropdown next to it picks what left-clicking
  the viewport places: Circle, Box, Triangle, Pentagon, or Hexagon. Soft
  bodies (Ring/Jelly/Cloth) and UI Elements aren't here — see Features
  above for where to create those instead.
- **Tool: Resize** — click a body, then drag to resize its radius (circles/
  regular polygons) or half-extents (boxes); mass/inertia are recomputed on
  release.
- **Tool: Rotate** — click a body, then drag around it; it rotates to face
  wherever the mouse currently is.
- **Tool: Box Select** — drag a rectangle in the viewport; every body whose
  center falls inside it is selected together (Hierarchy/Inspector switch
  to a group view with a **Delete All Selected** button).
- **Pan** — right or middle mouse drag in the viewport
- **Zoom** — mouse wheel over the viewport
- **Hierarchy panel** — lists every body in the scene (click to select, same
  as clicking it in the viewport); a UI Element is tagged `(UI)`. Its
  **Kind** dropdown (Circle/Box/Triangle/Pentagon/Hexagon, or
  Button/Text/Checkbox/Slider/Input Field/Panel) and **Create** button make
  one immediately — a physics shape at world (0, 0), or a script-free UI
  Element — no viewport click, no naming popup; the **"..."** button next
  to Create holds Matter Kind/Body Type/Quantity for the physics-shape case
  (see Features above).
- **Inspector** — edit the selected body's transform, velocity, type, and:
  - **Radius** (circles) or **Half Width / Height** (everything else,
    rebuilt as a box — same as the viewport's Resize tool) right under
    Rotation: a direct numeric alternative to dragging with Resize.
  - *Material*: Bounciness (restitution, slider 0-15) and Friction (slider
    0-20) both go well past their "physically sane" range (1 and ~1-2
    respectively) on purpose — a slider that stops exactly at realism
    can't reach the exaggerated, cartoonish extremes a toy/prototyping
    sandbox is often more fun with. Density.
  - *Appearance*: a **Color** picker, fixed once set — independent of the
    body's type or whether it's currently asleep (see the Awake/Asleep
    indicator below for that status instead) — and a **Texture** field
    (type/paste a path, or **drag an image straight in from the FileSystem
    panel**) that draws an image over the body instead of its flat color,
    mapped onto the body's own local shape so it stays fixed under
    rotation and camera movement rather than sliding; a **Clear Texture**
    button removes it and falls back to Color.
  - *Dynamics*: Linear/Angular Damping, Gravity Scale, Fixed Rotation, Is Sensor
  - **UI Text**/**Hide Text**/**UI Value**/**UI Min / Max**/**Panel Size**
    (only shown for a UI Element's host body, and only the fields its Kind
    actually uses — see Features above): UI Text/Value/Min/Max are read
    *and* written live every frame — by `drawNativeUiElement()` for a
    script-free element, or by the attached script for a scripted one —
    so editing them here changes the widget immediately, without opening
    the code editor. Panel repurposes UI Min/Max as width/height instead
    (its own **Panel Size** field) and has no text at all. Hide Text (not
    offered for Text/Input Field/Panel) drops the label, useful once
    Appearance's Texture field has given the element an icon of its own.
  - *Script*: pick a file from the **Script Library** dropdown, or **drag a
    `.lua` file from the FileSystem panel straight onto the Script path
    field** — either one attaches it immediately, no extra click needed.
    **Attach / Reload** is still there for typing/pasting a path by hand
    (or manually re-running a script after editing it outside the app).
    Click **Edit** to open the attached script in the built-in code editor.
    Script errors show up in red in the Console panel.
  - An **Awake / Asleep** indicator shows whether the body is currently
    being simulated (any edit here, or a collision, wakes it).
  - *Connections (springs)*: pick another body from the **Connect to**
    dropdown, set Stiffness/Damping, and **Connect** — ties the two
    together with a spring (rest length = their current distance apart);
    both stay themselves, nothing is replaced. Existing connections on the
    selected body are listed below with a per-connection **Disconnect**.
  - With a **Box Select** group active, the Inspector instead shows a body
    count, a **Delete All Selected** / **Clear Selection** pair, and
    **Connect Selected with Springs** (wires the whole group together,
    same bracing idea as a Ring).
- **FileSystem panel** (bottom-left) — browses your project's `scripts/`
  folder. Toolbar buttons or a right-click on any folder give you **New
  Folder** / **New Script** / **Upload Texture** / **Delete**. **New
  Script** opens a popup with a name field and a **Kind** dropdown: **Plain
  Script** scaffolds an ordinary `on_start`/`on_update` stub (no host body —
  the only option before UI Elements existed, and still the default);
  **Button**/**Text**/**Checkbox**/**Slider** instead write that Kind's
  `on_gui` template *and* create a live host body running it, selected
  immediately (this is the script-*generating* path — the Hierarchy panel's
  **Create** button, see above, makes a script-free UI Element instead,
  including the Input Field/Panel kinds this popup doesn't offer).
  Any `.lua` file the generated template wrote is tagged with a hidden
  marker comment, shown in this panel as a `[UI: Button]`-style suffix on
  the entry itself; drag any `.lua` entry onto the Inspector's Script path
  field to attach it (a marked one auto-configures the target body as that
  Kind of UI Element even if it wasn't created through this popup — see
  Features above), or **double-click it to open it in the code editor**.
  Image file entries (`.png`/`.jpg`/`.jpeg`/`.bmp`/`.gif`/`.tga`) are
  rendered in a distinct color so they stand out from scripts at a glance.
  Click any file/folder to select it, then **Ctrl+C** /
  **Ctrl+V** / **Delete** copy, paste, and remove it (paste lands in the
  selected folder, next to the selected file, or the project root if
  nothing's selected) — the same
  Copy/Paste/Delete buttons above the tree work with just the mouse.
  **Upload Texture** copies an image file from anywhere on disk into the
  project via a typed path — a reliable alternative to OS drag-and-drop,
  which isn't available/consistent on every desktop environment. **Drag an
  image file in from your desktop file manager** also works where XDND is
  supported, copying it straight into the project's `scripts/` folder,
  where it shows up like any other file (selectable, copyable, deletable) —
  see `app/src/FileDropWatcher.{hpp,cpp}` and "A note on OS drag-and-drop"
  below. Once it's in the project (either way), an image entry is itself a
  drag-*out* source (alongside `.lua`'s existing one) with its own payload —
  drag it onto the Inspector's **Texture** field to texture the selected
  body, or onto the Settings panel's **Background Texture** field to use it
  as the scene backdrop (tiled or stretched — see the Settings entry
  above).
  The keyboard shortcuts are gated on this panel being focused *or* hovered,
  not focus alone — a click on a tree row doesn't reliably hand this
  window keyboard focus away from whatever had it before (e.g. a text
  field left over from a just-closed popup), so focus-only silently
  dropped Ctrl+C/Ctrl+V in practice; hovering the panel is unambiguous
  enough to act as a safe fallback.
- **Code Editor** — closed by default; opens (and switches to) when you
  double-click a script in FileSystem or click **Edit** in the Inspector.
  Line-numbered, Lua-syntax-highlighted, scrollable. It's docked as a *tab
  of the Viewport itself* — so you're always looking at either the
  simulation or the code, never both at once, same as Godot's 2D/Script
  tabs — click **Save** to write immediately, or just keep typing: it
  **autosaves** ~2 seconds after your last keystroke (and auto-reloads onto
  the selected body, if that's the script it has attached).
- **Script UI panel** (tabbed with the Inspector) — shows whatever controls
  the currently attached scripts' `on_gui()` hooks define, one after
  another in a single stacked list; the bundled `spawn_cube_button.lua` and
  `spawn_1000_button.lua` add buttons here if attached. The *same* controls
  also render over the **Viewport** itself, so you don't have to switch
  panels to use them while watching the simulation — but there, unlike the
  Script UI panel's fixed list, **each attached script gets its own small,
  opaque window that you can drag anywhere over the viewport** (no
  title bar, but grab it anywhere and move it, Godot-Control-node style) —
  translucent only for the Panel UI Element kind, everything else is fully
  opaque (see Create/Features above).
  Wherever you drop one is remembered per-body (`Body::uiOverlayX/Y`, a
  screen-pixel offset from the viewport's own top-left corner, so it stays
  put across camera pan/zoom) and persists with the rest of the scene —
  first appearance cascades new ones so they don't all land in the exact
  same spot, but after that it's exactly where you last left it. The drag
  is clamped so it can't pull a window outside the viewport image (it
  snaps back in immediately rather than hanging off the edge into another
  panel), and it's only available while **Editing**: during **Play**, the
  same windows are still fully usable (a button still clicks, a slider
  still drags) but can no longer be repositioned, so you can't accidentally
  relocate a control mid-interaction while actively testing the scene.
- Every panel (Toolbar, Viewport/Code Editor, Hierarchy, FileSystem,
  Inspector/Script UI, Console) is a normal dockable ImGui window — drag its
  tab to rearrange the layout, same as Godot or any other docking editor.

Each project lives at `~/Physics2DProjects/<name>/` (see the project
manager above) with its own empty `scripts/` folder — separate from, and
untouched by, this repo's own `scripts/` folder below, which is example/
reference material only and is never copied into a project.

## Scripting API

Attach a script to a body (from C++: `scriptEngine.attachScript(body, path)`;
from the editor: pick or drag a script into the Inspector's Script path
field — both attach it immediately).

```lua
-- called once when the script is attached
function on_start(body)
    body.restitution = 0.8
end

-- called every fixed physics step while attached
function on_update(body, dt)
    body:apply_force(Vec2.new(0.0, 20.0))
end

-- called once per rendered frame (not per physics step) -- add controls to
-- the editor's Script UI panel; ui.button reads exactly like ImGui's own
-- immediate-mode idiom (true on the frame it's clicked)
function on_gui(body)
    if ui.button("Spawn Cube") then
        world:create_box(0.0, 8.0, 0.4, 0.4, BodyType.Dynamic)
    end
end
```

Bound API surface:

| Type/Global | Members |
|---|---|
| `Vec2` | `Vec2.new(x, y)`, `.x`, `.y`, `:length()`, `:normalized()`, `:dot(v)`, `+`, `-`, `*` |
| `Body` | `.position`, `.rotation`, `.velocity`, `.angular_velocity`, `.restitution`, `.friction`, `.density` (settable, recomputes mass), `.mass` (read), `.inertia` (read), `.linear_damping`, `.angular_damping`, `.gravity_scale`, `.fixed_rotation`, `.is_sensor`, `.is_awake` (read), `.name`, `.type` (settable -- recomputes mass immediately, e.g. `Static` \| `Kinematic` both go to mass 0), `.matter_kind` (`MatterKind.Rigidbody`/`.Matter`/`.OptiMatter` -- solver fidelity dial, see "`Body::matterKind`" above; doesn't remove/change any other field here), `.collision_category`/`.collision_mask` (Box2D-style filtering, see "Physics engine capabilities" above), `.radius` (0 for a non-circle shape; settable ONLY while already a circle -- silently ignored otherwise, use `:set_circle()` below to convert), `.color_r`/`.color_g`/`.color_b` (0-255, settable individually), `.texture_path` (a plain string -- empty means no texture; Inspector-editable via Appearance's "Texture" field, drag-and-drop included, see Features above), `.ui_text` (a plain string -- UI Element templates read this for their label; Inspector-editable via the "UI Text" field), `.ui_value` (a checkbox's checked state or a slider's current value; read AND written by those templates every frame, Inspector-editable via "UI Value"), `.ui_min`/`.ui_max` (a slider's range, Inspector-editable via "UI Min / Max"), `.ui_hide_text` (drops the label -- Inspector-editable via "Hide Text", not offered for the Text kind), `:set_color(r,g,b)` (all three channels at once, 0-255), `:set_circle(radius)`/`:set_box(hw,hh)`/`:set_polygon(sides,circumradius)` (rebuild this body's shape from scratch -- unlike `.radius`, these convert BETWEEN shape kinds too, e.g. a box into a circle -- and recompute mass/inertia), `:apply_force(v)`, `:apply_force_at_point(v, worldPoint)`, `:apply_impulse(v)`, `:apply_torque(t)`, `:wake()`, `:set_velocity(x,y)`, `:set_position(x,y)` |
| `World` (global `world`) | `.gravity`, `.enable_ccd` (settable -- see "Physics engine capabilities" above; off by default), `:create_circle(x,y,radius,BodyType)`, `:create_box(x,y,hw,hh,BodyType)`, `:create_polygon(x,y,sides,circumradius,BodyType)` (any regular N-gon -- triangle, pentagon, hexagon, ... in one function), `:find_body(name)` (first match only), `:count()` (total body count), `:remove_body(body)`, `:bodies()` (every body as a 1-indexed table, for `ipairs` -- use `.radius > 0` to tell a circle from a polygon, since `.radius` reads 0 for non-circle shapes), `:create_matter(x,y,radius,MatterKind)`, `:find_matter(name)`, `:matter_count()`, `:remove_matter(m)`, `:matter()` (every Matter object, 1-indexed, for `ipairs`) — plus tracking helpers for when names aren't unique or you just want a headcount: `:count_by_name(name)`/`:count_by_type(BodyType)`/`:count_by_matter_kind(MatterKind)` (all return an int) and `:find_bodies_by_name(name)`/`:find_bodies_by_type(BodyType)`/`:find_bodies_by_matter_kind(MatterKind)` (all return a 1-indexed table of every match, same convention as `:bodies()`), plus the Matter-object equivalents `:count_matter_by_kind(MatterKind)`/`:find_matter_by_kind(MatterKind)`. Queries: `:raycast_closest(x1,y1,x2,y2,[mask])` (nil, or a table with `body`/`matter`/`x`/`y`/`nx`/`ny`/`fraction`), `:raycast_all(x1,y1,x2,y2,[mask])` (same table shape, 1-indexed, nearest-first), `:query_point(x,y,[mask])` (the topmost `Body` there, or nil). Joints: `:create_distance_joint(a,b,ax,ay,bx,by)`, `:create_revolute_joint(a,b,ax,ay)`, `:create_weld_joint(a,b,ax,ay)`, `:create_prismatic_joint(a,b,ax,ay,axisX,axisY)` (world-space anchors; each returns a joint handle, see below), and the matching `:remove_distance_joint(j)`/`:remove_revolute_joint(j)`/`:remove_weld_joint(j)`/`:remove_prismatic_joint(j)` |
| `BodyType` | `.Static`, `.Kinematic`, `.Dynamic` |
| `Matter` | a point-mass object, NOT a rigidbody -- see Features above (Matter section) for what's genuinely different about it. `.position`, `.velocity`, `.restitution`, `.friction`, `.linear_damping`, `.gravity_scale`, `.name`, `.matter_kind` (`MatterKind.Matter`/`MatterKind.OptiMatter`), `.collision_category`/`.collision_mask` (Box2D-style filtering, see "Physics engine capabilities" above), `.radius` (settable -- recomputes mass, like `.density` below), `.texture_path`, `.color_r`/`.color_g`/`.color_b` (0-255, settable individually), `.density` (settable, recomputes mass), `.mass` (read), `.is_awake` (read), `:set_color(r,g,b)` (all three channels at once), `:apply_force(v)`, `:apply_impulse(v)`, `:wake()`, `:set_velocity(x,y)`, `:set_position(x,y)` -- deliberately no `.rotation`/`.angular_velocity`/`.type`/`:apply_torque`, none of which mean anything for something that never rotates |
| `MatterKind` | `.Rigidbody`, `.Matter`, `.OptiMatter` -- see Features above (Matter section); `.Rigidbody` only makes sense on `Body` (a `Matter` object defaults to `.Matter` and has no reason to be `.Rigidbody`) |
| `DistanceJoint`/`RevoluteJoint`/`WeldJoint`/`PrismaticJoint` | rigid, bilateral constraints -- see "Physics engine capabilities" above for what each one constrains. All four: `.body_a`/`.body_b` (read). `DistanceJoint` additionally: `.length` (settable -- the rope/rod's rest length). Created via `world:create_*_joint(...)` (above), removed via `world:remove_*_joint(j)` -- also cleaned up automatically if either body is removed with `world:remove_body()`. |
| `SCRIPT_PATH` | the currently-running script's own path — pass it to `attach_script` to make a spawned body self-replicate with the same script |
| `attach_script(body, path)` | attaches/reloads a script on a body spawned from within another script |
| `create_spring(bodyA, bodyB, stiffness, damping)` | ties two *existing* bodies together with a spring (rest length = their current distance apart) -- editor-only, like `soft_body.*`/`ui.*` below (needs `EditorApp`'s bookkeeping to persist/simulate) |
| `soft_body.create_ring(x,y,radius,segments)`, `.create_jelly(x,y,radius,segments)`, `.create_cloth(x,y,cols,rows,spacing,pin_top)` | spawn a Ring/Jelly/Cloth from a script, same shapes the editor's Spawn tool places; each returns a 1-indexed table of every particle Body created (`particles[1]` is the hub for a jelly) -- editor-only |
| `print(...)` | routed to the editor's Console panel (or stdout when run headless) |
| `ui.button(label) -> bool` | true on the frame it's clicked; only meaningful inside `on_gui`, and only in the editor (the `ui` table is bound by the app, not the core scripting library -- see below) |
| `ui.text(str)`, `ui.same_line()`, `ui.separator()` | layout/display helpers, same semantics as the identically-named ImGui calls |
| `ui.checkbox(label, value) -> bool`, `ui.slider_float(label, value, min, max) -> float` | return the (possibly-changed) value; store it back into a script-local variable yourself, same pattern as any immediate-mode UI |
| `ui.image(path, size)` | draws `path` (loaded/cached via the same `Renderer` texture cache the viewport uses) as a `size`x`size` square; a missing/unloadable path draws `[missing image]` instead of nothing |
| `ui.image_button(path, id, size) -> bool` | an image-only button (`id` is the ImGui id, independent of any visible label -- pass e.g. `body.name`); true on the frame it's clicked, same as `ui.button`; falls back to a plain `ui.button(id)` if `path` fails to load |
| `ui.is_item_hovered() -> bool`, `ui.is_item_clicked() -> bool`, `ui.is_item_active() -> bool` | generic interaction detection on whatever `ui.*` widget was drawn last -- e.g. detecting a click on `ui.text(...)`, which isn't a button and so has no return value of its own |

There's deliberately no `ui.input_text`/`ui.panel` (or similar) here: Input
Field and Panel (the two newest UI Element kinds) only exist as
`EditorApp::drawNativeUiElement()`'s native C++ rendering, not through the
`ui` table at all -- see "Create" in Features above.

**Tuning (C++ side, not exposed to Lua):** `World` has public fields for
`allowSleeping`, `linearSleepThreshold`, `angularSleepThreshold`,
`timeToSleep`, `velocityIterations`, `continuousDisplacementFraction`, and
`maxSubsteps` if you're embedding the engine and need to tune sleeping or
substepping behavior per-scene, plus `optiMatterLinearSleepThreshold`/
`optiMatterAngularSleepThreshold`/`optiMatterTimeToSleep`/
`optiMatterMaxLinearCorrection` for the shared `MatterKind::OptiMatter`
dial and `matterLinearSleepThreshold`/`matterAngularSleepThreshold`/
`matterTimeToSleep`/`matterMaxLinearCorrection`/
`matterContinuousDisplacementFraction` for the shared `MatterKind::Matter`
dial (pulls the opposite direction — stricter, not looser) — both used by
either `Matter` or any `Body` with `matterKind` set accordingly (see
Features above); the angular-threshold fields only ever matter for a
`Body`, since `Matter` has no rotation to check. None of these World-wide
fields are exposed to Lua individually, but the practical way most scenes
want to tune this — per-body, not scene-wide — is already scripted: set
`body.matter_kind = MatterKind.OptiMatter` (or `.Matter`) on whichever
bodies need it.

`World::onBodyRemoved` (C++ side) fires just before a body is erased, so a
host can detach any script attachment for that body's id — the editor wires
this up once so `world:remove_body(...)` from Lua cleans up correctly, and
you'll want the same hook if you embed the engine standalone.

**Reentrancy:** `ScriptEngine::update()` *and* `updateGui()` are both safe
for scripts that spawn new bodies and attach scripts to them (self-
replication) or remove bodies (death) from inside `on_update`/`on_gui` —
exactly what `bacteria.lua` and `spawn_cube_button.lua` do. If you extend
`ScriptEngine`, never replace it wholesale with `scriptEngine =
ScriptEngine()`; use `scriptEngine.reset()` instead, which tears down script
attachments before the Lua state they reference is destroyed (the wrong
order is a use-after-free that only shows up once scripts are doing enough
work to trigger it).

**`on_gui`/`ui.*`/`soft_body.*`/`create_spring` are editor-only:**
`ScriptEngine::updateGui()` (which calls `on_gui`) lives in the core,
dependency-free scripting library and is tested headlessly, but `ui`,
`soft_body`, and `create_spring` are all registered by
`EditorApp::bindUiApi()` instead of `ScriptEngine::bindWorld()` — `ui` is a
thin wrapper directly issuing ImGui calls, and `soft_body`/`create_spring`
need `EditorApp`'s own bookkeeping (`softBodies_`/`springJoints_`) to
actually simulate and persist through Play/Stop/Reset, none of which a
plain `World` has access to. All three only exist when running inside the
editor, not if you embed `p2d_scripting` standalone without also binding
your own equivalents.

This repo's own `scripts/` folder is reference material/test fixtures only
(headless tests like `bacteria_test.cpp` run against it directly) — it is
NOT copied into new projects; copy a file into a project's FileSystem panel
yourself if you want to build on one. The one script every NEW project gets
automatically, `spawn_balls.lua`, is written fresh into that project's own
`scripts/` folder by `resetScene()` (see "Getting started" above) — it
doesn't live in this repo's `scripts/` folder at all.

Example scripts in this repo's `scripts/`:
- `bacteria.lua` — grows a population fast by budding off children (which
  reproduce themselves too) on a random timer, capped at a max population,
  with cells dying off after a lifespan
- `spawn_cube_button.lua` — a "Spawn Cube" button, demonstrating `on_gui`
- `spawn_1000_button.lua` — a "Spawn 1000" button dropping a 40x25 grid of
  circles; each click spawns a bit higher than the last (script-local
  upvalues track the current height across calls), capped at a maximum so
  it doesn't climb forever
- `spawn_by_matter_kind.lua` — a slider (0-1000) plus three buttons that
  spawn that many circles as Rigidbody/Matter/OptiMatter respectively (see
  "`Body::matterKind`" above); marked as a UI Element (slider kind) so
  dragging it onto any body auto-configures that body's Inspector/Hierarchy
  entries, not just its `on_gui`
- `bouncy_ball.lua` — sets high restitution on start
- `oscillating_platform.lua` — drives a Kinematic body back and forth
- `spawner.lua` — periodically spawns new circles via the `world` API

## Project layout

```
engine/      p2d_engine    -- Vec2, Shape, Body, World, Collision (no dependencies)
scripting/   p2d_scripting -- sol2 bindings, ScriptEngine (depends on p2d_engine)
app/         p2d_editor    -- SFML + ImGui visual editor (depends on both)
  src/ProjectPaths.{hpp,cpp}         -- project listing/scaffolding, no SFML/ImGui dependency
  src/ProjectManagerScreen.{hpp,cpp} -- the Godot-like project browser screen
  src/ProjectBundle.{hpp,cpp}        -- single-file project save/open (Save Project As.../Open Project File...)
  src/ScenePersistence.{hpp,cpp}     -- scene save/load (JSON); no SFML/ImGui/ScriptEngine dependency
  src/Theme.{hpp,cpp}                -- the editor's custom ImGui color/style palette
cmake/patch_texteditor.cmake -- source patch applied to the fetched code-editor dependency
tests/                     -- headless tests for engine + scripting + project scaffolding
scripts/                   -- example .lua scripts (reference material/test fixtures, not copied anywhere)
```

`~/Physics2DProjects/<project name>/scripts/` (created when a project is
made) is where each project's own scripts actually live and where that
project's FileSystem panel points -- starting with just `spawn_balls.lua`
(written there directly by `resetScene()`, see "Getting started" above),
not a copy of this repo's own `scripts/` folder. `~/Physics2DProjects/
<project name>/scene.json`, alongside it, is the autosaved scene (see
"Project persistence" above) -- delete it (and `scripts/spawn_balls.lua`,
if you want that regenerated too) if you want a project to go back to the
starter scene next time you open it.

### Sharing a project as a single file

The Project Manager's **Save Project As...** bundles a whole project --
`scene.json` plus every file under `scripts/`, recursively -- into one
self-contained `.p2dproj` JSON file (`app/src/ProjectBundle.{hpp,cpp}`):
`.lua` files are embedded as plain JSON strings (readable, diffable, since
JSON already escapes newlines/quotes safely); anything else (images) is
base64-encoded. **Open Project File...** reverses this, unpacking a bundle
into a brand-new project directory under `~/Physics2DProjects/<name>/` --
it refuses to overwrite an existing project (checked before writing
anything). Both are typed-path popups rather than a real file-picker
dialog, for the same reason "Upload Texture" above is: there's no native OS
file dialog library linked into this project. Verified with a throwaway
round-trip test (save a project with a `.lua` file and a binary PNG,
load it back into a new directory, and confirm the script content, the
PNG's raw bytes, and the scene JSON all match byte-for-byte) before
wiring it into the UI.

## Known limitations

- Polygon shapes are assumed to be defined with their centroid at the local
  origin (true for `MakeBox`); off-center convex polygons will have
  incorrect rotational dynamics.
- Rigid joints (`DistanceJoint`/`RevoluteJoint`/`WeldJoint`/`PrismaticJoint`,
  see "Physics engine capabilities" above) have no motors or limits (angle
  ranges, distance ranges) yet — each is exactly the bare constraint its
  name implies, nothing more. `WeldJoint`/`PrismaticJoint` also solve their
  two constraints as independent sequential passes rather than one coupled
  system (a deliberate simplification, see that section). No editor
  support yet either (Inspector/Spawn-tool/persistence) — script-only, like
  `p2d::Matter` below.
- Continuous collision detection (`World::enableCcd`, see "Physics engine
  capabilities" above) is real (a genuine swept time-of-impact check, not
  just finer substepping) but deliberately scoped: circle movers only (no
  swept polygon-vs-polygon), off by default and OptiMatter-exempt (no
  broadphase acceleration of its own, so it's a real per-mover cost, not a
  free upgrade), and treats the swept-against target as momentarily
  stationary. With it off (or for a polygon mover, or an OptiMatter one),
  adaptive substepping alone still applies, same as before — an extreme
  enough speed vs. a thin enough wall can still tunnel (substeps are capped
  at `World::maxSubsteps`, 8 by default, for a worst-case performance
  bound).
- No multithreading -- deliberately not attempted alongside the above (see
  "Physics engine capabilities" above for why); everything in this engine
  runs single-threaded.
- Warm-starting matches contacts between frames by nearest point position
  (not full feature IDs like Box2D) — effective in practice, but a contact
  configuration that changes drastically in a single step could match the
  wrong prior point. `Matter`'s contacts skip this matching entirely instead
  of needing it: a circle-circle (or circle-vs-polygon) contact only ever
  produces one point, so there's nothing ambiguous to match against.
- `p2d::Matter` (see Features above) is script-only for now: no Inspector
  editing, no Spawn tool placement, no Hierarchy listing, no selection in
  the viewport, and no scene persistence (a Matter object created by a
  script vanishes on reload, unlike a Body). It does render, and fully
  participates in physics (collides with both Matter and Body, sleeps,
  respects its own MatterKind dial) — the gap is purely in editor tooling
  around it, not in the simulation itself.
- The FileSystem panel has no rename or move-by-drag yet, only create/delete;
  only `.lua` and image files are drag *sources* out of the panel (not
  folders or other file types) — dropping files *in* from the OS file
  manager is separate and only recognizes image extensions (see "A note on
  OS drag-and-drop" below).
- The dark theme's exact colors were written to approximate a dark, rounded,
  blue-violet-accented look but were never seen rendered in this sandbox
  (no display) — tweak the constants at the top of `app/src/Theme.cpp` if
  it's not quite to taste.
- The code editor has no "unsaved changes?" confirmation: opening a
  different script while the current one is dirty (shown with a `*` in its
  tab title) silently discards the in-memory edit. Save before switching
  files if it matters.
- The project manager has no rename, and "Delete" removes a project's
  folder from disk immediately after the confirmation popup (no undo/trash).
- Within a single script's `on_gui()`, `ui.*` widgets are placed in call
  order with no layout control beyond `same_line()`/`separator()` -- fine
  for a handful of controls, not a general-purpose UI layout system. (Each
  *script's* whole block can be dragged as a unit in the Viewport overlay
  -- see Features above -- but what's laid out within one script's own
  on_gui call is still just top-to-bottom/same-line.)
- `scene.json`'s format has no version field or migration path -- a scene
  saved by a future version of this app with new/renamed fields would just
  have those fields ignored/defaulted on load by an older version (and vice
  versa), rather than erroring or auto-upgrading.
- **Panel** (see Create/Features above) is a plain sized rectangle, not a
  real container -- other UI Elements dragged over it visually layer on
  top (each still has its own separate floating window), but there's no
  actual parent-child relationship: moving a Panel doesn't move whatever's
  sitting on it, and there's no "add to this Panel" operation.
- Project bundles (`.p2dproj`, see "Sharing a project as a single file"
  above) are plain, uncompressed JSON with base64-inflated binary content
  (image files end up roughly 4/3 their original size) -- fine for typical
  script-heavy projects, less efficient for one with many/large images.
  There's also no version field, same caveat as `scene.json` above.
