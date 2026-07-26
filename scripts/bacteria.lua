-- Fast-reproducing bacteria: each instance waits a random interval, then
-- buds off a nearby child running this same script (binary-fission style),
-- and dies off after a lifespan. World population is capped so physics
-- stays smooth even with exponential-looking growth.
local MAX_POPULATION = 140
local MIN_REPRO_TIME = 1.2
local MAX_REPRO_TIME = 2.6
local MAX_AGE = 18.0

age = 0.0
repro_timer = MIN_REPRO_TIME + math.random() * (MAX_REPRO_TIME - MIN_REPRO_TIME)

function on_start(body)
    body.restitution = 0.2
    body.friction = 0.5
end

function on_update(body, dt)
    age = age + dt
    repro_timer = repro_timer - dt

    if age > MAX_AGE then
        world:remove_body(body)
        return
    end

    if repro_timer <= 0.0 and world:count() < MAX_POPULATION then
        repro_timer = MIN_REPRO_TIME + math.random() * (MAX_REPRO_TIME - MIN_REPRO_TIME)

        local ox = (math.random() - 0.5) * 0.6
        local oy = math.random() * 0.3
        local child = world:create_circle(body.position.x + ox, body.position.y + oy, body.radius, BodyType.Dynamic)
        child.restitution = 0.2
        child.friction = 0.5
        child:set_velocity((math.random() - 0.5) * 1.0, math.random() * 0.8)

        -- Re-attach this same script (by its own path) so the child can
        -- reproduce too -- SCRIPT_PATH is injected automatically on attach.
        attach_script(child, SCRIPT_PATH)
    end
end
