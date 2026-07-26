-- Attach to a Kinematic box: sweeps it back and forth sinusoidally.
-- Demonstrates driving physics motion from script rather than from forces.
t = 0.0
start_x = nil
amplitude = 3.0
speed = 1.5

function on_start(body)
    start_x = body.position.x
end

function on_update(body, dt)
    t = t + dt
    local target_x = start_x + math.sin(t * speed) * amplitude
    -- Set the velocity needed to reach target_x this step; the engine
    -- integrates kinematic bodies the same way as dynamic ones.
    body:set_velocity((target_x - body.position.x) / dt, 0.0)
end
