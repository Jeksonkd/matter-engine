-- Used by script_test.cpp to verify the C++/Lua binding surface.
call_log = {}

function on_start(body)
    body.restitution = 0.42
    body.linear_damping = 1.5
    body.angular_damping = 0.5
    body.gravity_scale = 0.0
    body.is_sensor = true
    body.fixed_rotation = true
    table.insert(call_log, "start:" .. body.name)
end

function on_update(body, dt)
    body:apply_force(Vec2.new(10.0, 0.0))
    table.insert(call_log, "update:" .. tostring(dt))
end

-- Distinguishable from on_update's effect (which pushes along X) so a C++
-- test can observe that on_gui actually ran via updateGui().
function on_gui(body)
    body:apply_impulse(Vec2.new(0.0, 0.01))
end
