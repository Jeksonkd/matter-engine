-- Attach to any body: periodically spawns new circles from the World API.
timer = 0.0
interval = 1.5

function on_update(body, dt)
    timer = timer + dt
    if timer >= interval then
        timer = 0.0
        local x = body.position.x + (math.random() - 0.5) * 2.0
        local radius = 0.25 + math.random() * 0.15
        local c = world:create_circle(x, body.position.y, radius, BodyType.Dynamic)
        c.restitution = 0.5
        c.friction = 0.3
        c:set_velocity((math.random() - 0.5) * 2.0, 0.0)
    end
end
