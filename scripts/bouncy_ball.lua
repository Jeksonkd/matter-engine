-- Attach to a circle body: makes it bounce energetically.
function on_start(body)
    body.restitution = 0.9
    body.friction = 0.2
end
