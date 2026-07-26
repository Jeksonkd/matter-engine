-- Adds a "Spawn 1000" button (Script UI panel) that drops 1000 small
-- circles into the scene in a grid pattern, a bit higher each time the
-- button is clicked (so repeated drops stack visibly on top of the last
-- one) -- capped at maxHeight so it doesn't keep climbing forever.
-- currentHeight/heightStep/maxHeight are script-local (upvalues), so they
-- persist across clicks for as long as this script stays attached; they
-- reset back to startHeight if the script is detached/re-attached.
local startHeight = 25.0
local heightStep = 15.0 -- how much higher each click spawns, until the cap
local maxHeight = 200.0 -- the cap -- further clicks keep spawning at this height, not higher
local currentHeight = startHeight

function on_gui(body)
    ui.text(string.format("Spawns 1000 circles at height %.0f (cap %.0f).", currentHeight, maxHeight))

    if ui.button("Spawn 1000") then
        local cols = 40
        local rows = 25 -- cols * rows = 1000
        local spacing = 0.35
        local startX = -(cols - 1) * spacing * 0.5
        local startY = currentHeight

        for row = 0, rows - 1 do
            for col = 0, cols - 1 do
                local x = startX + col * spacing + (math.random() - 0.5) * 0.05
                local y = startY + row * spacing + (math.random() - 0.5) * 0.05

                local c = world:create_circle(x, y, 0.15, BodyType.Dynamic)
                c.restitution = 0.2
                c.friction = 0.4
                c:set_velocity((math.random() - 0.5) * 0.5, 0.0)
            end
        end

        currentHeight = math.min(currentHeight + heightStep, maxHeight)
    end

    if currentHeight >= maxHeight then
        ui.text("(at the height cap -- further clicks spawn here, not higher)")
    end
end
