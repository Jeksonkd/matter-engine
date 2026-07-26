-- @p2d_ui_kind: slider
-- Attach this to ANY body (drag it onto one from the FileSystem panel, or
-- pick it from the Inspector's Script Library / Script path + Attach) --
-- the marker line above is the same mechanism the FileSystem's "New
-- Script > Slider" template uses, so EditorApp::applyUiKindMarkerOnAttach()
-- recognizes it and flips that body into a UI Element automatically (badged
-- in the FileSystem/Hierarchy, and its on_gui below is picked up by the
-- Viewport's floating overlay AND the Script UI panel). No separate setup
-- step needed on the body itself.
--
-- A slider (0-1000) picks how many objects the three buttons below spawn,
-- one as ordinary Rigidbody bodies, one as Matter, one as OptiMatter --
-- see the README's "Body::matterKind" section for what those mean. All
-- three spawn PLAIN circles through world:create_circle and just set the
-- resulting body's matter_kind -- Matter/OptiMatter are a solver fidelity
-- dial on an ordinary rigidbody now (rotation, any shape, scripting, UI
-- Elements all keep working), not a different kind of object, so nothing
-- else needs to differ between the three spawn functions below.
--
-- body.ui_value holds the slider's amount, body.ui_min/ui_max its range --
-- all three are also visible/editable from the Inspector ("UI Value"/"UI
-- Min / Max"), and saved with the scene. currentHeight climbs a bit after
-- every spawn (any of the three buttons), capped at maxHeight, so repeated
-- clicks stack visibly instead of piling on the exact same spot -- same
-- idea as scripts/spawn_1000_button.lua.

local heightStep = 6.0
local maxHeight = 200.0
local currentHeight = 25.0

-- Runs once, right when this script is attached -- gives the slider a
-- sensible 0-1000 range and a non-zero starting amount immediately,
-- instead of Body::uiMin/uiMax's generic 0-1 default (fine for a plain
-- 0-1 slider template, useless here). Left alone on every later attach/
-- reload of the SAME body, so manual Inspector edits afterward stick.
function on_start(body)
    body.ui_min = 0
    body.ui_max = 1000
    if body.ui_value <= 0 then body.ui_value = 100 end
end

local function spawnBatch(count, kind, label)
    if count <= 0 then return end

    -- A roughly square grid, capped at 40 columns (same cap
    -- spawn_1000_button.lua uses for its own 1000-circle grid) so a small
    -- amount doesn't spawn one absurdly long single-row line.
    local cols = math.min(40, math.max(1, math.ceil(math.sqrt(count))))
    local spacing = 0.3
    local startX = -(cols - 1) * spacing * 0.5
    local startY = currentHeight

    local spawned = 0
    local row = 0
    while spawned < count do
        for col = 0, cols - 1 do
            if spawned >= count then break end

            local x = startX + col * spacing + (math.random() - 0.5) * 0.05
            local y = startY + row * spacing + (math.random() - 0.5) * 0.05

            local c = world:create_circle(x, y, 0.15, BodyType.Dynamic)
            c.matter_kind = kind
            c.restitution = 0.2
            c.friction = 0.4
            c:set_velocity((math.random() - 0.5) * 0.5, 0.0)

            spawned = spawned + 1
        end
        row = row + 1
    end

    currentHeight = math.min(currentHeight + heightStep, maxHeight)
    print("Spawned " .. count .. " " .. label .. ".")
end

function on_gui(body)
    ui.text(string.format("Amount: %d", math.floor(body.ui_value + 0.5)))
    body.ui_value = ui.slider_float("Amount", body.ui_value, body.ui_min, body.ui_max)

    ui.separator()

    local amount = math.floor(body.ui_value + 0.5)
    if ui.button("Spawn Rigidbody") then
        spawnBatch(amount, MatterKind.Rigidbody, "Rigidbody(s)")
    end
    if ui.button("Spawn Matter") then
        spawnBatch(amount, MatterKind.Matter, "Matter body/bodies")
    end
    if ui.button("Spawn OptiMatter") then
        spawnBatch(amount, MatterKind.OptiMatter, "OptiMatter body/bodies")
    end

    if currentHeight >= maxHeight then
        ui.text("(at the height cap -- further clicks spawn here, not higher)")
    end
end
