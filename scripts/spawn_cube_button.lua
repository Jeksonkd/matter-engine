-- Demonstrates the scriptable UI API: on_gui(body) runs once per rendered
-- frame (not per physics step) and can call ui.* functions, which map
-- directly onto the editor's ImGui panel for this frame. Attach this to
-- any body (it doesn't have to be visible or even Dynamic -- a tiny static
-- marker works fine) to add controls to the "Script UI" panel.
function on_gui(body)
    ui.text("Spawns a box above the scene each click.")

    if ui.button("Spawn Cube") then
        local x = (math.random() - 0.5) * 6.0
        local cube = world:create_box(x, 9.0, 0.4, 0.4, BodyType.Dynamic)
        cube.restitution = 0.3
        cube.friction = 0.5
        cube:set_velocity((math.random() - 0.5) * 1.0, 0.0)
    end
end
