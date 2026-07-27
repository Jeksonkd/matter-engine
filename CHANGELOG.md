# Changelog

Notable changes per release. Each GitHub Release's description is generated
from the matching section here (see `.github/workflows/release.yml`).

## v0.3.0

- **Rigid joints can have motors and limits now.** `DistanceJoint`/
  `RevoluteJoint`/`PrismaticJoint` each get one `enable_motor_limit` switch
  (off by default, same cost as before when off): on, the joint's one free
  DOF (length, relative angle, or slide translation) can be driven by a
  motor (target speed + a max torque/force) and/or bounded by a limit
  (lower/upper). A door that swings free but stops at 90°, a continuously
  driven wheel, a winch, a piston with a fixed stroke. `WeldJoint` doesn't
  get this -- it has no free DOF left to drive or bound. Fully scriptable
  (`enable_motor_limit`, `motor_speed`, `max_motor_torque`/`max_motor_force`,
  and the matching `lower_*`/`upper_*` or `min_length`/`max_length` fields).

## v0.2.0

- **OptiMatter is now the default Matter Kind.** New bodies spawned via the
  Spawn tool get OptiMatter (cheapest, highest-performance fidelity tier)
  unless changed in Settings, instead of the previous Rigidbody default.
- **New projects now start genuinely empty.** Only the one seeded sample
  project (shown the first time you launch the editor) has the starter
  scene (floor, walls, 5 balls, a Spawn Ball button). Every project you
  create yourself via "New Project" now opens with zero objects and zero
  scripts.
- **Upload Texture opens a native file picker.** Both the FileSystem panel
  toolbar button and its folder-context-menu equivalent now open your OS's
  real "Open File" dialog (via tinyfiledialogs) instead of a type-the-path
  popup.
- **Save Project As... / Open Project File... do too.** Saving a project
  bundle opens the native "Save As" dialog; opening one opens the native
  "Open File" dialog and names the new project after the bundle's own
  filename (no separate name-entry step).
- **Fixed the FileSystem panel's toolbar overflowing off-screen.** Its
  buttons (New Folder/New Script/Upload Texture/Refresh/Copy/Paste/Delete)
  now wrap onto additional rows based on the panel's actual width, and
  collapse back down as you widen it.

## v0.1.1

- First public release: Linux binary + Windows .exe, built and published
  automatically by GitHub Actions from a pushed `v*` tag.
