# Foxglove visualization implementation plan

**Goal:** Add a separate ROS 2 visualization package that turns the existing saved RViz configuration into a Foxglove layout and connects the desktop client to the running Odin stack.

**Architecture:** A pure Python converter maps RViz displays to native Foxglove 3D and Image panels. A ROS node exports and refreshes the layout on saved configuration changes, and adapts Odometry to dedicated visualization-only PoseStamped topics. An independent launch starts that node and the installed official Foxglove Bridge. The bridge streams existing topics without adding any flight or perception behavior.

**Constraints:** Add files only. Do not edit existing tracking, prediction, planning, flight, driver, RViz, or launch files. Reuse installed Foxglove Desktop 2.59.0 and foxglove_bridge 3.4.2. Preserve display topic names and enabled states. Connect in ROS domain 66 by default when no domain is configured. Desktop layout import is an explicit Foxglove UI operation.

## Work

- [x] Test and implement conversion of nested display visibility, topic styles, camera state, image panels, duplicate topics, and unsupported-display reporting.
- [x] Add an exporter ROS node with atomic JSON writes, saved-file watching, and a reload service; add a separate bridge/desktop launch and a one-command runner.
- [x] Generate a ready-to-import layout from the current RViz file and document setup/use and conversion limits.
- [x] Build only the new package, run its tests, and verify the WebSocket advertises and streams real ROS messages without command-publishing capability.

## Verification

17 unit tests pass. Official SDK WebSocket handshake, channel advertising, original TF messages, exported layout CDR messages, and a synthetic isolated Odometry-to-PoseStamped round trip were verified against the installed bridge. Both visualization processes exit cleanly. The official Linux x64 Desktop 3.5.0 installer is cached under `.local/deploy/downloads/`. Existing tracked-file hashes remained unchanged throughout this task. Desktop layout import remains a manual UI operation; no desktop rendering validation is claimed.

## New files

- `ros2/flyco_foxglove/`: independent package, converter, exporter node, launch, generated layout, and tests.
- `tools/run_foxglove.sh`: launch wrapper using the existing ROS installation.
- `docs/foxglove.md`: local setup, launch, layout import, and troubleshooting.

## Review focus

Parent group visibility must constrain children. Duplicate topic entries must not turn an enabled display off. Unsupported RViz plugins must be reported. Failed reloads must preserve the last valid layout. Custom `quadrotor_msgs/Mesh` is not a native 3D schema: the existing `/prediction/mesh_vis` Marker is the visual representation. Importing a layout does not generate missing prediction or trajectory data.
