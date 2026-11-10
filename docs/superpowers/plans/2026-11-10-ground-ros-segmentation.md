# Ground ROS image segmentation implementation plan

**Goal:** Validate text segmentation on ground by subscribing to Odin images directly, without HTTP or onboard inference requests.

**Architecture:** A ROS Python 3.10 node receives images and `/query_text`, keeps only the newest pending image, and exchanges PNG payloads with one persistent Python 3.12 model worker over local pipes. The worker reuses SAM_AD_Perception backend A. Results preserve the input header and image size and publish mask, overlay, and runtime status.

**Scope:** Ground only. Orin mask consumption and 3D tracking integration remain a later task. The user explicitly authorized this ground implementation after reviewing the inline architecture.

**Constraints:** Reuse the installed uv environments and model assets. Do not run manifest/hash verification, download weights, or repeat GPU/model validation. Preserve existing user edits and the existing HTTP launch entrypoint. Default image input is `/odin1/image` for this 2D test; allow `/flyco/camera/rgb` as an override.

**Workspace decision:** Implement in the user's active `dev-real` checkout because this task adapts its existing uncommitted launch/environment changes and installed uv environments. Do not switch branches, commit unrelated edits, or alter onboard files.

## Task 1: Local worker and bounded image handoff

- [x] Add tests for replacing pending frames, rejecting old-prompt results, pipe framing, worker errors and shutdown.
- [x] Observe the new tests fail before implementing the missing modules.
- [x] Implement `semantic_ipc.py` and `semantic_worker.py`; keep ROS and GPU imports in separate processes.
- [x] Verify the worker with a controlled model boundary; do not load real model weights.

## Task 2: ROS input and result publication

- [x] Test mask size, encoding, original timestamp/frame preservation, and empty-mask behavior.
- [x] Implement `semantic_image.py` and its ROS executable; subscribe with sensor QoS, use a single inference worker, and expose loading/waiting/inference/error status.
- [x] Reject results from an earlier prompt generation and close the subprocess on shutdown.

## Task 3: Launch, installation and instructions

- [x] Add `segmentation.launch.py`, a small RViz configuration and editable node defaults. Allow UI/RViz to be disabled.
- [x] Add a ground launch segmentation-only option that bypasses predictor/CUDA/mesh-checkpoint validation.
- [x] Register the executable and allow the existing text UI to use the selected image topic.
- [x] Run focused new/affected tests, install `flyco_real`, and verify launch argument discovery without starting models.
- [x] Document the launch command, text input, output topics and limits of the ground-only test.

## Review focus

Model logs must not corrupt the pipe protocol; request failures must remain visible; a text change must invalidate an in-flight result; pending images must not grow without bounds; raw and rectified images must retain their own header and dimensions. Tests exercise these behaviors without revalidating the neural models.

## Verification and decisions

- Focused verification: 15 tests passed in 2.93s with the existing uv predictor environment; pytest plugin auto-discovery was disabled because the system pytest and an unrelated anyio plugin are incompatible.
- A fresh read-only reviewer identified inherited launch configuration and a UI text-topic mismatch. Both were reproduced, fixed, and covered by tests evaluating the child configuration and actual UI remapping.
- Native stdout diagnostics were reproduced as a pipe-protocol failure and fixed by duplicating the protocol descriptor before redirecting stdout.
- The colcon frontend stalled after configuring the existing workspace. Its configured CMake build and install steps both completed successfully when invoked directly. Installed launch discovery and the `semantic_image.py` executable were verified.
- Existing user deletions were retained, including the obsolete zero-camera wrapper and old tests. Its stale CMake program-install reference was removed.
- No manifest verification, model weight loading, real GPU inference, live Odin test, GUI launch, or onboard change was performed. Real image segmentation is the user's next validation step.
