# Capture Script Guide

This document describes the capture workflow implemented by `capture_ring_v4.py` and the helper launcher `run_capture.sh`.

## What the capture script does

`capture_ring_v4.py` performs orbit capture around a target object in Isaac Lab / Isaac Sim.

Main features:

- Load a task config and spawn the scene/object setting.
- Find a target object by name (for example `apple`).
- Spawn and control an orbit camera around the target.
- Capture RGB + depth images for each view.
- Save per-view metadata (`metadata.json`) with:
	- camera pose,
	- object pose,
	- object world AABB bounding box.
- Set an oblique overview viewport focused on the target region.
- During capture, show the current orbit camera position as a live red marker in the viewport (if debug draw is available).
- Save a static setup visualization image (`capture_setup_visualization.png`) if `matplotlib` is available.

## Output layout

By default, outputs are written to `outputs/captures`.

For each ring/view, one folder is created:

- `r00_v000/`
- `r00_v001/`
- ...

Inside each view folder:

- `rgb.png`
- `depth.png`
- `metadata.json`

And one setup-level file is written to the output root:

- `capture_setup_visualization.png`

## Quick start

### Option A: run with helper script

```bash
./run_capture.sh
```

Example with explicit arguments:

```bash
./run_capture.sh \
	configs/tasks/object_movement/arnold_room_0.yaml \
	1 \
	apple \
	outputs/captures \
	0.6 \
	24 \
	3 \
	0.05 \
	1.2 \
	0.6 \
	1.0 \
	1.8
```

### Option B: run Python script directly

```bash
python3 capture_ring_v4.py \
	--cfg configs/tasks/object_movement/arnold_room_0.yaml \
	--setting-id 1 \
	--target apple \
	--out outputs/captures \
	--radius 0.6 \
	--views 24 \
	--rings 3 \
	--view-center-z-offset 0.05 \
	--overview-height-scale 1.2 \
	--overview-radius-scale 0.6 \
	--overview-min-height 1.0 \
	--overview-side-scale 1.8
```

## Parameters

### Core capture parameters

- `--cfg` (str)
	- Task YAML config path.
	- Default: `configs/tasks/object_movement/arnold_room_0.yaml`

- `--setting-id` (int)
	- Which task setting to use from the config.
	- Default: `1`

- `--target` (str)
	- Target object name in `task.scene_entities`.
	- Default: `apple`

- `--out` (str)
	- Output root directory.
	- Default: `outputs/captures`

- `--radius` (float)
	- Orbit radius in meters.
	- Default: `0.6`

- `--views` (int)
	- Number of camera views per ring.
	- Default: `24`

- `--rings` (int)
	- Number of elevation rings.
	- Default: `3`

### View-center / overview viewport parameters

- `--view-center-z-offset` (float)
	- Z offset added to target center for orbit look-at center.
	- Useful to look slightly above object root.
	- Default: `0.05`

- `--overview-height-scale` (float)
	- Scale factor for overview camera height based on orbit horizontal radius.
	- Higher value => more top-down.
	- Lower value => more side/oblique.
	- Default: `1.2`

- `--overview-radius-scale` (float)
	- Extra overview height term based on orbit radius.
	- Helps keep full orbit area visible for larger radii.
	- Default: `0.6`

- `--overview-min-height` (float)
	- Minimum overview camera height floor.
	- Default: `1.0`

- `--overview-side-scale` (float)
	- Horizontal side offset scale for oblique overview camera.
	- Higher value => stronger side view.
	- Default: `1.8`

### AppLauncher / Isaac arguments

`capture_ring_v4.py` also accepts Isaac AppLauncher arguments via `AppLauncher.add_app_launcher_args(parser)`.
Use them when needed (for example device/headless options supported by your Isaac Lab setup).

## Practical tuning tips

- If the overview is too top-down:
	- decrease `--overview-height-scale`,
	- increase `--overview-side-scale`.

- If the full orbit path is not visible:
	- increase `--overview-height-scale` or `--overview-min-height`.

- If the camera seems to look too low/high on the object:
	- adjust `--view-center-z-offset`.

## Notes

- If `matplotlib` is not installed, setup visualization PNG is skipped.
- If Isaac debug draw module is unavailable, live red marker is skipped.
- The script still captures RGB/depth/metadata normally in both cases.
