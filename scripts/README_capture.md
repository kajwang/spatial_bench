# Ring Capture Utility

Environmental 3D scanning tool that captures RGBD images by placing a camera on circular
trajectories around a target object over multiple elevation rings. 

## Key Features

- **Follows eval_tasks.py pattern**: Uses `AppLauncher` + `SimulationContext` for proper initialization
- **Multi-ring capture**: Multiple elevation levels (configurable number of rings)
- **High-resolution scans**: Configurable views per ring
- **Task integration**: Reuses existing task configs and scene loading from `tasks/`

## Usage

### Basic Usage (from repo root)

```bash
conda run -n env_isaaclab-5.0 ./run_capture.sh
```

This captures the default object (apple) using setting 1 with default parameters:
- Config: `configs/tasks/object_movement/arnold_room_0.yaml`
- Setting: 1 (loads 'apple' object; use 0 for 'apple_static')
- Target: `apple`
- Output: `outputs/captures/`
- Radius: 0.5m, Views: 24/ring, Rings: 3

### Object Settings Reference

For `configs/tasks/object_movement/arnold_room_0.yaml`:

```
Setting 0: apple_static (static object, cannot be manipulated)
Setting 1: apple (dynamic, preferred for capturing)
Setting 2: avocado
Setting 3: coke
Setting 4: lime
Setting 5: sponge
Setting 6: potato
... and more
```

### Advanced Usage with Different Object

Capture 'apple_static' from setting 0:

```bash
conda run -n env_isaaclab-5.0 ./run_capture.sh \
	configs/tasks/object_movement/arnold_room_0.yaml \
	0 \
	apple_static \
	outputs/apple_static_scans \
	0.5 \
	24 \
	3
```

Capture 'coke' from setting 3 with custom parameters:

```bash
conda run -n env_isaaclab-5.0 ./run_capture.sh \
	configs/tasks/object_movement/arnold_room_0.yaml \
	3 \
	coke \
	outputs/coke_scans \
	0.8 \
	36 \
	4
```

Parameters (in order):
1. `cfg_file`: Task YAML config file (default: `configs/tasks/object_movement/arnold_room_0.yaml`)
2. `setting_id`: Task setting index (default: 1); selects which object group to load
   - Each setting corresponds to a different rigid object configuration in the YAML
   - Run script once to see which setting contains your target object
3. `target`: Object name in the selected setting (e.g., `apple`, `coke`, `lime`, `sponge`)
   - Must match an actual object loaded by the selected setting
4. `output_dir`: Output directory (default: `outputs/captures`)
5. `radius`: Distance from object center in meters (default: 0.5)
6. `views`: Views per ring (default: 24)
7. `rings`: Number of elevation rings (default: 3)

### Direct Python Execution

```bash
conda run -n env_isaaclab-5.0 python3 capture_ring_v2.py \
	--cfg configs/tasks/object_movement/arnold_room_0.yaml \
	--setting-id 1 \
	--target apple \
	--out outputs/captures \
	--radius 0.5 \
	--views 24 \
	--rings 3 \
	--elev_start 10 \
	--elev_delta 20
```

### Available Objects (arnold_room_0.yaml)

First, check which objects are available in each setting by examining the YAML:

```yaml
tasks:
  settings:
    - rigid: ["apple_static"]    # Setting 0
    - rigid: ["apple"]            # Setting 1 (default)
    - rigid: ["avocado"]
    - rigid: ["coke"]
    - rigid: ["lime"]
    - rigid: ["sponge"]
    - rigid: ["potato"]
    # ... and more
```

Run the capture script with the correct `--setting-id` for your target object.
If you get "Target not found" error, check which setting ID contains your object in the YAML.

## Output Structure


```
outputs/captures/
├── ring_00/          (lowest elevation ring)
│   ├── view_000_rgb.png
│   ├── view_001_rgb.png
│   └── ...
├── ring_01/          (middle elevation ring)
│   └── ...
└── ring_02/          (highest elevation ring)
		└── ...
```

Each ring captures the object from multiple azimuth angles (determined by `--views`).

## Implementation Notes

- **AppLauncher pattern**: Follows the same initialization as `eval_tasks.py`:
	1. Parse arguments and create `AppLauncher` BEFORE omni.isaac imports
	2. Launch simulation app
	3. Import omni.isaac and related modules
	4. Load task and initialize scene
  
- **Task integration**: Reuses `tasks/geometry/object_movement.py` to load scene, physics, and objects

- **Camera control**: Computes look-at matrix to orient camera toward target center

## Troubleshooting

- **"Target not found"**: Check object name in the config file
- **No viewport**: Ensure headless mode is disabled (camera capture requires rendering)
- **Missing dependencies**: Run with correct conda environment: `conda run -n env_isaaclab-5.0 ...`

