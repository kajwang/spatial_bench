#!/bin/bash
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6

# Ring capture script - runs capture_ring_v4.py with sensible defaults

### Usage: ./run_capture.sh [cfg_file] [setting_id] [target_object] [output_dir] [radius] [views] [rings] [view_center_z_offset] [overview_height_scale] [overview_radius_scale] [overview_min_height] [overview_side_scale]

### Examples:
#   ./run_capture.sh                                      # Use all defaults
#   ./run_capture.sh configs/tasks/object_movement/arnold_room_0.yaml 1 apple
#   ./run_capture.sh configs/tasks/object_movement/arnold_room_0.yaml 0 apple_static outputs/apple_static 0.5 24 3 0.05 1.2 0.6 1.0 1.8

# Default argument values
CFG_FILE="${1:-configs/tasks/object_movement/arnold_room_0.yaml}"
SETTING_ID="${2:-1}"
TARGET="${3:-apple}"
OUT_DIR="${4:-outputs/captures}"
RADIUS="${5:-0.5}"
VIEWS="${6:-24}"
RINGS="${7:-3}"
VIEW_CENTER_Z_OFFSET="${8:-0.05}"
OVERVIEW_HEIGHT_SCALE="${9:-1.2}"
OVERVIEW_RADIUS_SCALE="${10:-0.6}"
OVERVIEW_MIN_HEIGHT="${11:-1.0}"
OVERVIEW_SIDE_SCALE="${12:-1.8}"

echo "Ring Capture Configuration:"
echo "  Config file: $CFG_FILE"
echo "  Setting ID: $SETTING_ID"
echo "  Target object: $TARGET"
echo "  Output directory: $OUT_DIR"
echo "  Radius: ${RADIUS}m, Views: $VIEWS/ring, Rings: $RINGS"
echo "  View center Z offset: $VIEW_CENTER_Z_OFFSET"
echo "  Overview scales: height=$OVERVIEW_HEIGHT_SCALE, radius=$OVERVIEW_RADIUS_SCALE, min_height=$OVERVIEW_MIN_HEIGHT, side=$OVERVIEW_SIDE_SCALE"
echo ""

python3 capture_ring.py \
  --cfg "$CFG_FILE" \
  --setting-id "$SETTING_ID" \
  --target "$TARGET" \
  --out "$OUT_DIR" \
  --radius "$RADIUS" \
  --views "$VIEWS" \
  --rings "$RINGS" \
  --view-center-z-offset "$VIEW_CENTER_Z_OFFSET" \
  --overview-height-scale "$OVERVIEW_HEIGHT_SCALE" \
  --overview-radius-scale "$OVERVIEW_RADIUS_SCALE" \
  --overview-min-height "$OVERVIEW_MIN_HEIGHT" \
  --overview-side-scale "$OVERVIEW_SIDE_SCALE"
