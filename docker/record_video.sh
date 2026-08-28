#!/bin/bash
# Record RViz window to video using ffmpeg (X11 screen capture).
#
# Usage:
#   ./record_video.sh <bag_file> [output_video] [playback_rate]
#
# Example:
#   ./record_video.sh /home/kota/data_static/hard_static.../bags/trial_0.bag /home/kota/data_video/hard_forest.mp4
#   ./record_video.sh /home/kota/data_static/hard_static.../bags/trial_0.bag /home/kota/data_video/hard_forest.mp4 0.5
#
# Requirements: ffmpeg, xdotool (both installed in this script if missing)

set -euo pipefail

BAG_FILE="${1:?Usage: $0 <bag_file> [output_video] [playback_rate]}"
OUTPUT="${2:-/home/kota/data_video/output.mp4}"
RATE="${3:-1.0}"

# Ensure output directory exists
mkdir -p "$(dirname "$OUTPUT")"

# Source ROS
source /home/kota/ego_swarm_v2_ws/src/EGO-Planner-v2/swarm-playground/main_ws/devel/setup.bash

echo "=== EGO-Swarm v2 Video Recorder ==="
echo "  Bag:    $BAG_FILE"
echo "  Output: $OUTPUT"
echo "  Rate:   ${RATE}x"
echo ""

# Install ffmpeg and xdotool if not available
apt-get install -y -qq ffmpeg xdotool 2>/dev/null || true

# Start roscore if not running
if ! pgrep -x rosmaster > /dev/null; then
    roscore &
    sleep 3
fi

# Launch RViz with playback config
RVIZ_CONFIG=/home/kota/ego_swarm_v2_ws/src/EGO-Planner-v2/docker/playback_rviz.rviz
rviz -d "$RVIZ_CONFIG" &
RVIZ_PID=$!
sleep 5

# Get bag duration
DURATION=$(rosbag info --yaml "$BAG_FILE" | grep "duration:" | head -1 | awk '{print $2}')
# Add buffer for playback rate
RECORD_DURATION=$(python3 -c "print(int(float('$DURATION') / float('$RATE') + 5))")
echo "  Bag duration: ${DURATION}s, recording for: ${RECORD_DURATION}s"

# Start screen recording (full screen)
echo "  Starting screen recording..."
ffmpeg -y -video_size 1920x1080 -framerate 30 -f x11grab -i :0.0 \
    -t "$RECORD_DURATION" \
    -c:v libx264 -preset fast -crf 23 -pix_fmt yuv420p \
    "$OUTPUT" &
FFMPEG_PID=$!
sleep 2

# Play the bag
echo "  Playing bag..."
rosbag play "$BAG_FILE" --clock -r "$RATE"

# Wait a few seconds after bag finishes
sleep 3

# Stop recording
kill $FFMPEG_PID 2>/dev/null || true
wait $FFMPEG_PID 2>/dev/null || true

# Stop RViz
kill $RVIZ_PID 2>/dev/null || true

echo ""
echo "=== Video saved to: $OUTPUT ==="
echo "  File size: $(du -h "$OUTPUT" | cut -f1)"
