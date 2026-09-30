#!/usr/bin/env bash
# Mount this check read-only; it is deliberately not installed into runtime.
set -euo pipefail
output=${1:?Usage: bash runtime-smoke.sh /output/NEW_DIRECTORY}
output=$(realpath -m -- "$output")
case "$output" in /output/*) ;; *) echo 'Output must be below /output' >&2; exit 2 ;; esac
[[ ! -e "$output" ]] || { echo 'Output already exists' >&2; exit 2; }
mkdir -p -- "$output"
exec > >(tee "$output/smoke.log") 2>&1
set -x
[[ $(id -u) -ne 0 ]]
! command -v g++
! command -v cmake
[[ ! -e /workspace/src/CMakeLists.txt ]]
config=/opt/autoaim/share/autoaim/config/offline/armor.yaml
synthetic_sim --config "$config" --output "$output/dataset" --frames 6
sha256sum "$output/dataset/"* > "$output/source-before.sha256"
offline_replay --config "$config" --input "$output/dataset/events.yaml" \
  --output "$output/commands.tsv" --uart-output "$output/uart.hex"
replay_visualizer --config "$config" --input "$output/dataset/events.yaml" \
  --output "$output/visualized"
annotate_session --self-test
annotate_session --export "$output/annotation-draft" --session "$output/dataset"
bench_detector --dataset "$output/dataset/events.yaml" --config "$config" \
  --iou 0.5 --output "$output/report"
mv -- "$output/report" "$output/report-first"
bench_detector --dataset "$output/dataset/events.yaml" --config "$config" \
  --iou 0.5 --output "$output/report"
cmp -- "$output/report-first/report.yaml" "$output/report/report.yaml"
sha256sum -c "$output/source-before.sha256"
for detector in yolov5 yolo11; do
  if [[ ! -f "/models/$detector.xml" ]]; then
    printf '%s: runtime CPU inference NOT VERIFIED (model absent)\n' "$detector"
    continue
  fi
  [[ -f "/models/$detector.bin" ]] || { echo "Missing BIN for $detector" >&2; exit 1; }
  # The mounted configuration must explicitly resolve to the mounted model.
  # Do not edit user configuration or silently substitute another detector.
  sha256sum "/models/$detector.xml" "/models/$detector.bin" \
    >> "$output/model-sha256.txt"
  bench_detector --dataset "$output/dataset/events.yaml" \
    --config "/config/offline/$detector.yaml" --iou 0.5 --output "$output/$detector"
done
if autoaim_node --config /opt/autoaim/share/autoaim/config/hardware/disabled.yaml; then
  echo 'Hardware rejection was lost' >&2
  exit 1
fi
printf '%s\n' 'Runtime smoke passed; absent models and real hardware remain unverified.'
