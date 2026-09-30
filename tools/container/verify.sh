#!/usr/bin/env bash
# Run explicitly inside dev, with read-only source/models and writable build/output mounts.
set -euo pipefail
source_dir=/workspace/src
build_root=/workspace/build
output=${1:?Usage: bash tools/container/verify.sh /output/NEW_DIRECTORY}
output=$(realpath -m -- "$output")
case "$output" in /output/*) ;; *) echo 'Output must be below /output' >&2; exit 2 ;; esac
[[ ${KEEP_BUILD_ARTIFACTS:-1} == 0 || ${KEEP_BUILD_ARTIFACTS:-1} == 1 ]] || exit 2
[[ ! -e "$output" ]] || { echo 'Output already exists' >&2; exit 2; }
run_name=$(basename -- "$output")
build_root="$build_root/$run_name"
[[ ! -e "$build_root" ]] || { echo 'Build run already exists; choose a new name' >&2; exit 2; }
mkdir -p -- "$output" "$build_root"
exec > >(tee "$output/commands.log") 2>&1
set -x
models=()
for specification in 'YOLOV5:yolov5.xml' 'YOLO11:yolo11.xml'; do
  key=${specification%%:*}
  model="/models/${specification#*:}"
  if [[ -f "$model" ]]; then
    [[ -f "${model%.xml}.bin" ]] || { echo "Missing BIN for $model" >&2; exit 1; }
    models+=("-DAUTOAIM_TEST_${key}_MODEL=$model")
    sha256sum -- "$model" "${model%.xml}.bin" >> "$output/model-sha256.txt"
  else
    printf '%s: external model tests NOT VERIFIED (no %s)\n' "$key" "$model"
  fi
done
for profile in debug release cxx20 sanitize; do
  type=Debug
  standard=17
  openvino=ON
  sanitizers=OFF
  model_options=("${models[@]}")
  case "$profile" in
    release) type=Release ;;
    cxx20) standard=20 ;;
    sanitize) openvino=OFF; sanitizers=ON; model_options=() ;;
  esac
  build="$build_root/$profile"
  cmake -S "$source_dir" -B "$build" -DCMAKE_BUILD_TYPE="$type" \
    -DCMAKE_CXX_STANDARD="$standard" -DBUILD_TESTING=ON \
    -DAUTOAIM_OPENVINO="$openvino" -DAUTOAIM_HIKROBOT=OFF \
    -DAUTOAIM_SANITIZERS="$sanitizers" "${model_options[@]}"
  cmake --build "$build" --parallel "${BUILD_JOBS:-2}"
  ctest --test-dir "$build" --show-only=json-v1 > "$output/$profile-tests.json"
  ASAN_OPTIONS=detect_leaks=1 UBSAN_OPTIONS=halt_on_error=1 \
    ctest --test-dir "$build" --output-on-failure | tee "$output/$profile-ctest.log"
  cmake --build "$build" --target verification_manifest
  cp -- "$build/verification_manifest.json" "$output/$profile-manifest.json"
  if [[ ${KEEP_BUILD_ARTIFACTS:-1} == 0 ]]; then
    # Only this new run's completed profile: keep CMakeCache and verification records.
    cmake --build "$build" --target clean
  fi
done
printf '%s\n' 'All registered container tests passed; missing external assets remain unverified.'
