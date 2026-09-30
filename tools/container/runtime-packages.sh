#!/usr/bin/env bash
# Emit the exact installed runtime package versions. No installation is performed.
set -euo pipefail
version=${1:?OpenVINO version is required}
[[ "$version" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]] || exit 2
packages=(
  libstdc++6 libgcc-s1 libgomp1
  libopencv-core4.5d libopencv-calib3d4.5d libopencv-imgproc4.5d libopencv-imgcodecs4.5d
  libyaml-cpp0.7
  "libopenvino-$version"
  "libopenvino-intel-cpu-plugin-$version"
  "libopenvino-ir-frontend-$version"
)
for package in "${packages[@]}"; do
  [[ $(dpkg-query -W -f='${db:Status-Status}' "$package") == installed ]] || {
    printf 'Required runtime package is not installed: %s\n' "$package" >&2
    exit 1
  }
done
dpkg-query -W -f='${Package}=${Version}\n' "${packages[@]}" | LC_ALL=C sort
