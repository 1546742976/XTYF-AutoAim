#!/usr/bin/env bash
# XTYF-AutoAim 依赖安装；按用户指定保留 indenpence 文件名。
# 仅人工运行本脚本才会安装系统包；不启动相机、CAN、串口或自瞄程序。
# 来源：
# https://apt.kitware.com/
# https://docs.openvino.ai/2026/get-started/install-openvino/install-openvino-apt.html
# https://docs.openvino.ai/2026/get-started/install-openvino/configurations/configurations-intel-gpu.html
# https://www.hikrobotics.com/en/machinevision/service/download?module=0
set -Eeuo pipefail
trap 'printf "安装失败（第 %s 行）；停止后续步骤，已经安装的包不会自动卸载。\n" "$LINENO" >&2' ERR

dry_run=false
assume_yes=false
with_openvino=true
with_camera=true
with_gpu=false
openvino_version=2026.3.1
mvs_deb=
mvs_root=/opt/MVS
mvs_library=
sudo_cmd=()
apt_yes=()

usage() {
  cat <<'HELP'
用法：bash install_indenpence.sh [选项]
平台：Ubuntu 22.04/24.04 amd64（原生 Linux 或 WSL）。

默认：安装 C++17/20 工具链、CMake、OpenCV/Eigen/yaml-cpp、Python3、
      OpenVINO 2026.3.1、USB/CAN 配套库，并要求 Hikrobot MVS SDK。
      MVS 已安装时复用，否则必须传入官方 Linux amd64 的本地 .deb 包。

  --dry-run                  只显示计划，不联网、不 sudo、不修改系统配置
  --yes                      不询问主脚本/APT 确认；不代替厂商许可协议
  --skip-camera              只装软件依赖，跳过 MVS（适合 WSL 离线开发）
  --mvs-deb FILE             安装从 Hikrobot 官网下载、解压得到的 .deb
  --mvs-root DIR             已安装 SDK 根目录，默认 /opt/MVS
  --skip-openvino            不安装 OpenVINO，构建须 AUTOAIM_OPENVINO=OFF
  --openvino-version X.Y.Z   选择显式版本；默认项目已验证的 2026.3.1
  --intel-gpu                附加 Ubuntu 的 Intel OpenCL 运行库及 clinfo
  -h, --help                 显示帮助

例：
  bash install_indenpence.sh --dry-run --skip-camera
  bash install_indenpence.sh --yes --skip-camera
  bash install_indenpence.sh --mvs-deb /absolute/path/MvCamCtrlSDK_Runtime.deb

不会执行 apt upgrade/remove、禁用服务、修改 .bashrc、自动重启、
调整 USB/CAN 参数或自动给用户增权。安装 MVS 会运行厂商包的维护脚本；
请先检查包来源及许可，安装后的设备权限/驱动加载/硬件效果由人验收。
预览读取本地 .deb 元数据时，dpkg-deb 可能使用并清理自己的临时文件。
HELP
}

die() { printf '错误：%s\n' "$*" >&2; exit 1; }
log() { printf '\n[依赖安装] %s\n' "$*"; }
run() {
  printf '  +'; printf ' %q' "$@"; printf '\n'
  if ! "$dry_run"; then "$@"; fi
}
need_value() {
  [[ $# -ge 2 && -n "$2" && "$2" != --* ]] || die "$1 缺少参数值"
}

while (($#)); do
  case "$1" in
    --dry-run) dry_run=true; shift ;;
    --yes) assume_yes=true; shift ;;
    --skip-camera) with_camera=false; shift ;;
    --skip-openvino) with_openvino=false; shift ;;
    --intel-gpu) with_gpu=true; shift ;;
    --mvs-deb) need_value "$@"; mvs_deb=$2; shift 2 ;;
    --mvs-root) need_value "$@"; mvs_root=$2; shift 2 ;;
    --openvino-version) need_value "$@"; openvino_version=$2; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) die "未知参数：$1（使用 --help 查看用法）" ;;
  esac
done

[[ "$openvino_version" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]] || die 'OpenVINO 版本必须形如 2026.3.1'
if ! "$with_camera" && [[ -n "$mvs_deb" ]]; then die '--skip-camera 与 --mvs-deb 不能同时使用'; fi
if "$with_gpu" && ! "$with_openvino"; then die '--intel-gpu 与 --skip-openvino 不能同时使用'; fi
[[ $(uname -s) == Linux && -r /etc/os-release ]] || die '请在 Ubuntu Linux/WSL 中运行'
# shellcheck source=/dev/null
source /etc/os-release
[[ "$ID" == ubuntu ]] || die '本脚本只支持 Ubuntu，不修改其它发行版的软件源'
case "$VERSION_ID" in
  22.04) suite=jammy; intel_suite=ubuntu22 ;;
  24.04) suite=noble; intel_suite=ubuntu24 ;;
  *) die "未支持的 Ubuntu 版本：$VERSION_ID（需要 22.04 或 24.04）" ;;
esac
[[ $(dpkg --print-architecture) == amd64 ]] || die '当前脚本面向 NUC/WSL amd64，不安装其它架构的驱动'

mvs_root=$(realpath -m -- "$mvs_root")
find_mvs_library() {
  local directory
  for directory in "$mvs_root/lib/64" "$mvs_root/lib/amd64" "$mvs_root/lib64" "$mvs_root/lib"; do
    if [[ -f "$directory/libMvCameraControl.so" ]]; then
      mvs_library=$directory/libMvCameraControl.so
      return 0
    fi
  done
  return 1
}
if "$with_camera"; then
  if [[ -n "$mvs_deb" ]]; then
    [[ -f "$mvs_deb" && "$mvs_deb" == *.deb ]] || die '--mvs-deb 必须为存在的本地 .deb 文件，不能是 ZIP 或 URL'
    mvs_deb=$(realpath -e -- "$mvs_deb")
    package_arch=$(dpkg-deb -f "$mvs_deb" Architecture)
    [[ "$package_arch" == amd64 || "$package_arch" == all ]] || die "MVS 包架构不匹配：$package_arch"
    package_name=$(dpkg-deb -f "$mvs_deb" Package)
    case "${package_name,,}" in
      *mvs*|*mvcamctrl*) ;;
      *) die "不是可识别的 MVS 包：$package_name；请检查下载文件" ;;
    esac
  elif [[ -f "$mvs_root/include/MvCameraControl.h" ]] && find_mvs_library; then
    log "复用已有 MVS：$mvs_root（文件存在不等于驱动已通过实机验收）"
  elif "$dry_run"; then
    log 'MVS 尚缺：实际安装前请提供 --mvs-deb FILE，或明确 --skip-camera'
  else
    die '未找到完整 MVS SDK。请从 Hikrobot 官网下载 Linux amd64 SDK 并传 --mvs-deb；离线开发可用 --skip-camera'
  fi
fi

if ((EUID != 0)); then
  if ! "$dry_run"; then command -v sudo >/dev/null || die '需要 sudo 或 root 安装系统包'; fi
  sudo_cmd=(sudo)
fi
if "$assume_yes"; then apt_yes=(-y); fi
log "Ubuntu $VERSION_ID / amd64；OpenVINO=$with_openvino；相机=$with_camera；Intel GPU=$with_gpu"
if grep -qi microsoft /proc/sys/kernel/osrelease; then
  log 'WSL 只作为离线构建环境；本脚本不安装 Windows 驱动，也不执行 USB 转接或设备测试。'
fi
if ! "$dry_run" && ! "$assume_yes"; then
  read -r -p '将安装系统包，必要时添加官方 APT 源；继续？[y/N] ' answer
  [[ "$answer" == y || "$answer" == Y ]] || die '用户取消，未开始安装'
fi

# 公钥限定于对应仓库的 signed-by，不将下载脚本直接交给 root 执行。
# 只使用本项目命名的源/公钥文件，不改写用户已有配置。
add_repository() (
  local url=$2 repository=$3 work
  local keyring=/usr/share/keyrings/xtyf-$1.gpg
  local list=/etc/apt/sources.list.d/xtyf-$1.list
  local line="deb [arch=amd64 signed-by=$keyring] $repository"
  if "$dry_run"; then
    printf '  [仓库] %s\n  [公钥] %s -> %s\n  [源文件] %s\n' "$line" "$url" "$keyring" "$list"
    exit 0
  fi
  work=$(mktemp -d -t xtyf-dependencies.XXXXXXXX)
  trap 'rm -rf -- "$work"' EXIT
  curl --fail --location --proto '=https' --tlsv1.2 --connect-timeout 20 --max-time 120 --retry 2 \
    "$url" -o "$work/key.asc"
  # 临时 GNUPGHOME 避免以安装脚本身份创建/修改用户的 ~/.gnupg。
  mkdir -m 700 "$work/gnupg"
  gpg --homedir "$work/gnupg" --batch --yes --dearmor --output "$work/key.gpg" "$work/key.asc"
  printf '%s\n' "$line" > "$work/source.list"
  "${sudo_cmd[@]}" install -m 0644 "$work/key.gpg" "$keyring"
  "${sudo_cmd[@]}" install -m 0644 "$work/source.list" "$list"
)

log '安装编译工具与当前九模块实际使用的库'
run "${sudo_cmd[@]}" apt-get update
run "${sudo_cmd[@]}" apt-get install "${apt_yes[@]}" --no-install-recommends \
  ca-certificates curl gnupg software-properties-common
run "${sudo_cmd[@]}" add-apt-repository -y universe
run "${sudo_cmd[@]}" apt-get update
run "${sudo_cmd[@]}" apt-get install "${apt_yes[@]}" --no-install-recommends \
  build-essential cmake ninja-build pkg-config git python3 \
  libopencv-dev libeigen3-dev libyaml-cpp-dev libusb-1.0-0-dev can-utils usbutils

# 项目自身需要 3.22；新 OpenVINO 官方工具链要求至少 3.26。
# Ubuntu 22.04 的系统 CMake 较旧，仅在需要时使用 Kitware 官方源升级。
minimum_cmake=3.22
if "$with_openvino"; then minimum_cmake=3.26; fi
if "$dry_run"; then
  log "实际安装后检查 CMake >= $minimum_cmake；不足时才添加以下 Kitware 源"
  add_repository kitware https://apt.kitware.com/keys/kitware-archive-latest.asc "https://apt.kitware.com/ubuntu/ $suite main"
  run "${sudo_cmd[@]}" apt-get update
  run "${sudo_cmd[@]}" apt-get install "${apt_yes[@]}" --no-install-recommends cmake
else
  cmake_version=$(cmake --version | awk 'NR == 1 {print $3}')
  if ! dpkg --compare-versions "$cmake_version" ge "$minimum_cmake"; then
    add_repository kitware https://apt.kitware.com/keys/kitware-archive-latest.asc "https://apt.kitware.com/ubuntu/ $suite main"
    run "${sudo_cmd[@]}" apt-get update
    run "${sudo_cmd[@]}" apt-get install "${apt_yes[@]}" --no-install-recommends cmake
  fi
fi

if "$with_openvino"; then
  log "安装 OpenVINO C/C++ Runtime 与开发文件：$openvino_version"
  # 已有仓库能提供指定包时直接复用，避免与用户现有 signed-by 配置冲突。
  if "$dry_run" || ! apt-cache show "openvino-$openvino_version" >/dev/null 2>&1; then
    add_repository openvino https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB \
      "https://apt.repos.intel.com/openvino $intel_suite main"
    run "${sudo_cmd[@]}" apt-get update
  fi
  run "${sudo_cmd[@]}" apt-get install "${apt_yes[@]}" --no-install-recommends "openvino-$openvino_version"
fi
if "$with_gpu"; then
  log '安装 Ubuntu Intel OpenCL 用户态运行库；不升级内核、不加载模块、不查询 GPU'
  run "${sudo_cmd[@]}" apt-get install "${apt_yes[@]}" --no-install-recommends intel-opencl-icd ocl-icd-libopencl1 clinfo
fi
if "$with_camera" && [[ -n "$mvs_deb" ]]; then
  log "安装本地 MVS 官方包（会运行厂商维护脚本）：$package_name"
  run "${sudo_cmd[@]}" apt-get install "${apt_yes[@]}" -- "$mvs_deb"
fi

if "$dry_run"; then
  log '预览结束：未安装软件、未访问网络、未修改系统配置；MVS/下载/安装效果未验证。'
  exit 0
fi

log '检查工具和开发文件（不连接任何设备）'
cmake_version=$(cmake --version | awk 'NR == 1 {print $3}')
dpkg --compare-versions "$cmake_version" ge "$minimum_cmake" || die "PATH 中 CMake 仍低于 $minimum_cmake，请检查旧安装覆盖"
g++ --version
cmake --version
pkg-config --modversion opencv4 eigen3 yaml-cpp
python3 --version
cmake_args=()
if "$with_openvino"; then
  openvino_config=$(dpkg-query -L "libopenvino-dev-$openvino_version" | awk '/\/OpenVINOConfig\.cmake$/ {if (!seen++) print}')
  [[ -n "$openvino_config" && -f "$openvino_config" ]] || die '没有找到所选 OpenVINO 版本的 CMake 开发配置'
  cmake_args+=("-DOpenVINO_DIR=$(dirname -- "$openvino_config")")
else
  cmake_args+=(-DAUTOAIM_OPENVINO=OFF)
fi
if "$with_camera"; then
  [[ -f "$mvs_root/include/MvCameraControl.h" ]] && find_mvs_library || die "MVS 安装后仍缺头文件/库；检查 $mvs_root 或传 --mvs-root"
  cmake_args+=(-DAUTOAIM_HIKROBOT=ON "-DAUTOAIM_HIKROBOT_INCLUDE=$mvs_root/include" "-DAUTOAIM_HIKROBOT_LIBRARY=$mvs_library")
else
  cmake_args+=(-DAUTOAIM_HIKROBOT=OFF)
fi
log '依赖文件检查完成。在项目根目录可使用以下配置命令（这里只显示，不执行）：'
printf 'cmake -S . -B build-local -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_STANDARD=20'
printf ' %q' "${cmake_args[@]}"; printf '\n'
printf '%s\n' '设备权限仍需人工核实：串口 dialout、GPU render、USB 使用 MVS 厂商 udev 规则。'
printf '%s\n' '不因安装成功开放硬件入口；不代表相机、GPU、CAN、固件或实机能力已经通过验收。'
