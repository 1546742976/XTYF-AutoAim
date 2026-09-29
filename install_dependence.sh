#!/usr/bin/env bash
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
mvs_archive=
mvs_url=
check_camera_only=false
camera_work=
mvs_root=/opt/MVS
mvs_library=

sudo_cmd=()
apt_yes=()

usage() {
  cat <<'HELP'
用法：bash install_dependence.sh [选项]
平台：Ubuntu 22.04/24.04 amd64（原生 Linux 或 WSL）。

默认：安装 C++17/20 工具链、CMake、OpenCV/Eigen/yaml-cpp、Python3、
      OpenVINO 2026.3.1、USB/CAN 配套库，并要求 Hikrobot MVS SDK。
      MVS 已安装时复用，否则须提供官方 Linux amd64 的 .deb、ZIP 或 HTTPS 直链。

  --dry-run                  只显示计划，不联网、不 sudo、不修改系统配置
  --yes                      不询问主脚本/APT 确认；不代替厂商许可协议
  --skip-camera              只装软件依赖，跳过 MVS（适合 WSL 离线开发）
  --mvs-deb FILE             安装从 Hikrobot 官网下载、解压得到的 .deb
  --mvs-archive FILE         从本地 ZIP 中选择唯一的 amd64 MVS .deb（支持子目录）
  --mvs-url URL              下载指定的 HTTPS .deb/ZIP，校验成功后才开始系统安装
  --check-camera-only        只检查 SDK/相机包；URL 会下载，不 sudo、不安装
  --mvs-root DIR             已安装 SDK 根目录，默认 /opt/MVS
  --skip-openvino            不安装 OpenVINO，构建须 AUTOAIM_OPENVINO=OFF
  --openvino-version X.Y.Z   选择显式版本；默认项目已验证的 2026.3.1
  --intel-gpu                附加 Ubuntu 的 Intel OpenCL 运行库及 clinfo
  -h, --help                 显示帮助

例：
  bash install_dependence.sh --dry-run --skip-camera
  bash install_dependence.sh --yes --skip-camera
  bash install_dependence.sh --mvs-archive /absolute/path/MvCamCtrlSDK.zip
  bash install_dependence.sh --check-camera-only --mvs-deb /absolute/path/MvCamCtrlSDK_Runtime.deb

不会执行 apt upgrade/remove、禁用服务、修改 .bashrc、自动重启、
调整 USB/CAN 参数或自动给用户增权。安装 MVS 会运行厂商包的维护脚本；
请先检查包来源及许可，安装后的设备权限/驱动加载/硬件效果由人验收。
相机包检查需要已有 python3/dpkg-deb；URL 下载还需要 curl/系统 CA 证书。
本地包检查（包括预览）会使用并清理临时文件；不会执行其中的 setup.sh。
直链由使用者从官网下载页取得，不绕过网站验证，不使用第三方镜像。
若遇 403/登录页，请用浏览器按官方流程下载，再传 --mvs-archive/--mvs-deb。
--dry-run 不联网，不能据此确认远程下载有效；与 --check-camera-only 互斥。
HELP
}

die() {
  printf '错误：%s\n' "$*" >&2
  exit 1
}

log() {
  printf '\n[依赖安装] %s\n' "$*"
}

run() {
  printf '  +'
  printf ' %q' "$@"
  printf '\n'

  if ! "$dry_run"; then
    "$@"
  fi
}

need_value() {
  [[ $# -ge 2 && -n "$2" && "$2" != --* ]] || die "$1 缺少参数值"
}

while (($#)); do
  case "$1" in
    --dry-run)
      dry_run=true
      shift
      ;;
    --yes)
      assume_yes=true
      shift
      ;;
    --skip-camera)
      with_camera=false
      shift
      ;;
    --skip-openvino)
      with_openvino=false
      shift
      ;;
    --intel-gpu)
      with_gpu=true
      shift
      ;;
    --mvs-deb)
      need_value "$@"
      mvs_deb=$2
      shift 2
      ;;
    --mvs-archive)
      need_value "$@"
      mvs_archive=$2
      shift 2
      ;;
    --mvs-url)
      need_value "$@"
      mvs_url=$2
      shift 2
      ;;
    --check-camera-only)
      check_camera_only=true
      shift
      ;;
    --mvs-root)
      need_value "$@"
      mvs_root=$2
      shift 2
      ;;
    --openvino-version)
      need_value "$@"
      openvino_version=$2
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die "未知参数：$1（使用 --help 查看用法）"
      ;;
  esac
done

[[ "$openvino_version" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]] || die 'OpenVINO 版本必须形如 2026.3.1'

camera_sources=0

for camera_source in "$mvs_deb" "$mvs_archive" "$mvs_url"; do
  if [[ -n "$camera_source" ]]; then
    camera_sources=$((camera_sources + 1))
  fi
done

((camera_sources <= 1)) || die '--mvs-deb、--mvs-archive、--mvs-url 只能选择一个'

if ! "$with_camera" && { ((camera_sources > 0)) || "$check_camera_only"; }; then
  die '--skip-camera 不能与相机包选项或 --check-camera-only 同时使用'
fi
if "$check_camera_only" && "$dry_run"; then
  die '--check-camera-only 与 --dry-run 不能同时使用'
fi

if [[ -n "$mvs_url" ]]; then
  [[ "$mvs_url" == https://* && ! "$mvs_url" =~ [[:space:]] ]] || die '--mvs-url 必须是 HTTPS 直链'
fi
if "$with_gpu" && ! "$with_openvino"; then
  die '--intel-gpu 与 --skip-openvino 不能同时使用'
fi

[[ $(uname -s) == Linux && -r /etc/os-release ]] || die '请在 Ubuntu Linux/WSL 中运行'

# shellcheck source=/dev/null
source /etc/os-release
[[ "$ID" == ubuntu ]] || die '本脚本只支持 Ubuntu，不修改其它发行版的软件源'

case "$VERSION_ID" in
  22.04)
    suite=jammy
    intel_suite=ubuntu22
    ;;
  24.04)
    suite=noble
    intel_suite=ubuntu24
    ;;
  *)
    die "未支持的 Ubuntu 版本：$VERSION_ID（需要 22.04 或 24.04）"
    ;;
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

# 相机包在任何 APT/sudo 操作之前准备完毕。临时目录仅由 mktemp 创建，
# 失败/中断后清理自己的下载和候选包；不覆盖用户文件或留下可复用的半包。
cleanup_camera() {
  if [[ -n "$camera_work" ]]; then
    rm -rf -- "$camera_work"
  fi
}

trap cleanup_camera EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

prepare_camera_package() {
  local input=$1

  command -v python3 >/dev/null ||
    die '相机包检查需要 python3；请先安装它，或用 --skip-camera 准备软件环境'
  command -v dpkg-deb >/dev/null || die '相机包检查需要 dpkg-deb'
  camera_work=$(mktemp -d -t xtyf-mvs.XXXXXXXX)

  if [[ -n "$mvs_url" ]]; then
    command -v curl >/dev/null || die 'MVS 下载需要 curl 和系统 CA 证书；也可提供本地包'
    log '下载 MVS；不使用旧的固定直链，不绕过官网访问控制'
    if ! curl --fail --location --proto '=https' --proto-redir '=https' --tlsv1.2 \
      --connect-timeout 20 --max-time 600 --retry 2 --retry-max-time 120 \
      --output "$camera_work/download.part" -- "$mvs_url"; then
      die 'MVS 下载失败（例如 HTTP 403/404、连接中断）；未开始系统安装。请按官网流程下载后传 --mvs-archive 或 --mvs-deb'
    fi
    input=$camera_work/download.part
  fi

  # 不使用 ZIP 内路径写文件，不 extractall，不执行安装脚本。按 DEB 元数据
  # 而非文件名判定架构；多个匹配包必须由人明确选择，不能随便安装第一个。
  mvs_deb=$(python3 - "$input" "$camera_work" <<'PY'
from pathlib import Path, PurePosixPath
import shutil
import subprocess
import sys
import zipfile

source, work = map(Path, sys.argv[1:])


def metadata(path):
    fields = []

    for field in ("Package", "Architecture"):
        result = subprocess.run(
            ["dpkg-deb", "-f", str(path), field], capture_output=True, text=True
        )

        if result.returncode:
            raise ValueError("损坏或无效的 DEB，无法读取元数据")

        fields.append(result.stdout.strip())

    return fields


def is_mvs(name):
    return "mvs" in name.lower() or "mvcamctrl" in name.lower()


try:
    with source.open("rb") as stream:
        magic = stream.read(8)

    if magic == b"!<arch>\n":
        name, arch = metadata(source)

        if not is_mvs(name):
            raise ValueError(f"不是可识别的 MVS 包：{name}；请检查来源")
        if arch != "amd64":
            raise ValueError(f"MVS 包架构不匹配：{arch}（需要 amd64）")

        selected = source
    elif zipfile.is_zipfile(source):
        candidates = []

        with zipfile.ZipFile(source) as archive:
            for index, entry in enumerate(archive.infolist()):
                if entry.is_dir() or not entry.filename.lower().endswith(".deb"):
                    continue

                parts = PurePosixPath(entry.filename)
                if parts.is_absolute() or ".." in parts.parts:
                    raise ValueError("ZIP 中的 DEB 路径非法")

                candidate = work / f"candidate-{index}.deb"
                with archive.open(entry) as packed, candidate.open("wb") as output:
                    shutil.copyfileobj(packed, output)  # 读到 EOF 同时检查所选成员的 CRC。

                name, arch = metadata(candidate)
                if is_mvs(name) and arch == "amd64":
                    candidates.append(candidate)

            if len(candidates) != 1:
                raise ValueError(
                    f"ZIP 中找到 {len(candidates)} 个 amd64 MVS DEB；"
                    "需要唯一匹配，请解压并用 --mvs-deb 明确选择；"
                    "仅含 tar.gz/setup.sh 的发行包须按厂商说明人工处理"
                )

            selected = candidates[0]
    else:
        raise ValueError(
            "不是有效 DEB/ZIP（可能是 HTML 登录/拦截页或未下载完整）；"
            "请按官网流程重新获取安装包"
        )

    # -f 只检查控制归档；还要读取数据归档，避免 DEB 尾部截断留到安装时才暴露。
    result = subprocess.run(
        ["dpkg-deb", "--contents", str(selected)],
        stdout=subprocess.DEVNULL, stderr=subprocess.PIPE
    )

    if result.returncode:
        raise ValueError("DEB 数据归档损坏或不完整")

    # HTTP URL/临时文件未必带 .deb 后缀，而 apt-get 按后缀识别本地包。
    # 只复制到自己的临时目录，绝不改名/移动使用者提供的文件。
    if selected.suffix != ".deb":
        prepared = work / "selected.deb"
        shutil.copyfile(selected, prepared)
        selected = prepared

    print(selected.resolve())
except (OSError, ValueError, RuntimeError, zipfile.BadZipFile, EOFError) as error:
    sys.exit(f"错误：MVS 包检查失败：{error}")
PY
  ) || die '相机包检查未通过，未开始系统安装'
  package_name=$(dpkg-deb -f "$mvs_deb" Package)
  log "已检查 MVS 包结构与 amd64 架构：$package_name；安装文件：$mvs_deb（不证明包来源或设备可用）"
}

if "$with_camera"; then
  if [[ -n "$mvs_url" ]] && "$dry_run"; then
    log "计划下载 MVS HTTPS 直链：$mvs_url；实际安装前检查 DEB/ZIP 内容与架构（本次不联网）"
  elif ((camera_sources > 0)); then
    camera_input=${mvs_deb:-$mvs_archive}

    if [[ -z "$mvs_url" ]]; then
      [[ -f "$camera_input" ]] || die 'MVS 本地包不存在或不是普通文件'
      camera_input=$(realpath -e -- "$camera_input")
    fi

    prepare_camera_package "$camera_input"
  elif [[ -f "$mvs_root/include/MvCameraControl.h" ]] && find_mvs_library; then
    log "复用已有 MVS：$mvs_root（文件存在不等于驱动已通过实机验收）"
  elif "$dry_run"; then
    log 'MVS 尚缺：实际安装前请提供 --mvs-deb/--mvs-archive/--mvs-url，或明确 --skip-camera'
  else
    die '未找到完整 MVS SDK。请从 Hikrobot 官网下载 Linux amd64 SDK 并传 --mvs-deb/--mvs-archive/--mvs-url；离线开发可用 --skip-camera'
  fi
fi

if "$check_camera_only"; then
  log '相机文件检查完成：未 sudo、未安装、未连接设备；临时下载/解包文件退出时清理。'
  exit 0
fi

if ((EUID != 0)); then
  if ! "$dry_run"; then
    command -v sudo >/dev/null || die '需要 sudo 或 root 安装系统包'
  fi

  sudo_cmd=(sudo)
fi

if "$assume_yes"; then
  apt_yes=(-y)
fi

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

if "$with_openvino"; then
  minimum_cmake=3.26
fi

if "$dry_run"; then
  log "实际安装后检查 CMake >= $minimum_cmake；不足时才添加以下 Kitware 源"
  add_repository kitware https://apt.kitware.com/keys/kitware-archive-latest.asc \
    "https://apt.kitware.com/ubuntu/ $suite main"
  run "${sudo_cmd[@]}" apt-get update
  run "${sudo_cmd[@]}" apt-get install "${apt_yes[@]}" --no-install-recommends cmake
else
  cmake_version=$(cmake --version | awk 'NR == 1 {print $3}')
  if ! dpkg --compare-versions "$cmake_version" ge "$minimum_cmake"; then
    add_repository kitware https://apt.kitware.com/keys/kitware-archive-latest.asc \
      "https://apt.kitware.com/ubuntu/ $suite main"
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
  run "${sudo_cmd[@]}" apt-get install "${apt_yes[@]}" --no-install-recommends \
    intel-opencl-icd ocl-icd-libopencl1 clinfo
fi

if "$with_camera" && [[ -n "$mvs_deb" ]]; then
  log "安装选定 MVS 包（会运行厂商维护脚本，请自行核实来源）：$package_name"
  run "${sudo_cmd[@]}" apt-get install "${apt_yes[@]}" -- "$mvs_deb"
elif "$with_camera" && [[ -n "$mvs_url" ]] && "$dry_run"; then
  log '下载及包检查成功后，使用 apt-get install 安装选定的 amd64 MVS DEB；此处不执行。'
fi

if "$dry_run"; then
  log '预览结束：未安装软件、未访问网络、未修改系统配置；MVS/下载/安装效果未验证。'
  exit 0
fi

log '检查工具和开发文件（不连接任何设备）'
cmake_version=$(cmake --version | awk 'NR == 1 {print $3}')
dpkg --compare-versions "$cmake_version" ge "$minimum_cmake" ||
  die "PATH 中 CMake 仍低于 $minimum_cmake，请检查旧安装覆盖"

g++ --version
cmake --version
pkg-config --modversion opencv4 eigen3 yaml-cpp
python3 --version

cmake_args=()

if "$with_openvino"; then
  openvino_config=$(dpkg-query -L "libopenvino-dev-$openvino_version" |
    awk '/\/OpenVINOConfig\.cmake$/ {if (!seen++) print}')
  [[ -n "$openvino_config" && -f "$openvino_config" ]] || die '没有找到所选 OpenVINO 版本的 CMake 开发配置'
  cmake_args+=("-DOpenVINO_DIR=$(dirname -- "$openvino_config")")
else
  cmake_args+=(-DAUTOAIM_OPENVINO=OFF)
fi

if "$with_camera"; then
  [[ -f "$mvs_root/include/MvCameraControl.h" ]] && find_mvs_library ||
    die "MVS 安装后仍缺头文件/库；检查 $mvs_root 或传 --mvs-root"
  cmake_args+=(
    -DAUTOAIM_HIKROBOT=ON
    "-DAUTOAIM_HIKROBOT_INCLUDE=$mvs_root/include"
    "-DAUTOAIM_HIKROBOT_LIBRARY=$mvs_library"
  )
else
  cmake_args+=(-DAUTOAIM_HIKROBOT=OFF)
fi

log '依赖文件检查完成。在项目根目录可使用以下配置命令（这里只显示，不执行）：'
printf 'cmake -S . -B build-local -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_STANDARD=20'
printf ' %q' "${cmake_args[@]}"
printf '\n'
printf '%s\n' '设备权限仍需人工核实：串口 dialout、GPU render、USB 使用 MVS 厂商 udev 规则。'
printf '%s\n' '不因安装成功开放硬件入口；不代表相机、GPU、CAN、固件或实机能力已经通过验收。'
