"""安装器的无安装测试：系统安装/联网命令用失败替身拦截，允许 dpkg-deb 读取临时元数据。"""
import os
from pathlib import Path
import platform
import subprocess
import tempfile
import unittest
import zipfile


SCRIPT = Path(__file__).resolve().parents[2] / "install_dependence.sh"
OS_RELEASE = Path("/etc/os-release")
SUPPORTED = (
    platform.machine() == "x86_64"
    and OS_RELEASE.exists()
    and 'ID=ubuntu\n' in OS_RELEASE.read_text()
    and any(
        f'VERSION_ID="{version}"' in OS_RELEASE.read_text()
        for version in ("22.04", "24.04")
    )
)


@unittest.skipUnless(SUPPORTED, "Dependency installer targets Ubuntu 22.04/24.04 amd64")
class InstallerTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="xtyf-installer-test-")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.calls = self.root / "unexpected-system-call"
        binary = self.root / "bin"
        self.binary = binary
        binary.mkdir()

        for name in (
            "sudo", "apt-get", "add-apt-repository", "curl", "gpg", "install",
            "usermod", "systemctl"
        ):
            command = binary / name
            command.write_text('#!/bin/sh\nprintf "%s\\n" "$0 $*" >> "$INSTALL_TEST_CALLS"\nexit 97\n')
            command.chmod(0o755)

        self.environment = dict(
            os.environ,
            PATH=f"{binary}:{os.environ['PATH']}",
            INSTALL_TEST_CALLS=str(self.calls),
            TMPDIR=str(self.root)
        )

    def invoke(self, *arguments, success=True):
        result = subprocess.run(
            ["bash", str(SCRIPT), *arguments], env=self.environment,
            cwd=self.root, text=True, capture_output=True, timeout=15
        )
        self.assertFalse(self.calls.exists(), self.calls.read_text() if self.calls.exists() else "")
        self.assertEqual(list(self.root.glob("xtyf-mvs.*")), [], "相机临时文件必须在成功/失败后清理")

        if success:
            self.assertEqual(result.returncode, 0, result.stderr)
        else:
            self.assertNotEqual(result.returncode, 0, result.stdout)

        return result.stdout + result.stderr

    def package(self, architecture):
        # 只构建测试元数据包供 dpkg-deb 读取；安装命令被替身阻止，绝不安装它。
        payload = self.root / f"payload-{architecture}"
        control = payload / "DEBIAN"
        control.mkdir(parents=True)
        (control / "control").write_text(
            "Package: mvcamctrlsdk-xtyf-test\nVersion: 1.0\n"
            f"Architecture: {architecture}\nMaintainer: Test <test@example.invalid>\n"
            "Description: Metadata fixture; never install\n"
        )
        output = self.root / f"camera package {architecture}.deb"
        subprocess.run(
            ["dpkg-deb", "--build", str(payload), str(output)],
            check=True, stdout=subprocess.DEVNULL, timeout=15
        )

        return output

    def archive(self, members):
        output = self.root / "camera bundle.zip"

        with zipfile.ZipFile(output, "w") as archive:
            for name, content in members.items():
                archive.writestr(name, content)

        return output

    def mock_download(self, source, status=0):
        # 仅 curl 被定向为本地复制；sudo/APT 等仍然是会导致测试失败的替身。
        command = self.binary / "curl"
        command.write_text(
            '#!/bin/sh\nprintf "%s\\n" "$@" > "$MVS_TEST_CURL_ARGS"\n'
            'while [ "$#" -gt 0 ]; do\n'
            '  if [ "$1" = "--output" ]; then output=$2; shift; fi\n'
            '  shift\ndone\n'
            'cp -- "$MVS_TEST_DOWNLOAD" "$output"\nexit "$MVS_TEST_CURL_STATUS"\n'
        )
        self.environment.update(
            MVS_TEST_DOWNLOAD=str(source),
            MVS_TEST_CURL_STATUS=str(status),
            MVS_TEST_CURL_ARGS=str(self.root / "curl-args")
        )

    def test_syntax_and_help(self):
        subprocess.run(["bash", "-n", str(SCRIPT)], check=True, timeout=15)
        self.assertIn("--mvs-deb", self.invoke("--help"))

    def test_default_software_preview(self):
        output = self.invoke("--dry-run", "--skip-camera", "--yes")

        for package in (
            "build-essential", "cmake", "python3", "libopencv-dev", "libeigen3-dev",
            "libyaml-cpp-dev", "libusb-1.0-0-dev", "can-utils", "openvino-2026.3.1"
        ):
            self.assertIn(package, output)

        self.assertIn("signed-by=", output)
        self.assertIn("预览结束", output)

    def test_traditional_only_preview(self):
        output = self.invoke("--dry-run", "--skip-camera", "--skip-openvino")
        self.assertNotIn("apt.repos.intel.com", output)
        self.assertNotIn("openvino-2026", output)

    def test_gpu_preview(self):
        output = self.invoke("--dry-run", "--skip-camera", "--intel-gpu")
        self.assertIn("intel-opencl-icd", output)
        self.assertIn("ocl-icd-libopencl1", output)

    def test_camera_missing_is_explicit(self):
        root = str(self.root / "missing MVS")
        self.assertIn("MVS 尚缺", self.invoke("--dry-run", "--mvs-root", root))
        self.assertIn("未找到完整 MVS", self.invoke("--yes", "--mvs-root", root, success=False))

    def test_existing_sdk_preview(self):
        sdk = self.root / "MVS existing"
        (sdk / "include").mkdir(parents=True)
        (sdk / "lib/64").mkdir(parents=True)
        (sdk / "include/MvCameraControl.h").touch()
        (sdk / "lib/64/libMvCameraControl.so").touch()
        self.assertIn("复用已有 MVS", self.invoke("--dry-run", "--mvs-root", str(sdk)))

    def test_local_package_and_architecture(self):
        output = self.invoke("--dry-run", "--mvs-deb", str(self.package("amd64")))
        self.assertIn("安装选定 MVS 包", output)
        self.assertIn("camera\\ package\\ amd64.deb", output)
        self.assertIn(
            "架构不匹配",
            self.invoke("--dry-run", "--mvs-deb", str(self.package("arm64")), success=False)
        )

    def test_url_preview_never_downloads(self):
        output = self.invoke("--dry-run", "--mvs-url", "https://www.hikrobotics.com/example.zip")
        self.assertIn("本次不联网", output)
        self.assertIn("下载及包检查成功后", output)

    def test_check_only_local_package(self):
        output = self.invoke("--check-camera-only", "--mvs-deb", str(self.package("amd64")))
        self.assertIn("相机文件检查完成", output)
        self.assertNotIn("apt-get", output)

    def test_zip_nested_paths_and_metadata_selection(self):
        archive = self.archive({
            # 文件名与架构故意相反，必须看实际 DEB 元数据。
            "SDK/Linux arm64/driver.deb": self.package("amd64").read_bytes(),
            "SDK/Linux x86_64/driver.deb": self.package("arm64").read_bytes(),
            "../../must-not-extract": b"not extracted",
            "SDK/setup.sh": b"exit 97\n",
        })
        output = self.invoke("--dry-run", "--mvs-archive", str(archive))
        self.assertIn("已检查 MVS 包结构与 amd64 架构", output)
        self.assertIn("candidate-0.deb", output)
        self.assertFalse((self.root / "SDK").exists())

    def test_zip_wrong_architecture_or_multiple_packages(self):
        arm = self.package("arm64").read_bytes()
        amd = self.package("amd64").read_bytes()

        for members, count in (
            ({"arm.deb": arm}, 0),
            ({"a.deb": amd, "b.deb": amd}, 2),
            ({"SDK/setup.sh": b"exit 97"}, 0)
        ):
            with self.subTest(count=count):
                output = self.invoke(
                    "--check-camera-only", "--mvs-archive",
                    str(self.archive(members)), success=False
                )
                self.assertIn(f"找到 {count} 个 amd64", output)

    def test_html_or_truncated_zip_is_rejected(self):
        path = self.root / "fake SDK.zip"

        for content in (b"<!doctype html><title>403 Forbidden</title>", b"PK\x03\x04truncated"):
            with self.subTest(content=content):
                path.write_bytes(content)
                output = self.invoke("--yes", "--mvs-archive", str(path), success=False)
                self.assertIn("不是有效 DEB/ZIP", output)

    def test_zip_crc_error_is_rejected(self):
        archive = self.archive({"runtime.deb": self.package("amd64").read_bytes()})
        content = archive.read_bytes()
        # ZIP_STORED 将 DEB 原样嵌入；损坏一个字节后，ZipFile 必须拒绝 CRC 不符。
        position = content.index(b"!<arch>")
        archive.write_bytes(content[:position] + b"?" + content[position + 1:])
        output = self.invoke("--check-camera-only", "--mvs-archive", str(archive), success=False)
        self.assertIn("CRC", output)

    def test_truncated_deb_data_is_rejected(self):
        package = self.package("amd64")
        package.write_bytes(package.read_bytes()[:-40])
        output = self.invoke("--yes", "--mvs-deb", str(package), success=False)
        self.assertIn("损坏或不完整", output)

    def test_zip_deb_path_traversal_is_rejected(self):
        archive = self.archive({"../runtime.deb": self.package("amd64").read_bytes()})
        output = self.invoke("--check-camera-only", "--mvs-archive", str(archive), success=False)
        self.assertIn("路径非法", output)

    def test_downloaded_deb_and_zip_check_without_installation(self):
        package = self.package("amd64")
        archive = self.archive({"nested/runtime.deb": package.read_bytes()})

        for source in (package, archive):
            with self.subTest(source=source):
                self.mock_download(source)
                output = self.invoke(
                    "--check-camera-only", "--mvs-url",
                    "https://www.hikrobotics.com/example?download=1"
                )
                self.assertIn("相机文件检查完成", output)
                self.assertIn("selected.deb" if source == package else "candidate-0.deb", output)
                self.assertNotIn("download.part", output)
                arguments = (self.root / "curl-args").read_text().splitlines()

                for flag in ("--fail", "--location", "--proto", "--proto-redir", "--max-time"):
                    self.assertIn(flag, arguments)

                self.assertEqual(arguments[arguments.index("--proto-redir") + 1], "=https")

    def test_download_failure_or_html_never_reaches_installation(self):
        html = self.root / "response.html"
        html.write_text("<html>Access Restricted</html>")

        for status in (22, 18, 0):  # HTTP 403/404、传输中断、HTTP 200 的 HTML 拦截页。
            with self.subTest(status=status):
                self.mock_download(html, status)
                output = self.invoke(
                    "--yes", "--mvs-url",
                    "https://www.hikrobotics.com/example.zip", success=False
                )
                self.assertIn("未开始系统安装", output)
                self.assertIn("下载失败" if status else "不是有效 DEB/ZIP", output)

    def test_bad_arguments_fail_before_system_changes(self):
        for arguments in (
            ("--unknown",), ("--mvs-deb",), ("--openvino-version", "latest"),
            ("--openvino-version", "1.2.3;bad"), ("--mvs-deb", "absent.deb"),
            ("--skip-camera", "--mvs-deb", "absent.deb"),
            ("--mvs-url", "http://www.hikrobotics.com/example.zip"),
            ("--mvs-url", "https://example.invalid/a b.zip"),
            ("--mvs-url", "https://example.invalid/a", "--mvs-archive", "a.zip"),
            ("--skip-camera", "--check-camera-only"),
            ("--check-camera-only",),  # 与 invoke 添加的 --dry-run 互斥。
            ("--mvs-archive",), ("--mvs-url",),
            ("--skip-openvino", "--intel-gpu")
        ):
            with self.subTest(arguments=arguments):
                self.invoke("--dry-run", *arguments, success=False)


if __name__ == "__main__":
    unittest.main()
