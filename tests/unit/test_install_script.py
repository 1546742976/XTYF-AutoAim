"""安装器的无安装测试：系统安装/联网命令用失败替身拦截，允许 dpkg-deb 读取临时元数据。"""
import os
from pathlib import Path
import platform
import subprocess
import tempfile
import unittest


SCRIPT = Path(__file__).resolve().parents[2] / "install_indenpence.sh"
OS_RELEASE = Path("/etc/os-release")
SUPPORTED = (platform.machine() == "x86_64" and OS_RELEASE.exists()
             and 'ID=ubuntu\n' in OS_RELEASE.read_text()
             and any(f'VERSION_ID="{version}"' in OS_RELEASE.read_text()
                     for version in ("22.04", "24.04")))


@unittest.skipUnless(SUPPORTED, "Dependency installer targets Ubuntu 22.04/24.04 amd64")
class InstallerTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="xtyf-installer-test-")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.calls = self.root / "unexpected-system-call"
        binary = self.root / "bin"
        binary.mkdir()
        for name in ("sudo", "apt-get", "add-apt-repository", "curl", "gpg", "install",
                     "usermod", "systemctl", "mktemp", "mkdir"):
            command = binary / name
            command.write_text('#!/bin/sh\nprintf "%s\\n" "$0 $*" >> "$INSTALL_TEST_CALLS"\nexit 97\n')
            command.chmod(0o755)
        self.environment = dict(os.environ, PATH=f"{binary}:{os.environ['PATH']}",
                                INSTALL_TEST_CALLS=str(self.calls))

    def invoke(self, *arguments, success=True):
        result = subprocess.run(["bash", str(SCRIPT), *arguments], env=self.environment,
                                cwd=self.root, text=True, capture_output=True, timeout=15)
        self.assertFalse(self.calls.exists(), self.calls.read_text() if self.calls.exists() else "")
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
            "Description: Metadata fixture; never install\n")
        output = self.root / f"camera package {architecture}.deb"
        subprocess.run(["dpkg-deb", "--build", str(payload), str(output)],
                       check=True, stdout=subprocess.DEVNULL, timeout=15)
        return output

    def test_syntax_and_help(self):
        subprocess.run(["bash", "-n", str(SCRIPT)], check=True, timeout=15)
        self.assertIn("--mvs-deb", self.invoke("--help"))

    def test_default_software_preview(self):
        output = self.invoke("--dry-run", "--skip-camera", "--yes")
        for package in ("build-essential", "cmake", "python3", "libopencv-dev", "libeigen3-dev",
                        "libyaml-cpp-dev", "libusb-1.0-0-dev", "can-utils", "openvino-2026.3.1"):
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
        self.assertIn("安装本地 MVS 官方包", output)
        self.assertIn("camera\\ package\\ amd64.deb", output)
        self.assertIn("架构不匹配", self.invoke("--dry-run", "--mvs-deb",
                                               str(self.package("arm64")), success=False))

    def test_bad_arguments_fail_before_system_changes(self):
        for arguments in (("--unknown",), ("--mvs-deb",), ("--openvino-version", "latest"),
                          ("--openvino-version", "1.2.3;bad"), ("--mvs-deb", "absent.deb"),
                          ("--skip-camera", "--mvs-deb", "absent.deb"),
                          ("--skip-openvino", "--intel-gpu")):
            with self.subTest(arguments=arguments):
                self.invoke("--dry-run", *arguments, success=False)


if __name__ == "__main__":
    unittest.main()
