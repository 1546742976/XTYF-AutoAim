"""Local bridge contract tests; no SSH server, camera, or serial device is used."""

import copy
import io
import json
import os
from pathlib import Path
import shlex
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from unittest import mock

import yaml

from workbench.backend import nuc, remote_runner


class TargetTests(unittest.TestCase):
    def test_save_and_probe_allow_missing_paths(self):
        target = nuc.validate_target({"host": "10.141.143.124", "user": "xtyf"})
        self.assertEqual(target["port"], 22)
        self.assertEqual(target["executable"], "autoaim_node")
        self.assertEqual(target["workspace_dir"], "")
        with self.assertRaisesRegex(ValueError, "project_dir"):
            nuc.validate_target(target, require_paths=True)
        with mock.patch.object(remote_runner.os, "uname", return_value=("Linux", "nuc", "1", "1", "x86_64"), create=True):
            report = remote_runner.dispatch("probe", {"job_id": "probe", "target": target})
        self.assertEqual(report["status"], "incomplete")
        self.assertFalse(report["ready"])
        self.assertEqual(set(report["missing"]), set(nuc.REQUIRED_PATHS))
        self.assertNotIn("build_information", report)

    def test_target_rejects_options_injection_and_bad_port(self):
        for key, bad in (("host", "-oProxyCommand=evil"), ("host", "host;echo evil"),
                         ("host", "host name"), ("host", "host\nname"),
                         ("user", "user@host"), ("user", "-root"), ("user", "$(id)"),
                         ("port", True), ("port", 0), ("port", 65536), ("port", "22")):
            with self.subTest(key=key, bad=bad), self.assertRaises(ValueError):
                nuc.validate_target({"host": "nuc.local", "user": "xtyf", key: bad})
        with self.assertRaises(ValueError):
            nuc.validate_target({"host": "nuc", "user": "xtyf", "build_dir": "relative/build"})
        with self.assertRaises(ValueError):
            nuc.validate_target({"host": "nuc", "user": "xtyf", "executable": "../outside"})

    def test_ssh_argv_uses_fixed_bootstrap_and_preserves_unicode_paths(self):
        with tempfile.TemporaryDirectory() as temporary:
            identity = Path(temporary) / "私人 密钥"
            identity.write_text("test key", encoding="utf-8")
            target = nuc.validate_target({"host": "nuc.local", "user": "xtyf", "port": 2222,
                                          "identity_file": str(identity), "project_dir": "/家/巡天 御风",
                                          "build_dir": "/家/build;literal", "device_config": "/家/设备.yaml",
                                          "workspace_dir": "/家/运行", "executable": "bin/自瞄程序"}, True)
            argv = nuc.build_ssh_argv(target)
            self.assertIn("StrictHostKeyChecking=yes", argv)
            self.assertIn("BatchMode=yes", argv)
            self.assertIn("ConnectTimeout=10", argv)
            self.assertEqual(argv[-2], "xtyf@nuc.local")
            self.assertEqual(shlex.split(argv[-1]), ["python3", "-u", "-c", nuc.BOOTSTRAP])
            self.assertEqual(argv[argv.index("-i") + 1], str(identity.resolve()))
            self.assertNotIn("巡天", argv[-1])
            self.assertEqual(target["project_dir"], "/家/巡天 御风")

    def test_transport_framing_and_local_report(self):
        class Input(io.StringIO):
            payload = ""
            def close(self):
                self.payload = self.getvalue()
                super().close()

        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "request.json"
            path.write_text(json.dumps({"job_id": "probe-test", "target": {"host": "nuc", "user": "xtyf"}}))
            report = {"schema_version": 1, "action": "probe", "job_id": "probe-test", "status": "incomplete", "ready": False}
            process = mock.Mock()
            process.stdin = Input()
            process.stdout = io.StringIO(
                nuc.EVENT_PREFIX + json.dumps({"event": "log", "text": "NUC_REPORT {fake}\n"}) + "\n" +
                nuc.EVENT_PREFIX + json.dumps({"event": "report", "report": report}) + "\n")
            process.wait.return_value = 0
            process.poll.return_value = 0
            with mock.patch.object(nuc.subprocess, "Popen", return_value=process) as launch, mock.patch("sys.stdout", new=io.StringIO()) as output:
                result = nuc.execute_request("probe", path)
            self.assertEqual(result["status"], "incomplete")
            self.assertEqual(json.loads((path.parent / "nuc_report.json").read_text()), result)
            payload = json.loads(process.stdin.payload)
            self.assertEqual(payload["action"], "probe")
            self.assertIn("def dispatch", payload["source"])
            self.assertIn("NUC_REPORT {fake}", output.getvalue())
            self.assertNotIn("shell", launch.call_args.kwargs)
            # A finishing run/probe and a concurrent stop cannot overwrite the
            # stop acknowledgement or share the same temporary report file.
            path.write_text(json.dumps({"job_id": "probe-test", "target": {
                "host": "nuc", "user": "xtyf", "workspace_dir": "/tmp/nuc-runs"}}))
            cancellation = {"schema_version": 1, "action": "cancel", "job_id": "probe-test",
                            "status": "cancelled", "stopped": True}
            stopped_process = mock.Mock()
            stopped_process.stdin = Input()
            stopped_process.stdout = io.StringIO(nuc.EVENT_PREFIX + json.dumps({"event": "report", "report": cancellation}) + "\n")
            stopped_process.wait.return_value = stopped_process.poll.return_value = 0
            with mock.patch.object(nuc.subprocess, "Popen", return_value=stopped_process), mock.patch("sys.stdout", new=io.StringIO()):
                stopped = nuc.execute_request("cancel", path)
            self.assertEqual(json.loads((path.parent / "nuc_cancel_report.json").read_text()), stopped)
            self.assertEqual(json.loads((path.parent / "nuc_report.json").read_text()), result)


FAKE_PROGRAM = r'''#!__PYTHON__
import copy,json,os,signal,subprocess,sys,time
from pathlib import Path
import yaml
ROOT = Path(__ROOT__)
config = Path(sys.argv[sys.argv.index('--config') + 1])
fast = yaml.safe_load(config.read_text())
base = yaml.safe_load((config.parent / fast['detectors'][fast['active_detector']]['config_file']).read_text())
def merge(dst, src):
    for k,v in src.items():
        if isinstance(v,dict):
            merge(dst.setdefault(k,{}),v)
        else:
            dst[k] = v
effective = copy.deepcopy(base)
merge(effective,fast['common'])
merge(effective.setdefault('detector',{}),{k:v for k,v in fast['detectors'][fast['active_detector']].items() if k!='config_file'})
merge(effective.setdefault('eso',{}),fast['eso'])
stage = 'check' if '--check-config' in sys.argv else 'launch'
with (ROOT/'trace.jsonl').open('a') as f:
    f.write(json.dumps({'stage':stage,'brightness':effective['detector'].get('brightness'),'execution':effective['execution']})+'\n')
if stage == 'check':
    if base.get('fake_check_delay'):
        (ROOT/'check_waiting').write_text(str(os.getpid()))
        while True: time.sleep(.05)
    effective['execution'] = base.get('fake_check_execution',effective['execution'])
    print(yaml.safe_dump({'check_schema_version':1,'valid':base.get('fake_valid',True),'effective_configuration':effective}),flush=True)
    sys.exit(base.get('fake_check_exit',0))
(ROOT/'applied.json').write_text(json.dumps(effective))
print('NUC fake stdout 参数已应用',flush=True)
print('NUC fake stderr',file=sys.stderr,flush=True)
if base.get('fake_behavior') == 'tree':
    signal.signal(signal.SIGTERM,signal.SIG_IGN)
    grand_code = "import signal,time; signal.signal(signal.SIGTERM,signal.SIG_IGN); time.sleep(120)"
    child_code = "import json,os,signal,subprocess,sys,time; from pathlib import Path; signal.signal(signal.SIGTERM,signal.SIG_IGN); p=subprocess.Popen([sys.executable,'-c',"+repr(grand_code)+"]); Path("+repr(str(ROOT/'tree_pids.json'))+").write_text(json.dumps([os.getpid(),p.pid])); time.sleep(120)"
    subprocess.Popen([sys.executable,'-c',child_code])
    while True: time.sleep(.05)
sys.exit(base.get('fake_run_exit',0))
'''


@unittest.skipUnless(os.name == "posix", "Remote helper runs on Linux/POSIX")
class RemoteRunnerTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.project = self.root / "NUC 巡天御风"
        self.build = self.root / "build"
        self.workspace = self.root / "runs"
        (self.project / "config").mkdir(parents=True)
        (self.build / "generated").mkdir(parents=True)
        template = Path(__file__).resolve().parents[3] / "config" / "fast_choose.yaml"
        self.profile = yaml.safe_load(template.read_text(encoding="utf-8"))
        (self.project / "config" / "fast_choose.yaml").write_text(yaml.safe_dump(self.profile, allow_unicode=True))
        self.metadata = {key: "OFF" for key in remote_runner.FLAGS}
        self.metadata.update(openvino="OFF", build_type="Release", hikrobot="ON")
        (self.build / "generated" / "autoaim_build_information.json").write_text(json.dumps(self.metadata))
        self.device = self.project / "config" / "device.yaml"
        self.base = {"execution": "hardware", "device_id": "camera-physical-123", "role": "infantry",
                     "configuration_id": "NUC-device", "camera": {"serial_number": "真实相机", "exposure_us": 1200},
                     "uart": {"device": "/dev/ttyUSB0", "baud_rate": 115200},
                     "calibration_file": "physical/calibration.yaml", "geometry_files": ["physical/geometry.yaml"],
                     "corners": {"indices": [0, 1, 2, 3], "mapping_id": "measured"},
                     "detector": {"kind": "traditional", "plate_type": "small", "brightness": 10},
                     "motion": {"maximum_horizon_s": 1.2}, "fake_behavior": "once"}
        self.write_device()
        self.program = self.build / "autoaim_node"
        self.program.write_text(FAKE_PROGRAM.replace("__PYTHON__", sys.executable).replace("__ROOT__", repr(str(self.project))), encoding="utf-8")
        self.program.chmod(0o755)
        self.request = {"job_id": "test-job", "target": {
            "host": "nuc.local", "user": "xtyf", "port": 22, "project_dir": str(self.project),
            "build_dir": str(self.build), "workspace_dir": str(self.workspace), "device_config": str(self.device),
            "executable": "autoaim_node", "yolov5_model": "", "yolo11_model": ""},
            "profile_values": self.profile, "selection": {"flags": {key: False for key in remote_runner.FLAGS},
                                                          "openvino": False, "build_type": "Release"}}
        self.request["profile_values"]["active_detector"] = "traditional"
        self.directory = self.workspace / "jobs" / "test-job"
        self.children = []
        self.addCleanup(self.stop_children)

    def write_device(self):
        self.device.write_text(yaml.safe_dump(self.base, allow_unicode=True), encoding="utf-8")

    def stop_children(self):
        for process in self.children:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
            if process.stdout:
                process.stdout.close()
            if process.stderr:
                process.stderr.close()

    def start_helper(self):
        path = self.root / "request.json"
        path.write_text(json.dumps(self.request, ensure_ascii=False), encoding="utf-8")
        helper = Path(remote_runner.__file__).resolve()
        process = subprocess.Popen([sys.executable, "-B", "-u", str(helper), "run", str(path)],
                                   stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        self.children.append(process)
        return process

    def wait_for(self, predicate, timeout=8):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            value = predicate()
            if value:
                return value
            time.sleep(.02)
        self.fail("Timed out waiting for fake helper state")

    def state(self):
        try:
            return json.loads((self.directory / "state.json").read_text())
        except (FileNotFoundError, ValueError):
            return {}

    def test_check_before_launch_applies_profile_and_preserves_hardware(self):
        self.request["profile_values"]["detectors"]["traditional"]["brightness"] = 254
        self.request["profile_values"]["common"]["motion"]["maximum_horizon_s"] = .8
        events = []
        source_before = self.device.read_bytes()
        report = remote_runner.dispatch("run", self.request, events.append)
        self.assertEqual(report["status"], "succeeded", report)
        trace = [json.loads(line) for line in (self.project / "trace.jsonl").read_text().splitlines()]
        self.assertEqual([row["stage"] for row in trace], ["check", "launch"])
        self.assertEqual([row["brightness"] for row in trace], [254, 254])
        applied = json.loads((self.project / "applied.json").read_text())
        self.assertEqual(applied["motion"]["maximum_horizon_s"], .8)
        self.assertEqual(applied["camera"], self.base["camera"])
        self.assertEqual(applied["uart"], self.base["uart"])
        self.assertFalse(applied["program_fire_requested"])
        self.assertEqual(applied["calibration_file"], str(self.device.parent / "physical/calibration.yaml"))
        self.assertEqual(self.device.read_bytes(), source_before)
        fast = yaml.safe_load((self.directory / "config/fast_choose.yaml").read_text())
        hardware = yaml.safe_load((self.directory / "config/hardware.yaml").read_text())
        self.assertEqual(hardware["fast_choose_file"], "fast_choose.yaml")
        self.assertNotIn("brightness", hardware["detector"])
        self.assertNotIn("maximum_horizon_s", hardware.get("motion", {}))
        self.assertEqual(fast["detectors"]["traditional"]["config_file"], "hardware.yaml")
        self.assertEqual([c["kind"] for c in report["commands"]], ["check_config", "hardware_run"])
        self.assertEqual(report["commands"][1]["argv"], [str(self.program), "--config", report["config_path"]])
        self.assertIn("stderr", (self.directory / "run.log").read_text())
        self.assertTrue(any("参数已应用" in e.get("text", "") for e in events))
        saved_state = (self.directory / "state.json").read_bytes()
        saved_config = (self.directory / "config/fast_choose.yaml").read_bytes()
        repeat = remote_runner.dispatch("run", self.request)
        self.assertEqual(repeat["status"], "failed")
        self.assertEqual((self.directory / "state.json").read_bytes(), saved_state)
        self.assertEqual((self.directory / "config/fast_choose.yaml").read_bytes(), saved_config)

    def test_build_mismatch_never_invokes_program(self):
        self.request["selection"]["flags"]["AUTOAIM_I1_CONTRAST_IRLS"] = True
        report = remote_runner.dispatch("run", self.request)
        self.assertEqual(report["status"], "failed")
        self.assertIn("AUTOAIM_I1_CONTRAST_IRLS", report["error"])
        self.assertFalse((self.project / "trace.jsonl").exists())
        self.assertEqual(report["commands"], [])

    def test_workspace_cannot_contain_or_be_inside_source_or_build(self):
        for workspace in (self.project, self.project / "out", self.build / "runs", self.root):
            with self.subTest(workspace=workspace):
                request = copy.deepcopy(self.request)
                request["target"]["workspace_dir"] = str(workspace)
                report = remote_runner.dispatch("run", request)
                self.assertEqual(report["status"], "failed")
                self.assertIn("independent", report["error"])
                self.assertFalse((workspace / "jobs").exists())
        self.assertFalse((self.project / "trace.jsonl").exists())

    def test_executable_must_belong_to_selected_build(self):
        other = self.root / "different-program"
        other.write_text(self.program.read_text())
        other.chmod(0o755)
        self.request["target"]["executable"] = str(other)
        report = remote_runner.dispatch("probe", self.request)
        self.assertFalse(report["ready"])
        self.assertTrue(any("selected build_dir" in message for message in report["diagnostics"]))
        report = remote_runner.dispatch("run", self.request)
        self.assertEqual(report["status"], "failed")
        self.assertIn("selected build_dir", report["error"])
        self.assertFalse((self.project / "trace.jsonl").exists())

    def test_replay_device_configuration_is_rejected_before_check(self):
        self.base["execution"] = "replay"
        self.write_device()
        report = remote_runner.dispatch("run", self.request)
        self.assertEqual(report["status"], "failed")
        self.assertIn("replay", report["error"])
        self.assertFalse((self.project / "trace.jsonl").exists())

    def test_checker_cannot_claim_valid_offline_configuration(self):
        self.base["fake_check_execution"] = "replay"
        self.write_device()
        report = remote_runner.dispatch("run", self.request)
        self.assertEqual(report["status"], "failed")
        self.assertIn("offline entry refused", report["error"])
        trace = [json.loads(line) for line in (self.project / "trace.jsonl").read_text().splitlines()]
        self.assertEqual([row["stage"] for row in trace], ["check"])
        self.assertFalse((self.project / "applied.json").exists())

    def test_checker_invalid_or_error_prevents_launch(self):
        self.base["fake_valid"] = False
        self.write_device()
        report = remote_runner.dispatch("run", self.request)
        self.assertEqual(report["status"], "failed")
        self.assertIn("valid=true", report["error"])
        self.assertEqual(len(report["commands"]), 1)
        self.assertFalse((self.project / "applied.json").exists())

    def test_probe_is_read_only_and_does_not_invoke_check(self):
        before = sorted(str(p.relative_to(self.root)) for p in self.root.rglob("*"))
        report = remote_runner.dispatch("probe", self.request)
        after = sorted(str(p.relative_to(self.root)) for p in self.root.rglob("*"))
        self.assertEqual(report["status"], "ready", report)
        self.assertEqual(report["device_execution"], "hardware")
        self.assertEqual(before, after)
        self.assertFalse((self.project / "trace.jsonl").exists())
        self.assertFalse(self.workspace.exists())

    def test_selected_model_uses_nuc_path_and_unused_models_are_optional(self):
        model = self.root / "NUC 模型.xml"
        model.write_text("fake xml")
        model.with_suffix(".bin").write_bytes(b"fake weights")
        self.request["target"]["yolov5_model"] = str(model)
        self.request["profile_values"]["active_detector"] = "yolov5"
        self.request["profile_values"]["detectors"]["yolov5"]["model_path"] = "D:/Windows/ignored.xml"
        self.metadata["openvino"] = "ON"
        (self.build / "generated/autoaim_build_information.json").write_text(json.dumps(self.metadata))
        self.request["selection"]["openvino"] = True
        report = remote_runner.dispatch("run", self.request)
        self.assertEqual(report["status"], "succeeded", report)
        applied = json.loads((self.project / "applied.json").read_text())
        self.assertEqual(applied["detector"]["model_path"], str(model))
        self.assertEqual(applied["detector"]["kind"], "yolov5")
        self.assertFalse((self.project / "models/yolo11.xml").exists())

    def test_cancel_terminates_check_and_never_launches_hardware(self):
        self.base["fake_check_delay"] = True
        self.write_device()
        process = self.start_helper()
        self.wait_for(lambda: (self.project / "check_waiting").exists())
        check_pid = int((self.project / "check_waiting").read_text())
        report = remote_runner.dispatch("cancel", self.request)
        self.assertEqual(report["status"], "cancelled", report)
        process.communicate(timeout=8)
        self.assertEqual(self.state()["status"], "cancelled")
        self.assertFalse(remote_runner._group_alive(check_pid))
        self.assertFalse((self.project / "applied.json").exists())

    def test_cancel_terminates_child_and_grandchild_group(self):
        self.base["fake_behavior"] = "tree"
        self.write_device()
        process = self.start_helper()
        self.wait_for(lambda: (self.project / "tree_pids.json").exists())
        descendants = json.loads((self.project / "tree_pids.json").read_text())
        state = self.wait_for(lambda: self.state() if self.state().get("status") == "running" else None)
        report = remote_runner.dispatch("cancel", self.request)
        self.assertEqual(report["status"], "cancelled", report)
        self.assertTrue(report["stopped"])
        process.communicate(timeout=8)
        self.assertEqual(self.state()["status"], "cancelled")
        self.assertFalse(remote_runner._group_alive(state["pgid"]))
        for pid in [state["pid"]] + descendants:
            stat_path = Path("/proc", str(pid), "stat")
            if stat_path.exists():
                self.assertEqual(stat_path.read_text().rsplit(")", 1)[1].split()[0], "Z")

    def test_cancel_before_pid_prevents_every_future_launch(self):
        self.directory.mkdir(parents=True)
        remote_runner._write_json(self.directory / "state.json", {"status": "checking"})
        report = remote_runner.dispatch("cancel", self.request)
        self.assertEqual(report["status"], "cancelled")
        self.assertTrue((self.directory / "cancel_requested").exists())
        # The launch boundary checks the same marker under the same lock.
        with remote_runner._launch_lock(self.directory):
            self.assertTrue((self.directory / "cancel_requested").exists())
        missing = copy.deepcopy(self.request)
        missing["job_id"] = "not-created"
        report = remote_runner.dispatch("cancel", missing)
        self.assertEqual(report["status"], "failed")
        self.assertIn("not acknowledged", report["error"])

    def test_helper_sigterm_finally_terminates_process_tree(self):
        self.base["fake_behavior"] = "tree"
        self.write_device()
        process = self.start_helper()
        self.wait_for(lambda: (self.project / "tree_pids.json").exists())
        state = self.state()
        process.send_signal(signal.SIGTERM)
        process.communicate(timeout=8)
        self.assertEqual(self.state()["status"], "cancelled")
        self.assertFalse(remote_runner._group_alive(state["pgid"]))

    def test_ssh_output_disconnection_finally_terminates_process_tree(self):
        self.base["fake_behavior"] = "tree"
        self.write_device()
        process = self.start_helper()
        self.wait_for(lambda: (self.project / "tree_pids.json").exists())
        state = self.state()
        process.stdout.close()
        process.wait(timeout=8)
        self.assertEqual(self.state()["status"], "failed")
        self.assertFalse(remote_runner._group_alive(state["pgid"]))


if __name__ == "__main__":
    unittest.main()
