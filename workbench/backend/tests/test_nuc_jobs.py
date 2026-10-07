import copy
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import shutil

from fastapi.testclient import TestClient

from workbench.backend.app import create_app
from workbench.backend.store import FLAGS, read_json, read_yaml, write_json


SOURCE = Path(__file__).resolve().parents[3]


class NucJobsTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="autoaim-nuc-jobs-")
        root = Path(self.temporary.name)
        source = root / "source"
        shutil.copytree(SOURCE / "config", source / "config")
        self.app = create_app(source, root / "workspace", start_worker=False)
        self.store = self.app.state.store
        self.tasks = self.app.state.tasks
        self.client = TestClient(self.app)
        self.target = {"host": "192.0.2.10", "user": "fixture", "port": 2222,
                       "project_dir": "/home/fixture/项目", "build_dir": "/home/fixture/build",
                       "device_config": "/home/fixture/hardware.yaml",
                       "workspace_dir": "/home/fixture/runs"}
        self.client.post("/api/nuc", json=self.target).raise_for_status()
        values = copy.deepcopy(self.store.values)
        values["active_detector"] = "traditional"
        self.profile = self.store.save_profile("Hardware parameter fixture", values)
        self.options = {"profile_id": self.profile["id"], "flags": {flag: False for flag in FLAGS},
                        "openvino": False, "build_type": "Release"}

    def tearDown(self):
        self.tasks.stop()
        self.client.close()
        self.temporary.cleanup()

    def test_target_can_be_partial_for_probe_but_not_run(self):
        response = self.client.post("/api/nuc", json={"host": "192.0.2.11", "user": "fixture"})
        self.assertEqual(response.status_code, 200)
        self.assertFalse(response.json()["configured"])
        probe = self.client.post("/api/jobs", json={"kind": "nuc_probe", "options": {}})
        self.assertEqual(probe.status_code, 201)
        run = self.client.post("/api/jobs", json={"kind": "nuc_run", "options": self.options})
        self.assertEqual(run.status_code, 422)

    def test_nuc_run_freezes_profile_target_and_selection_without_local_build(self):
        job = self.tasks.submit("nuc_run", self.options)
        request_path = Path(job["directory"]) / "nuc_request.json"
        frozen = read_json(request_path)
        self.assertEqual(frozen["profile_values"]["detectors"]["traditional"]["brightness"], 100)
        values = read_yaml(Path(self.profile["config_path"]))
        values["detectors"]["traditional"]["brightness"] = 255
        Path(self.profile["config_path"]).write_text("changed after submit", encoding="utf-8")
        self.client.post("/api/nuc", json={**self.target, "host": "192.0.2.12"}).raise_for_status()
        self.assertEqual(read_json(request_path), frozen)
        self.assertEqual(frozen["target"]["host"], self.target["host"])
        self.assertEqual(frozen["selection"]["flags"], self.options["flags"])
        label, argv, output = self.tasks.command_plan(job)[0]
        self.assertEqual(label, "nuc_run")
        self.assertEqual(argv[-2:], ["run", str(request_path)])
        self.assertIn("workbench.backend.nuc", argv)
        self.assertNotIn("build_id", job["options"])
        self.assertEqual(self.tasks.builds(), [])

    def test_nuc_constraints_and_missing_profile(self):
        for changed in ({"flags": {}}, {"openvino": "false"}, {"build_type": "Unknown"}):
            with self.assertRaises(ValueError):
                self.tasks.submit("nuc_run", {**self.options, **changed})
        flags = {**self.options["flags"], "AUTOAIM_I3_LINEAR_CA": True, "AUTOAIM_USE_ESO": True}
        with self.assertRaisesRegex(ValueError, "mutually exclusive"):
            self.tasks.submit("nuc_run", {**self.options, "flags": flags})

    def test_failed_remote_stop_keeps_job_running(self):
        job = self.tasks.submit("nuc_run", self.options)
        self.tasks.jobs[job["id"]]["status"] = "running"
        self.tasks._save(self.tasks.jobs[job["id"]])
        with patch.object(self.tasks, "_cancel_nuc", side_effect=ValueError("SSH disconnected")):
            response = self.client.post("/api/jobs/" + job["id"] + "/cancel")
        self.assertEqual(response.status_code, 422)
        record = self.tasks.get(job["id"])
        self.assertEqual(record["status"], "running")
        self.assertFalse(record["cancel_requested"])
        self.assertNotIn(job["id"], self.tasks.remote_stops)
        self.tasks.jobs[job["id"]]["status"] = "interrupted"

    def test_remote_stop_requires_cancel_acknowledgement(self):
        job = self.tasks.submit("nuc_run", self.options)
        root = Path(job["directory"])
        write_json(root / "nuc_cancel_report.json", {"action": "run", "status": "running"})
        with patch("workbench.backend.jobs.subprocess.run") as run:
            run.return_value.returncode = 0
            run.return_value.stdout = b"no stop acknowledgement"
            with self.assertRaisesRegex(ValueError, "停止尚未确认"):
                self.tasks._cancel_nuc(self.tasks.jobs[job["id"]])
            write_json(root / "nuc_cancel_report.json", {"action": "cancel", "status": "cancelled"})
            self.tasks._cancel_nuc(self.tasks.jobs[job["id"]])
        commands = self.tasks.get(job["id"])["commands"]
        self.assertEqual(len(commands), 2)
        self.assertEqual(commands[0]["name"], "nuc_cancel")
        self.assertIn("remote stop", (root / "log.txt").read_text())


if __name__ == "__main__":
    unittest.main()
