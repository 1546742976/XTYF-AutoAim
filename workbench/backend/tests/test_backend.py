import copy
import hashlib
import os
from pathlib import Path
import shutil
import sys
import tempfile
import time
import unittest

from fastapi.testclient import TestClient
import yaml

from workbench.backend import create_app
from workbench.backend.jobs import JobQueue
from workbench.backend.results import artifact_file, comparison, read_commands, results
from workbench.backend.store import Workspace, FLAGS, leaves, read_json, write_json


SOURCE = Path(__file__).resolve().parents[3]


class BackendTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="autoaim-workbench-test-")
        self.root = Path(self.temporary.name)
        self.source = self.root / "source"
        shutil.copytree(SOURCE / "config", self.source / "config")
        self.workspace = self.root / "workspace"
        self.store = Workspace(self.source, self.workspace)
        self.tasks = JobQueue(self.store)
        self.initial = self.source_inventory()

    def tearDown(self):
        self.tasks.stop()
        self.assertEqual(self.initial, self.source_inventory(), "Backend changed source inputs")
        self.temporary.cleanup()

    def source_inventory(self):
        return {str(path.relative_to(self.source)): hashlib.sha256(path.read_bytes()).hexdigest()
                for path in self.source.rglob("*") if path.is_file()}

    def profile(self, name="Traditional", **changes):
        values = copy.deepcopy(self.store.values)
        values["active_detector"] = "traditional"
        for path, value in changes.items():
            keys = path.split(".")
            target = values
            for key in keys[:-1]:
                target = target[key]
            target[keys[-1]] = value
        return self.store.save_profile(name, values)

    def ready_build(self):
        build = self.tasks.submit("build", {"name": "Fixture build"})
        self.tasks.jobs[build["id"]]["status"] = "succeeded"
        self.tasks._save(self.tasks.jobs[build["id"]])
        directory = Path(build["directory"]) / "acceptance/build"
        directory.mkdir(parents=True)
        for name in ("autoaim_node", "synthetic_sim", "offline_replay", "replay_visualizer", "pipeline_metrics", "bench_detector"):
            (directory / name).touch()
        return build["id"]

    def test_workspace_must_be_outside_source(self):
        for path in (self.source, self.source / "out", self.root):
            with self.assertRaises(ValueError):
                Workspace(self.source, path)

    def test_template_covers_every_leaf_and_comments(self):
        template = self.store.template()
        self.assertEqual(set(dict(leaves(template["values"]))), {field["path"] for field in template["fields"]})
        fields = {field["path"]: field for field in template["fields"]}
        self.assertIn("整数[0,255]", fields["common.refinement.brightness"]["description"])
        self.assertIn("顺序", fields["common.tracker.initial_variance"]["description"])
        self.assertEqual(fields["common.refinement.brightness"]["type"], "integer")
        self.assertEqual(fields["common.pnp.maximum_rms_px"]["type"], "number")
        self.assertEqual(len(template["contracts"]), 9)

    def test_profile_package_backlinks_local_resources_missing_unselected_models(self):
        saved = self.profile(**{"common.pnp.maximum_rms_px": 2.5, "common.motion.maximum_horizon_s": 0.8})
        fast = yaml.safe_load(Path(saved["config_path"]).read_text(encoding="utf-8"))
        self.assertEqual(fast["common"]["pnp"]["maximum_rms_px"], 2.5)
        for key in ("traditional", "yolov5", "yolo11"):
            base_path = Path(saved["config_path"]).parent / fast["detectors"][key]["config_file"]
            base = yaml.safe_load(base_path.read_text(encoding="utf-8"))
            self.assertEqual((base_path.parent / base["fast_choose_file"]).resolve(), Path(saved["config_path"]))
            self.assertFalse(Path(base["calibration_file"]).is_absolute())
            calibration = (base_path.parent / base["calibration_file"]).resolve()
            self.assertTrue(calibration.is_relative_to(Path(saved["config_path"]).parent / "resources"))
            self.assertEqual(calibration.read_bytes(), (self.source / "config/offline/calibration.yaml").read_bytes())
            for reference in base["geometry_files"]:
                geometry = (base_path.parent / reference).resolve()
                self.assertTrue(geometry.is_relative_to(Path(saved["config_path"]).parent / "resources"))
                self.assertEqual(geometry.read_bytes(), (self.source / "config/offline/geometry.yaml").read_bytes())
        self.assertFalse(Path(fast["detectors"]["yolov5"]["model_path"]).exists())
        second = self.profile()
        self.assertNotEqual(saved["id"], second["id"])
        self.assertEqual(len(self.store.list_profiles()), 2)

    def test_selected_base_and_resource_files_are_preserved(self):
        external = self.root / "inputs"
        external.mkdir()
        shutil.copy2(self.source / "config/offline/calibration.yaml", external / "custom-calibration.yaml")
        shutil.copy2(self.source / "config/offline/geometry.yaml", external / "custom-geometry.yaml")
        self.store.register_root(str(external))
        values = copy.deepcopy(self.store.values)
        values["active_detector"] = "traditional"
        selected = {"traditional.calibration_file": str(external / "custom-calibration.yaml"),
                    "traditional.geometry_files.0": str(external / "custom-geometry.yaml")}
        saved = self.store.save_profile("Custom resources", values, selected)
        base_path = Path(saved["config_path"]).parent / "base/traditional.yaml"
        base = yaml.safe_load(base_path.read_text(encoding="utf-8"))
        self.assertEqual((base_path.parent / base["calibration_file"]).read_bytes(), Path(selected["traditional.calibration_file"]).read_bytes())
        self.assertEqual((base_path.parent / base["geometry_files"][0]).read_bytes(), Path(selected["traditional.geometry_files.0"]).read_bytes())
        resources = {entry["key"]: entry for entry in saved["contracts"]}
        self.assertEqual(resources["traditional.calibration_file"]["path"], selected["traditional.calibration_file"])

    def test_profile_and_submitted_snapshot_resources_survive_original_edits(self):
        external = self.root / "mutable-inputs"
        external.mkdir()
        originals = {}
        for name in ("calibration", "geometry"):
            path = external / (name + ".yaml")
            shutil.copy2(self.source / "config/offline" / (name + ".yaml"), path)
            originals[name] = path
        self.store.register_root(str(external))
        values = copy.deepcopy(self.store.values)
        values["active_detector"] = "traditional"
        selected = {"traditional.calibration_file": str(originals["calibration"]),
                    "traditional.geometry_files.0": str(originals["geometry"])}
        saved = self.store.save_profile("Frozen resources", values, selected)
        build = self.ready_build()
        job = self.tasks.submit("check_config", {"build_id": build, "profile_id": saved["id"]})
        expected = {name: path.read_bytes() for name, path in originals.items()}
        bases = [Path(saved["config_path"]).parent / "base/traditional.yaml",
                 Path(job["snapshots"][saved["id"]]).parent / "base/traditional.yaml"]
        base_before = [path.read_bytes() for path in bases]
        for name, path in originals.items():
            path.write_text("modified: " + name, encoding="utf-8")
        for base_path, previous in zip(bases, base_before):
            self.assertEqual(base_path.read_bytes(), previous)
            document = yaml.safe_load(previous)
            for name, reference in (("calibration", document["calibration_file"]),
                                    ("geometry", document["geometry_files"][0])):
                copied = (base_path.parent / reference).resolve()
                self.assertTrue(copied.is_relative_to(base_path.parent.parent / "resources"))
                self.assertEqual(copied.read_bytes(), expected[name])
        contracts = {entry["key"]: entry for entry in saved["contracts"]}
        for name, key in (("calibration", "traditional.calibration_file"),
                          ("geometry", "traditional.geometry_files.0")):
            self.assertEqual(contracts[key]["path"], str(originals[name]))
            self.assertEqual(contracts[key]["content"].encode("utf-8"), expected[name])

    def test_calibration_report_references_are_snapshotted_with_calibration(self):
        external = self.root / "calibration-with-reports"
        (external / "evidence").mkdir(parents=True)
        calibration_path = external / "calibration.yaml"
        calibration = yaml.safe_load((self.source / "config/offline/calibration.yaml").read_text(encoding="utf-8"))
        reports = {}
        for field in ("intrinsic_report_file", "extrinsic_report_file"):
            report = external / "evidence" / (field + ".yaml")
            report.write_text("report_kind: " + field + "\nprovenance: original-evidence\n", encoding="utf-8")
            calibration["calibration"][field] = "evidence/" + report.name
            reports[field] = (report, report.read_bytes())
        calibration_path.write_text(yaml.safe_dump(calibration), encoding="utf-8")
        original_calibration = calibration_path.read_bytes().decode("utf-8")
        self.store.register_root(str(external))
        values = copy.deepcopy(self.store.values)
        values["active_detector"] = "traditional"
        saved = self.store.save_profile("Evidence snapshot", values,
                                        {"traditional.calibration_file": str(calibration_path)})
        job = self.tasks.submit("check_config", {"build_id": self.ready_build(), "profile_id": saved["id"]})
        for report, _ in reports.values():
            report.write_text("evidence changed after submission", encoding="utf-8")
        for fast_choose in (Path(saved["config_path"]), Path(job["snapshots"][saved["id"]])):
            base = yaml.safe_load((fast_choose.parent / "base/traditional.yaml").read_text(encoding="utf-8"))
            copied_calibration = (fast_choose.parent / "base" / base["calibration_file"]).resolve()
            copied = yaml.safe_load(copied_calibration.read_text(encoding="utf-8"))["calibration"]
            for field, (_, expected) in reports.items():
                copied_report = (copied_calibration.parent / copied[field]).resolve()
                self.assertTrue(copied_report.is_relative_to(fast_choose.parent / "resources"))
                self.assertEqual(copied_report.read_bytes(), expected)
        selected = {entry["key"]: entry for entry in saved["contracts"]}
        self.assertEqual(selected["traditional.calibration_file"]["content"], original_calibration)

    def test_profile_types_and_shape_are_validated(self):
        for path, value in (("common.refinement.brightness", 1.5), ("common.queue.maximum_age_s", float("nan")),
                            ("common.impact.aim_variance_rad2", [0, float("inf"), 0])):
            with self.assertRaises(ValueError):
                self.profile(**{path: value})
        values = copy.deepcopy(self.store.values)
        del values["common"]["queue"]
        with self.assertRaises(ValueError):
            self.store.save_profile("Incomplete", values)

    def test_duplicate_base_keys_are_rejected_before_packaging(self):
        external = self.root / "duplicate-contract"
        external.mkdir()
        self.store.register_root(str(external))
        base = yaml.safe_load((self.source / "config/offline/armor.yaml").read_text(encoding="utf-8"))
        base["calibration_file"] = str(self.source / "config/offline/calibration.yaml")
        base["geometry_files"] = [str(self.source / "config/offline/geometry.yaml")]
        selected = external / "armor.yaml"
        selected.write_text(yaml.safe_dump(base) + "\nrole: sentry\n", encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "Duplicate YAML key"):
            self.profile(**{"detectors.traditional.config_file": str(selected)})
        self.assertEqual(self.store.list_profiles(), [])

    def test_build_constraints_and_selection_command(self):
        options = {"flags": {flag: False for flag in FLAGS}, "openvino": False}
        options["flags"]["AUTOAIM_I3_LINEAR_CA"] = True
        options["flags"]["AUTOAIM_USE_ESO"] = True
        with self.assertRaisesRegex(ValueError, "mutually exclusive"):
            self.tasks.submit("build", options)
        for flag in ("AUTOAIM_I9_THROUGHPUT", "AUTOAIM_I9_PREALLOC"):
            options = {"flags": {key: key == flag for key in FLAGS}, "openvino": True}
            with self.assertRaisesRegex(ValueError, "both YOLO"):
                self.tasks.submit("build", options)
        build = self.tasks.submit("build", {})
        argv = self.tasks.command_plan(build)[0][1]
        self.assertIn("--selection-file", argv)
        self.assertEqual(argv[argv.index("--build-type") + 1], "Release")
        self.assertFalse((Path(build["directory"]) / "acceptance").exists())
        selection = read_json(Path(build["directory"]) / "selection.json")
        self.assertEqual(set(selection["options"]), set(FLAGS))

    def test_jobs_snapshot_profiles_and_replay_commands(self):
        build = self.ready_build()
        saved = self.profile()
        dataset = self.source / "config/offline/calibration-dataset.yaml"
        job = self.tasks.submit("replay", {"build_id": build, "profile_id": saved["id"], "dataset": str(dataset)})
        snapshot = Path(job["snapshots"][saved["id"]])
        original = snapshot.read_bytes()
        Path(saved["config_path"]).write_text("later edits", encoding="utf-8")
        self.assertEqual(snapshot.read_bytes(), original)
        plans = self.tasks.command_plan(job)
        self.assertEqual([step[0] for step in plans], ["offline_replay", "replay_visualizer", "pipeline_metrics"])
        self.assertIn("--record-session", plans[0][1])
        self.assertIn("--uart-output", plans[0][1])
        self.assertIn(str(snapshot), plans[0][1])

    def test_benchmark_groups_configurations_per_build(self):
        first, second = self.ready_build(), self.ready_build()
        profiles = [self.profile("one"), self.profile("two")]
        runs = [{"build_id": first, "profile_id": item["id"]} for item in profiles]
        runs.append({"build_id": second, "profile_id": profiles[0]["id"]})
        job = self.tasks.submit("benchmark", {"runs": runs, "dataset": str(self.source / "config/offline/calibration-dataset.yaml"), "iou": 0.5})
        plans = self.tasks.command_plan(job)
        self.assertEqual(len(plans), 2)
        self.assertEqual(plans[0][1].count("--config"), 2)
        self.assertEqual(plans[1][1].count("--config"), 1)
        with self.assertRaisesRegex(ValueError, "both uncertainty"):
            self.tasks.submit("benchmark", {"runs": runs, "dataset": str(self.source / "config/offline/calibration-dataset.yaml"), "pose_reference": "truth"})

    def test_restart_interrupts_queued_and_running_and_preserves_commands(self):
        queued = self.tasks.submit("build", {})
        running = self.tasks.submit("build", {})
        self.tasks.jobs[running["id"]]["status"] = "running"
        self.tasks.jobs[running["id"]]["commands"] = [{"argv": ["cmake"], "cwd": running["directory"], "exit_code": None}]
        self.tasks._save(self.tasks.jobs[running["id"]])
        restarted = JobQueue(self.store)
        self.assertEqual(restarted.get(queued["id"])["status"], "interrupted")
        self.assertEqual(restarted.get(running["id"])["status"], "interrupted")
        self.assertEqual(restarted.get(running["id"])["commands"][0]["argv"], ["cmake"])
        restarted.stop()

    def test_queue_serial_execution_failure_and_cancellation(self):
        self.tasks.command_plan = lambda job: [("queue infrastructure", [sys.executable, "-B", "-c",
            "import time; print('started', flush=True); time.sleep(0.05); print('finished')"], None)]
        first = self.tasks.submit("build", {})
        second = self.tasks.submit("build", {})
        cancelled = self.tasks.submit("build", {})
        self.tasks.cancel(cancelled["id"])
        self.tasks.start()
        deadline = time.monotonic() + 8
        while self.tasks.get(second["id"])["status"] not in {"succeeded", "failed"} and time.monotonic() < deadline:
            time.sleep(0.02)
        one, two = self.tasks.get(first["id"]), self.tasks.get(second["id"])
        self.assertEqual(one["status"], "succeeded")
        self.assertEqual(two["status"], "succeeded")
        self.assertLessEqual(one["finished_at"], two["started_at"])
        self.assertEqual(one["commands"][0]["exit_code"], 0)
        self.assertIn("finished", (Path(first["directory"]) / "log.txt").read_text())
        self.assertEqual(self.tasks.get(cancelled["id"])["status"], "cancelled")

    def test_nonzero_process_failure_is_persisted_with_log(self):
        self.tasks.command_plan = lambda job: [("failing process", [sys.executable, "-B", "-c",
            "import sys; print('failure diagnostic',flush=True); sys.exit(7)"], None)]
        job = self.tasks.submit("build", {})
        self.tasks.start()
        deadline = time.monotonic() + 5
        while self.tasks.get(job["id"])["status"] in {"queued", "running"} and time.monotonic() < deadline:
            time.sleep(0.01)
        record = self.tasks.get(job["id"])
        self.assertEqual(record["status"], "failed")
        self.assertEqual(record["commands"][0]["exit_code"], 7)
        self.assertIn("failure diagnostic", (Path(job["directory"]) / "log.txt").read_text())
        self.assertEqual(read_json(Path(job["directory"]) / "job.json")["status"], "failed")

    @unittest.skipUnless(os.name == "posix", "Process-group cancellation is for Linux/WSL")
    def test_running_cancel_terminates_process_group(self):
        self.tasks.command_plan = lambda job: [("long process", [sys.executable, "-B", "-c",
            "import subprocess,time; subprocess.Popen(['sleep','60']); print('ready',flush=True); time.sleep(60)"], None)]
        job = self.tasks.submit("build", {})
        self.tasks.start()
        deadline = time.monotonic() + 5
        while not self.tasks.process and time.monotonic() < deadline:
            time.sleep(0.01)
        self.tasks.cancel(job["id"])
        while self.tasks.get(job["id"])["status"] == "running" and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertEqual(self.tasks.get(job["id"])["status"], "cancelled")

    def test_artifacts_cannot_escape_and_missing_metrics_remain_null(self):
        job = self.tasks.submit("build", {})
        root = Path(job["directory"])
        (root / "commands.tsv").write_text("frame_id\tage_s\tspace\n0\tNA\trelative\n", encoding="utf-8")
        self.assertIsNone(read_commands(root / "commands.tsv")[0]["age_s"])
        with self.assertRaises(KeyError):
            artifact_file(job, "../../roots.json")
        (root / "visualized").mkdir()
        (root / "visualized/1_2.png").write_bytes(b"PNG fixture")
        (root / "recording").mkdir()
        (root / "recording/frame_2.png").write_bytes(b"Original PNG fixture")
        parsed = results(job)
        self.assertEqual(parsed["frames"][0]["source"], "independent_replay")
        self.assertEqual(parsed["frames"][0]["frame_id"], 2)
        self.assertTrue(parsed["frames"][0]["original_url"].endswith("recording/frame_2.png"))

    def test_partial_benchmark_artifacts_bind_group_not_sorted_position(self):
        build = self.ready_build()
        profile = self.profile()
        job = self.tasks.submit("benchmark", {"runs": [{"build_id": build, "profile_id": profile["id"]}],
            "dataset": str(self.source / "config/offline/calibration-dataset.yaml")})
        job["options"]["runs"] = [{"build_id": "build-" + str(index), "profile_id": profile["id"]}
                                    for index in range(11)]
        root = Path(job["directory"])
        for index in (0, 2, 10):
            folder = root / ("benchmark_" + str(index))
            folder.mkdir()
            report = {"dataset_fingerprint": "same", "minimum_iou": 0.5, "pose_metrics": "not_produced",
                      "runs": [{"configuration_path": job["snapshots"][profile["id"]],
                                "effective_configuration": {"detector": {"enemy": "red"}},
                                "summary": {"marker": index}}]}
            (folder / "report.yaml").write_text(yaml.safe_dump(report), encoding="utf-8")
            if index != 0:
                costs = {"runs": [{"configuration_path": job["snapshots"][profile["id"]], "marker": index}]}
                (folder / "timing.yaml").write_text(yaml.safe_dump(costs), encoding="utf-8")
        compared = results(job)["comparison"]
        self.assertFalse(compared["compatible"])
        self.assertEqual([row["build_id"] for row in compared["rows"]], ["build-0", "build-2", "build-10"])
        self.assertIsNone(compared["rows"][0]["timing"])
        self.assertEqual(compared["rows"][1]["timing"]["marker"], 2)
        self.assertEqual(compared["rows"][2]["timing"]["marker"], 10)

    def test_comparison_rejects_different_conditions(self):
        job = {"options": {"runs": [{"build_id": "a"}, {"build_id": "b"}]}, "snapshots": {}}
        report = {"dataset_fingerprint": {"hash": "one"}, "minimum_iou": 0.5, "pose_reference": None,
                  "pose_metrics": "not_produced", "runs": [{"effective_configuration": {"detector": {"enemy": "red"}},
                  "summary": {"precision": None, "negative_frames": 0, "unlabeled_frames": 2}}]}
        equal = comparison(job, [report, copy.deepcopy(report)], [])
        self.assertTrue(equal["compatible"])
        self.assertIsNone(equal["rows"][0]["summary"]["precision"])
        for change in ("fingerprint", "enemy", "iou", "pose"):
            other = copy.deepcopy(report)
            if change == "fingerprint":
                other["dataset_fingerprint"] = {"hash": "two"}
            elif change == "enemy":
                other["runs"][0]["effective_configuration"]["detector"]["enemy"] = "blue"
            elif change == "iou":
                other["minimum_iou"] = 0.7
            else:
                other["pose_reference"] = "different reference"
            checked = comparison(job, [report, other], [])
            self.assertFalse(checked["compatible"], change)
            self.assertTrue(checked["reasons"])

    def test_api_routes_paths_log_and_registered_roots(self):
        app = create_app(self.source, self.workspace, start_worker=False)
        with TestClient(app) as client:
            context = client.get("/api/context").json()
            self.assertEqual(context["source_root"], str(self.source))
            self.assertEqual(len(context["flags"]), 6)
            values = client.get("/api/config/template").json()["values"]
            values["active_detector"] = "traditional"
            saved = client.post("/api/profiles", json={"name": "API", "values": values})
            self.assertEqual(saved.status_code, 201)
            self.assertEqual(len(client.get("/api/profiles").json()), 1)
            job = client.post("/api/jobs", json={"kind": "build", "options": {}}).json()
            self.assertEqual(client.get("/api/jobs/" + job["id"]).status_code, 200)
            self.assertEqual(client.get("/api/jobs/" + job["id"] + "/log").json(), {"text": "", "next_offset": 0})
            self.assertEqual(client.post("/api/jobs/" + job["id"] + "/cancel").json()["status"], "cancelled")
            self.assertEqual(client.get("/api/files", params={"path": str(self.root)}).status_code, 422)
            self.assertEqual(client.get("/api/artifacts/" + job["id"] + "/../../roots.json").status_code, 404)
            self.assertEqual(client.post("/api/roots", json={"path": str(self.root)}).status_code, 201)
            self.assertEqual(client.get("/api/files", params={"path": str(self.root)}).status_code, 200)


if __name__ == "__main__":
    unittest.main()
