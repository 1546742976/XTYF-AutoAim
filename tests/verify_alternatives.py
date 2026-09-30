"""Select one CMake alternative in an unchanged independent copy and run full CTest.

I1-CONTRAST-IRLS, I2-LM and I3-LINEAR-CA use OpenVINO OFF.
I9-THROUGHPUT and I9-PREALLOC use OpenVINO ON with both YOLO XML/BIN pairs.
CMake registers alternative-specific tests only for the selected implementation.
ESO and DEFAULT also use OpenVINO OFF. No source or CMake file is rewritten.
Use the repository's build toolchain (for example WSL); output must be a new directory
outside the source tree. Logs and inventories are retained, including after failures.
"""

import argparse
import json
from pathlib import Path
import re
import shutil
import sys
import tempfile

# Importing the shared verification helpers must not write into the source tree.
sys.dont_write_bytecode = True
from verify_eso import checked_path, digest, inventory, run_command, write_json


METHODS = {
    "I1-CONTRAST-IRLS": "AUTOAIM_I1_CONTRAST_IRLS",
    "I2-LM": "AUTOAIM_I2_LM",
    "I3-LINEAR-CA": "AUTOAIM_I3_LINEAR_CA",
    "I9-THROUGHPUT": "AUTOAIM_I9_THROUGHPUT",
    "I9-PREALLOC": "AUTOAIM_I9_PREALLOC",
    "ESO": "AUTOAIM_USE_ESO",
    "DEFAULT": None,
}
OPENVINO_METHODS = {"I9-THROUGHPUT", "I9-PREALLOC"}
SPECIAL_TESTS = {
    "I1-CONTRAST-IRLS": ("test_corner_refine_irls", "unit", "autoaim_vision"),
    "I2-LM": ("test_pnp_lm", "unit", "autoaim_vision"),
    "I3-LINEAR-CA": ("test_linear_ca", "unit", "autoaim_pipeline"),
    "I9-PREALLOC": ("test_queue_prealloc", "contract", "autoaim_pipeline"),
    "ESO": ("test_eso", "unit", "autoaim_decision"),
}
I9_TESTS = {
    "test_openvino_model", "test_openvino_async", "test_pipeline_async",
    "test_openvino_yolo11_model", "test_openvino_yolo11_async", "test_pipeline_yolo11_async",
    "test_batch_external_models",
}


def selection(method):
    if method not in METHODS:
        raise ValueError("Unsupported method: " + str(method))
    # Explicit OFF values prevent a previous cache selection leaking into a run.
    return {option: "ON" if option == METHODS[method] else "OFF"
            for option in METHODS.values() if option is not None}


def validate_source(source):
    text = (source / "CMakeLists.txt").read_text(encoding="utf-8")
    for option in selection("DEFAULT"):
        declarations = re.findall(r"option\(\s*" + option + r'\s+"[^"\n]*"\s+(ON|OFF)\s*\)', text)
        if len(declarations) != 1:
            raise ValueError("Expected one boolean CMake selection: " + option)
    for name, folder, _ in SPECIAL_TESTS.values():
        if not (source / "tests" / folder / (name + ".cpp")).is_file():
            raise ValueError("Missing alternative test: " + name)


def model_inputs(method, yolov5, yolo11, openvino_dir):
    if method not in OPENVINO_METHODS:
        if yolov5 or yolo11 or openvino_dir:
            raise ValueError(method + " runs OpenVINO OFF; model/OpenVINO options apply only to I9")
        return {}, None
    models = {}
    for name, supplied in (("YOLOV5", yolov5), ("YOLO11", yolo11)):
        if supplied is None:
            raise ValueError(method + " requires both --yolov5-model and --yolo11-model")
        xml = checked_path(supplied)
        binary = checked_path(xml.with_suffix(".bin"))
        if xml.suffix.lower() != ".xml" or not xml.is_file() or not binary.is_file():
            raise ValueError("Model needs an existing XML and adjacent BIN: " + str(xml))
        models[name] = {"xml": str(xml), "bin": str(binary),
                        "xml_sha256": digest(xml.read_bytes()),
                        "bin_sha256": digest(binary.read_bytes())}
    directory = checked_path(openvino_dir) if openvino_dir else None
    if directory and not directory.is_dir():
        raise ValueError("--openvino-dir must be an existing directory")
    return models, directory


def verify(source, output, method, jobs=2, cmake="cmake", ctest="ctest",
           yolov5=None, yolo11=None, openvino_dir=None, build_type="Debug"):
    if method not in METHODS or jobs < 1:
        raise ValueError("Select a supported method and a positive --jobs value")
    if build_type not in {"Debug", "Release"}:
        raise ValueError("--build-type must be Debug or Release")
    source, output = checked_path(source), checked_path(output)
    if not (source / "CMakeLists.txt").is_file():
        raise ValueError("--source must name the repository root")
    if source == output or source in output.parents or output in source.parents:
        raise ValueError("--output must be independent of, and must not contain, --source")
    if output.exists():
        raise ValueError("--output already exists; refusing to overwrite acceptance evidence")
    models, openvino_dir = model_inputs(method, yolov5, yolo11, openvino_dir)
    validate_source(source)
    original = inventory(source)
    output.mkdir(parents=True, exist_ok=False)
    copied, build = output / "source", output / "build"
    commands = []
    configuration = {
        "AUTOAIM_OPENVINO": "ON" if method in OPENVINO_METHODS else "OFF",
        "AUTOAIM_HIKROBOT": "OFF", "CMAKE_CXX_STANDARD": "17",
        "CMAKE_BUILD_TYPE": build_type, "BUILD_TESTING": "ON",
    }
    configuration.update(selection(method))
    for name, model in models.items():
        configuration["AUTOAIM_TEST_" + name + "_MODEL"] = model["xml"]
    if openvino_dir:
        configuration["OpenVINO_DIR"] = str(openvino_dir)
    summary = {
        "schema_version": 2, "method": method,
        "scope": "isolated alternative software acceptance; not NUC performance or hardware evidence",
        "original_source": str(source), "copied_source": str(copied),
        "configuration": configuration, "jobs": jobs, "models": models,
        "original_sha256": original["sha256"], "exit_code": 1,
    }
    write_json(output / "original_before.json", original)
    try:
        for relative in original["files"]:
            destination = copied / relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source / relative, destination)
        copied_before = inventory(copied)
        write_json(output / "copied_before.json", copied_before)
        if copied_before != original:
            raise ValueError("Source changed while copying, or copy content differs")
        experiment = inventory(copied)
        write_json(output / "experimental_source.json", experiment)
        summary["experimental_sha256"] = experiment["sha256"]
        if experiment != original:
            raise ValueError("Candidate selection must not modify copied source")
        status = run_command(output, commands, "configure", [
            cmake, "-S", str(copied), "-B", str(build),
            *("-D" + key + "=" + value for key, value in configuration.items())])
        if status == 0:
            cache = (build / "CMakeCache.txt").read_text(encoding="utf-8")
            for option, value in selection(method).items():
                if re.findall(r"^" + option + r":BOOL=(\w+)$", cache, re.M) != [value]:
                    raise ValueError("CMake did not apply selection: " + option)
            status = run_command(output, commands, "build", [
                cmake, "--build", str(build), "--config", build_type, "--parallel", str(jobs)])
            registry_status = run_command(output, commands, "ctest_registry", [
                ctest, "--test-dir", str(build), "-C", build_type, "--show-only=json-v1"])
            if registry_status == 0:
                registry = json.loads((output / "ctest_registry.log").read_text(encoding="utf-8"))
                write_json(output / "ctest_registry.json", registry)
                summary["registered_tests"] = len(registry["tests"])
                expected = set(I9_TESTS) if method in OPENVINO_METHODS else set()
                if method in SPECIAL_TESTS:
                    expected.add(SPECIAL_TESTS[method][0])
                registered = {test["name"] for test in registry["tests"]}
                special_names = {item[0] for item in SPECIAL_TESTS.values()}
                unexpected = (registered & special_names) - expected
                if unexpected:
                    raise ValueError("Unexpected active candidate tests: " + str(sorted(unexpected)))
                missing = expected - registered
                if missing:
                    raise ValueError("Required alternative tests are unregistered: " +
                                     ", ".join(sorted(missing)))
            if status == 0 and registry_status == 0:
                status = run_command(output, commands, "ctest", [
                    ctest, "--test-dir", str(build), "-C", build_type, "--output-on-failure",
                    "--parallel", str(jobs)])
                manifest_status = run_command(output, commands, "verification_manifest", [
                    cmake, "--build", str(build), "--config", build_type,
                    "--target", "verification_manifest"])
                status = status or manifest_status
            else:
                status = status or registry_status
        summary["exit_code"] = status
    except Exception as error:
        summary["error"] = str(error)
        summary["exit_code"] = 1
    finally:
        try:
            after = inventory(source)
            write_json(output / "original_after.json", after)
            summary["original_source_unchanged"] = after == original
            if after != original:
                names = set(original["files"]) | set(after["files"])
                summary["changed_original_files"] = sorted(
                    name for name in names if original["files"].get(name) != after["files"].get(name))
                summary["exit_code"] = 1
        except Exception as error:
            summary["original_source_unchanged"] = False
            summary["original_inventory_error"] = str(error)
            summary["exit_code"] = 1
        if copied.is_dir():
            copied_after = inventory(copied)
            write_json(output / "copied_after.json", copied_after)
            summary["copied_source_unchanged"] = copied_after == original
            if copied_after != original:
                summary["exit_code"] = 1
        for relative, name in (
            ("CMakeCache.txt", "CMakeCache.txt"),
            ("Testing/Temporary/LastTestsFailed.log", "failed_tests.log"),
            ("Testing/Temporary/LastTest.log", "ctest_last.log"),
            ("generated/autoaim_build_information.json", "build_information.json"),
            ("verification_manifest.json", "verification_manifest.json"),
        ):
            if (build / relative).is_file():
                shutil.copy2(build / relative, output / name)
        summary["commands"] = commands
        write_json(output / "summary.json", summary)
    print(json.dumps(summary, ensure_ascii=False, indent=2), flush=True)
    return summary["exit_code"]


def self_test():
    cases = 0
    source = Path(__file__).resolve().parents[1]
    validate_source(source)
    before = inventory(source)
    for method, selected in METHODS.items():
        flags = selection(method)
        assert len(flags) == 6
        assert [key for key, value in flags.items() if value == "ON"] == ([selected] if selected else [])
        cases += 1
    try:
        selection("unknown")
    except ValueError:
        cases += 1
    else:
        raise RuntimeError("Unknown selection accepted")
    with tempfile.TemporaryDirectory(prefix="autoaim-selection-") as folder:
        root = Path(folder)
        fake = root / "source"
        fake.mkdir()
        (fake / "CMakeLists.txt").write_text("# fixture", encoding="utf-8")
        for method in METHODS:
            for output in (fake, fake / "nested", root):
                try:
                    verify(fake, output, method)
                except ValueError as error:
                    assert "independent" in str(error)
                    cases += 1
                else:
                    raise RuntimeError("Source/output overlap accepted")
        existing = root / "existing"
        existing.mkdir()
        try:
            verify(fake, existing, "DEFAULT")
        except ValueError as error:
            assert "already exists" in str(error)
            cases += 1
        else:
            raise RuntimeError("Existing evidence overwrite accepted")
        for method in OPENVINO_METHODS:
            try:
                model_inputs(method, None, None, None)
            except ValueError:
                cases += 1
            else:
                raise RuntimeError("Missing models accepted")
        try:
            model_inputs("ESO", root, None, None)
        except ValueError:
            cases += 1
        else:
            raise RuntimeError("OFF run accepted unused model inputs")
        # Duplicate/missing selections fail before creating acceptance output.
        original = (source / "CMakeLists.txt").read_text(encoding="utf-8")
        for content in (original + '\noption(AUTOAIM_USE_ESO "duplicate" OFF)\n',
                        original.replace('option(AUTOAIM_USE_ESO "Use experimental ESO instead of EKF" OFF)',
                                         '# removed ESO option')):
            (fake / "CMakeLists.txt").write_text(content, encoding="utf-8")
            try:
                validate_source(fake)
            except ValueError:
                cases += 1
            else:
                raise RuntimeError("Invalid central declaration accepted")
        # Editing a central default to ON is valid; explicit per-run flags override it.
        (fake / "CMakeLists.txt").write_text(original.replace(
            'option(AUTOAIM_USE_ESO "Use experimental ESO instead of EKF" OFF)',
            'option(AUTOAIM_USE_ESO "Use experimental ESO instead of EKF" ON)'), encoding="utf-8")
        for name, folder, _ in SPECIAL_TESTS.values():
            test = fake / "tests" / folder / (name + ".cpp")
            test.parent.mkdir(parents=True, exist_ok=True)
            test.write_text("// fixture", encoding="utf-8")
        validate_source(fake)
        cases += 1
    assert inventory(source) == before
    print(f"Alternative verifier self-test passed ({cases} cases); source unchanged")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--source", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--output", type=Path)
    parser.add_argument("--method", choices=tuple(METHODS))
    parser.add_argument("--jobs", type=int, default=2)
    parser.add_argument("--build-type", choices=("Debug", "Release"), default="Debug")
    parser.add_argument("--cmake", default="cmake")
    parser.add_argument("--ctest", default="ctest")
    parser.add_argument("--yolov5-model", type=Path)
    parser.add_argument("--yolo11-model", type=Path)
    parser.add_argument("--openvino-dir", type=Path)
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return 0
    if args.method is None or args.output is None:
        parser.error("--method and --output are required unless --self-test is used")
    if args.jobs < 1:
        parser.error("--jobs must be positive")
    try:
        return verify(args.source, args.output, args.method, args.jobs, args.cmake, args.ctest,
                      args.yolov5_model, args.yolo11_model, args.openvino_dir, args.build_type)
    except (OSError, ValueError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    sys.exit(main())
