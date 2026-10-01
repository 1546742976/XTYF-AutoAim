"""Select CMake alternatives in an unchanged independent copy and run full CTest.

I1-CONTRAST-IRLS, I2-LM and I3-LINEAR-CA use OpenVINO OFF.
I9-THROUGHPUT and I9-PREALLOC use OpenVINO ON with both YOLO XML/BIN pairs.
CMake registers alternative-specific tests only for the selected implementation.
ESO and DEFAULT also use OpenVINO OFF. No source or CMake file is rewritten.
Use --selection-file for a complete six-option combination with explicit OpenVINO.
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
CANDIDATE_OPTIONS = tuple(option for option in METHODS.values() if option is not None)
MODEL_TESTS = {
    "YOLOV5": {"test_openvino_model", "test_openvino_async", "test_pipeline_async"},
    "YOLO11": {"test_openvino_yolo11_model", "test_openvino_yolo11_async",
               "test_pipeline_yolo11_async"},
}


def selection(method):
    if method not in METHODS:
        raise ValueError("Unsupported method: " + str(method))
    # Explicit OFF values prevent a previous cache selection leaking into a run.
    return {option: "ON" if option == METHODS[method] else "OFF"
            for option in METHODS.values() if option is not None}


def reject_duplicate_keys(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("Duplicate JSON key: " + key)
        result[key] = value
    return result


def validate_selection(document):
    if not isinstance(document, dict):
        raise ValueError("Selection must be a JSON object")
    required = {"schema_version", "options", "openvino"}
    if set(document) != required:
        raise ValueError("Selection keys must be exactly schema_version, options, openvino")
    if type(document["schema_version"]) is not int or document["schema_version"] != 1:
        raise ValueError("Selection schema_version must be integer 1")
    options = document["options"]
    if not isinstance(options, dict) or set(options) != set(CANDIDATE_OPTIONS):
        raise ValueError("Selection options must contain all six CMake candidate names and no others")
    if any(type(value) is not bool for value in options.values()):
        raise ValueError("Every selection option must be a JSON boolean")
    if type(document["openvino"]) is not bool:
        raise ValueError("Selection openvino must be a JSON boolean")
    if options["AUTOAIM_I3_LINEAR_CA"] and options["AUTOAIM_USE_ESO"]:
        raise ValueError("AUTOAIM_I3_LINEAR_CA and AUTOAIM_USE_ESO are mutually exclusive")
    if not document["openvino"] and any(options[METHODS[method]] for method in OPENVINO_METHODS):
        raise ValueError("I9-THROUGHPUT and I9-PREALLOC acceptance require OpenVINO ON")
    return {"schema_version": 1, "options": dict(options), "openvino": document["openvino"]}


def load_selection_file(path):
    with checked_path(path).open(encoding="utf-8-sig") as stream:
        return validate_selection(json.load(stream, object_pairs_hook=reject_duplicate_keys))


def resolve_selection(method=None, selection_file=None):
    if (method is None) == (selection_file is None):
        raise ValueError("Select exactly one of --method and --selection-file")
    if selection_file is not None:
        return load_selection_file(selection_file)
    flags = selection(method)
    return {"schema_version": 1,
            "options": {key: value == "ON" for key, value in flags.items()},
            "openvino": method in OPENVINO_METHODS}


def expected_tests(options, models):
    expected = {test[0] for method, test in SPECIAL_TESTS.items() if options[METHODS[method]]}
    for name in models:
        expected.update(MODEL_TESTS[name])
    if set(models) == set(MODEL_TESTS):
        expected.add("test_batch_external_models")
    return expected


def validate_registry(registry, options, models):
    tests = registry["tests"]
    registered = {test["name"] for test in tests}
    expected = expected_tests(options, models)
    special_names = {item[0] for item in SPECIAL_TESTS.values()} | I9_TESTS
    unexpected = (registered & special_names) - expected
    if unexpected:
        raise ValueError("Unexpected active candidate/model tests: " + str(sorted(unexpected)))
    missing = expected - registered
    if missing:
        raise ValueError("Required alternative/model tests are unregistered: " + ", ".join(sorted(missing)))
    for test in tests:
        if test["name"] in expected:
            for property_ in test.get("properties", []):
                if property_["name"] == "DISABLED" and str(property_["value"]).upper() in {
                        "TRUE", "ON", "1", "YES"}:
                    raise ValueError("Required alternative/model test is disabled: " + test["name"])
    return expected


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
    return combination_model_inputs(True, True, yolov5, yolo11, openvino_dir, method)


def combination_model_inputs(openvino, require_models, yolov5, yolo11, openvino_dir,
                             context="Selection"):
    if not openvino:
        if yolov5 or yolo11 or openvino_dir:
            raise ValueError(context + " runs OpenVINO OFF; model/OpenVINO options require ON")
        return {}, None
    models = {}
    for name, supplied in (("YOLOV5", yolov5), ("YOLO11", yolo11)):
        if supplied is None:
            if require_models:
                raise ValueError(context + " requires both --yolov5-model and --yolo11-model")
            continue
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


def verify(source, output, method=None, jobs=2, cmake="cmake", ctest="ctest",
           yolov5=None, yolo11=None, openvino_dir=None, build_type="Debug", selection_file=None):
    chosen = resolve_selection(method, selection_file)
    flags = {key: "ON" if value else "OFF" for key, value in chosen["options"].items()}
    require_models = any(chosen["options"][METHODS[name]] for name in OPENVINO_METHODS)
    if jobs < 1:
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
    if selection_file is None:
        models, openvino_dir = model_inputs(method, yolov5, yolo11, openvino_dir)
    else:
        models, openvino_dir = combination_model_inputs(
            chosen["openvino"], require_models, yolov5, yolo11, openvino_dir)
    validate_source(source)
    original = inventory(source)
    output.mkdir(parents=True, exist_ok=False)
    copied, build = output / "source", output / "build"
    commands = []
    configuration = {
        "AUTOAIM_OPENVINO": "ON" if chosen["openvino"] else "OFF",
        "AUTOAIM_HIKROBOT": "OFF", "CMAKE_CXX_STANDARD": "17",
        "CMAKE_BUILD_TYPE": build_type, "BUILD_TESTING": "ON",
    }
    configuration.update(flags)
    for name, model in models.items():
        configuration["AUTOAIM_TEST_" + name + "_MODEL"] = model["xml"]
    if openvino_dir:
        configuration["OpenVINO_DIR"] = str(openvino_dir)
    summary = {
        "schema_version": 2, "method": method, "selection": chosen,
        "selection_file": str(checked_path(selection_file)) if selection_file is not None else None,
        "openvino": chosen["openvino"],
        "scope": "isolated alternative software acceptance; not NUC performance or hardware evidence",
        "original_source": str(source), "copied_source": str(copied),
        "configuration": configuration, "jobs": jobs, "models": models,
        "original_sha256": original["sha256"], "exit_code": 1,
        "model_acceptance": {
            "required": require_models, "provided": sorted(models),
            "missing": sorted(set(MODEL_TESTS) - set(models)),
            "required_tests": sorted(expected_tests({key: False for key in CANDIDATE_OPTIONS}, models)),
            "status": "not_run" if models else "not_provided",
        },
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
            for option, value in {**flags, "AUTOAIM_OPENVINO": configuration["AUTOAIM_OPENVINO"]}.items():
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
                expected = validate_registry(registry, chosen["options"], models)
                summary["required_tests"] = sorted(expected)
            if status == 0 and registry_status == 0:
                status = run_command(output, commands, "ctest", [
                    ctest, "--test-dir", str(build), "-C", build_type, "--output-on-failure",
                    "--parallel", str(jobs)])
                if models:
                    summary["model_acceptance"]["status"] = "passed" if status == 0 else "failed_or_incomplete"
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
    from contextlib import redirect_stderr, redirect_stdout
    import io
    from unittest.mock import patch
    import verify_alternatives as verifier_module
    import verify_eso

    cases = 0

    def rejected(function, *args, **kwargs):
        nonlocal cases
        try:
            function(*args, **kwargs)
        except ValueError:
            cases += 1
        else:
            raise RuntimeError("Invalid verifier input accepted")

    def document(enabled=(), openvino=False):
        return {"schema_version": 1,
                "options": {name: name in enabled for name in CANDIDATE_OPTIONS},
                "openvino": openvino}

    source = Path(__file__).resolve().parents[1]
    validate_source(source)
    before = inventory(source)
    for method, selected in METHODS.items():
        flags = selection(method)
        assert len(flags) == 6
        assert [key for key, value in flags.items() if value == "ON"] == ([selected] if selected else [])
        chosen = resolve_selection(method)
        assert chosen["options"] == {key: value == "ON" for key, value in flags.items()}
        assert chosen["openvino"] == (method in OPENVINO_METHODS)
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

        # Check every switch combination, independently spelling out the constraints.
        for mask in range(1 << len(CANDIDATE_OPTIONS)):
            enabled = {name for index, name in enumerate(CANDIDATE_OPTIONS) if mask & (1 << index)}
            for openvino in (False, True):
                chosen = document(enabled, openvino)
                invalid = ({"AUTOAIM_I3_LINEAR_CA", "AUTOAIM_USE_ESO"} <= enabled or
                           (not openvino and bool(enabled & {
                               "AUTOAIM_I9_THROUGHPUT", "AUTOAIM_I9_PREALLOC"})))
                if invalid:
                    rejected(validate_selection, chosen)
                else:
                    assert validate_selection(chosen) == chosen
                    cases += 1

        invalid_documents = []
        missing = document()
        del missing["options"]["AUTOAIM_I2_LM"]
        invalid_documents.append(missing)
        unknown = document()
        unknown["options"]["AUTOAIM_UNKNOWN"] = False
        invalid_documents.append(unknown)
        for value in (0, 1, "false", None, []):
            malformed = document()
            malformed["options"]["AUTOAIM_I2_LM"] = value
            invalid_documents.append(malformed)
        for key, value in (("schema_version", 2), ("schema_version", True),
                           ("schema_version", "1"), ("schema_version", 1.0),
                           ("openvino", "ON"), ("openvino", 0), ("options", [])):
            malformed = document()
            malformed[key] = value
            invalid_documents.append(malformed)
        missing = document()
        del missing["openvino"]
        invalid_documents.append(missing)
        unknown = document()
        unknown["extra"] = True
        invalid_documents.extend((unknown, [], None))
        selection_path = root / "selection.json"
        for malformed in invalid_documents:
            write_json(selection_path, malformed)
            rejected(load_selection_file, selection_path)
        serialized = json.dumps(document())
        for malformed in (serialized.replace('"schema_version": 1',
                                              '"schema_version": 1, "schema_version": 1'),
                          serialized.replace('"AUTOAIM_I2_LM": false',
                                              '"AUTOAIM_I2_LM": false, "AUTOAIM_I2_LM": false')):
            selection_path.write_text(malformed, encoding="utf-8")
            rejected(load_selection_file, selection_path)
        rejected(resolve_selection)
        rejected(resolve_selection, "DEFAULT", selection_path)
        write_json(selection_path, document({"AUTOAIM_I1_CONTRAST_IRLS", "AUTOAIM_I2_LM"}))
        assert resolve_selection(selection_file=selection_path) == document({
            "AUTOAIM_I1_CONTRAST_IRLS", "AUTOAIM_I2_LM"})
        cases += 1

        v5, y11 = root / "yolov5.xml", root / "yolo11.xml"
        for xml in (v5, y11):
            xml.write_text("<fixture />", encoding="utf-8")
            xml.with_suffix(".bin").write_bytes(b"model fixture")
        for method in METHODS:
            models, _ = model_inputs(method, v5 if method in OPENVINO_METHODS else None,
                                     y11 if method in OPENVINO_METHODS else None, None)
            assert set(models) == (set(MODEL_TESTS) if method in OPENVINO_METHODS else set())
            cases += 1
        for v5_input, y11_input in ((None, None), (v5, None), (None, y11)):
            rejected(combination_model_inputs, True, True, v5_input, y11_input, None)
        for supplied in ((v5, None, None), (None, y11, None), (None, None, root)):
            rejected(combination_model_inputs, False, False, *supplied)
        for v5_input, y11_input, wanted in ((None, None, set()), (v5, None, {"YOLOV5"}),
                                          (None, y11, {"YOLO11"}), (v5, y11, set(MODEL_TESTS))):
            models, directory = combination_model_inputs(True, False, v5_input, y11_input, root)
            assert set(models) == wanted and directory == root.resolve()
            cases += 1
        rejected(combination_model_inputs, True, False, v5, None, v5)
        orphan = root / "orphan.xml"
        orphan.write_text("<fixture />", encoding="utf-8")
        rejected(combination_model_inputs, True, False, orphan, None, None)

        options = document({"AUTOAIM_I1_CONTRAST_IRLS", "AUTOAIM_I2_LM",
                            "AUTOAIM_I3_LINEAR_CA", "AUTOAIM_I9_PREALLOC"}, True)["options"]
        candidate_tests = {"test_corner_refine_irls", "test_pnp_lm", "test_linear_ca",
                           "test_queue_prealloc"}
        for model_names, model_tests in (({}, set()), ({"YOLOV5": {}}, {
                "test_openvino_model", "test_openvino_async", "test_pipeline_async"}),
                ({"YOLO11": {}}, {"test_openvino_yolo11_model", "test_openvino_yolo11_async",
                                  "test_pipeline_yolo11_async"}),
                ({"YOLOV5": {}, "YOLO11": {}}, I9_TESTS)):
            expected = candidate_tests | model_tests
            registry = {"tests": [{"name": name} for name in sorted(expected | {"test_smoke"})]}
            assert validate_registry(registry, options, model_names) == expected
            cases += 1
            for name in expected:
                rejected(validate_registry, {"tests": [test for test in registry["tests"]
                                                        if test["name"] != name]}, options, model_names)
            rejected(validate_registry, {"tests": registry["tests"] + [{"name": "test_eso"}]},
                     options, model_names)
            rejected(validate_registry, {"tests": registry["tests"] + [{
                "name": "test_pnp_lm", "properties": [{"name": "DISABLED", "value": True}]}]},
                     options, model_names)
            for unwanted in I9_TESTS - model_tests:
                rejected(validate_registry, {"tests": registry["tests"] + [{"name": unwanted}]},
                         options, model_names)

        # Exercise actual copy/cache/registry/summary orchestration without C++ commands.
        def exercise(method=None, selected_document=None, fail_ctest=False, optional_models=()):
            nonlocal cases
            if selected_document is not None:
                write_json(selection_path, selected_document)
                chosen = selected_document
            else:
                chosen = resolve_selection(method)
            output = root / ("evidence-" + str(cases))
            enabled = chosen["options"]
            i9 = enabled["AUTOAIM_I9_THROUGHPUT"] or enabled["AUTOAIM_I9_PREALLOC"]
            supplied_models = set(MODEL_TESTS) if i9 else set(optional_models)
            fixture_tests = {name for option, name in (
                ("AUTOAIM_I1_CONTRAST_IRLS", "test_corner_refine_irls"),
                ("AUTOAIM_I2_LM", "test_pnp_lm"), ("AUTOAIM_I3_LINEAR_CA", "test_linear_ca"),
                ("AUTOAIM_I9_PREALLOC", "test_queue_prealloc"), ("AUTOAIM_USE_ESO", "test_eso"))
                if enabled[option]}
            if "YOLOV5" in supplied_models:
                fixture_tests |= {"test_openvino_model", "test_openvino_async", "test_pipeline_async"}
            if "YOLO11" in supplied_models:
                fixture_tests |= {"test_openvino_yolo11_model", "test_openvino_yolo11_async",
                                  "test_pipeline_yolo11_async"}
            if supplied_models == {"YOLOV5", "YOLO11"}:
                fixture_tests.add("test_batch_external_models")

            def fixture_command(output_dir, records, name, argv):
                if name == "configure":
                    build = Path(argv[argv.index("-B") + 1])
                    build.mkdir()
                    flags = {item[2:].split("=", 1)[0]: item.split("=", 1)[1]
                             for item in argv if item.startswith("-D")}
                    assert all(flags[key] == ("ON" if value else "OFF")
                               for key, value in enabled.items())
                    assert flags["AUTOAIM_OPENVINO"] == ("ON" if chosen["openvino"] else "OFF")
                    assert flags["CMAKE_CXX_STANDARD"] == "17"
                    (build / "CMakeCache.txt").write_text("\n".join(
                        key + ":BOOL=" + value for key, value in flags.items()), encoding="utf-8")
                log = output_dir / (name + ".log")
                if name == "ctest_registry":
                    write_json(log, {"tests": [{"name": item} for item in sorted(fixture_tests)]})
                else:
                    log.write_text("Mock verifier self-test; no C++ execution\n", encoding="utf-8")
                code = 4 if name == "ctest" and fail_ctest else 0
                records.append({"name": name, "argv": argv, "exit_code": code, "log": log.name})
                write_json(output_dir / "commands.json", records)
                return code

            with patch.dict(globals(), {"run_command": fixture_command}), redirect_stdout(io.StringIO()):
                code = verify(fake, output, method, 2, "fixture-cmake", "fixture-ctest",
                              v5 if "YOLOV5" in supplied_models else None,
                              y11 if "YOLO11" in supplied_models else None, build_type="Release",
                              selection_file=selection_path if selected_document is not None else None)
            summary = json.loads((output / "summary.json").read_text(encoding="utf-8"))
            assert code == (4 if fail_ctest else 0), summary
            assert summary["selection"] == chosen
            assert summary["original_source_unchanged"] and summary["copied_source_unchanged"]
            assert summary["copied_source"] == str(output / "source")
            assert summary["required_tests"] == sorted(fixture_tests)
            assert [record["name"] for record in summary["commands"]] == [
                "configure", "build", "ctest_registry", "ctest", "verification_manifest"]
            acceptance = summary["model_acceptance"]
            assert acceptance["provided"] == sorted(supplied_models)
            assert acceptance["missing"] == sorted({"YOLOV5", "YOLO11"} - supplied_models)
            assert acceptance["status"] == ("failed_or_incomplete" if fail_ctest and supplied_models else
                                             "passed" if supplied_models else "not_provided")
            cases += 1

        for method in METHODS:
            exercise(method)
        exercise(selected_document=document({"AUTOAIM_I1_CONTRAST_IRLS", "AUTOAIM_I2_LM"}))
        exercise(selected_document=document(openvino=True))
        for supplied_models in (("YOLOV5",), ("YOLO11",), ("YOLOV5", "YOLO11")):
            exercise(selected_document=document(openvino=True), optional_models=supplied_models)
        exercise(selected_document=document(set(CANDIDATE_OPTIONS) - {"AUTOAIM_USE_ESO"}, True))
        exercise("I9-PREALLOC", fail_ctest=True)
        with patch.object(verifier_module, "verify", return_value=9) as compatible:
            assert verify_eso.verify(fake, root / "compatibility", 3, "cmake-fixture",
                                     "ctest-fixture", "Release") == 9
            compatible.assert_called_once_with(fake, root / "compatibility", "ESO", 3,
                                               "cmake-fixture", "ctest-fixture", build_type="Release")
        cases += 1
        with patch.object(sys, "argv", ["verify_alternatives.py", "--method", "DEFAULT",
                                       "--selection-file", str(selection_path), "--output", str(root)]), \
                redirect_stderr(io.StringIO()):
            try:
                main()
            except SystemExit as error:
                assert error.code == 2
                cases += 1
            else:
                raise RuntimeError("Mutually exclusive CLI selectors accepted")
    assert inventory(source) == before
    print(f"Alternative verifier self-test passed ({cases} cases); source unchanged")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--source", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--output", type=Path)
    chosen = parser.add_mutually_exclusive_group()
    chosen.add_argument("--method", choices=tuple(METHODS))
    chosen.add_argument("--selection-file", type=Path,
                        help="JSON schema 1 with all six boolean options and explicit openvino")
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
    if (args.method is None and args.selection_file is None) or args.output is None:
        parser.error("--method or --selection-file, and --output, are required unless --self-test is used")
    if args.jobs < 1:
        parser.error("--jobs must be positive")
    try:
        return verify(args.source, args.output, args.method, args.jobs, args.cmake, args.ctest,
                      args.yolov5_model, args.yolo11_model, args.openvino_dir, args.build_type,
                      selection_file=args.selection_file)
    except (OSError, ValueError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    sys.exit(main())
