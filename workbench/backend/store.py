"""Configuration packages and durable workspace records.

The source and registered input directories are always read-only. Every mutable
record is placed under the independent workspace and replaced atomically.
"""

import copy
import json
import math
import os
from pathlib import Path
import re
import threading
from datetime import datetime, timezone
import uuid

import yaml


FLAGS = (
    "AUTOAIM_I1_CONTRAST_IRLS", "AUTOAIM_I2_LM", "AUTOAIM_I3_LINEAR_CA",
    "AUTOAIM_I9_THROUGHPUT", "AUTOAIM_I9_PREALLOC", "AUTOAIM_USE_ESO",
)
DETECTORS = ("traditional", "yolov5", "yolo11")


def now():
    return datetime.now(timezone.utc).isoformat()


def inside(path, root):
    return path == root or root in path.parents


def read_json(path, default=None):
    if not path.exists():
        return default
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path, value):
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False),
                         encoding="utf-8")
    os.replace(temporary, path)


def read_yaml(path):
    try:
        value = yaml.load(path.read_text(encoding="utf-8"), Loader=UniqueSafeLoader)
    except yaml.YAMLError as error:
        raise ValueError("Invalid YAML in " + str(path) + ": " + str(error)) from error
    if not isinstance(value, dict):
        raise ValueError("YAML document must be a mapping: " + str(path))
    return value


class UniqueSafeLoader(yaml.SafeLoader):
    """Preserve the C++ loader's rejection of duplicate mapping keys."""

    def construct_mapping(self, node, deep=False):
        self.flatten_mapping(node)
        keys = set()
        for key_node, _ in node.value:
            key = self.construct_object(key_node, deep=True)
            if key in keys:
                raise yaml.constructor.ConstructorError("mapping", node.start_mark,
                                                        "Duplicate YAML key: " + str(key), key_node.start_mark)
            keys.add(key)
        return super().construct_mapping(node, deep=deep)


def write_yaml(path, value):
    path.write_text(yaml.safe_dump(value, allow_unicode=True, sort_keys=False), encoding="utf-8")


def leaves(value, prefix=""):
    for key, item in value.items():
        path = prefix + "." + key if prefix else key
        if isinstance(item, dict):
            yield from leaves(item, path)
        else:
            yield path, item


def value_type(value):
    if isinstance(value, bool):
        return "boolean"
    if isinstance(value, int):
        return "integer"
    if isinstance(value, float):
        return "number"
    if isinstance(value, list):
        return "array"
    return "string"


def template_fields(text, values):
    """Use YAML marks, rather than indentation guesses, to bind leaf comments."""
    lines = text.splitlines()
    document = yaml.compose(text)
    descriptions = {}

    def walk(node, prefix=""):
        if not isinstance(node, yaml.MappingNode):
            return
        for key, item in node.value:
            path = prefix + "." + key.value if prefix else key.value
            if isinstance(item, yaml.MappingNode):
                walk(item, path)
                continue
            row = item.start_mark.line
            before = []
            cursor = row - 1
            while cursor >= 0 and lines[cursor].lstrip().startswith("#"):
                before.insert(0, lines[cursor].lstrip()[1:].strip())
                cursor -= 1
            # YAML source marks end before inline comments, including flow arrays.
            tail = lines[item.end_mark.line][item.end_mark.column:]
            comment = tail.split("#", 1)[1].strip() if "#" in tail else ""
            descriptions[path] = " ".join(before + ([comment] if comment else []))

    walk(document)
    fields = []
    for path, value in leaves(values):
        kind = value_type(value)
        description = descriptions.get(path, "")
        if kind in {"integer", "number"}:
            kind = "integer" if path == "fast_choose_version" or "整数" in description else "number"
        fields.append({"path": path, "type": kind,
                       "group": path.rsplit(".", 1)[0] if "." in path else "general",
                       "description": description})
    return fields


class Workspace:
    def __init__(self, source, workspace):
        self.source = Path(source).expanduser().resolve()
        self.root = Path(workspace).expanduser().resolve()
        if inside(self.root, self.source) or inside(self.source, self.root):
            raise ValueError("Workspace must be independent of, and must not contain, source")
        if not (self.source / "config/fast_choose.yaml").is_file():
            raise ValueError("Source must contain config/fast_choose.yaml")
        self.root.mkdir(parents=True, exist_ok=True)
        self.profiles_dir = self.root / "profiles"
        self.jobs_dir = self.root / "jobs"
        self.profiles_dir.mkdir(exist_ok=True)
        self.jobs_dir.mkdir(exist_ok=True)
        self.lock = threading.RLock()
        roots = read_json(self.root / "roots.json", [])
        self.roots = list(dict.fromkeys([str(self.source), str(self.root)] + roots))
        self.template_path = self.source / "config/fast_choose.yaml"
        self.template_text = self.template_path.read_text(encoding="utf-8")
        self.values = read_yaml(self.template_path)
        self.fields = template_fields(self.template_text, self.values)

    def allowed_path(self, supplied, owner=None, *, exists=True, directory=False):
        if not isinstance(supplied, (str, Path)) or not str(supplied).strip():
            raise ValueError("A nonempty server path is required")
        path = Path(supplied).expanduser()
        if not path.is_absolute():
            path = (owner or self.source) / path
        path = path.resolve()
        if not any(inside(path, Path(root)) for root in self.roots):
            raise ValueError("Path is outside registered roots: " + str(path))
        if exists and not (path.is_dir() if directory else path.is_file()):
            raise ValueError("Path is not an existing " + ("directory: " if directory else "file: ") + str(path))
        return path

    def register_root(self, supplied):
        if not isinstance(supplied, (str, Path)) or not str(supplied).strip():
            raise ValueError("Register an existing absolute server directory")
        path = Path(supplied).expanduser()
        if not path.is_absolute() or not path.resolve().is_dir():
            raise ValueError("Register an existing absolute server directory")
        path = str(path.resolve())
        with self.lock:
            if path not in self.roots:
                self.roots.append(path)
                write_json(self.root / "roots.json", self.roots)
        return {"path": path, "roots": self.roots}

    def base_contracts(self, key, supplied):
        if key not in DETECTORS:
            raise ValueError("Unknown detector contract")
        path = self.allowed_path(supplied, self.template_path.parent)
        document = read_yaml(path)
        entries = [{"key": key, "path": str(path), "content": path.read_text(encoding="utf-8")}]
        resources = [("calibration_file", document.get("calibration_file"))]
        resources += [("geometry_files." + str(index), value)
                      for index, value in enumerate(document.get("geometry_files", []))]
        for field, value in resources:
            if value:
                resource = self.allowed_path(value, path.parent)
                entries.append({"key": key + "." + field, "path": str(resource),
                                "content": resource.read_text(encoding="utf-8")})
        return entries

    def contracts(self):
        return [entry for key in DETECTORS
                for entry in self.base_contracts(key, self.values["detectors"][key]["config_file"])]

    def template(self):
        return {"values": copy.deepcopy(self.values), "fields": self.fields, "contracts": self.contracts()}

    def list_profiles(self):
        return sorted((read_json(path) for path in self.profiles_dir.glob("*/profile.json")),
                      key=lambda item: item["created_at"], reverse=True)

    def profile(self, identifier):
        self.validate_id(identifier)
        item = read_json(self.profiles_dir / identifier / "profile.json")
        if item is None:
            raise KeyError("Unknown profile: " + identifier)
        return item

    @staticmethod
    def validate_id(identifier):
        if not re.fullmatch(r"[a-f0-9]{32}", identifier):
            raise KeyError("Unknown identifier")

    def validate_values(self, values):
        if not isinstance(values, dict):
            raise ValueError("values must be a complete mapping")
        provided = dict(leaves(values))
        expected = dict(leaves(self.values))
        if provided.keys() != expected.keys():
            raise ValueError("Configuration leaves differ from template; missing=" +
                             str(sorted(expected.keys() - provided.keys())) + "; extra=" +
                             str(sorted(provided.keys() - expected.keys())))
        field_types = {field["path"]: field["type"] for field in self.fields}
        for path, reference in expected.items():
            value = provided[path]
            kind = field_types[path]
            valid = ((kind == "number" and type(value) in (int, float)) or
                     (kind == "integer" and type(value) is int) or
                     (kind == "boolean" and type(value) is bool) or
                     (kind == "array" and isinstance(value, list)) or
                     (kind == "string" and isinstance(value, str) and bool(value.strip())))
            if not valid:
                raise ValueError(path + " must have type " + kind)
            invalid_numbers = (any(type(number) not in (int, float) or not math.isfinite(number)
                                   for number in value) if isinstance(value, list) else
                               type(value) in (int, float) and not math.isfinite(value))
            if invalid_numbers:
                raise ValueError(path + " must contain finite numbers")
        if values["active_detector"] not in DETECTORS:
            raise ValueError("active_detector must be traditional, yolov5, or yolo11")
        if values["fast_choose_version"] != 1:
            raise ValueError("fast_choose_version must be 1")

    def save_profile(self, name, values, contracts=None):
        if not isinstance(name, str) or not name.strip():
            raise ValueError("Profile name is required")
        self.validate_values(values)
        chosen = {}
        if isinstance(contracts, list):
            chosen = {item["key"]: item["path"] for item in contracts}
        elif isinstance(contracts, dict):
            chosen = {key: value.get("path") if isinstance(value, dict) else value
                      for key, value in contracts.items()}
        elif contracts is not None:
            raise ValueError("contracts must be a list or mapping")
        if any(not re.fullmatch(r"(traditional|yolov5|yolo11)(\.(calibration_file|geometry_files\.\d+))?", key)
               for key in chosen):
            raise ValueError("Unknown base contract key")
        documents = {}
        selected = []
        captured_resources = {}
        for key in DETECTORS:
            supplied = chosen.get(key, values["detectors"][key]["config_file"])
            original = self.allowed_path(supplied, self.template_path.parent)
            document = read_yaml(original)
            if document.get("detector", {}).get("kind") != key:
                raise ValueError("Base contract detector.kind differs from " + key)
            # Resolve selections against their original base before packaging them.
            for field in ("calibration_file", "input_manifest"):
                if field in document:
                    document[field] = str(self.allowed_path(chosen.get(key + "." + field, document[field]), original.parent,
                                                           exists=field != "input_manifest"))
            if "geometry_files" in document:
                document["geometry_files"] = [str(self.allowed_path(chosen.get(
                    key + ".geometry_files." + str(index), path), original.parent))
                    for index, path in enumerate(document["geometry_files"])]
                if any(entry.startswith(key + ".geometry_files.") and
                       int(entry.rsplit(".", 1)[1]) >= len(document["geometry_files"])
                       for entry in chosen):
                    raise ValueError("Selected geometry index is absent from base contract")
            document["fast_choose_file"] = "../fast_choose.yaml"
            documents[key] = document
            selected.append({"key": key, "path": str(original),
                             "content": original.read_text(encoding="utf-8")})
            resources = [("calibration_file", document.get("calibration_file"))]
            resources += [("geometry_files." + str(index), value)
                          for index, value in enumerate(document.get("geometry_files", []))]
            for field, value in resources:
                if value:
                    resource = Path(value)
                    contents = resource.read_bytes()
                    selected.append({"key": key + "." + field, "path": value,
                                     "content": contents.decode("utf-8")})
                    relative = Path("resources") / key / (field.replace(".", "_") + resource.suffix)
                    if field == "calibration_file":
                        calibration_document = read_yaml(resource)
                        calibration = calibration_document.get("calibration", {})
                        report_paths_changed = False
                        for report_field in ("intrinsic_report_file", "extrinsic_report_file"):
                            if report_field in calibration:
                                report = self.allowed_path(calibration[report_field], resource.parent)
                                report_relative = relative.parent / (report_field + report.suffix)
                                captured_resources[report_relative] = report.read_bytes()
                                calibration[report_field] = report_relative.name
                                report_paths_changed = True
                        if report_paths_changed:
                            contents = yaml.safe_dump(calibration_document, allow_unicode=True,
                                                      sort_keys=False).encode("utf-8")
                    captured_resources[relative] = contents
                    packaged_reference = "../" + relative.as_posix()
                    if field == "calibration_file":
                        document[field] = packaged_reference
                    else:
                        document["geometry_files"][int(field.rsplit(".", 1)[1])] = packaged_reference
        packaged = copy.deepcopy(values)
        packaged["common"]["input_manifest"] = str(self.allowed_path(
            packaged["common"]["input_manifest"], self.template_path.parent, exists=False))
        for key in DETECTORS:
            packaged["detectors"][key]["config_file"] = "base/" + key + ".yaml"
            if key != "traditional":
                packaged["detectors"][key]["model_path"] = str(self.allowed_path(
                    packaged["detectors"][key]["model_path"], self.template_path.parent, exists=False))
        identifier = uuid.uuid4().hex
        directory = self.profiles_dir / identifier
        directory.mkdir()
        package = directory / "config"
        (package / "base").mkdir(parents=True)
        for relative, contents in captured_resources.items():
            destination = package / relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_bytes(contents)
        for key, document in documents.items():
            write_yaml(package / "base" / (key + ".yaml"), document)
        write_yaml(package / "fast_choose.yaml", packaged)
        profile = {"id": identifier, "name": name.strip(), "values": copy.deepcopy(values),
                   "contracts": selected, "config_path": str(package / "fast_choose.yaml"),
                   "created_at": now()}
        write_json(directory / "profile.json", profile)
        return profile
