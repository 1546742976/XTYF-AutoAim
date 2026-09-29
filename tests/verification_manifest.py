"""Offline acceptance inventory, separate from production run provenance. No tests are executed."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile


def digest(data):
    return hashlib.sha256(data).hexdigest()


def test_inventory(source):
    return {path.relative_to(source).as_posix(): digest(path.read_bytes())
            for path in sorted((source / "tests").rglob("*"))
            if path.is_file() and "__pycache__" not in path.parts}


def manifest(source, build_information, registered, cache, configuration):
    inventory = build_information["source_inventory"]
    if digest(inventory.encode()) != build_information["source_sha256"]:
        raise ValueError("Production source inventory digest mismatch")
    for record in inventory.splitlines():
        name, expected = record.rsplit(":", 1)
        if digest((source / name).read_bytes()) != expected:
            raise ValueError("Build is stale; rebuild before acceptance: " + name)
    files = test_inventory(source)
    encoded = json.dumps(files, sort_keys=True, separators=(",", ":")).encode()
    return {
        "schema_version": 1,
        "scope": "registration and content inventory; not a test-pass assertion",
        "configuration": configuration,
        "production_build": build_information,
        "test_files_sha256": digest(encoded),
        "test_files": files,
        "ctest_registered_count": len(registered["tests"]),
        "ctest_registry": registered,
        "cmake_cache": cache,
    }


def read_cache(path):
    result = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line or line.startswith(("#", "//")) or "=" not in line:
            continue
        key, value = line.split("=", 1)
        result[key] = value
    return result


def self_test():
    # 只在临时目录建立最小材料；测试修改应改变测试标识而不改变生产标识。
    with tempfile.TemporaryDirectory(prefix="autoaim-acceptance-") as folder:
        source = Path(folder)
        production = source / "a.cpp"
        production.write_bytes(b"production\n")
        tests = source / "tests/fixtures"
        tests.mkdir(parents=True)
        fixture = tests / "frame.bin"
        fixture.write_bytes(b"image bytes")
        inventory = "a.cpp:" + digest(production.read_bytes()) + "\n"
        build = {"source_inventory": inventory, "source_sha256": digest(inventory.encode())}
        registry = {"tests": [{"name": "offline", "command": ["test_offline"]}]}
        first = manifest(source, build, registry, {"CMAKE_CXX_STANDARD:STRING": "17"}, "Debug")
        second = manifest(source, build, registry, first["cmake_cache"], "Debug")
        if first != second or first["ctest_registered_count"] != 1:
            raise RuntimeError("Manifest is not reproducible or registration is missing")
        fixture.write_bytes(b"modified image bytes")
        changed = manifest(source, build, registry, first["cmake_cache"], "Debug")
        if changed["test_files_sha256"] == first["test_files_sha256"]:
            raise RuntimeError("Fixture content change was not recorded")
        if changed["production_build"] != first["production_build"]:
            raise RuntimeError("Test fingerprint polluted production fingerprint")
        production.write_bytes(b"new source\n")
        try:
            manifest(source, build, registry, {}, "Debug")
        except ValueError as error:
            if "stale" not in str(error):
                raise
        else:
            raise RuntimeError("Stale production build accepted")
    print("Acceptance manifest self-test passed")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--source", type=Path)
    parser.add_argument("--build", type=Path)
    parser.add_argument("--ctest", default="ctest")
    parser.add_argument("--configuration", default="")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return
    if not args.source or not args.build or not args.output:
        parser.error("--source, --build and --output are required")
    source, build = args.source.resolve(), args.build.resolve()
    information = json.loads((build / "generated/autoaim_build_information.json").read_text())
    command = [args.ctest, "--test-dir", str(build), "--show-only=json-v1"]
    if args.configuration:
        command += ["-C", args.configuration]
    registered = json.loads(subprocess.check_output(command, text=True))
    result = manifest(source, information, registered, read_cache(build / "CMakeCache.txt"),
                      args.configuration)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n",
                           encoding="utf-8")
    print(f"Saved {result['ctest_registered_count']} registered tests to {args.output}")


if __name__ == "__main__":
    main()
