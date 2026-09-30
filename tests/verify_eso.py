"""Select experimental ESO through CMake in an unchanged independent source copy.

Run with the same Python/CMake toolchain used to build the project (for example WSL).
The output directory must not exist and must not overlap the original source tree.
All command logs, failed tests, source inventories and build files are retained.
"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import stat
import subprocess
import sys
import time


SOURCE_DIRS = ("apps", "cmake", "config", "include", "src", "tests", "tools")
SKIP_DIRS = {".git", "__pycache__", ".pytest_cache", "CMakeFiles"}


def digest(data):
    return hashlib.sha256(data).hexdigest()


def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def reject_link(path):
    info = path.lstat()
    if path.is_symlink() or getattr(info, "st_file_attributes", 0) & getattr(
            stat, "FILE_ATTRIBUTE_REPARSE_POINT", 0):
        raise ValueError("Symlinks/reparse points are unsupported: " + str(path))


def checked_path(path):
    # Resolve only after examining every existing ancestor, including Windows junctions.
    absolute = Path(os.path.abspath(path))
    for candidate in (absolute, *absolute.parents):
        if candidate.exists() or candidate.is_symlink():
            reject_link(candidate)
    return absolute.resolve()


def source_files(source):
    result = []
    # Root files include the actual edited CMakeLists.txt, toolchain files and documentation.
    for path in sorted(source.iterdir()):
        if path.is_file() or path.is_symlink():
            reject_link(path)
            result.append(path)
    for name in SOURCE_DIRS:
        directory = source / name
        if not directory.exists():
            continue
        reject_link(directory)
        for parent, directories, files in os.walk(directory, followlinks=False):
            directories[:] = sorted(d for d in directories if d not in SKIP_DIRS)
            for child in directories:
                reject_link(Path(parent) / child)
            for child in sorted(files):
                path = Path(parent) / child
                reject_link(path)
                if path.suffix != ".pyc":
                    result.append(path)
    return sorted(result)


def inventory(source):
    files = {p.relative_to(source).as_posix(): digest(p.read_bytes())
             for p in source_files(source)}
    encoded = json.dumps(files, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return {"sha256": digest(encoded), "files": files}


def run_command(output, records, name, argv):
    log = output / (name + ".log")
    started = time.monotonic()
    print("Running " + name + ": " + subprocess.list2cmdline(argv), flush=True)
    record = {"name": name, "argv": argv, "cwd": str(output), "log": log.name}
    try:
        with log.open("w", encoding="utf-8") as stream:
            process = subprocess.run(argv, cwd=output, stdout=stream,
                                     stderr=subprocess.STDOUT, check=False)
        record["exit_code"] = process.returncode
    except OSError as error:
        log.write_text(str(error) + "\n", encoding="utf-8")
        record["exit_code"] = 127
        record["launch_error"] = str(error)
    record["elapsed_seconds"] = time.monotonic() - started
    records.append(record)
    write_json(output / "commands.json", records)
    print(name + " exit=" + str(record["exit_code"]) + " log=" + str(log), flush=True)
    return record["exit_code"]


def verify(source, output, jobs=2, cmake="cmake", ctest="ctest", build_type="Debug"):
    # Compatibility entry point; activation only changes CMake cache values.
    sys.dont_write_bytecode = True
    from verify_alternatives import verify as verify_alternative
    return verify_alternative(source, output, "ESO", jobs, cmake, ctest, build_type=build_type)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--jobs", type=int, default=2)
    parser.add_argument("--build-type", choices=("Debug", "Release"), default="Debug")
    parser.add_argument("--cmake", default="cmake")
    parser.add_argument("--ctest", default="ctest")
    args = parser.parse_args()
    if args.jobs < 1:
        parser.error("--jobs must be positive")
    try:
        return verify(args.source, args.output, args.jobs, args.cmake, args.ctest, args.build_type)
    except (OSError, ValueError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    sys.exit(main())
