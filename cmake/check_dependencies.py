"""检查项目内包含边界；不将文本检查冒充完整的 C++ 语义分析。"""
import pathlib
import re
import sys
import tempfile

EDGES = {
    "core": set(), "math": {"core"}, "hal": {"core"},
    "vision": {"math"}, "estimation": {"vision"},
    "decision": {"estimation"}, "control": {"hal"},
    "mission": {"decision"}, "pipeline": {"mission", "control", "hal"},
}


def allowed(module):
    result = {module}
    for child in EDGES[module]:
        result |= allowed(child)
    return result


def owner(path, root):
    parts = path.relative_to(root).parts
    if parts[:2] == ("include", "autoaim"):
        return parts[2]
    if parts[0] == "src":
        return parts[1]
    return "pipeline"  # apps 是编排层的薄入口。


def check(root):
    failures = []
    cmake = root / "CMakeLists.txt"
    if cmake.exists():
        content = re.sub(r"#[^\n]*", "", cmake.read_text(encoding="utf-8"))
        for declaration in re.findall(r"autoaim_dependencies\(([^)]+)\)", content):
            source, *dependencies = declaration.split()
            for destination in dependencies:
                if destination not in EDGES.get(source, set()):
                    failures.append(f"{cmake}: forbidden target {source} -> {destination}")
    for directory in ("include/autoaim", "src", "apps"):
        for path in (root / directory).rglob("*"):
            if path.suffix not in (".hpp", ".cpp", ".h"):
                continue
            source = owner(path, root)
            content = re.sub(r"/\*.*?\*/|//[^\n]*", "", path.read_text(encoding="utf-8"), flags=re.S)
            for name in re.findall(r'^\s*#\s*include\s*[<"]([^>"\n]+)[>"]', content, re.M):
                candidates = (path.parent / name, root / "include" / name, root / name)
                target = next((p.resolve() for p in candidates if p.is_file()), None)
                if target is None:
                    if name.startswith("autoaim/"):
                        failures.append(f"{path}: missing {name}")
                    continue
                if root not in target.parents:
                    failures.append(f"{path}: project include escapes root: {name}")
                    continue
                destination = owner(target, root)
                if source not in EDGES or destination not in allowed(source):
                    failures.append(f"{path}: forbidden {source} -> {destination}: {name}")
    return failures


def self_test():
    # 临时负例证明检查器能拒绝逆向依赖和相对路径绕过；不修改项目源码。
    with tempfile.TemporaryDirectory() as tmp:
        root = pathlib.Path(tmp).resolve()
        core = root / "include/autoaim/core/a.hpp"
        control = root / "include/autoaim/control/b.hpp"
        core.parent.mkdir(parents=True)
        control.parent.mkdir(parents=True)
        control.write_text('#include "autoaim/core/a.hpp"', encoding="utf-8")
        core.write_text("", encoding="utf-8")
        if check(root):
            raise RuntimeError("checker rejected permitted edge")
        for name in ("autoaim/control/b.hpp", "../control/b.hpp"):
            core.write_text(f'#include "{name}"', encoding="utf-8")
            if not check(root):
                raise RuntimeError("checker missed forbidden edge")
        core.write_text("", encoding="utf-8")
        cmake = root / "CMakeLists.txt"
        cmake.write_text("autoaim_dependencies(control hal)", encoding="utf-8")
        if check(root):
            raise RuntimeError("checker rejected permitted target edge")
        cmake.write_text("autoaim_dependencies(hal vision)", encoding="utf-8")
        if not check(root):
            raise RuntimeError("checker missed forbidden target edge")


if __name__ == "__main__":
    if sys.argv[1] == "--self-test":
        self_test()
    else:
        errors = check(pathlib.Path(sys.argv[1]).resolve())
        print("\n".join(errors) if errors else "Module include boundaries passed")
        sys.exit(bool(errors))
