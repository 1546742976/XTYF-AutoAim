"""Install and relocate the Runtime component without Docker or root privileges."""

import argparse
import json
from pathlib import Path
import subprocess
import tempfile


def run(*args):
  result = subprocess.run([str(arg) for arg in args], capture_output=True, text=True)
  if result.returncode:
    raise RuntimeError(f"{args}: exit {result.returncode}\n{result.stdout}\n{result.stderr}")
  return result.stdout


def verify(cmake, build, config):
  applications = {
    "autoaim_node", "offline_replay", "autoaim_infantry", "autoaim_sentry", "autoaim_rune",
    "calibration_tool", "bench_detector", "annotate_session", "synthetic_sim",
    "protocol_tester", "replay_visualizer", "pipeline_metrics",
  }
  with tempfile.TemporaryDirectory(prefix="autoaim-install-") as temporary:
    root = Path(temporary)
    prefix = root / "install"
    run(cmake, "--install", build, "--prefix", prefix, "--component", "Runtime",
        "--config", config)
    relocated = root / "relocated 空格"
    prefix.rename(relocated)
    bindir = relocated / "bin"
    if {path.name for path in bindir.iterdir()} != applications:
      raise AssertionError("Runtime binary set differs from the public application list")
    files = list(relocated.rglob("*"))
    if any(path.suffix in {".cpp", ".hpp", ".a"} for path in files):
      raise AssertionError("Runtime must not install source, public headers or static libraries")
    share = relocated / "share" / "autoaim"
    metadata = json.loads((share / "autoaim_build_information.json").read_text())
    if len(metadata["source_sha256"]) != 64:
      raise AssertionError("Missing production source identity")
    configuration = share / "config" / "fast_choose.yaml"
    if not configuration.is_file():
      raise AssertionError("Missing fast_choose configuration")
    for filename in ("calibration.yaml", "geometry.yaml", "yolov5.yaml", "yolo11.yaml"):
      if not (configuration.parent / "offline" / filename).is_file():
        raise AssertionError(f"Missing configuration: {filename}")
    for application in ("offline_replay", "annotate_session", "bench_detector", "autoaim_rune"):
      run(bindir / application, "--help")
    # Runtime relocation is checked without external models, independently of active_detector.
    configuration = configuration.parent / "offline" / "armor.yaml"
    rune = subprocess.run([str(bindir / "autoaim_rune"), "--config", str(configuration)],
                          capture_output=True, text=True)
    if rune.returncode != 2 or rune.stdout or "Rune mission is not implemented" not in rune.stderr:
      raise AssertionError("Rune placeholder must reject running an armor configuration")
    dataset = root / "synthetic"
    run(bindir / "synthetic_sim", "--config", configuration,
        "--output", dataset, "--frames", "3")
    run(bindir / "offline_replay", "--config", configuration,
        "--input", dataset / "events.yaml", "--output", root / "commands.tsv")
    result = subprocess.run(
      [str(bindir / "autoaim_node"), "--config",
       str(share / "config" / "hardware" / "disabled.yaml")],
      capture_output=True, text=True)
    if result.returncode == 0:
      raise AssertionError("Installation must not enable the hardware entry")
    print("Runtime install, relocation, metadata, replay and hardware rejection passed")


if __name__ == "__main__":
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--cmake", required=True)
  parser.add_argument("--build", type=Path, required=True)
  parser.add_argument("--config", required=True)
  options = parser.parse_args()
  verify(options.cmake, options.build.resolve(), options.config)
