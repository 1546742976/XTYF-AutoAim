from contextlib import asynccontextmanager
from pathlib import Path
import platform
import shutil
from urllib.parse import quote

from fastapi import FastAPI, Body, Query
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles

from .jobs import JobQueue
from .results import artifact_file, artifacts, results
from .store import Workspace, FLAGS, read_json, write_json
from .nuc import validate_target


def create_app(source, workspace, *, start_worker=True):
    store = Workspace(source, workspace)
    tasks = JobQueue(store)

    @asynccontextmanager
    async def lifespan(app):
        if start_worker:
            tasks.start()
        try:
            yield
        finally:
            tasks.stop()

    app = FastAPI(title="AutoAim offline workbench", lifespan=lifespan)
    app.state.store = store
    app.state.tasks = tasks

    @app.exception_handler(ValueError)
    async def invalid(request, error):
        return JSONResponse(status_code=422, content={"detail": str(error)})

    @app.exception_handler(KeyError)
    async def absent(request, error):
        return JSONResponse(status_code=404, content={"detail": str(error)})

    @app.get("/api/context")
    def context():
        presets = [{"id": "baseline", "name": "Baseline", "flags": {flag: False for flag in FLAGS},
                    "openvino": False}]
        for flag in FLAGS:
            presets.append({"id": flag, "name": flag.removeprefix("AUTOAIM_"),
                            "flags": {key: key == flag for key in FLAGS},
                            "openvino": flag in {"AUTOAIM_I9_THROUGHPUT", "AUTOAIM_I9_PREALLOC"}})
        return {"source_root": str(store.source), "workspace_root": str(store.root),
                "tools": {name: shutil.which(name) for name in ("python3", "cmake", "ctest")},
                "platform": platform.system(), "flags": list(FLAGS), "presets": presets, "roots": store.roots}

    @app.get("/api/config/template")
    def template():
        return store.template()

    @app.get("/api/config/contracts")
    def contract_selection(path: str, key: str):
        return store.base_contracts(key, path)

    @app.get("/api/profiles")
    def profiles():
        return store.list_profiles()

    @app.post("/api/profiles", status_code=201)
    def save_profile(body: dict = Body(...)):
        return store.save_profile(body.get("name"), body.get("values"), body.get("contracts"))

    @app.get("/api/builds")
    def builds():
        return tasks.builds()

    def nuc_configuration():
        target = read_json(store.root / "nuc_target.json", {})
        return {"target": target,
                "configured": all(target.get(key) for key in
                                  ("host", "user", "project_dir", "build_dir", "device_config", "workspace_dir")),
                "ssh_available": shutil.which("ssh") is not None,
                "live_entry_note": "当前项目入口只支持离线；NUC 需要支持设备配置及 --check-config/--config 的实时程序。"}

    @app.get("/api/nuc")
    def nuc_target():
        return nuc_configuration()

    @app.post("/api/nuc")
    def save_nuc_target(body: dict = Body(...)):
        target = validate_target(body)
        with store.lock:
            write_json(store.root / "nuc_target.json", target)
        return nuc_configuration()

    @app.get("/api/jobs")
    def jobs():
        return tasks.list()

    @app.post("/api/jobs", status_code=201)
    def submit_job(body: dict = Body(...)):
        return tasks.submit(body.get("kind"), body.get("options", {}))

    @app.get("/api/jobs/{identifier}")
    def job(identifier: str):
        return tasks.get(identifier)

    @app.get("/api/jobs/{identifier}/log")
    def log(identifier: str, offset: int = Query(default=0, ge=0)):
        record = tasks.get(identifier)
        path = Path(record["directory"]) / "log.txt"
        with path.open("rb") as stream:
            stream.seek(offset)
            text = stream.read(256 * 1024)
            next_offset = stream.tell()
        return {"text": text.decode("utf-8", errors="replace"), "next_offset": next_offset}

    @app.post("/api/jobs/{identifier}/cancel")
    def cancel(identifier: str):
        return tasks.cancel(identifier)

    @app.get("/api/jobs/{identifier}/artifacts")
    def list_artifacts(identifier: str):
        return artifacts(tasks.get(identifier))

    @app.get("/api/artifacts/{identifier}/{path:path}")
    def download_artifact(identifier: str, path: str):
        return FileResponse(artifact_file(tasks.get(identifier), path))

    @app.get("/api/jobs/{identifier}/results")
    def job_results(identifier: str):
        return results(tasks.get(identifier))

    @app.get("/api/files")
    def files(path: str | None = None):
        directory = store.allowed_path(path or store.source, directory=True)
        entries = []
        for child in sorted(directory.iterdir(), key=lambda item: (not item.is_dir(), item.name.lower())):
            try:
                resolved = store.allowed_path(child, exists=False)
                if not resolved.exists():
                    continue
                item = {"name": child.name, "path": str(resolved), "directory": resolved.is_dir(),
                        "size": None if resolved.is_dir() else resolved.stat().st_size}
                if resolved.suffix.lower() in {".png", ".jpg", ".jpeg", ".webp"} and resolved.is_file():
                    item["url"] = "/api/file?path=" + quote(str(resolved), safe="")
                entries.append(item)
            except (ValueError, OSError):
                continue
        try:
            parent = str(store.allowed_path(directory.parent, directory=True))
        except ValueError:
            parent = None
        return {"path": str(directory), "parent": parent, "entries": entries}

    @app.get("/api/file")
    def preview_file(path: str):
        selected = store.allowed_path(path)
        if selected.suffix.lower() not in {".png", ".jpg", ".jpeg", ".webp"}:
            raise ValueError("File preview supports images only")
        return FileResponse(selected)

    @app.post("/api/roots", status_code=201)
    def register_root(body: dict = Body(...)):
        return store.register_root(body.get("path", ""))

    dist = store.source / "workbench/frontend/dist"
    if dist.is_dir():
        app.mount("/", StaticFiles(directory=dist, html=True), name="frontend")
    else:
        @app.get("/")
        def frontend_not_built():
            return JSONResponse(status_code=503, content={"detail":
                "Build the frontend with npm --prefix workbench/frontend ci and npm --prefix workbench/frontend run build"})

    return app
