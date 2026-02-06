import argparse
import asyncio
import os
import pty
import signal
from pathlib import Path
from typing import Optional, Set

from fastapi import FastAPI
from fastapi.responses import FileResponse, Response
from fastapi.staticfiles import StaticFiles
from fastapi.websockets import WebSocket
import uvicorn

app = FastAPI()

# ---- Paths / static serving ----
BASE_DIR = Path(__file__).resolve().parent
VIEWER_HTML = BASE_DIR / "Viewer.html"
ASSETS_DIR = BASE_DIR / "assets"
ROBOT_IMG = BASE_DIR / "robot_image.png"

if ASSETS_DIR.exists():
    app.mount("/assets", StaticFiles(directory=str(ASSETS_DIR)), name="assets")
else:
    print(f"WARNING: assets dir not found at {ASSETS_DIR}")

@app.get("/")
async def index():
    return FileResponse(str(VIEWER_HTML))

@app.get("/robot_image.png")
async def robot_image():
    if ROBOT_IMG.exists() and ROBOT_IMG.is_file():
        return FileResponse(str(ROBOT_IMG))
    return Response(status_code=404)

# ---- WebSocket clients ----
clients: Set[WebSocket] = set()

@app.websocket("/ws")
async def ws(websocket: WebSocket):
    await websocket.accept()
    clients.add(websocket)
    try:
        while True:
            # We don't require inbound messages, but keeping this receive loop
            # prevents some intermediaries from closing idle connections.
            await websocket.receive_text()
    except Exception:
        pass
    finally:
        clients.discard(websocket)

async def broadcast(line: str):
    dead = []
    for ws in clients:
        try:
            await ws.send_text(line)
        except Exception:
            dead.append(ws)
    for ws in dead:
        clients.discard(ws)

# ---- PROS terminal process + PTY plumbing ----
proc_pid: Optional[int] = None
pty_master_fd: Optional[int] = None

_loop: Optional[asyncio.AbstractEventLoop] = None
_pty_buf = b""

@app.on_event("startup")
async def _startup():
    global _loop
    _loop = asyncio.get_running_loop()

def _on_pty_data_ready():
    """
    Called by asyncio when PTY master fd is readable.
    Must NOT block. Reads available bytes, splits into lines, broadcasts.
    """
    global pty_master_fd, _pty_buf, _loop

    if pty_master_fd is None or _loop is None:
        return

    try:
        data = os.read(pty_master_fd, 4096)
    except OSError:
        return

    if not data:
        return

    _pty_buf += data

    # Split on newlines; tolerate CRLF
    while b"\n" in _pty_buf:
        raw, _pty_buf = _pty_buf.split(b"\n", 1)
        line = raw.decode("utf-8", errors="replace").rstrip("\r").strip()
        if line:
            # schedule async broadcast without blocking the reader callback
            _loop.create_task(broadcast(line))

@app.post("/api/start")
async def start():
    global proc_pid, pty_master_fd, _loop, _pty_buf

    if proc_pid is not None:
        return {"ok": True, "status": "already running", "pid": proc_pid}

    if _loop is None:
        return {"ok": False, "status": "event loop not ready"}

    # Create PTY
    master_fd, slave_fd = pty.openpty()
    pty_master_fd = master_fd
    _pty_buf = b""

    pid = os.fork()
    if pid == 0:
        # Child: attach slave PTY to stdin/out/err and exec `pros terminal`
        try:
            os.setsid()
        except Exception:
            pass

        os.dup2(slave_fd, 0)
        os.dup2(slave_fd, 1)
        os.dup2(slave_fd, 2)

        # Close fds we don't need in child
        try:
            os.close(master_fd)
        except Exception:
            pass
        try:
            os.close(slave_fd)
        except Exception:
            pass

        # Exec PROS terminal
        os.execvp("pros", ["pros", "terminal"])

        # os.execvp("python3", ["python3", "-u", "simulate-log.py"]) # Test log

    # Parent
    os.close(slave_fd)
    proc_pid = pid

    # Start watching the PTY master fd for readability
    _loop.add_reader(pty_master_fd, _on_pty_data_ready)

    return {"ok": True, "status": "started", "pid": proc_pid}

@app.post("/api/stop")
async def stop():
    global proc_pid, pty_master_fd, _loop, _pty_buf

    if proc_pid is None:
        return {"ok": True, "status": "not running"}

    # Stop process
    try:
        os.kill(proc_pid, signal.SIGTERM)
    except Exception:
        pass
    proc_pid = None

    # Remove reader + close fd
    if _loop is not None and pty_master_fd is not None:
        try:
            _loop.remove_reader(pty_master_fd)
        except Exception:
            pass

    try:
        if pty_master_fd is not None:
            os.close(pty_master_fd)
    except Exception:
        pass

    pty_master_fd = None
    _pty_buf = b""

    return {"ok": True, "status": "stopped"}

@app.get("/api/status")
async def status():
    return {
        "running": (proc_pid is not None),
        "pid": proc_pid,
        "clients": len(clients),
        "pty_open": (pty_master_fd is not None),
    }

@app.post("/api/kill")
async def api_kill():
    global proc_pid
    if proc_pid is None:
        return {"ok": True, "status": "not running"}

    try:
        os.kill(proc_pid, signal.SIGKILL)
    except Exception:
        pass

    proc_pid = None
    return {"ok": True, "status": "killed"}

@app.websocket("/ws")
async def ws(websocket: WebSocket):
    print("WS: connection attempt")
    await websocket.accept()
    print("WS: accepted")
    ...

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    args = parser.parse_args()
    try:
        uvicorn.run(app, host=args.host, port=args.port, ws="websockets")
    except Exception as e:
        print(f"Exception running uvicorn: {e}")

if __name__ == "__main__":
    main()
