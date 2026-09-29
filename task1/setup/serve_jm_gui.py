#!/usr/bin/env python3
"""Local-only HTTP bridge to the actual Isaac Sim GUI; access over SSH.

Uses Kit's own swapchain capture and input provider. No WebRTC, X server,
third-party web assets, arbitrary file serving, or remote command endpoint.
"""
import argparse
import ctypes
import io
import json
from pathlib import Path
import mimetypes
import queue
import secrets
import signal
import threading
import time
import traceback
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit

parser = argparse.ArgumentParser()
parser.add_argument("--world", type=Path, required=True)
parser.add_argument("--output", type=Path, required=True)
parser.add_argument("--gpu", type=int, default=0)
parser.add_argument("--port", type=int, default=8765)
args, _ = parser.parse_known_args()
args.output.mkdir(parents=True, exist_ok=True)
task1_results = Path(__file__).resolve().parent.parent / "outputs" / "task1_results"
token = secrets.token_urlsafe(32)
html = Path(__file__).with_name("jm_gui.html").read_text().replace("__TOKEN__", token).encode()
events = queue.Queue(maxsize=1024)
state = {"ready": False, "frames": 0, "inputs": 0, "camera": "factory_aisle", "error": None}
frame = b""
last_request = 0.0
stop = threading.Event()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *unused):
        pass

    def allowed(self):
        return self.headers.get("Host") in (f"127.0.0.1:{args.port}", f"localhost:{args.port}")

    def reply(self, code, data, content_type):
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("X-Frame-Options", "DENY")
        self.end_headers()
        try:
            self.wfile.write(data)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def do_GET(self):
        global last_request
        if not self.allowed():
            return self.reply(403, b"Use localhost via SSH", "text/plain")
        path = urlsplit(self.path).path
        if path == "/":
            self.reply(200, html, "text/html; charset=utf-8")
        elif path in ("/task1", "/task1/") or path.startswith("/task1/"):
            relative = "index.html" if path in ("/task1", "/task1/") else path[len("/task1/"):]
            target = (task1_results / relative).resolve()
            if not target.is_relative_to(task1_results.resolve()) or not target.is_file():
                return self.reply(404, b"Not found", "text/plain")
            if target.suffix not in (".html", ".json", ".png", ".jpg", ".npz", ".txt", ".obj", ".mtl"):
                return self.reply(404, b"Not found", "text/plain")
            mime = mimetypes.guess_type(str(target))[0] or "application/octet-stream"
            if target.suffix == ".html":
                mime = "text/html; charset=utf-8"
            self.reply(200, target.read_bytes(), mime)
        elif path == "/frame.jpg":
            last_request = time.monotonic()
            snapshot = frame
            self.reply(200 if snapshot else 503, snapshot, "image/jpeg")
        elif path == "/status":
            self.reply(200, json.dumps(state).encode(), "application/json")
        else:
            self.reply(404, b"Not found", "text/plain")

    def do_POST(self):
        global last_request
        if not self.allowed() or not secrets.compare_digest(self.headers.get("X-Factor-Token", ""), token):
            return self.reply(403, b"Forbidden", "text/plain")
        if self.path != "/input":
            return self.reply(404, b"Not found", "text/plain")
        try:
            size = int(self.headers.get("Content-Length", "0"))
            if not 0 < size <= 32768:
                raise ValueError("Invalid request size")
            items = json.loads(self.rfile.read(size))
            if not isinstance(items, list) or len(items) > 128 or any(not isinstance(e, dict) for e in items):
                raise ValueError("Invalid events")
            for item in items:
                events.put_nowait(item)
        except (ValueError, queue.Full):
            return self.reply(400, b"Invalid input or busy", "text/plain")
        last_request = time.monotonic()
        self.reply(200, b"{}", "application/json")


# Binding first prevents a second instance from starting an unnecessary GPU app.
server = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
threading.Thread(target=server.serve_forever, daemon=True).start()
for sig in (signal.SIGINT, signal.SIGTERM):
    signal.signal(sig, lambda *_: stop.set())

from isaacsim import SimulationApp

app = SimulationApp({
    "headless": True, "hide_ui": False,
    "width": 1280, "height": 720, "window_width": 1600, "window_height": 900,
    "active_gpu": args.gpu, "physics_gpu": args.gpu, "multi_gpu": False,
    "renderer": "RaytracedLighting", "sync_loads": True,
})

try:
    import carb
    import carb.input
    import omni.appwindow
    import omni.usd
    import omni.kit.renderer_capture
    from omni.kit.viewport.utility import get_active_viewport
    from PIL import Image
    from pxr import Gf, Usd, UsdGeom

    carb.settings.get_settings().set("/rtx/post/histogram/enabled", False)
    context = omni.usd.get_context()
    if not context.open_stage(str(args.world.resolve())):
        raise RuntimeError("Could not open JM world")
    for _ in range(40):
        app.update()
    stage = context.get_stage()
    stage.SetEditTarget(stage.GetSessionLayer())
    # Optional visualization only; source factory layers and H1 articulation are unchanged.
    task1_preview = task1_results / "grasp_preview.usda"
    if task1_preview.is_file():
        stage.GetSessionLayer().subLayerPaths.append(str(task1_preview))
        UsdGeom.Imageable(stage.GetPrimAtPath("/World/Task1GraspCandidates")).MakeInvisible()
    viewport = get_active_viewport()
    if viewport is None:
        raise RuntimeError("No active GUI viewport")
    camera = UsdGeom.Camera.Define(stage, "/World/RemoteViewCamera")
    camera.CreateClippingRangeAttr(Gf.Vec2f(0.05, 1000.0))
    camera.CreateHorizontalApertureAttr(20.955)
    camera.CreateVerticalApertureAttr(20.955 * 720 / 1280)
    transform = UsdGeom.Xformable(camera).AddTransformOp()

    def set_view(name):
        if task1_preview.is_file():
            preview_root = UsdGeom.Imageable(stage.GetPrimAtPath("/World/Task1GraspCandidates"))
            if name == "task1_pcb":
                preview_root.MakeVisible()
            else:
                preview_root.MakeInvisible()
        if name in ("factory_top", "factory_aisle"):
            source = UsdGeom.Camera(stage.GetPrimAtPath("/World/FloorCam" if name == "factory_top" else "/World/RenderCam"))
            matrix = source.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            focal = source.GetFocalLengthAttr().Get()
        elif name == "h1_close":
            robot = stage.GetPrimAtPath("/World/Robots/H1_1")
            cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
            bounds = cache.ComputeWorldBound(robot).ComputeAlignedRange()
            center = (bounds.GetMin() + bounds.GetMax()) * 0.5
            matrix = Gf.Matrix4d().SetLookAt(center + Gf.Vec3d(-2.5, -3.0, 1.4), center, Gf.Vec3d(0, 0, 1)).GetInverse()
            focal = 24.0
        elif name == "task1_pcb":
            target = stage.GetPrimAtPath("/World/Dadong/F2/PCBStacks/wet_recv/pcb7")
            center = UsdGeom.Xformable(target).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
            matrix = Gf.Matrix4d().SetLookAt(center + Gf.Vec3d(-0.8, -0.9, 0.75), center, Gf.Vec3d(0, 0, 1)).GetInverse()
            focal = 21.0
        else:
            return
        transform.Set(matrix)
        camera.CreateFocalLengthAttr(focal)
        viewport.camera_path = str(camera.GetPath())
        state["camera"] = name

    set_view("factory_aisle")
    window = omni.appwindow.get_default_app_window()
    mouse, keyboard = window.get_mouse(), window.get_keyboard()
    provider = carb.input.acquire_input_provider()
    capture = omni.kit.renderer_capture.acquire_renderer_capture_interface()
    capsule_pointer = ctypes.pythonapi.PyCapsule_GetPointer
    capsule_pointer.restype = ctypes.c_void_p
    capsule_pointer.argtypes = (ctypes.py_object, ctypes.c_char_p)
    capturing = False
    capture_modes = {}

    def on_capture(buffer, size, width, height, fmt):
        global frame, capturing
        try:
            raw = ctypes.string_at(capsule_pointer(buffer, None), size)
            if size != width * height * 4:
                raise RuntimeError(f"Unexpected capture format: {fmt}, {size}, {width}, {height}")
            # Kit's callback format label need not describe the byte order after readback.
            # Match its own conversion helper once per format, then retain the fast PIL path.
            key = str(fmt)
            if key not in capture_modes:
                reference = bytes(omni.kit.renderer_capture.convert_raw_bytes_to_list(buffer, size, width, height, fmt))
                if reference == raw:
                    capture_modes[key] = "RGBA"
                elif Image.frombytes("RGBA", (width, height), raw, "raw", "BGRA").tobytes() == reference:
                    capture_modes[key] = "BGRA"
                else:
                    raise RuntimeError(f"Unsupported readback byte order: {fmt}")
                state["capture_format"] = key
                state["capture_raw_mode"] = capture_modes[key]
                state["color_conversion_check"] = "matches Kit official RGBA conversion"
            mode = capture_modes[key]
            picture = Image.frombytes("RGBA", (width, height), raw, "raw", mode)
            output = io.BytesIO()
            picture.convert("RGB").save(output, "JPEG", quality=85)
            frame = output.getvalue()
            state.update(ready=True, width=width, height=height, frames=state["frames"] + 1)
            if state["frames"] == 1:
                (args.output / "gui_ready.jpg").write_bytes(frame)
                (args.output / "ready.json").write_text(json.dumps({**state, "url": f"http://127.0.0.1:{args.port}", "world": str(args.world)}, indent=2))
                print(f"FACTOR_GUI_READY http://127.0.0.1:{args.port}", flush=True)
        except Exception as exc:
            state["error"] = str(exc)
            print(f"Capture error: {exc}", flush=True)
        finally:
            capturing = False

    pressed_keys = set()
    pressed_buttons = set()
    position = (0.0, 0.0)

    def mouse_event(kind, x, y, mods=0):
        provider.buffer_mouse_event(mouse, kind, (x / 1600, y / 900), mods, (x, y))

    def release_all():
        for key in pressed_keys:
            provider.buffer_keyboard_key_event(keyboard, carb.input.KeyboardEventType.KEY_RELEASE, key, 0)
        for button in pressed_buttons:
            mouse_event(getattr(carb.input.MouseEventType, button + "_UP"), *position)
        pressed_keys.clear()
        pressed_buttons.clear()

    def handle(event):
        global position
        kind = event.get("kind")
        mods = int(event.get("mods", 0)) & 15
        if kind == "view":
            set_view(event.get("name"))
        elif kind == "release":
            release_all()
        elif kind == "mouse":
            x = min(1599.0, max(0.0, float(event["x"])))
            y = min(899.0, max(0.0, float(event["y"])))
            position = (x, y)
            mouse_event(carb.input.MouseEventType.MOVE, x, y, mods)
            action = event.get("action", "move")
            if action in ("down", "up"):
                button = ["LEFT_BUTTON", "MIDDLE_BUTTON", "RIGHT_BUTTON"][int(event["button"])]
                mouse_event(getattr(carb.input.MouseEventType, button + "_" + action.upper()), x, y, mods)
                if action == "down":
                    pressed_buttons.add(button)
                else:
                    pressed_buttons.discard(button)
        elif kind == "wheel":
            delta = min(5.0, max(-5.0, float(event["delta"])))
            provider.buffer_mouse_event(mouse, carb.input.MouseEventType.SCROLL, (0, delta), mods, (0, delta))
        elif kind == "key":
            key = getattr(carb.input.KeyboardInput, str(event["code"]), None)
            if key is None:
                return
            down = event.get("down") is True
            event_type = carb.input.KeyboardEventType.KEY_PRESS if down else carb.input.KeyboardEventType.KEY_RELEASE
            provider.buffer_keyboard_key_event(keyboard, event_type, key, mods)
            if down:
                pressed_keys.add(key)
                char = event.get("char", "")
                if isinstance(char, str) and len(char) == 1 and not mods & 14:
                    provider.buffer_keyboard_char_event(keyboard, char, mods)
            else:
                pressed_keys.discard(key)
        state["inputs"] += 1

    for _ in range(60):
        app.update()
    last_capture = 0.0
    while app.is_running() and not stop.is_set():
        tick = time.monotonic()
        # One transition per Kit update preserves a down/up pair queued together.
        if not events.empty():
            try:
                handle(events.get_nowait())
            except (KeyError, ValueError, TypeError, IndexError) as exc:
                print(f"Ignored invalid input: {exc}", flush=True)
        active = tick - last_request < 15.0
        if not active and (pressed_keys or pressed_buttons):
            release_all()
        if not capturing and tick - last_capture >= (0.12 if active else 2.0):
            capturing = True
            capture.capture_next_frame_swapchain_callback(on_capture, window)
            last_capture = tick
        app.update()
        time.sleep(max(0.0, (1 / 30 if active else 0.5) - (time.monotonic() - tick)))
except Exception:
    traceback.print_exc()
    raise
finally:
    stop.set()
    server.shutdown()
    server.server_close()
    app.close()
