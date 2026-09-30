"""Minimal MJPEG-over-HTTP server: two endpoints, one per camera, each an
independent multipart/x-mixed-replace stream that any number of browsers or
VLC ("Open Network Stream") can connect to at once.

Deliberately plain stdlib http.server rather than a framework - the whole
project has one external dependency short of it (opencv/numpy/pyusb) and a
long-lived multipart response doesn't need routing, templating, etc.
"""

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from frame_bus import FrameBus

BOUNDARY = "FRAME"

_INDEX_HTML = """<!doctype html>
<html>
<head><title>Raspi4 P1 + Pi Camera V2 - live streams</title></head>
<body style="background:#111;color:#eee;font-family:sans-serif">
<h1>Live streams</h1>
<div style="display:flex;gap:16px;flex-wrap:wrap">
  <div><h3>Thermal (P1)</h3><img src="/thermal.mjpg" style="max-width:100%"></div>
  <div><h3>RGB (Camera Module 2)</h3><img src="/rgb.mjpg" style="max-width:100%"></div>
</div>
</body>
</html>
"""


def make_handler(thermal_bus: FrameBus, rgb_bus: FrameBus):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass  # the default per-request access log is just noise for a live video stream

        def do_GET(self):
            if self.path == "/":
                body = _INDEX_HTML.encode()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif self.path == "/thermal.mjpg":
                self._stream(thermal_bus, "thermal")
            elif self.path == "/rgb.mjpg":
                self._stream(rgb_bus, "rgb")
            else:
                self.send_error(404)

        def _stream(self, bus: FrameBus, name: str):
            self.send_response(200)
            self.send_header("Age", "0")
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header(
                "Content-Type", f"multipart/x-mixed-replace; boundary={BOUNDARY}"
            )
            self.end_headers()
            last_id = -1
            try:
                while True:
                    jpg, last_id = bus.get_latest(last_id)
                    if jpg is None:
                        continue  # source not producing frames yet; keep waiting
                    self.wfile.write(f"--{BOUNDARY}\r\n".encode())
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", str(len(jpg)))
                    self.end_headers()
                    self.wfile.write(jpg)
                    self.wfile.write(b"\r\n")
            except OSError:
                # Any socket-level failure means the same thing here: the client is
                # gone, nothing more to send. BrokenPipeError/ConnectionResetError/
                # TimeoutError are the common cases, but a flaky link (e.g. the
                # client's interface briefly losing its route) can also surface as
                # a plain OSError (errno ENETUNREACH) that isn't one of those named
                # subclasses - catch OSError itself so it doesn't print an unhandled
                # traceback for what is, from here, an ordinary disconnect.
                pass
            finally:
                print(f"[http] client {self.client_address[0]} disconnected from /{name}")

    return Handler


def serve(host: str, port: int, thermal_bus: FrameBus, rgb_bus: FrameBus) -> ThreadingHTTPServer:
    handler = make_handler(thermal_bus, rgb_bus)
    httpd = ThreadingHTTPServer((host, port), handler)
    httpd.daemon_threads = True
    return httpd
