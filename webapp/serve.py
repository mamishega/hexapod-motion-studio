"""Local Motion Studio launcher for Windows and macOS."""
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import os
import shutil
import subprocess
import sys
import threading

ROOT = Path(__file__).resolve().parent / 'dist'
URL = 'http://localhost:5180'


def find_edge():
    candidates = []
    for variable in ('ProgramFiles(x86)', 'ProgramFiles', 'LOCALAPPDATA'):
        base = os.environ.get(variable)
        if base:
            candidates.append(Path(base) / 'Microsoft' / 'Edge' / 'Application' / 'msedge.exe')
    for candidate in candidates:
        if candidate.is_file():
            return str(candidate)
    return shutil.which('msedge.exe')


def open_browser():
    if sys.platform == 'win32':
        executable = find_edge()
        if executable:
            try:
                subprocess.Popen([executable, URL])
                return
            except OSError as error:
                print('Could not launch Edge:', error)
        # Use the registered Edge URL handler for installations in other paths.
        try:
            os.startfile('microsoft-edge:' + URL)
            return
        except OSError as error:
            print('Could not open the Edge URL handler:', error)
    if sys.platform == 'darwin':
        for name in ('Microsoft Edge', 'Google Chrome'):
            if any((folder / (name + '.app')).exists() for folder in (Path('/Applications'), Path.home() / 'Applications')):
                subprocess.run(['open', '-a', name, URL], check=False)
                return
    print('Open Microsoft Edge or Chrome and enter:', URL)


class AppHandler(SimpleHTTPRequestHandler):
    """Serve the app with fixed MIME types and no caching.

    On Windows, Python reads MIME types from the registry; if .js is registered
    as text/plain the browser refuses to load ES modules and the page is blank.
    """
    TYPES = {'.js': 'text/javascript', '.mjs': 'text/javascript',
             '.css': 'text/css', '.html': 'text/html', '.json': 'application/json'}

    def guess_type(self, path):
        return self.TYPES.get(os.path.splitext(str(path))[1].lower()) or super().guess_type(path)

    def end_headers(self):
        self.send_header('Cache-Control', 'no-store')
        super().end_headers()


class AppServer(ThreadingHTTPServer):
    # On Windows, SO_REUSEADDR lets a second server bind a port that is already
    # in use, so two copies of the app (or another program) could share 5180.
    allow_reuse_address = sys.platform != 'win32'


def main():
    handler = partial(AppHandler, directory=str(ROOT))
    try:
        server = AppServer(('127.0.0.1', 5180), handler)
    except OSError as error:
        print('Could not start the local app:', error)
        print('Close any previous calibration server, then try again.')
        return 1
    print('Hexapod Motion Studio:', URL)
    print('Keep this window open. Press Ctrl+C to stop.')
    timer = threading.Timer(0.5, open_browser)
    timer.start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        timer.cancel()
        server.server_close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
