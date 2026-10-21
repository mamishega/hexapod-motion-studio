#!/bin/zsh
# Mac launcher: starts the local Motion Studio server and opens Edge or Chrome.
pause() { read -k1 "?Press any key to close this window."; echo; }
cd -- "$(dirname -- "$0")/webapp" || { echo "Could not find the webapp folder. Extract the complete ZIP first."; pause; exit 1; }
if ! python3 -c 'import sys; sys.exit(sys.version_info < (3, 8))' >/dev/null 2>&1; then
  echo "Python 3.8 or newer was not found."
  echo "Install it from https://www.python.org/downloads/macos/ and open this file again."
  pause; exit 1
fi
python3 serve.py || pause
