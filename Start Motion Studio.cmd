@echo off
setlocal
 title Hexapod - Motion Studio
pushd "%~dp0webapp"
if errorlevel 1 goto folder_error
py -3 -c "import sys; sys.exit(sys.version_info < (3, 8))" >nul 2>&1
if not errorlevel 1 goto use_py
python -c "import sys; sys.exit(sys.version_info < (3, 8))" >nul 2>&1
if not errorlevel 1 goto use_python
echo Python 3.8 or newer was not found.
echo Install Python 3 from https://www.python.org/downloads/windows/
echo Enable Add Python to PATH during installation, then reopen this file.
popd
pause
exit /b 1
:use_py
py -3 serve.py
goto finished
:use_python
python serve.py
:finished
set "calibration_exit=%errorlevel%"
popd
if not "%calibration_exit%"=="0" pause
exit /b %calibration_exit%
:folder_error
echo Could not find the webapp folder.
echo Extract the complete ZIP before starting the app.
pause
exit /b 1
