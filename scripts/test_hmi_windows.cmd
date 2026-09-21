@echo off
setlocal

for %%I in ("%~dp0..") do set "SAFESTRIDE_ROOT=%%~fI"

set "SAFESTRIDE_PYTHON=%SAFESTRIDE_ROOT%\.venv\Scripts\python.exe"
if not exist "%SAFESTRIDE_PYTHON%" set "SAFESTRIDE_PYTHON=%LOCALAPPDATA%\Programs\Python\Python312\python.exe"
if not exist "%SAFESTRIDE_PYTHON%" (
  echo Python 3.12 was not found. Select the project interpreter or create .venv.
  exit /b 1
)

echo [1/4] ROS-topic samples to 16 HMI registers
pushd "%SAFESTRIDE_ROOT%\src\safestride_bridge"
"%SAFESTRIDE_PYTHON%" -m unittest discover -s test -p test_hmi_scenarios.py -v
set "SAFESTRIDE_RESULT=%ERRORLEVEL%"
popd
if not "%SAFESTRIDE_RESULT%"=="0" exit /b %SAFESTRIDE_RESULT%

echo [2/4] VisualTFT project and register contract
pushd "%SAFESTRIDE_ROOT%"
"%SAFESTRIDE_PYTHON%" -m unittest discover -s test -p test_visualtft_project.py -v
set "SAFESTRIDE_RESULT=%ERRORLEVEL%"
popd
if not "%SAFESTRIDE_RESULT%"=="0" exit /b %SAFESTRIDE_RESULT%

echo [3/4] Production Lua behavior
pushd "%SAFESTRIDE_ROOT%"
"%SAFESTRIDE_PYTHON%" -m unittest discover -s test -p test_display_lua.py -v
set "SAFESTRIDE_RESULT=%ERRORLEVEL%"
popd
if not "%SAFESTRIDE_RESULT%"=="0" exit /b %SAFESTRIDE_RESULT%

echo [4/4] Terrain MCU state and Modbus frame
call "%SAFESTRIDE_ROOT%\scripts\test_firmware_windows.cmd"
if errorlevel 1 exit /b %ERRORLEVEL%

echo SafeStride HMI host tests: OK
exit /b 0
