@echo off
setlocal

where cl >nul 2>&1
if not errorlevel 1 goto :compiler_ready
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if not exist "%VSWHERE%" exit /b 2
for /f "usebackq delims=" %%I in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VSROOT=%%I"
if not defined VSROOT exit /b 2
call "%VSROOT%\Common7\Tools\VsDevCmd.bat" -arch=x64 -host_arch=x64 >nul
if errorlevel 1 exit /b %errorlevel%

:compiler_ready

for %%I in ("%~dp0..") do set "ROOT=%%~fI"
set "OUT=%TEMP%\safestride_pdj1_host"
if not exist "%OUT%" mkdir "%OUT%"
pushd "%OUT%"

cl /nologo /utf-8 /std:c++14 /EHsc /W4 /I"%ROOT%\test\arduino_stub" /I"%ROOT%\firmware\safestride_mcu" "%ROOT%\test\firmware_protocol_test.cpp" "%ROOT%\firmware\safestride_mcu\protocol.cpp" /Fe:"%OUT%\protocol_test.exe"
if errorlevel 1 goto :fail
cl /nologo /utf-8 /std:c++14 /EHsc /W4 /I"%ROOT%\test\arduino_stub" /I"%ROOT%\firmware\safestride_mcu" "%ROOT%\test\firmware_pressure_sensor_test.cpp" "%ROOT%\firmware\safestride_mcu\pressure_sensor.cpp" /Fe:"%OUT%\pressure_test.exe"
if errorlevel 1 goto :fail
cl /nologo /utf-8 /std:c++14 /EHsc /W4 /I"%ROOT%\test\arduino_stub" /I"%ROOT%\firmware\safestride_mcu" "%ROOT%\test\firmware_analog_hall_sensor_test.cpp" "%ROOT%\firmware\safestride_mcu\analog_hall_sensor.cpp" "%ROOT%\firmware\safestride_mcu\motor_control.cpp" /Fe:"%OUT%\analog_hall_test.exe"
if errorlevel 1 goto :fail
cl /nologo /utf-8 /std:c++14 /EHsc /W4 /I"%ROOT%\test\arduino_stub" /I"%ROOT%\firmware\safestride_mcu" "%ROOT%\test\firmware_motor_control_test.cpp" "%ROOT%\firmware\safestride_mcu\motor_control.cpp" /Fe:"%OUT%\motor_test.exe"
if errorlevel 1 goto :fail
cl /nologo /utf-8 /std:c++14 /EHsc /W4 /I"%ROOT%\test\arduino_stub" /I"%ROOT%\firmware\safestride_mcu" "%ROOT%\test\firmware_state_machine_test.cpp" "%ROOT%\firmware\safestride_mcu\analog_hall_sensor.cpp" "%ROOT%\firmware\safestride_mcu\motor_control.cpp" "%ROOT%\firmware\safestride_mcu\pressure_sensor.cpp" "%ROOT%\firmware\safestride_mcu\protocol.cpp" /Fe:"%OUT%\state_machine_test.exe"
if errorlevel 1 goto :fail
cl /nologo /utf-8 /std:c++14 /EHsc /W4 /I"%ROOT%\test\arduino_stub" /I"%ROOT%\firmware\terrain_mcu" "%ROOT%\test\firmware_tof10120_test.cpp" "%ROOT%\firmware\terrain_mcu\tof10120_sensor.cpp" /Fe:"%OUT%\tof_test.exe"
if errorlevel 1 goto :fail
cl /nologo /utf-8 /std:c++14 /EHsc /W4 /I"%ROOT%\test\arduino_stub" /I"%ROOT%\firmware\terrain_mcu" "%ROOT%\test\firmware_mpu6050_test.cpp" "%ROOT%\firmware\terrain_mcu\mpu6050_sensor.cpp" /Fe:"%OUT%\mpu_test.exe"
if errorlevel 1 goto :fail
cl /nologo /utf-8 /std:c++14 /EHsc /W4 /I"%ROOT%\test\arduino_stub" /I"%ROOT%\firmware\terrain_mcu" "%ROOT%\test\firmware_terrain_state_test.cpp" "%ROOT%\firmware\terrain_mcu\mpu6050_sensor.cpp" "%ROOT%\firmware\terrain_mcu\tof10120_sensor.cpp" "%ROOT%\firmware\terrain_mcu\protocol.cpp" /Fe:"%OUT%\terrain_state_test.exe"
if errorlevel 1 goto :fail

"%OUT%\protocol_test.exe"
if errorlevel 1 goto :fail
if not errorlevel 0 goto :fail
"%OUT%\pressure_test.exe"
if errorlevel 1 goto :fail
if not errorlevel 0 goto :fail
"%OUT%\analog_hall_test.exe"
if errorlevel 1 goto :fail
if not errorlevel 0 goto :fail
"%OUT%\motor_test.exe"
if errorlevel 1 goto :fail
if not errorlevel 0 goto :fail
"%OUT%\state_machine_test.exe"
if errorlevel 1 goto :fail
if not errorlevel 0 goto :fail
"%OUT%\tof_test.exe"
if errorlevel 1 goto :fail
if not errorlevel 0 goto :fail
"%OUT%\mpu_test.exe"
if errorlevel 1 goto :fail
if not errorlevel 0 goto :fail
"%OUT%\terrain_state_test.exe"
if errorlevel 1 goto :fail
if not errorlevel 0 goto :fail
popd
exit /b 0

:fail
set "RESULT=%errorlevel%"
popd
exit /b %RESULT%
