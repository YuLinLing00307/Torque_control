@echo off
if "%1" == "h" goto begin
	mshta vbscript:createobject("wscript.shell").run("""%~nx0"" h",0)(window.close)&&exit
:begin
cd /d %~dp0
set TEMP=%CD%
set PATHDIR=%TEMP%\temp\update_temp
robocopy  "%PATHDIR%"   "%CD%"  *  /mt /np /move /e /z /r:1 /w:1
start MotorHost.exe