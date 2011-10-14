@echo off
if %1!==! goto message
for %%i in ("%1") do set fwname=%%~di%%~pi%%~ni
avr32-objcopy -O binary %1 "%fwname%.bin"
..\bin2dvfw.exe -d R -f "%fwname%.bin" -n "%fwname%"
goto ende
:message
echo Call "makedvfw.bat <name of elf file>"
:ende
