@echo off
::This file was created automatically by CrossIDE to compile with C51.
C:
cd "\CrossIDE\AVR\Project2\"
"C:\CrossIDE\avr-gcc-15.2.0-x64-windows\avr-gcc-15.2.0-x64-windows\bin\avr-gcc.exe" --use-stdout  "C:\CrossIDE\AVR\Project2\project.c"
if not exist hex2mif.exe goto done
if exist project.ihx hex2mif project.ihx
if exist project.hex hex2mif project.hex
:done
echo done
echo Crosside_Action Set_Hex_File C:\CrossIDE\AVR\Project2\project.hex
