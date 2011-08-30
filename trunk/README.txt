
Standard DV-RPTR Firmware
--------------------------
This project is firmware of the AVR32 DV-RPTR, developed by Jan Alte.

See www.dvrptr.de for more information about DV-RPTR Board and Development.

Copyright (c) 2011 by Jan Alte, DO1FJN

The source and the compiled firmware (binary files) a licensed under GPL
verson 2 except all files in the folder "SOFTWARE_FRAMEWORK" and subfolders
of it. Files in these folders are copyrighted by Atmel Corporation.


This firmware is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License version 2 as
published by the Free Software Foundation.

This firmware is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You can find a copy of the GNU General Public License in the "license" 
folder. If not, see <http://www.gnu.org/licenses/>.


All source files of DV-RPTR firmware are written in UTF-8 charset.

---

I recommended AVR32-Studios 2.7beta (2.6 works also).


Importing source in AVR32 Studio
--------------------------------
Start AVR32Studio.

Select 'File' menue -> 'Import...'.
Select item 'General -> Existing Projects into workspace' and press 'Next'.

If you have checkout the source from sourceforge:
  Select 'Select root directory', disable 'Copy projects into workspace' if your
  copy is already in it.

If you have a zipped archive file:
  Select 'Select archive file'.

Select all files you needed (default = 'Select All') and click on 'Finish'.


AVR Dragon oder JTAG ICE mkII setup
-----------------------------------
Select a JTAG-Adapter from 'AVR32 Targets'.
Click right and select 'Properties'.

Enter correct properties in the Adaptor:
Details     - Device = UC3B0256
Details     - Clock Source = "Crystal connected to OSC0"
Daisy Chain - Enable Daisy Chain = OFF



Creation of a own Firmware (for downloading with bootloader)
------------------------------------------------------------
With a simple tool and a shell script a automatic conversion ELF file to fw file
is possible.

Linux:
Copy 'bin2dvfw' in a directory for binaries (usr/local/bin).
Copy shell script 'makedvfw' in your project directory and change the file
access modes to executable (755).

Windows:
Copy 'bin2dvfw.exe' in a directory and add it to the PATH variable.
 
Type in '[Properties]-[Settings]-[Build Steps]' dialog a call to the script:

Windows:
  "${ProjDirPath}/makedvfw.bat" ${ProjName}_HW10.elf
Linux:
  ${ProjDirPath}/makedvfw ${ProjName}_HW10.elf


How it works:
avr32-objcopy -O binary %1 "%fwname%.bin"
avr32-objcopy copy out a binary image from elf-file (file suffix .bin).

bin2dvfw -d R -f "%fwname%.bin" "%fwname%.fw"
A binary image created with the command above includes a placeholder for a boot-
loader (trampoline). bin2dvfw removes it and append a DFU suffix (DFU v1.1).

removing the placeholder under Linux w/o bin2dvfw: 

  tail -c+16385 DV-RPTR_HW10.bin >DV-RPTR-FW.bin

(this removes 16384 bytes from beginning)


Hints:
'dfu-loader' checks the DFU suffix and accept only correct firmware files.
'dfu-util' (openMoko) can load ANY file as a firmware! Be CAREFULLY.

If you change #defines in the source, it is useful to 'Clean...' the project
before build. Some dependencies are not notice by the AVR32 Studio, changes in
other files are ignored sometimes.



Jan Alte, DO1FJN
