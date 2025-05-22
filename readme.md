Sources for Emute Lab Instruments Myriad eurorack module

### Architecture

There are two ELI2040 modules, linked by serial.  Module A (on the left when the module is facing down) handles IO and display on core 0 and synthesis on core 1. Module B handles synthesis on both cores.


### Developer Notes

This compiles using Arduino enviroment, with the Earle Philhower Arduino Pico Core.

There are two sketches, Myriad_A and Myriad_B

There are shared files between the two sketches.  In the Arduino system, these files need to be in the libraries folder, so create a symbolic link from the Arduino/libraries folder to ```arduino_libmyriad``` in this folder.

On unix: 

```
ln -s [path to this repo]/arduino_libmyriad/ 
```

You need to install the TFT_eSPI library, and configure to use the following driver in Arduino/libraries/TFT_eSPT/User_Setup_Select.h by making this edit:

```
//#include <User_Setup.h>           // Default setup is root library folder

#include <User_Setups/Setup303_RP2040_GC9A01.h> //for myriad round TFT

```
