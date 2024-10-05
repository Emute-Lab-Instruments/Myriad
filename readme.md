Sources for Emute Lab Instruments Myriad eurorack module

### Architecture

There are two ELI2040 modules, linked on SPI1.  Module A (on the left when the module is facing down) handles IO and display on core 0 and synthesis on core 1. Module B handles synthesis on both cores.


### Developer Notes

This compiles using Arduino enviroment, with the Earle Philhower Arduino Pico Core.

There are two sketches, Myriad_A and Myriad_B

There are shared files between the two sketches.  In the Arduino system, these files need to be in the libraries folder, so create a symbolic link from the Arduino/libraries folder to ```arduino_libmyriad``` in this folder.

On unix: 

```
ln -s [path to this repo]/arduino_libmyriad/ 
```