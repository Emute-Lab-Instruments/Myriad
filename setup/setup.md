# Setting Up A New Myriad Module

## Flashing the Firmware

With the unit facing screen down, there are two RP2040 microcontroller boards on the back.  Flash the ```A``` firmware to the unit on the left and the ```B``` firmware to the other unit.

To flash a firmware, hook the unit up to a computer with the USB connection. There's no need for external power.  

If the unit doesn't appear as a drive on your computer then you need to put it in boot select mode, but holding down the ```reset``` button while pressing the ```boot select``` button.  The ```reset`` button is the one closest to the power connector, and ```boot select``` is next to it.

## Test Procedure

### Needed

1. Eurorack power
2. Stereo output module
3. A single modulation source e.g. and LFO
4. 3 patch cables

### Tests

1. *Action*: Power up the unit by connecting a power cable to the power connector on the ```A``` unit. The red wire should be closest to the top of the module. 
2. *Action*: connect the ```CLEAN L``` output to an output module
3. *Action*: Turn up the ```VCA/SHAPE``` knob and set the ```FREQUENCY``` knob to 20%.
    1. Is the screen on?
    2. Can you hear sound?
