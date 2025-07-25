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
4. *Action*: press the callibration button on the pcb on the back of the module.
    1. Can you see the calibration screen?
5. *Action*: turn each of the pink knobs to the extremes.  The first column is the raw value of the knobs, the second is the smoothed value. The third is the minimum observed value, and the forth is the maximum observed value.
    1. Did the values change when you moved the knobs?
6. *Action*: Turn the rotary encoders.
    1. Did you see the values on the 5th row change?  They should show some 1s when moving right, and -1s when moving left
7. *Action*: Press the rotary encoders:
    1. Did the values in the lowest row change when you pressed them?
8. *Action*: press the calibration button again. This will save the calibration data.
9. *Action*: connect the ```CLEAN R``` output to your right output channel
    1. When you move the ```SPREAD``` control, can you hear the stereo field changing?
10. *Action*: For each CV input in turn, connect the modulation source.
    1. Does the modulation source change the sound?
    2. For everything except ```VCA```, does the corresponding knob act as an attenverter?
    3. Does the ```VCA/SHAPE``` control scale the ```VCA``` CV level?
11. *Action*: connect the outputs to ```SHAPED L``` and ```SHAPED R```.
    1. Does the sound become overdriven?
    2. Does the ```DRIVE``` control change the sound?
    3. Do the four LEDs respond to changes in the sound?

