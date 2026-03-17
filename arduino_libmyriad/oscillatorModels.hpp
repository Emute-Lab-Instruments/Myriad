#pragma once

#include "hardware/pio.h"
#include <array>
#include "oscDisplayModes.hpp"
#include "clockfreq.h"
#include <limits>
#include "utils.hpp"
#include "oscmodels/oscillatorModel.hpp"

using WvlenFPType = Fixed<20,11>;
using oscModelPtr = oscillatorModel*;

const size_t __not_in_flash("mydata") N_OSCILLATOR_MODELS = 13;

#include "oscmodels/silentOscillatorModel.hpp"
#include "oscmodels/sawOscillatorModel.hpp"
#include "oscmodels/triOscillatorModel.hpp"
#include "oscmodels/noiseOscillatorModelSD.hpp"
#include "oscmodels/triSDVar1OscillatorModel.hpp"
#include "oscmodels/smoothThreshSDOscillatorModel.hpp"
#include "oscmodels/whiteNoiseOscillatorModel.hpp"
#include "oscmodels/pulseSDOscillatorModel.hpp"
#include "oscmodels/expPulseSDOscillatorModel.hpp"
#include "oscmodels/parasineSDOscillatorModel.hpp"
#include "oscmodels/pulsePWOscillatorModel.hpp"
#include "oscmodels/expPulse2SDOscillatorModel.hpp"
#include "oscmodels/formantSDOscillatorModel.hpp"
#include "oscmodels/bellSDOscillatorModel.hpp"
#include "oscmodels/sharkTeethSDOscillatorModel.hpp"
