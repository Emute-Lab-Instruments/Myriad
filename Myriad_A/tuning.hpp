#ifndef MYRIAD_A_TUNING_HPP
#define MYRIAD_A_TUNING_HPP

#define TUNING_MEM __not_in_flash("tuningdata")

namespace TuningSettings {
    static TUNING_MEM int octaves=0;
    static TUNING_MEM int semitones=0;
    static TUNING_MEM int cents=0;
    static TUNING_MEM float adjustment = 0.0f;
};

#endif // MYRIAD_A_TUNING_HPP