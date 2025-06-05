#ifndef RESPONSIVEFILTER_HPP
#define RESPONSIVEFILTER_HPP

#include <cmath>

/**
 * @class ResponsiveFilter
 * @brief A class to filter and smooth analog readings with a responsive algorithm.
 *
 * This class provides a method to process raw analog readings and produce a smoothed output
 * that responds to changes in the input value. The smoothing is controlled by a snap multiplier
 * and an activity threshold.
 *
 * @param snapMultiplier A multiplier that controls the responsiveness of the filter. Default is 0.01.
 * @param activityThreshold A threshold that determines the minimum change required to update the output value. Default is 4.0.
 *
 * @method process Processes a raw analog value and returns a smoothed output value.
 * @param rawValue The raw analog value to be processed.
 * @return The smoothed output value.
 */

class ResponsiveFilter {
public:
    // Constructor to initialize the filter with optional snap multiplier and activity threshold
    ResponsiveFilter(float snapMultiplier = 0.1, float activityThreshold = 0.1)
        : smoothValue(0), outputValue(0), snapMultiplier(snapMultiplier), activityThreshold(activityThreshold) {}

    // Method to process a raw analog value and return a smoothed output value
    float process(const float rawValue) {
        float diff = fabs(rawValue - smoothValue); // Calculate the difference between raw value and smoothed value
        float snap  = diff * snapMultiplier; // Calculate the snap value based on the difference and snap multiplier
        if (snap < 0.0f) { // If the snap value is too small, set it to a minimum value
            snap = 0.0f;
        }else if (snap > 1.0f) { // If the snap value is too large, set it to a maximum value
            snap = 1.0f;
        }

        smoothValue += (rawValue - smoothValue) * snap; // Update the smoothed value

        if (fabs(smoothValue - outputValue) >= activityThreshold) { // Check if the change is above the activity threshold
            outputValue = smoothValue; // Update the output value
        }
        if (!std::isfinite(smoothValue) || !std::isfinite(smoothValue)) {
            // Reset or handle error
            smoothValue = rawValue;
            outputValue = rawValue;
        }        
        return outputValue; // Return the output value
    }

private:
    float smoothValue; // The smoothed value
    float outputValue; // The output value
    float snapMultiplier; // The snap multiplier
    float activityThreshold; // The activity threshold
};


#endif // RESPONSIVEFILTER_HPP