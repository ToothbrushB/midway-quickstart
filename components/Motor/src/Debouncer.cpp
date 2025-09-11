#include "Debouncer.hpp"
#include <esp_timer.h>


// Returns true if input has been active for 'threshold' consecutive calls
bool Debouncer::calculate(bool active) {
    int64_t now = esp_timer_get_time();
    if (active) {
        if (!currentState) {
            // Start timing when signal goes active
            if (now - lastTime >= interval) {
                currentState = true;
            }
        }
        // Reset timer if signal is not yet debounced
        if (!currentState) {
            lastTime = now;
        }
    } else {
        // Reset state and timer if signal goes inactive
        currentState = false;
        lastTime = now;
    }
    return currentState;
}