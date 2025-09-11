#pragma once

class Debouncer {
    public:
        Debouncer(long interval) : interval(interval), lastTime(0) {}

        bool calculate(bool active);

    private:
        long interval;
        long lastTime;
        bool currentState = false;
};