#pragma once
#include <Arduino.h>
#include "config.h"

// Phase label printed in HX_TIMEOUT diagnostics (set by callers).
extern const char *hxPhase;

class ForceSensor {
    private:
        int dataPin, clockPin;
        int prevForceRaw;
        uint32_t prevForceRawMs;
        bool hasFirstSample;
        bool isReversed;

        // TODO : interval stats

    public:
        ForceSensor(int dataPin, int clockPin);

        // statics
        static double rawToNewton(int32_t raw);

        // helpers
        bool isFresh();

        // blocking function
        bool readForceRaw(int32_t& output);
        bool readForce(double& output);

        // non-blocking function
        void serviceUpdateForce();
        bool getPrevForceRaw(int32_t& output);
        bool getPrevForce(double& output);
};