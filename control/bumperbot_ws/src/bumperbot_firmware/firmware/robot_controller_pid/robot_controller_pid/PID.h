#ifndef PID_H
#define PID_H

struct PID {
    float kp, ki, kd;           // PID gains
    float outputMin, outputMax; // Output limits
    bool inAuto;                // PID mode: manual or automatic

    float eprev;                // Previous error
    float eintegral;            // Integral of error
    float vFilt, vPrev;         // Low-pass filter variables

    PID(float Kp, float Ki, float Kd, float minOutput, float maxOutput)
        : kp(Kp), ki(Ki), kd(Kd), outputMin(minOutput), outputMax(maxOutput),
          inAuto(false), eprev(0), eintegral(0), vFilt(0), vPrev(0) {}

    void setMode(bool autoMode) {
        inAuto = autoMode;
        if (!autoMode) { // Reset when switching to manual
            eprev = 0;
            eintegral = 0;
            vFilt = 0;
            vPrev = 0;
        }
    }

    void setOutputLimits(float minOutput, float maxOutput) {
        outputMin = minOutput;
        outputMax = maxOutput;
    }

    float compute(float v_d, float meas_vel, float deltaT, bool derivativeOnMeasurement = true) {
        if (!inAuto) {
            return 0; // In manual mode, PID does not compute
        }

        // Low-pass filter for measured velocity (25 Hz cutoff)
        vFilt = 0.854 * vFilt + 0.0728 * meas_vel + 0.0728 * vPrev;
        vPrev = meas_vel;

        // Error calculation
        float e = v_d - vFilt;

        // Derivative term (choose between error or measurement)
        float dedt;
        if (derivativeOnMeasurement) {
            dedt = -(vFilt - vPrev) / deltaT; // Derivative on measurement
        } else {
            dedt = (e - eprev) / deltaT;      // Derivative on error
        }

        // Proportional term
        float pTerm = kp * e;

        // Integral term (anti-windup included)
        eintegral += e * deltaT;
        if (eintegral * ki > outputMax) {
            eintegral = outputMax / ki;
        } else if (eintegral * ki < outputMin) {
            eintegral = outputMin / ki;
        }
        float iTerm = ki * eintegral;

        // Derivative term
        float dTerm = kd * dedt;

        // PID output
        float u = pTerm + iTerm + dTerm;

        // Constrain output to limits
        u = constrain(u, outputMin, outputMax);

        // Store previous error
        eprev = e;

        return u;
    }
};

#endif
