#ifndef PID_CUSTOM_H
#define PID_CUSTOM_H

#include <Arduino.h>

class PID_Custom {
private:
    float kp, ki, kd;           // PID gains
    float outputMin, outputMax; // Output limits
    bool inAuto;                // PID mode: manual or automatic

    float eprev;                // Previous error
    float eintegral;            // Integral of error
    float vFilt, vPrev;         // Low-pass filter variables

    // Private function to compute PID
    float computePID(float v_d, float meas_vel, float deltaT, bool derivativeOnMeasurement) {
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

        return u;
    }

public:
    float *input;     // Input variable
    float *output;    // Output variable (controller result)
    float *setpoint;  // Setpoint (desired value)

    // Constructor to initialize PID with gains and output limits
    PID_Custom(float *Input, float *Output, float *Setpoint, float Kp, float Ki, float Kd,
               float MinOutput = 0.0, float MaxOutput = 180.0) :
        input(Input), output(Output), setpoint(Setpoint),
        kp(Kp), ki(Ki), kd(Kd), outputMin(MinOutput), outputMax(MaxOutput),
        inAuto(false), eprev(0), eintegral(0), vFilt(0), vPrev(0) {}

    // Function to initialize the PID in automatic mode
    void setMode(bool autoMode) {
        inAuto = autoMode;
    }

    // Function to update the PID output based on the current input, setpoint, and PID gains
    void compute(float deltaT, bool derivativeOnMeasurement = true) {
        if (!inAuto) {
            return; // In manual mode, PID does not compute
        }

        // Call computePID to calculate the control output
        *output = computePID(*setpoint, *input, deltaT, derivativeOnMeasurement);

        // Store previous error
        eprev = *setpoint - *input;
    }

    // Function to set output limits
    void setOutputLimits(float minOutput, float maxOutput) {
        outputMin = minOutput;
        outputMax = maxOutput;
    }
    // **NEW** Reset integral and other variables
    void reset() {
        eintegral = 0;
        eprev = 0;
        vFilt = 0;
        vPrev = 0;
    }
};

#endif // PID_CUSTOM_H
