using System;
using System.Collections.Generic;
using UnityEngine;

[Serializable]
public class PIDController {
    //PID coefficients
    public float proportionalGain;
    public float integralGain;
    public float derivativeGain;

    public float outputMin = -1;
    public float outputMax = 1;
    public float integralSaturation;

    float integrationStored;

    public float Update(float dt, float currentValue, float targetValue, float velocity) {
        if (dt <= 0) throw new ArgumentOutOfRangeException(nameof(dt));

        float error = targetValue - currentValue;

        //calculate P term
        float P = proportionalGain * error;

        //calculate I term
        integrationStored = Mathf.Clamp(integrationStored + (error * dt), -integralSaturation, integralSaturation);
        float I = integralGain * integrationStored;

        float D = derivativeGain * velocity;

        float result = P + I + D;

        return Mathf.Clamp(result, outputMin, outputMax);
    }

    float AngleDifference(float a, float b) {
        return (a - b + 540) % 360 - 180;   //calculate modular difference, and remap to [-180, 180]
    }

    public float UpdateAngle(float dt, float currentAngle, float targetAngle, float velocity) {
        if (dt <= 0) throw new ArgumentOutOfRangeException(nameof(dt));
        float error = AngleDifference(targetAngle, currentAngle);

        //calculate P term
        float P = proportionalGain * error;

        //calculate I term
        integrationStored = Mathf.Clamp(integrationStored + (error * dt), -integralSaturation, integralSaturation);
        float I = integralGain * integrationStored;

        float D = derivativeGain * -velocity;

        float result = P + I + D;

        return Mathf.Clamp(result, outputMin, outputMax);
    }
}
