using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AutopilotController : MonoBehaviour {
    public enum AutopilotState {
        Idle,
        Takeoff,
        Landing,
        Navigate,
    }

    [SerializeField]
    Plane plane;
    [SerializeField]
    AutopilotState state;
    [SerializeField]
    PIDController pitchHoldController;
    [SerializeField]
    PIDController climbRateController;
    [SerializeField]
    TakeoffModeState takeoffMode;
    [SerializeField]
    NavigateModeState navigateMode;

    [Serializable]
    public class TakeoffModeState {
        public enum TakeoffState {
            Idle,
            StartTakeoff,
            Rotate,
            FinishTakeoff
        }

        public TakeoffState state;
        [Tooltip("Knots")]
        public float rotationSpeed;
        public float rotationAngle;
        [Tooltip("Feet/min")]
        public float finishTakeoffClimbRate;
        [Tooltip("Feet Above Ground Level (runway)")]
        public float finishTakeoffTargetAGL;
        [Tooltip("Knots")]
        public float finishTakeoffTargetSpeed;

        public float runwayAltitude;
    }

    [Serializable]
    public class NavigateModeState {
        public enum PitchControlMode {
            PitchMode,
            FlightPathMode,
            AltitudeMode
        }

        public enum AltitudeControlMode {
            AltitudeHold,
            ClimbRateHold
        }

        public PitchControlMode pitchControlMode;
        public AltitudeControlMode altitudeControlMode;
        public float targetPitch;
        [Tooltip("Knots")]
        public float targetSpeed;
        [Tooltip("Feet")]
        public float targetAltitude;
        [Tooltip("Feet/min")]
        public float targetClimbRate;
        public float targetHeading;
    }

    public AutopilotState State {
        get {
            return state;
        }
    }

    void Start() {

    }

    void FixedUpdate() {
        float dt = Time.fixedDeltaTime;

        switch (state) {
            case AutopilotState.Idle:
                break;
            case AutopilotState.Takeoff:
                HandleTakeoff(dt);
                break;
            case AutopilotState.Landing:
                break;
            case AutopilotState.Navigate:
                HandleNavigate(dt);
                break;
        }
    }

    float GetPitchRate(Plane plane) {
        return -plane.LocalAngularVelocity.x * Mathf.Rad2Deg;
    }

    public void EnterTakeoffMode() {
        if (state == AutopilotState.Takeoff) return;

        state = AutopilotState.Takeoff;
        takeoffMode.state = TakeoffModeState.TakeoffState.Idle;
    }

    public void EnterNavigateMode() {
        if (state == AutopilotState.Navigate) return;

        state = AutopilotState.Navigate;

        ResetNavigation();
    }

    public void ResetNavigation() {
        SetPitchControlMode(NavigateModeState.PitchControlMode.FlightPathMode);
        SetAltitudeControlMode(NavigateModeState.AltitudeControlMode.AltitudeHold);

        navigateMode.targetAltitude = plane.Rigidbody.position.y * Units.metersToFeet;
        navigateMode.targetHeading = plane.PitchYawRoll.y;
        navigateMode.targetSpeed = plane.LocalVelocity.z * Units.metersToKnots;
    }

    public void SetPitchControlMode(NavigateModeState.PitchControlMode mode) {
        if (navigateMode.pitchControlMode == mode) return;

        navigateMode.pitchControlMode = mode;
        pitchHoldController.Reset();
        climbRateController.Reset();
    }

    public void SetAltitudeControlMode(NavigateModeState.AltitudeControlMode mode) {
        if (navigateMode.altitudeControlMode == mode) return;

        navigateMode.altitudeControlMode = mode;
        pitchHoldController.Reset();
        climbRateController.Reset();
    }

    void HandleNavigate(float dt) {
        HandleNavigatePitchControl(dt);
    }

    void HandleNavigatePitchControl(float dt) {
        switch (navigateMode.pitchControlMode) {
            case NavigateModeState.PitchControlMode.PitchMode:
                HandleNavigatePitchMode(dt);
                break;
            case NavigateModeState.PitchControlMode.FlightPathMode:
                HandleNavigateFlightPathMode(dt);
                break;
            case NavigateModeState.PitchControlMode.AltitudeMode:
                HandleNavigateAltitudeMode(dt);
                break;
        }
    }

    void HandleNavigatePitchMode(float dt) {

    }

    void HandleNavigateFlightPathMode(float dt) {
        var velocityDir = plane.Rigidbody.velocity.normalized;
        var vertical = Vector3.up;
        var flightPathAngle = 90 - Vector3.Angle(vertical, velocityDir);
        var pitchRate = GetPitchRate(plane);

        var pitchInput = pitchHoldController.Update(dt, flightPathAngle, navigateMode.targetPitch, pitchRate);

        var steering = new Vector3(-pitchInput, 0, 0);
        plane.SetControlInput(steering);
    }

    void HandleNavigateAltitudeMode(float dt) {

    }

    void HandleTakeoff(float dt) {
        switch (takeoffMode.state) {
            case TakeoffModeState.TakeoffState.Idle:
                break;
            case TakeoffModeState.TakeoffState.StartTakeoff:
                HandleStartTakeoff(dt);
                break;
            case TakeoffModeState.TakeoffState.Rotate:
                HandleRotateTakeoff(dt);
                break;
            case TakeoffModeState.TakeoffState.FinishTakeoff:
                HandleFinishTakeoff(dt);
                break;
        }
    }

    public void StartTakeoff() {
        takeoffMode.state = TakeoffModeState.TakeoffState.StartTakeoff;
        takeoffMode.runwayAltitude = plane.Rigidbody.position.y * Units.metersToFeet;
    }

    void HandleStartTakeoff(float dt) {
        plane.SetThrottleInput(1);

        var speedTarget = takeoffMode.rotationSpeed / Units.metersToKnots;

        if (plane.LocalVelocity.z > speedTarget) {
            takeoffMode.state = TakeoffModeState.TakeoffState.Rotate;
        }
    }

    void HandleRotateTakeoff(float dt) {
        var pitch = plane.PitchYawRoll.x;
        var pitchRate = GetPitchRate(plane);

        var pitchInput = pitchHoldController.Update(dt, pitch, takeoffMode.rotationAngle, pitchRate);

        var steering = new Vector3(-pitchInput, 0, 0);
        plane.SetControlInput(steering);

        if (!plane.Grounded) {
            takeoffMode.state = TakeoffModeState.TakeoffState.FinishTakeoff;
        }
    }

    void HandleFinishTakeoff(float dt) {
        // convert m/s to ft/min
        var verticalSpeedFt = plane.Rigidbody.velocity.y * Units.metersToFeet * 60;
        var verticalAccelFt = plane.GForce.y * Units.metersToFeet * 60;
        var pitch = plane.PitchYawRoll.x;
        var pitchRate = GetPitchRate(plane);

        var pitchTarget = climbRateController.Update(dt, verticalSpeedFt, takeoffMode.finishTakeoffClimbRate, verticalAccelFt);
        var pitchInput = pitchHoldController.Update(dt, pitch, pitchTarget, pitchRate);

        var steering = new Vector3(-pitchInput, 0, 0);
        plane.SetControlInput(steering);

        var alt = plane.Rigidbody.position.y * Units.metersToFeet;
        var targetAlt = takeoffMode.finishTakeoffTargetAGL + takeoffMode.runwayAltitude;
        var speed = plane.LocalVelocity.z * Units.metersToKnots;

        if (alt >= targetAlt && speed >= takeoffMode.finishTakeoffTargetSpeed) {
            takeoffMode.state = TakeoffModeState.TakeoffState.Idle;
            EnterNavigateMode();
        }
    }
}
