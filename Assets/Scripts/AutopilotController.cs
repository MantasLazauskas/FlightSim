using System;
using System.Collections;
using System.Collections.Generic;
using System.Text;
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
    PIDController altitudeHoldController;
    [SerializeField]
    PIDController speedHoldController;
    [SerializeField]
    TakeoffModeState takeoffMode;
    [SerializeField]
    NavigateModeState navigateMode;

    float internalTargetClimbRate;
    float internalTargetPitch;

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
        public float rotationSpeedKts;
        public float rotationAngle;
        [Tooltip("Feet/min")]
        public float finishTakeoffClimbRateFtPerMin;
        [Tooltip("Feet Above Ground Level (runway)")]
        public float finishTakeoffTargetFtAGL;
        [Tooltip("Knots")]
        public float finishTakeoffTargetSpeedKts;

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
        public float targetSpeedKts;
        [Tooltip("Feet")]
        public float targetAltitudeFt;
        [Tooltip("Feet/min")]
        public float targetClimbRateFtPerMin;
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

    public void WriteDebugString(StringBuilder builder) {
        if (state == AutopilotState.Navigate) {
            if (navigateMode.pitchControlMode == NavigateModeState.PitchControlMode.AltitudeMode) {
                builder.AppendLine(string.Format("Internal climb rate: {0:N0}", internalTargetClimbRate));
                builder.AppendLine(string.Format("Internal pitch: {0:N1}", internalTargetPitch));
            }
        }
    }

    float GetPitchRate(Plane plane) {
        return -plane.LocalAngularVelocity.x * Mathf.Rad2Deg;
    }

    float GetPitchInputForClimbRateHold(float dt, float targetClimbRate) {
        // convert m/s to ft/min
        var verticalSpeedFt = plane.Rigidbody.velocity.y * Units.metersToFeet * 60;
        var verticalAccelFt = plane.GForce.y * Units.metersToFeet * 60;
        var pitch = plane.PitchYawRoll.x;
        var pitchRate = GetPitchRate(plane);

        var pitchTarget = climbRateController.Update(dt, verticalSpeedFt, targetClimbRate, verticalAccelFt);
        var pitchInput = pitchHoldController.Update(dt, pitch, pitchTarget, pitchRate);

        internalTargetPitch = pitchTarget;

        return pitchInput;
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

        navigateMode.targetAltitudeFt = plane.Rigidbody.position.y * Units.metersToFeet;
        navigateMode.targetHeading = plane.PitchYawRoll.y;
        navigateMode.targetSpeedKts = plane.LocalVelocity.z * Units.metersToKnots;
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
        HandleNavigateSpeedControl(dt);
        HandleNavigatePitchControl(dt);
    }

    void HandleNavigatePitchControl(float dt) {
        switch (navigateMode.pitchControlMode) {
            case NavigateModeState.PitchControlMode.PitchMode:
            case NavigateModeState.PitchControlMode.FlightPathMode:
                HandleNavigateFlightPathMode(dt, navigateMode.pitchControlMode);
                break;
            case NavigateModeState.PitchControlMode.AltitudeMode:
                HandleNavigateAltitudeMode(dt);
                break;
        }
    }

    void HandleNavigateFlightPathMode(float dt, NavigateModeState.PitchControlMode mode) {
        var currentPitch = plane.PitchYawRoll.x;
        var pitchRate = GetPitchRate(plane);

        if (mode == NavigateModeState.PitchControlMode.FlightPathMode) {
            var velocityDir = plane.Rigidbody.velocity.normalized;
            var vertical = Vector3.up;
            var flightPathAngle = 90 - Vector3.Angle(vertical, velocityDir);
            currentPitch = flightPathAngle;
        }

        var pitchInput = pitchHoldController.Update(dt, currentPitch, navigateMode.targetPitch, pitchRate);

        var steering = new Vector3(-pitchInput, 0, 0);
        plane.SetControlInput(steering);
    }

    void HandleNavigateAltitudeMode(float dt) {
        var climbRate = navigateMode.targetClimbRateFtPerMin;

        if (navigateMode.altitudeControlMode == NavigateModeState.AltitudeControlMode.AltitudeHold) {
            // convert m to ft, m/s to ft/min
            var altitudeFt = plane.Rigidbody.position.y * Units.metersToFeet;
            var verticalSpeedFt = plane.Rigidbody.velocity.y * Units.metersToFeet * 60;

            climbRate = altitudeHoldController.Update(dt, altitudeFt, navigateMode.targetAltitudeFt, verticalSpeedFt);
            internalTargetClimbRate = climbRate;
        }

        var pitchInput = GetPitchInputForClimbRateHold(dt, climbRate);

        var steering = new Vector3(-pitchInput, 0, 0);
        plane.SetControlInput(steering);
    }

    void HandleNavigateSpeedControl(float dt) {
        var speed = plane.LocalVelocity.z * Units.metersToKnots;
        var accel = plane.LocalGForce.z * Units.metersToKnots;

        var throttleInput = speedHoldController.Update(dt, speed, navigateMode.targetSpeedKts, accel);
        plane.SetThrottleInput(throttleInput);
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

        var speedTarget = takeoffMode.rotationSpeedKts / Units.metersToKnots;

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
        var pitchInput = GetPitchInputForClimbRateHold(dt, takeoffMode.finishTakeoffClimbRateFtPerMin);

        var steering = new Vector3(-pitchInput, 0, 0);
        plane.SetControlInput(steering);

        var alt = plane.Rigidbody.position.y * Units.metersToFeet;
        var targetAlt = takeoffMode.finishTakeoffTargetFtAGL + takeoffMode.runwayAltitude;
        var speed = plane.LocalVelocity.z * Units.metersToKnots;

        if (alt >= targetAlt && speed >= takeoffMode.finishTakeoffTargetSpeedKts) {
            takeoffMode.state = TakeoffModeState.TakeoffState.Idle;
            EnterNavigateMode();
        }
    }
}
