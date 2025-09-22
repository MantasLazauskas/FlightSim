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

    [Serializable]
    class TakeoffModeState {
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
        [Tooltip("Feet Above Ground Level")]
        public float finishTakeoffTargetAGL;
        [Tooltip("Knots")]
        public float finishTakeoffTargetSpeed;
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
                break;
        }
    }

    public void EnterTakeoffMode() {
        if (state == AutopilotState.Takeoff) return;

        state = AutopilotState.Takeoff;
        takeoffMode.state = TakeoffModeState.TakeoffState.Idle;
    }

    public void StartTakeoff() {
        takeoffMode.state = TakeoffModeState.TakeoffState.StartTakeoff;
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

    void HandleStartTakeoff(float dt) {
        plane.SetThrottleInput(1);

        var speedTarget = takeoffMode.rotationSpeed / Units.metersToKnots;

        if (plane.LocalVelocity.z > speedTarget) {
            takeoffMode.state = TakeoffModeState.TakeoffState.Rotate;
        }
    }

    void HandleRotateTakeoff(float dt) {
        var pitch = plane.PitchYawRoll.x;
        var pitchRate = plane.LocalAngularVelocity.x * Mathf.Rad2Deg;

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
        var pitchRate = plane.LocalAngularVelocity.x * Mathf.Rad2Deg;

        var pitchTarget = climbRateController.Update(dt, verticalSpeedFt, takeoffMode.finishTakeoffClimbRate, verticalAccelFt);
        var pitchInput = pitchHoldController.Update(dt, pitch, pitchTarget, pitchRate);

        var steering = new Vector3(-pitchInput, 0, 0);
        plane.SetControlInput(steering);

        var radarAlt = plane.RadarAltimeter;
        var speed = plane.LocalVelocity.z * Units.metersToKnots;

        if (radarAlt >= takeoffMode.finishTakeoffTargetAGL && speed >= takeoffMode.finishTakeoffTargetSpeed) {
            takeoffMode.state = TakeoffModeState.TakeoffState.Idle;
            state = AutopilotState.Navigate;
        }
    }
}
