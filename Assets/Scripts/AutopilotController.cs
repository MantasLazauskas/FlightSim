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
    float deadzone;
    [SerializeField]
    float steeringPitchMaxHeadingError;
    [SerializeField]
    PIDController pitchHoldController;
    [SerializeField]
    PIDController climbRateController;
    [SerializeField]
    PIDController altitudeHoldController;
    [SerializeField]
    PIDController speedHoldController;
    [SerializeField]
    PIDController turnBankController;
    [SerializeField]
    PIDController rollController;
    [SerializeField]
    PIDController yawSlipController;
    [SerializeField]
    PIDController yawHeadingController;
    [SerializeField]
    PIDController glideSlopeController;
    [SerializeField]
    PIDController crossTrackController;
    [SerializeField]
    TakeoffModeState takeoffMode;
    [SerializeField]
    NavigateModeState navigateMode;
    [SerializeField]
    LandingModeState landingMode;

    float internalHeading;
    float internalFlightPath;
    float internalTargetClimbRate;
    float internalTargetPitch;
    float internalTargetFlightPath;
    float internalTargetRoll;

    float internalLandingDistance;
    float internalLandingCrossTrack;
    float internalLandingAltitude;
    float internalLandingHeading;
    float internalGlideSlope;
    float internalLandingAngle;

    bool hasFlightPathVelocity;
    float flightPathVelocity;
    bool hasCrossTrackVelocity;
    float crossTrackVelocity;

    [Serializable]
    public class TakeoffModeState {
        public enum TakeoffState {
            StartTakeoff,
            Rotate,
            FinishTakeoff
        }

        public TakeoffState state;
        [Tooltip("Knots")]
        public float rotationSpeedKts;
        public float rotationAngle;
        [Tooltip("Knots")]
        public float takeoffTargetSpeedKts;
        [Tooltip("Feet/min")]
        public float finishTakeoffClimbRateFtPerMin;
        [Tooltip("Feet Above Ground Level (runway)")]
        public float finishTakeoffMinFtAGL;
        [Tooltip("Knots")]
        public float finishTakeoffMinSpeedKts;

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

    [Serializable]
    public class LandingModeState {
        public enum LandingState {
            Align,
            Approach,
            Flare,
            Touchdown
        }

        public LandingState state;
        public List<Runway> runways;
        public Runway selectedRunway;
        public Vector3 touchdownPosition;
        public Vector3 touchdownDirection;

        public float idealGlideSlope;
        public float approachSpeedKts;
        public float approachDistance;
        public float approachMaxCrossTrackError;
        public float approachMinGlideSlope;
        public float approachMaxGlideSlope;
        public float approachMaxAngle;
        public float flareVerticalSpeedFtPerMin;
        public float flareAltitudeFt;

        public float captureMinDistance;
        public float captureMaxDistance;
        public float captureMaxAngle;
        public float captureMinGlideSlope;
        public float captureMaxGlideSlope;

        public float abortApproachMinGlideSlope;
        public float abortApproachMaxGlideSlope;
        public float abortApproachMaxAngle;
        public float abortApproachMaxCrossTrackError;
        public float abortTouchdownMaxAngle;
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
        UpdatePlaneData(dt);

        switch (state) {
            case AutopilotState.Idle:
                break;
            case AutopilotState.Takeoff:
                HandleTakeoff(dt);
                break;
            case AutopilotState.Landing:
                HandleLanding(dt);
                break;
            case AutopilotState.Navigate:
                HandleNavigate(dt);
                break;
        }
    }

    float CalculateHeading(Vector3 direction) {
        // calculate compass heading
        var heading = Vector3.SignedAngle(Vector3.forward, direction, Vector3.up);
        return Utilities.MapAngleTo180(heading);
    }

    void UpdateValueVelocity(float dt, float newValue, ref bool hasValue, ref float currentValue, ref float velocity) {
        if (hasValue && dt != 0) {
            velocity = (newValue - currentValue) / dt;
        } else {
            velocity = 0;
        }

        hasValue = true;
        currentValue = newValue;
    }

    void UpdatePlaneData(float dt) {
        var planeTrueDirection = plane.Rigidbody.rotation * Vector3.forward;

        var plane2DDirection = planeTrueDirection;
        plane2DDirection.y = 0;
        plane2DDirection = plane2DDirection.normalized;

        internalHeading = CalculateHeading(plane2DDirection);

        var velocityDir = plane.Rigidbody.velocity.normalized;

        var currentFlightPath = 90 - Vector3.Angle(Vector3.up, velocityDir);
        UpdateValueVelocity(dt, currentFlightPath, ref hasFlightPathVelocity, ref internalFlightPath, ref flightPathVelocity);
    }

    public void WriteDebugString(StringBuilder builder) {
        builder.AppendLine(string.Format("Mode: {0}", state));

        float pitch = plane.PitchYawRoll.x;
        builder.AppendLine(string.Format("Pitch: {0:N1}", pitch));

        float pitchRate = -plane.LocalAngularVelocity.x * Mathf.Rad2Deg;
        builder.AppendLine(string.Format("Pitch rate: {0:N1}", pitchRate));

        float agl = plane.RadarAltimeter * Units.metersToFeet;
        builder.AppendLine(string.Format("AGL: {0:N0} m", agl));

        float climbRate = plane.Rigidbody.velocity.y * Units.metersToFeet * 60;
        builder.AppendLine(string.Format("Climb rate: {0} fpm", (int)Mathf.Round(climbRate)));
        builder.AppendLine(string.Format("Heading: {0:N0}", internalHeading));

        if (state == AutopilotState.Navigate) {
            builder.AppendLine(string.Format("Pitch target: {0:N1}", internalTargetPitch));
            builder.AppendLine(string.Format("Roll target: {0:N1}", internalTargetRoll));

            if (navigateMode.pitchControlMode == NavigateModeState.PitchControlMode.AltitudeMode) {
                builder.AppendLine(string.Format("Climb rate target: {0:N0}", internalTargetClimbRate));
            }
        } else if (state == AutopilotState.Landing) {
            builder.AppendLine(string.Format("Landing: {0}", landingMode.state));
            builder.AppendLine(string.Format("  Distance: {0:N0} m", internalLandingDistance));
            builder.AppendLine(string.Format("  Cross track error: {0:N0} m", internalLandingCrossTrack));
            builder.AppendLine(string.Format("  Altitude: {0:N0} m", internalLandingAltitude));
            builder.AppendLine(string.Format("  Glide slope: {0:N1}", internalGlideSlope));
            builder.AppendLine(string.Format("  Angle: {0:N1}", internalLandingAngle));
            builder.AppendLine(string.Format("  Flight path target: {0:N1}", internalTargetFlightPath));
        }
    }

    float GetPitchRate(Plane plane) {
        return -plane.LocalAngularVelocity.x * Mathf.Rad2Deg;
    }

    float GetRollRate(Plane plane) {
        return -plane.LocalAngularVelocity.z * Mathf.Rad2Deg;
    }

    float GetYawRate(Plane plane) {
        return plane.LocalAngularVelocity.y * Mathf.Rad2Deg;
    }

    float ApplyDeadzone(float input) {
        if (Mathf.Abs(input) < deadzone) {
            return 0;
        }

        return input;
    }

    void SetThrottleSpeedHold(float dt, float targetSpeed) {
        var speed = plane.LocalVelocity.z * Units.metersToKnots;
        var accel = plane.LocalGForce.z * Units.metersToKnots;

        var throttleInput = speedHoldController.Update(dt, speed, targetSpeed, accel);
        plane.SetThrottleInput(throttleInput);
    }

    void SetControlInput(Plane plane, Vector3 input) {
        input.x = ApplyDeadzone(-input.x);
        input.y = ApplyDeadzone(input.y);
        input.z = ApplyDeadzone(-input.z);

        plane.SetControlInput(input);
    }

    float CalculatePitchClimbRate(float dt, float targetClimbRate) {
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

    float CalculatePitchFlightPath(float dt, float flightPath, float targetFlightPath, float pitchRate) {
        var pitchInput = pitchHoldController.Update(dt, flightPath, targetFlightPath, pitchRate);
        return pitchInput;
    }

    float CalculateGlideSlopeTarget(float dt, float glidePath, float targetGlidePath, float pitchRate) {
        // glide slope measures degrees downwards, convert to flight path which is degrees upward
        var bias = glideSlopeController.Update(dt, glidePath, targetGlidePath, pitchRate);
        return -targetGlidePath + bias;
    }

    float CalculateRollBank(float dt, float targetHeading) {
        var heading = plane.PitchYawRoll.y;
        var turnRate = plane.Rigidbody.angularVelocity.y * Mathf.Rad2Deg;
        var roll = plane.PitchYawRoll.z;
        var rollRate = GetRollRate(plane);

        var rollTarget = turnBankController.UpdateAngle(dt, heading, targetHeading, turnRate);
        var rollInput = rollController.Update(dt, roll, rollTarget, rollRate);
        internalTargetRoll = rollTarget;

        return rollInput;
    }

    float CalculateCrossTrackTarget(float dt, float targetHeading, float crossTrackError, float crossTrackVelocity) {
        float bias = crossTrackController.Update(dt, crossTrackError, 0, crossTrackVelocity);
        return targetHeading + bias;
    }

    float CalculateYawSlip(float dt, float targetSlip, float yawRate) {
        var slip = -plane.AngleOfAttackYaw * Mathf.Rad2Deg;

        var yawInput = yawSlipController.Update(dt, slip, targetSlip, yawRate);
        return yawInput;
    }

    float CalculateYawHeading(float dt, float heading, float targetHeading, float yawRate) {
        var yawInput = yawHeadingController.Update(dt, heading, targetHeading, yawRate);
        return yawInput;
    }

    public void EnterTakeoffMode() {
        if (state == AutopilotState.Takeoff) return;
        if (state == AutopilotState.Landing && landingMode.state != LandingModeState.LandingState.Touchdown) {
            AbortLandingToTakeoff();
        }
        if (!plane.Grounded) return;

        state = AutopilotState.Takeoff;
        takeoffMode.state = TakeoffModeState.TakeoffState.StartTakeoff;
        takeoffMode.runwayAltitude = plane.Rigidbody.position.y * Units.metersToFeet;
    }

    public void EnterNavigateMode() {
        if (state == AutopilotState.Navigate) return;

        state = AutopilotState.Navigate;

        ResetNavigation();

        if (plane.FlapsDeployed) {
            plane.ToggleFlaps();
        }
    }

    public void EnterLandingMode() {
        if (state == AutopilotState.Landing) return;

        TryLandingCapture();
    }

    public void ResetNavigation() {
        SetPitchControlMode(NavigateModeState.PitchControlMode.FlightPathMode);
        SetAltitudeControlMode(NavigateModeState.AltitudeControlMode.AltitudeHold);

        navigateMode.targetAltitudeFt = plane.Rigidbody.position.y * Units.metersToFeet;
        navigateMode.targetHeading = plane.PitchYawRoll.y;
        navigateMode.targetSpeedKts = plane.LocalVelocity.z * Units.metersToKnots;
    }

    void ResetLanding() {
        landingMode.selectedRunway = null;
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
        SetThrottleSpeedHold(dt, navigateMode.targetSpeedKts);

        var yawRate = GetYawRate(plane);

        var pitchInput = HandleNavigatePitchControl(dt);
        var rollInput = CalculateRollBank(dt, Utilities.MapAngleTo180(navigateMode.targetHeading));
        var yawInput = CalculateYawSlip(dt, 0, yawRate);

        var steering = new Vector3(pitchInput, yawInput, rollInput);
        SetControlInput(plane, steering);
    }

    float HandleNavigatePitchControl(float dt) {
        switch (navigateMode.pitchControlMode) {
            case NavigateModeState.PitchControlMode.PitchMode:
            case NavigateModeState.PitchControlMode.FlightPathMode:
                return HandleNavigateFlightPathMode(dt, navigateMode.pitchControlMode);
            case NavigateModeState.PitchControlMode.AltitudeMode:
                return HandleNavigateAltitudeMode(dt);
        }

        return 0;
    }

    float HandleNavigateFlightPathMode(float dt, NavigateModeState.PitchControlMode mode) {
        var pitchRate = GetPitchRate(plane);

        if (mode == NavigateModeState.PitchControlMode.FlightPathMode) {
            return CalculatePitchFlightPath(dt, internalFlightPath, navigateMode.targetPitch, pitchRate);
        } else {
            var pitch = plane.PitchYawRoll.x;
            var pitchInput = pitchHoldController.Update(dt, pitch, navigateMode.targetPitch, pitchRate);
            return pitchInput;
        }
    }

    float HandleNavigateAltitudeMode(float dt) {
        var climbRate = navigateMode.targetClimbRateFtPerMin;

        if (navigateMode.altitudeControlMode == NavigateModeState.AltitudeControlMode.AltitudeHold) {
            // convert m to ft, m/s to ft/min
            var altitudeFt = plane.Rigidbody.position.y * Units.metersToFeet;
            var verticalSpeedFt = plane.Rigidbody.velocity.y * Units.metersToFeet * 60;

            climbRate = altitudeHoldController.Update(dt, altitudeFt, navigateMode.targetAltitudeFt, verticalSpeedFt);
            internalTargetClimbRate = climbRate;
        }

        var pitchInput = CalculatePitchClimbRate(dt, climbRate);
        return pitchInput;
    }

    void HandleTakeoff(float dt) {
        switch (takeoffMode.state) {
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

        var pitch = plane.PitchYawRoll.x;
        var roll = plane.PitchYawRoll.z;
        var pitchRate = GetPitchRate(plane);
        var rollRate = GetRollRate(plane);

        var pitchInput = pitchHoldController.Update(dt, pitch, takeoffMode.rotationAngle, pitchRate);
        var rollInput = rollController.Update(dt, roll, 0, rollRate);

        var steering = new Vector3(pitchInput, 0, rollInput);
        SetControlInput(plane, steering);

        var speedTarget = takeoffMode.rotationSpeedKts / Units.metersToKnots;

        if (plane.LocalVelocity.z > speedTarget) {
            takeoffMode.state = TakeoffModeState.TakeoffState.Rotate;
        }
    }

    void HandleRotateTakeoff(float dt) {
        var pitch = plane.PitchYawRoll.x;
        var roll = plane.PitchYawRoll.z;
        var pitchRate = GetPitchRate(plane);
        var rollRate = GetRollRate(plane);

        var pitchInput = pitchHoldController.Update(dt, pitch, takeoffMode.rotationAngle, pitchRate);
        var rollInput = rollController.Update(dt, roll, 0, rollRate);

        var steering = new Vector3(pitchInput, 0, rollInput);
        SetControlInput(plane, steering);

        if (!plane.Grounded) {
            takeoffMode.state = TakeoffModeState.TakeoffState.FinishTakeoff;
        }
    }

    void HandleFinishTakeoff(float dt) {
        SetThrottleSpeedHold(dt, takeoffMode.takeoffTargetSpeedKts);

        var roll = plane.PitchYawRoll.z;
        var rollRate = GetRollRate(plane);

        var pitchInput = CalculatePitchClimbRate(dt, takeoffMode.finishTakeoffClimbRateFtPerMin);
        var rollInput = rollController.Update(dt, roll, 0, rollRate);

        var steering = new Vector3(pitchInput, 0, rollInput);
        SetControlInput(plane, steering);

        var alt = plane.Rigidbody.position.y * Units.metersToFeet;
        var targetAlt = takeoffMode.finishTakeoffMinFtAGL + takeoffMode.runwayAltitude;
        var speed = plane.LocalVelocity.z * Units.metersToKnots;

        if (alt >= targetAlt && speed >= takeoffMode.finishTakeoffMinSpeedKts) {
            EnterNavigateMode();
        }
    }

    void TryLandingCapture() {
        ResetLanding();

        var planePosition = plane.Rigidbody.position;
        var planeDirection = plane.Rigidbody.rotation * Vector3.forward;
        planeDirection.y = 0;

        if (planeDirection == Vector3.zero) return;

        planeDirection = planeDirection.normalized;

        Runway bestRunway = null;
        float bestDistance = float.PositiveInfinity;

        foreach (var runway in landingMode.runways) {
            var result = TryCapture(runway, planePosition, planeDirection);

            if (result.valid && result.distance < bestDistance) {
                bestRunway = runway;
                bestDistance = result.distance;
            }
        }

        if (bestRunway != null) {
            state = AutopilotState.Landing;

            var touchdownData = bestRunway.GetClosestTouchdown(planePosition);

            landingMode.selectedRunway = bestRunway;
            landingMode.touchdownPosition = touchdownData.position;
            landingMode.touchdownDirection = touchdownData.direction;
            hasCrossTrackVelocity = false;
        }
    }

    struct CaptureResult {
        public bool valid;
        public float distance;
    }

    CaptureResult TryCapture(Runway runway, Vector3 planePosition, Vector3 planeDirection) {
        CaptureResult result = new CaptureResult();
        var data = runway.GetClosestTouchdown(plane.Rigidbody.position);

        var diff = (data.position - planePosition);

        // below runway
        if (diff.y > 0) {
            return result;
        }

        // test horizontal distance
        diff.y = 0;
        var dist = diff.magnitude;

        if (dist < landingMode.captureMinDistance || dist > landingMode.captureMaxDistance) {
            return result;
        }

        var angleError = Vector3.Angle(data.direction, planeDirection);
        if (angleError > landingMode.captureMaxAngle) {
            return result;
        }

        var predictedPath = (data.position - planePosition).normalized;
        var glideSlope = CalculateGlideSlope(data.direction, predictedPath);

        if (glideSlope < landingMode.captureMinGlideSlope || glideSlope > landingMode.captureMaxGlideSlope) {
            return result;
        }

        result.distance = dist;
        result.valid = true;

        return result;
    }

    float CalculateGlideSlope(Vector3 runwayDirection, Vector3 planeDirection) {
        Vector3 axis = Vector3.Cross(runwayDirection, Vector3.down);
        Vector3 runwayDir = Vector3.ProjectOnPlane(runwayDirection, axis);
        Vector3 planeDir = Vector3.ProjectOnPlane(planeDirection, axis);

        return Vector3.SignedAngle(runwayDir, planeDir, axis);
    }

    void HandleLanding(float dt) {
        UpdateLandingData(dt);

        switch (landingMode.state) {
            case LandingModeState.LandingState.Align:
                HandleLandingAlign(dt);
                break;
            case LandingModeState.LandingState.Approach:
                HandleLandingApproach(dt);
                break;
            case LandingModeState.LandingState.Flare:
                HandleLandingFlare(dt);
                break;
            case LandingModeState.LandingState.Touchdown:
                HandleLandingTouchdown(dt);
                break;
        }
    }

    void UpdateLandingData(float dt) {
        var planePosition = plane.Rigidbody.position;
        var planeVelocityDirection = plane.Rigidbody.velocity.normalized;

        var planePosition2D = planePosition;
        planePosition2D.y = 0;

        var touchdownPosition2D = landingMode.touchdownPosition;
        touchdownPosition2D.y = 0;

        var crossDir = Vector3.Cross(landingMode.touchdownDirection, Vector3.up);

        var error2D = (touchdownPosition2D - planePosition2D);
        internalLandingDistance = Vector3.Dot(error2D, landingMode.touchdownDirection);

        var currentCrossTrack = Vector3.Dot(error2D, crossDir);
        UpdateValueVelocity(dt, currentCrossTrack, ref hasCrossTrackVelocity, ref internalLandingCrossTrack, ref crossTrackVelocity);

        var error = (landingMode.touchdownPosition - planePosition);
        var glideDirection = error.normalized;

        var glideSlope = CalculateGlideSlope(landingMode.touchdownDirection, glideDirection);
        internalGlideSlope = glideSlope;

        internalLandingHeading = CalculateHeading(landingMode.touchdownDirection);

        var altitude = -error.y;
        internalLandingAltitude = altitude;

        var planeVelocity2D = planeVelocityDirection;
        planeVelocity2D.y = 0;
        planeVelocity2D = planeVelocity2D.normalized;

        internalLandingAngle = Vector3.Angle(landingMode.touchdownDirection, planeVelocity2D);
    }

    void AbortLandingToTakeoff() {
        state = AutopilotState.Takeoff;
        takeoffMode.state = TakeoffModeState.TakeoffState.FinishTakeoff;
        takeoffMode.runwayAltitude = landingMode.touchdownPosition.y;
    }

    bool CheckLandingAbort() {
        bool crossTrackCheck = Mathf.Abs(internalLandingCrossTrack) > landingMode.abortApproachMaxCrossTrackError;
        bool glideSlopeCheck = internalGlideSlope < landingMode.abortApproachMinGlideSlope && internalGlideSlope > landingMode.abortApproachMaxGlideSlope;
        bool distanceCheck = internalLandingDistance < landingMode.approachDistance;
        bool angleCheck = internalLandingAngle > landingMode.abortApproachMaxAngle;

        if ((crossTrackCheck || glideSlopeCheck || angleCheck) && distanceCheck) {
            return true;
        }

        return false;
    }

    void SteerLandingApproach(float dt) {
        var pitchRate = GetPitchRate(plane);
        var yawRate = GetYawRate(plane);

        var targetFlightPath = CalculateGlideSlopeTarget(dt, internalGlideSlope, landingMode.idealGlideSlope, pitchRate);
        var targetHeading = CalculateCrossTrackTarget(dt, internalLandingHeading, internalLandingCrossTrack, crossTrackVelocity);
        var yawError = internalHeading - targetHeading;

        if (Mathf.Abs(yawError) > steeringPitchMaxHeadingError && Mathf.Abs(targetFlightPath) > Mathf.Abs(internalFlightPath)) {
            targetFlightPath = internalFlightPath;
        }

        internalTargetFlightPath = targetFlightPath;

        var pitchInput = CalculatePitchFlightPath(dt, internalFlightPath, targetFlightPath, flightPathVelocity);
        var rollInput = CalculateRollBank(dt, targetHeading);
        var yawInput = CalculateYawSlip(dt, 0, yawRate);

        var steering = new Vector3(pitchInput, yawInput, rollInput);
        SetControlInput(plane, steering);
    }

    void HandleLandingAlign(float dt) {
        SetThrottleSpeedHold(dt, landingMode.approachSpeedKts);
        SteerLandingApproach(dt);

        bool crossTrackCheck = Mathf.Abs(internalLandingCrossTrack) < landingMode.approachMaxCrossTrackError;
        bool glideSlopeCheck = internalGlideSlope > landingMode.approachMinGlideSlope && internalGlideSlope < landingMode.approachMaxGlideSlope;
        bool angleCheck = internalLandingAngle < landingMode.approachMaxAngle;

        if (crossTrackCheck && glideSlopeCheck && angleCheck) {
            landingMode.state = LandingModeState.LandingState.Approach;
        } else if (internalLandingDistance < landingMode.approachDistance) {
            AbortLandingToTakeoff();
        }
    }

    void HandleLandingApproach(float dt) {
        SetThrottleSpeedHold(dt, landingMode.approachSpeedKts);
        SteerLandingApproach(dt);

        if (!plane.FlapsDeployed) {
            // deploy flaps and landing gear;
            plane.ToggleFlaps();
        }

        float altitude = internalLandingAltitude * Units.metersToFeet;
        bool altitudeCheck = altitude <= landingMode.flareAltitudeFt;
        bool thresholdCheck = internalLandingDistance < 0;

        if (CheckLandingAbort()) {
            AbortLandingToTakeoff();
            return;
        } else if (altitudeCheck || thresholdCheck) {
            landingMode.state = LandingModeState.LandingState.Flare;
        }
    }

    void HandleLandingFlare(float dt) {
        SetThrottleSpeedHold(dt, landingMode.approachSpeedKts);

        var yawRate = GetYawRate(plane);

        var targetHeading = CalculateCrossTrackTarget(dt, internalLandingHeading, internalLandingCrossTrack, crossTrackVelocity);

        var pitchInput = CalculatePitchClimbRate(dt, landingMode.flareVerticalSpeedFtPerMin);
        var rollInput = CalculateRollBank(dt, targetHeading);
        var yawInput = CalculateYawHeading(dt, internalHeading, internalLandingHeading, yawRate);

        var steering = new Vector3(pitchInput, yawInput, rollInput);
        SetControlInput(plane, steering);

        if (CheckLandingAbort()) {
            AbortLandingToTakeoff();
            return;
        } else if (plane.Grounded) {
            landingMode.state = LandingModeState.LandingState.Touchdown;
        }
    }

    void HandleLandingTouchdown(float dt) {
        plane.SetThrottleInput(-1); // apply brakes

        var yawRate = GetYawRate(plane);
        var yawInput = CalculateYawHeading(dt, internalHeading, internalLandingHeading, yawRate);

        var steering = new Vector3(0, yawInput, 0);
        SetControlInput(plane, steering);

        if (plane.LocalVelocity.z < 1) {
            state = AutopilotState.Idle;
        }
    }
}
