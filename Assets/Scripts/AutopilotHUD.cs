using System.Collections;
using System.Collections.Generic;
using System.Text;
using UnityEngine;
using UnityEngine.UI;

public class AutopilotHUD : MonoBehaviour {
    [SerializeField]
    Plane plane;
    [SerializeField]
    AutopilotController autopilot;
    [SerializeField]
    Text infoText;

    StringBuilder builder;

    void Start() {
        builder = new StringBuilder();
    }

    void Update() {
        builder.Clear();
        builder.AppendLine(string.Format("Mode: {0}", autopilot.State));

        float pitch = plane.PitchYawRoll.x;
        builder.AppendLine(string.Format("Pitch: {0:N2}", pitch));

        float agl = plane.RadarAltimeter * Units.metersToFeet;
        builder.AppendLine(string.Format("AGL: {0:N0}", agl));

        float climbRate = plane.Rigidbody.velocity.y * Units.metersToFeet * 60;
        builder.AppendLine(string.Format("Climb rate: {0}", (int)Mathf.Round(climbRate)));

        infoText.text = builder.ToString();
    }

    public void OnSwitchTakeoff() {
        autopilot.EnterTakeoffMode();
    }

    public void OnStartTakeoff() {
        autopilot.StartTakeoff();
    }
}
