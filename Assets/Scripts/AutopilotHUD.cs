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
    [SerializeField]
    float updateInterval;

    StringBuilder builder;

    void Start() {
        builder = new StringBuilder();
    }

    void Update() {
        builder.Clear();
        autopilot.WriteDebugString(builder);
        infoText.text = builder.ToString();
    }

    public void OnSwitchTakeoff() {
        autopilot.EnterTakeoffMode();
    }

    public void OnSwitchNavigate() {
        autopilot.EnterNavigateMode();
    }

    public void OnSwitchLanding() {
        autopilot.EnterLandingMode();
    }
}
