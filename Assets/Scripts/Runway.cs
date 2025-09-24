using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Runway : MonoBehaviour {
    [SerializeField]
    float length;

    public Vector3 Direction { get; private set; }
    public Vector3 End1 { get; private set; }
    public Vector3 End2 { get; private set; }

    public struct TouchdownData {
        public Vector3 position;
        public Vector3 direction;
    }

    void Awake() {
        var transform = GetComponent<Transform>();
        Direction = transform.forward;

        var halfLength = length / 2;
        End1 = transform.position + Direction * halfLength;
        End2 = transform.position - Direction * halfLength;
    }

    public TouchdownData GetClosestTouchdown(Vector3 position) {
        var dist1 = (End1 - position).sqrMagnitude;
        var dist2 = (End2 - position).sqrMagnitude;

        if (dist1 < dist2) {
            return new TouchdownData { position = End1, direction = -Direction };
        } else {
            return new TouchdownData { position = End2, direction = Direction };
        }
    }
}
