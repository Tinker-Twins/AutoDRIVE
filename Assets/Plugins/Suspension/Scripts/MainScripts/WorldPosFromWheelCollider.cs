using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// This script is needed to change position and rotatin from WheelCollider
/// </summary>
public class WorldPosFromWheelCollider : MonoBehaviour {

	[SerializeField] WheelCollider WheelCollider;
	[SerializeField] bool SetPosition;
	[SerializeField] bool SetRotation;
	[SerializeField] Vector3 OffsetPosition;

	Vector3 Position;
	Quaternion Rotation;

	private void LateUpdate () {
		WheelCollider.GetWorldPose(out Position, out Rotation);

		if (SetPosition) {
			transform.position = Position;
			transform.localPosition += OffsetPosition;
		}

		if (SetRotation) {
			transform.rotation = Rotation;
		}
	}
}
