using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// This script is needed to move the object based on the angle of angle of the WheelCollider
/// </summary>
public class MoveBySteerAngle : MonoBehaviour {

	[SerializeField] WheelCollider WheelCollider;		//WheelCollider ref
	[SerializeField] Direction Direction;				//Move direction
	[SerializeField] float Mulriplier;					//Move multiplier
	
	Vector3 StartLocalPosition;

	private void Awake () {
		StartLocalPosition = transform.localPosition;
	}

	void LateUpdate () {
		transform.localPosition = StartLocalPosition + (Direction.vector3 * WheelCollider.steerAngle * Mulriplier);
	}
}
