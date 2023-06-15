using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

/// <summary>
/// This script is needed to rotate object from WheelCollider.rpm,
/// and LookAt Object rotation (optional)
/// </summary>
public class SetRotationByRPM: MonoBehaviour {

	[Header("Main settings")]
	[SerializeField] WheelCollider WheelCollider;		//WheelCollider ref
	[SerializeField] float RotationMultiplier = 1;		//Rotate multiplier
	[SerializeField] Axis AxisRotate;					//Axis of rotation

	[Header("Target object settings")]
	[SerializeField] Transform TargetTransform;			//Target transform (may be null)
	[SerializeField] Direction TargetDirection;			//Target direction need for rotate object to selected direction at the start

	Vector3 _Axis;										//Selected axis
	Func<float> SetRotate;								//SetRotate func, in order not to check the chosen direction every frame
	Transform RotateTransform;							//Transform for rotation
	float LerpPrm;

	private void Awake () {
		if (WheelCollider == null) {
			Debug.LogError("wheelCollider is null");
			enabled = false;
		}

		//Rotate the object to selected direction
		if (TargetTransform) {
			RotateTransform = new GameObject("AxleTransformHelper").transform;
			RotateTransform.SetParent(transform.parent);
			RotateTransform.position = transform.position;
			RotateTransform.rotation = transform.rotation;
			transform.SetParent(RotateTransform);
			switch (TargetDirection.direction) {
				case Direction.EnumDirection.Forward: break;
				case Direction.EnumDirection.Back: transform.localRotation = Quaternion.AngleAxis(180f, Vector3.up);  break;
				case Direction.EnumDirection.Left: transform.localRotation = Quaternion.AngleAxis(-90f, Vector3.up); break;
				case Direction.EnumDirection.Right: transform.localRotation = Quaternion.AngleAxis(90f, Vector3.up); break;
				case Direction.EnumDirection.Up: transform.localRotation = Quaternion.AngleAxis(-90f, Vector3.left); break;
				case Direction.EnumDirection.Down: transform.localRotation = Quaternion.AngleAxis(90f, Vector3.left); break;
			}
		} else {
			RotateTransform = transform;
		}

		//Save selected Axis, and RotateTransform LookAt method params
		switch (AxisRotate) {
			case Axis.X:
				_Axis = Vector3.left;
				SetRotate = () => {
					var oldRotate = RotateTransform.localEulerAngles.x;
					RotateTransform.LookAt(TargetTransform);
					RotateTransform.SetLocalEulerX(oldRotate);
					return oldRotate;
				};
				break;
			case Axis.Y:
				_Axis = Vector3.up;
				SetRotate = () => {
					var oldRotate = RotateTransform.localEulerAngles.y;
					RotateTransform.LookAt(TargetTransform);
					RotateTransform.SetLocalEulerY(oldRotate);
					return oldRotate;
				};
				break;
			case Axis.Z:
				_Axis = Vector3.forward;
				SetRotate = () => {
					var oldRotate = RotateTransform.localEulerAngles.z;
					RotateTransform.LookAt(TargetTransform);
					RotateTransform.SetLocalEulerZ(oldRotate);
					return oldRotate;
				};
				break;
		}
	}

	private void LateUpdate () {
		//Invoke LookAt method
		if (TargetTransform != null) {
			SetRotate();
		}

		//Raotate RotateTransform
		LerpPrm = Mathf.MoveTowardsAngle(LerpPrm, WheelCollider.rpm, 5f);
		float rotate = LerpPrm * RotationMultiplier * Time.deltaTime;
		RotateTransform.localRotation *= Quaternion.AngleAxis(rotate, _Axis);
	}
}
