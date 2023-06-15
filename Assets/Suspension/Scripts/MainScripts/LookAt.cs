using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// This script is needed to look an object while maintaining rotation along the z axis
/// </summary>
public class LookAt : MonoBehaviour {

	[SerializeField] Direction Direction;		//Direction
	[SerializeField] Transform Target;			//Target object ref
	[SerializeField] Vector3 Offset;			//Offset position, calculate from local space of Target
	[SerializeField] bool WithChangeScale;		//When enabled, the z-axis scale will change

	Vector3 TargetPoint { get { return Target.TransformPoint(Offset); } }

	Transform RotateTransform;			//Parent object that rotates along the z axis, created at the start
	float StartDistance;				//Distance to target object at the start

	float GetDistance { get { return (RotateTransform.transform.position - TargetPoint).magnitude; } }
	private void Awake () {
		//Create parent object
		RotateTransform = new GameObject("AxleTransformHelper").transform;
		RotateTransform.SetParent(transform.parent);
		RotateTransform.position = transform.position;
		transform.SetParent(RotateTransform);

		//Rotate the object in the selected direction
		switch (Direction.direction) {
			case Direction.EnumDirection.Forward: break;
			case Direction.EnumDirection.Back: transform.localRotation = Quaternion.AngleAxis(180f, Vector3.up);  break;
			case Direction.EnumDirection.Left: transform.localRotation = Quaternion.AngleAxis(-90f, Vector3.up); break;
			case Direction.EnumDirection.Right: transform.localRotation = Quaternion.AngleAxis(90f, Vector3.up); break;
			case Direction.EnumDirection.Up: transform.localRotation = Quaternion.AngleAxis(-90f, Vector3.left); break;
			case Direction.EnumDirection.Down: transform.localRotation = Quaternion.AngleAxis(90f, Vector3.left); break;
		}

		//Calcelate distance between this transform and target transform
		if (WithChangeScale) {
			StartDistance = GetDistance;
		}
	}

	private void LateUpdate () {
		//Save rotation along the z axis
		var oldEulerZ = RotateTransform.localEulerAngles.z;

		//Rotate parent tranfom to target
		RotateTransform.localRotation = Quaternion.identity;
		Vector3 targetPosLocal = RotateTransform.InverseTransformPoint(TargetPoint);

		//Find rotation in the direction of targetPosLocal
		Quaternion quat = Quaternion.LookRotation(targetPosLocal);
		RotateTransform.localRotation = quat;

		//RotateTransform.LookAt(TargetPoint);
		RotateTransform.SetLocalEulerZ(oldEulerZ);

		//Calculate new scale along the z axis
		if (WithChangeScale) {
			var newScale = GetDistance / StartDistance;
			RotateTransform.SetLocalScaleZ(newScale);
		}
	}

	private void OnDrawGizmosSelected () {
		Gizmos.color = Color.yellow;

		if (RotateTransform) {
			Gizmos.DrawLine(RotateTransform.position, RotateTransform.TransformPoint(Vector3.forward * 0.3f));
		} else {
			Gizmos.DrawLine(transform.position, transform.TransformPoint(Direction.vector3 * 0.3f));
		}

		Gizmos.DrawWireSphere(transform.position, 0.05f);

		Gizmos.color = Color.green;

		if (Target) {
			Gizmos.DrawWireSphere(TargetPoint, 0.02f);
		}

		Gizmos.color = Color.white;
	}
}
