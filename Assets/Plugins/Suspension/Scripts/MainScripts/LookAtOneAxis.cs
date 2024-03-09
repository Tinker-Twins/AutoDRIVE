using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// This script is needed to rotate object by only one axis
/// Works only 180 degrees (greater to zero or less if enabled Invertion)
/// </summary>
public class LookAtOneAxis : MonoBehaviour {

	[Header("Main settings")]
	[SerializeField] Transform TargetTransform;				//Target transform ref
	[SerializeField] Transform TransformClingingToTarget;	//The opposite side of the current object
	[SerializeField] Direction ForwardDirection;			//Direction
	[SerializeField] Vector3 OffsetPosition;				//Offset position, calculate from local space of TargetTransform
	[SerializeField] bool InvertionLookAtDirection;			//Invertion direction

	#region LookAt

	Transform RotateTransform;				//Transform for rotation
	Transform ParentTransform;				//Parent object that rotates along the x axis, created at the start
	Vector3 TargetPosition {
		get {
			return TargetTransform.TransformPoint(OffsetPosition);
		}
	}

	private void Awake () {

		if (TargetTransform == null) {
			Debug.LogError("targetTransform is null");
			enabled = false;
			return;
		}

		//Create parent object
		ParentTransform = new GameObject(string.Format("Parent_{0}", gameObject.name)).transform;
		ParentTransform.parent = transform.parent;
		ParentTransform.position = transform.position;
		ParentTransform.localRotation = Quaternion.identity;

		//Create rotation object
		RotateTransform = new GameObject(string.Format("RotateObject_{0}", gameObject.name)).transform;
		RotateTransform.parent = ParentTransform;
		RotateTransform.position = transform.position;
		transform.parent = RotateTransform;

		//Rotate the object to selected direction
		switch (ForwardDirection.direction) {
			case Direction.EnumDirection.Forward: { break; }
			case Direction.EnumDirection.Back: { transform.localRotation = Quaternion.AngleAxis(180, Vector3.up); break; }
			case Direction.EnumDirection.Right: { transform.localRotation = Quaternion.AngleAxis(-90, Vector3.up); break; }
			case Direction.EnumDirection.Left: { transform.localRotation = Quaternion.AngleAxis(-90, Vector3.up); break; }
			case Direction.EnumDirection.Up: { transform.localRotation = Quaternion.AngleAxis(-90, Vector3.left); break; }
			case Direction.EnumDirection.Down: { transform.localRotation = Quaternion.AngleAxis(90, Vector3.left); break; }
		}
	}

	private void LateUpdate () {
		Vector3 eulerAngles;

		//Find target position in local space with ignore local rotation
		Vector3 targetPosLocal = ParentTransform.InverseTransformPoint(TargetPosition);

		//Find rotation in the direction of targetPosLocal
		Quaternion quat = Quaternion.LookRotation(targetPosLocal);
		eulerAngles = quat.eulerAngles;
		if (InvertionLookAtDirection) {
			eulerAngles = - eulerAngles;
		}
		
		//Rotation alignment
		eulerAngles.z = 0;
		eulerAngles.y = 90;
		RotateTransform.localEulerAngles = eulerAngles;

		//Move opposite side to target point
		if (TransformClingingToTarget) {
			TransformClingingToTarget.position = TargetPosition;
		}
	}

	#endregion //LookAt

	private void OnDrawGizmosSelected () {
		Gizmos.color = Color.yellow;
		var direction = ForwardDirection.vector3;
		if (InvertionLookAtDirection) {
			direction = - direction;
		}
		Gizmos.DrawLine(transform.position, transform.TransformPoint(direction * 0.3f));
		Gizmos.DrawWireSphere(transform.position, 0.03f);

		Gizmos.color = Color.yellow;
		if (TargetTransform) {
			Gizmos.DrawWireSphere(TargetPosition, 0.03f);
		}

		Gizmos.color = Color.white;
	}

}
