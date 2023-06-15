using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// This script is needed to move the wheel in an arc
/// </summary>
public class ArcMoveFromWheelCollider : MonoBehaviour {

	[SerializeField] WheelCollider WheelCollider;				//WheelCollider ref
	[SerializeField] Vector3 OffsetPosition;					//Offset position, calculate from local space of WheelCollider
	[SerializeField] float ArmLength;							//Arm length, radius
	[SerializeField, Range(0f, 1f)] float MaxLengthOnDistance;  //The position of the wheel in which the arm is horizontal
	[SerializeField] bool Invertion;							//Invertion direction

	Vector3 _CenterArcPoint;									//Point the center of the circle
	Vector3 CenterArcPoint {
		get {
			if (_CenterArcPoint != Vector3.zero) {
				return _CenterArcPoint;
			} else {
				return (OffsetPosition + (Vector3.left * (Invertion ? ArmLength : -ArmLength)));
			}
		}
	}

	Vector3 Position;				//For getting position from WheelCollider
	Quaternion Rotation;			//For getting rotation from WheelCollider

	private void Awake () {
		//Saving center position on start
		_CenterArcPoint = CenterArcPoint;
	}

	private void LateUpdate () {

		// Get position and rotation feom WheelCollider
		WheelCollider.GetWorldPose(out Position, out Rotation);

		//Position translation to local space
		Position = WheelCollider.transform.InverseTransformPoint(Position) + OffsetPosition;
		Position -= CenterArcPoint;

		//Find Y of wheel position
		var offsetY = Position.y - MaxLengthOnDistance * WheelCollider.suspensionDistance;

		if (offsetY < ArmLength && offsetY > -ArmLength) {
			//Calcelate X, According to the formula X*X + Y*Y = R*R
			Position.x = OffsetPosition.x + Mathf.Sqrt((ArmLength * ArmLength) - (offsetY * offsetY));
		}
		if (Invertion) Position.x = -Position.x;
		transform.localPosition = Position;
	}

	List<Vector3> GizmoPoints = new List<Vector3>();
	float GizmoArmLength;
	float GizmoSuspensionDistance;
	float GizmoMaxLengthOnDistance;

	void CalculateGizmoPoints () {
		GizmoPoints.Clear();
		var centerArcWheelColliderPoint = Vector3.left * (Invertion? -ArmLength: ArmLength);
		Vector3 point;
		for (int i = 0; i < (int)(WheelCollider.suspensionDistance * 100); i++) {
			point = new Vector3(0, ((float)i / 100f) - (WheelCollider.suspensionDistance / 2), 0);
			var offsetY = point.y - MaxLengthOnDistance * (WheelCollider.suspensionDistance) + (WheelCollider.suspensionDistance / 2);
			point -= centerArcWheelColliderPoint;
			if (offsetY < ArmLength && offsetY > -ArmLength) {
				point.x = Mathf.Sqrt((ArmLength * ArmLength) - (offsetY * offsetY));
			}
			if (Invertion) point.x = -point.x;
			GizmoPoints.Add(point);
		}
		GizmoArmLength = ArmLength;
		GizmoSuspensionDistance = WheelCollider.suspensionDistance;
		GizmoMaxLengthOnDistance = MaxLengthOnDistance;
	}
	private void OnDrawGizmosSelected () {

		if (WheelCollider == null) return;

		var centerArcWheelColliderPoint = Vector3.left * (Invertion? -ArmLength: ArmLength);

		if (GizmoPoints.Count == 0 ||
		!Mathf.Approximately(GizmoArmLength, ArmLength)||
		!Mathf.Approximately(GizmoSuspensionDistance, WheelCollider.suspensionDistance) ||
		!Mathf.Approximately(GizmoMaxLengthOnDistance, MaxLengthOnDistance))
		{
			CalculateGizmoPoints ();
		}

		Gizmos.color = Color.green;
		Vector3 prevPoint = GizmoPoints[0];
		for (int i = 1; i < GizmoPoints.Count; i++) {
			Gizmos.DrawLine(WheelCollider.transform.TransformPoint(centerArcWheelColliderPoint + prevPoint), WheelCollider.transform.TransformPoint(centerArcWheelColliderPoint + GizmoPoints[i]));
			prevPoint = GizmoPoints[i];
		}
		Gizmos.DrawLine(WheelCollider.transform.TransformPoint(centerArcWheelColliderPoint), WheelCollider.transform.TransformPoint(centerArcWheelColliderPoint + (GizmoPoints[0])));
		Gizmos.DrawLine(WheelCollider.transform.TransformPoint(centerArcWheelColliderPoint), WheelCollider.transform.TransformPoint(centerArcWheelColliderPoint + GizmoPoints[GizmoPoints.Count - 1]));

		Gizmos.color = Color.yellow;

		Gizmos.DrawWireSphere(WheelCollider.transform.TransformPoint(centerArcWheelColliderPoint), 0.02f);

		Gizmos.color = Color.white;
	}
}
