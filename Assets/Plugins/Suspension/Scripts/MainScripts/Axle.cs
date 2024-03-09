using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// This script is needed to change the position and rotation the axle
/// </summary>
public class Axle : MonoBehaviour {

	[SerializeField] WheelCollider LeftWheelCollider;				//Left WheelCollider ref
	[SerializeField] WheelCollider RightWheelCollider;				//Right WheelCollider ref
	[SerializeField] Transform LeftWheelView;						//Left wheel view, to rotate the transform
	[SerializeField] Transform RightWheelView;						//Right wheel view, to rotate the transform
	[SerializeField] float AngleMiltiplier = 45;					//Rotation multiplier

	Vector3 LeftWheelPosition;		//For getting position from LeftWheelCollider
	Vector3 RightWheelPosition;		//For getting position from RightWheelCollider
	Transform TransformHelper;		//For save transform at the start
	float Distance;					//Distance between wheels, for the angle calculation formula

	private void Awake () {
		//Saving transform on start
		TransformHelper = new GameObject("AxleTransformHelper").transform;
		TransformHelper.SetParent(transform.parent);
		TransformHelper.position = transform.position;

		Quaternion rot;
		LeftWheelCollider.GetWorldPose(out LeftWheelPosition, out rot);
		RightWheelCollider.GetWorldPose(out RightWheelPosition, out rot);

		//Find distance between wheels
		Distance = Vector3.Distance(LeftWheelPosition, RightWheelPosition);
	}

	private void LateUpdate () {
		//Get wheels position
		Quaternion rot;

		LeftWheelCollider.GetWorldPose(out LeftWheelPosition, out rot);
		RightWheelCollider.GetWorldPose(out RightWheelPosition, out rot);

		LeftWheelPosition = TransformHelper.transform.InverseTransformPoint(LeftWheelPosition);
		RightWheelPosition = TransformHelper.transform.InverseTransformPoint(RightWheelPosition);

		//Calculate axle pos, position is considered the midpoint between the wheels
		Vector3 newAxlePos = TransformHelper.localPosition;
		newAxlePos.y += (LeftWheelPosition.y + RightWheelPosition.y) * 0.5f;
		transform.localPosition = newAxlePos;

		//Calculate axle rotation, the angle is calculated from the height difference of the wheels in local space
        float angle = (LeftWheelPosition.y > RightWheelPosition.y)? -AngleMiltiplier : AngleMiltiplier;
        angle = Mathf.Abs(LeftWheelPosition.y - RightWheelPosition.y) / Distance * angle;
		rot = Quaternion.AngleAxis(angle, Vector3.forward);

		transform.localRotation = rot;

		//Wheels rotation assignment, from WheelColliders
		LeftWheelView.localRotation *= Quaternion.AngleAxis(LeftWheelCollider.rpm * 6 * Time.deltaTime, Vector3.right);
		RightWheelView.localRotation *= Quaternion.AngleAxis(RightWheelCollider.rpm * 6 * Time.deltaTime, Vector3.right);
	}
}
