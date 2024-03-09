using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// This script is needed to move the object to target point with offset
/// </summary>
public class SetPosition : MonoBehaviour {

	[SerializeField] Transform TargetTransform;
	[SerializeField] Vector3 OffsetPosition;

	Vector3 TargetPoint { get { return TargetTransform.TransformPoint(OffsetPosition); } }

	private void OnEnable () {
		if (TargetTransform == null) {
			Debug.LogError("targetTransform is null");
			enabled = false;
		}
	}

	private void LateUpdate () {
		transform.position = TargetPoint;
	}

	private void OnDrawGizmosSelected () {

		if (TargetTransform == null) return;

		Gizmos.color = Color.yellow;
		Gizmos.DrawWireSphere(transform.position, 0.03f);
		Gizmos.color = Color.green;
		Gizmos.DrawWireSphere(TargetPoint, 0.02f);
		Gizmos.DrawLine(transform.position, TargetPoint);
		Gizmos.color = Color.white;
	}
}
