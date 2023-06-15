using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraController : MonoBehaviour {

	[SerializeField] float MoveSpeed = 25f;
	[SerializeField] float RotateSpeed = 0.2f;
	[SerializeField] float ZoomSpeed = 10f;

	Vector2 MousePos;
	ViewPoints ViewPoints;

	Vector3 TargetPoint {
		get {
			if (!ViewPoints) {
				return Vector3.zero;
			}
			return ViewPoints.GetCurrentPoint;
		}
	}

	public void SetViewPoints (ViewPoints vp) {
		ViewPoints = vp;
	}

	void LateUpdate () {
		transform.position = Vector3.Lerp(transform.position, TargetPoint, Time.unscaledDeltaTime * MoveSpeed);

		if (Input.GetKey(KeyCode.Mouse0)) {
			UpdateMouse();
		} else {
			MousePos = Vector2.zero;
		}

		if (Input.mouseScrollDelta.y > 0) {
			transform.GetChild(0).SetLocalZ(Mathf.MoveTowards(transform.GetChild(0).localPosition.z, 0, Time.unscaledDeltaTime * ZoomSpeed));
		} else if (Input.mouseScrollDelta.y < 0) {
			transform.GetChild(0).SetLocalZ(Mathf.MoveTowards(transform.GetChild(0).localPosition.z, -5, Time.unscaledDeltaTime * ZoomSpeed));
		}

		if (Input.touchCount > 0) {
			UpdateTouch();
		}
	}

	void UpdateMouse () {
		if (MousePos == Vector2.zero) {
			MousePos = Input.mousePosition;
		}
		Vector2 delta = (Vector2)Input.mousePosition - MousePos;
		MousePos = (Vector2)Input.mousePosition;
		Rotate(delta);
	}

	void UpdateTouch () {
		Vector2 delta = (Vector2)Input.touches[0].deltaPosition;
		Rotate(delta);
	}

	void Rotate (Vector2 delta) {
		var currentEulerZ = transform.eulerAngles.z;
		var rotate = Quaternion.AngleAxis(delta.x, Vector3.up);
		float eulerX = transform.eulerAngles.x;
		if (eulerX < 0 || eulerX > 180) {
			eulerX -= 360;
		}
		if (!(delta.y > 0 && eulerX < -85 || delta.y < 0 && eulerX > 85)) {
			rotate *= Quaternion.AngleAxis(delta.y, Vector3.left);
		}
		transform.rotation *= rotate;

		transform.SetEulerZ(currentEulerZ);
	}
}
