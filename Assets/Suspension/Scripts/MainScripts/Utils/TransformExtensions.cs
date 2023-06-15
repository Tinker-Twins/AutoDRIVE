using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public static class TransformExtensions {

	//Global position
	public static void SetPosX (this Transform transform, float x) {
		var p = transform.position;
		p.x = x;
		transform.position = p;
	}
	public static void SetPosY (this Transform transform, float y) {
		var p = transform.position;
		p.y = y;
		transform.position = p;
	}
	public static void SetPosZ (this Transform transform, float z) {
		var p = transform.position;
		p.z = z;
		transform.position = p;
	}

	//LocalPosition
	public static void SetLocalX (this Transform transform, float x) {
		var p = transform.localPosition;
		p.x = x;
		transform.localPosition = p;
	}
	public static void SetLocalY (this Transform transform, float y) {
		var p = transform.localPosition;
		p.y = y;
		transform.localPosition = p;
	}
	public static void SetLocalZ (this Transform transform, float z) {
		var p = transform.localPosition;
		p.z = z;
		transform.localPosition = p;
	}

	//GlobalEulerAngels
	public static void SetEulerX (this Transform transform, float x) {
		var p = transform.eulerAngles;
		p.x = x;
		transform.eulerAngles = p;
	}
	public static void SetEulerY (this Transform transform, float y) {
		var p = transform.eulerAngles;
		p.y = y;
		transform.eulerAngles = p;
	}
	public static void SetEulerZ (this Transform transform, float z) {
		var p = transform.eulerAngles;
		p.z = z;
		transform.eulerAngles = p;
	}

	//LocalEulerAngels
	public static void SetLocalEulerX (this Transform transform, float x) {
		var p = transform.localEulerAngles;
		p.x = x;
		transform.localEulerAngles = p;
	}
	public static void SetLocalEulerY (this Transform transform, float y) {
		var p = transform.localEulerAngles;
		p.y = y;
		transform.localEulerAngles = p;
	}
	public static void SetLocalEulerZ (this Transform transform, float z) {
		var p = transform.localEulerAngles;
		p.z = z;
		transform.localEulerAngles = p;
	}

	//LocalScale
	public static void SetLocalScaleX (this Transform transform, float x) {
		var p = transform.localScale;
		p.x = x;
		transform.localScale = p;
	}
	public static void SetLocalScaleY (this Transform transform, float y) {
		var p = transform.localScale;
		p.y = y;
		transform.localScale = p;
	}
	public static void SetLocalScaleZ (this Transform transform, float z) {
		var p = transform.localScale;
		p.z = z;
		transform.localScale = p;
	}
	
}
