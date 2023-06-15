using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ViewPoints : MonoBehaviour {

	[SerializeField] List<Vector3> ViewVectors = new List<Vector3>();
	[SerializeField] List<Transform> ViewTransforms = new List<Transform>();

	List<Vector3> AllPoints = new List<Vector3>();

	int CurrentPointIndex;

	public Vector3 GetCurrentPoint { get { return transform.TransformPoint(AllPoints[CurrentPointIndex]); } }

	private void Awake () {
		AllPoints.AddRange(ViewVectors);
		foreach (var t in ViewTransforms) {
			t.SetParent(transform);
			AllPoints.Add(t.localPosition);
		}
		if (AllPoints.Count == 0) {
			Debug.LogError("Points is empty");
		}
		CurrentPointIndex = 0;
	}

	public void SettNextPoint () {
		CurrentPointIndex++;
		if (CurrentPointIndex >= AllPoints.Count) {
			CurrentPointIndex = 0;
		}
	}

	private void OnDrawGizmosSelected () {
		Gizmos.color = Color.blue;

		foreach (var point in ViewVectors) {
			Gizmos.DrawWireSphere(transform.TransformPoint(point), 0.25f);
		}
		foreach (var point in ViewTransforms) {
			Gizmos.DrawWireSphere(transform.TransformPoint(point.localPosition), 0.25f);
		}

		Gizmos.color = Color.white;
	}
}
