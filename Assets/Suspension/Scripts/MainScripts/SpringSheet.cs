using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;

/// <summary>
/// This script is needed to change the geometry of the suspension sheets.
/// </summary>
[RequireComponent(typeof(MeshFilter))]
public class SpringSheet : MonoBehaviour {

	[SerializeField] Transform TargetTransform;									//Target transform ref
	[SerializeField] Vector3 OffsetTargetPosition;								//Offset position
	[SerializeField, Range(0.01f, 1f)] float MaxZDistBetweenVertices = 0.1f;	//The maximum distance on the z axis between the vertices in the group

	[SerializeField, HideInInspector]Vector3 StartTargetPosition;				//For saving TargetPosition at the start? in local space, сhange if there are changes in the settings

	List<Vector3> Vertices = new List<Vector3>();								//All vertices from mesh
	MeshFilter MeshFilter;
	List<VerticesGroup> VerticesGroups = new List<VerticesGroup>();

	Mesh Mesh {
		get {
			return MeshFilter.mesh;
		}
	}
	Vector3 TargetPosition {
		get {
			return TargetTransform.TransformPoint(OffsetTargetPosition);
		}
	}

	Vector3 DiffTargetPosition {
		get {
			return TargetPositionInLocalSpace - StartTargetPosition;
		}
	}

	Vector3 TargetPositionInLocalSpace {
		get {
			return transform.InverseTransformPoint(TargetPosition);
		}
	}

	//Invoke if there are changes in the settings
	public void UpdateStartPosition () {
		StartTargetPosition = transform.InverseTransformPoint(TargetTransform.position + OffsetTargetPosition);
	}

	void Start () {
		MeshFilter = GetComponent<MeshFilter>();
		MeshFilter.mesh = MeshFilter.sharedMesh;
		InitGroupingVertices();
	}

	//Create vertices groups
	void InitGroupingVertices () {

		VerticesGroups.Clear();
		Vertices.Clear();
		Vertices.AddRange(Mesh.vertices);

		//tempListVertices need for create VerticesGroups
		List<Vector3> tempListVertices = new List<Vector3>();
		
		tempListVertices.AddRange(Vertices);
		tempListVertices = tempListVertices.OrderBy(v => v.z).ToList();
		
		//Moving from the first vertex to the last
		while (tempListVertices.Count > 0) {

			//Find vertices in max distance by z
			float findDistance = tempListVertices.First().z + MaxZDistBetweenVertices;
			List<Vector3> groupVertices = new List<Vector3>();
			List<int> verticesIndexList = new List<int>();
			
			foreach (var vert in tempListVertices) {
				if (vert.z < findDistance || Mathf.Approximately(findDistance, vert.z)) {
					groupVertices.Add(vert);
				}
			}

			//Remove from temp list vertices found
			tempListVertices.RemoveAll(v => groupVertices.Contains(v));

			//create VerticesGroups
			VerticesGroups.Add(
				new VerticesGroup(
					Mesh,
					groupVertices
				)
			);
		}

		CalculateProcentDeformation();

		for (int i = 0; i < Mesh.vertices.Length; i++) {
			VerticesGroup nearestVg = VerticesGroups.First();
			float minDistance = (Mesh.vertices[i] - nearestVg.StartCenterPoint).sqrMagnitude;
			foreach (var vg in VerticesGroups) {
				float distance = (Mesh.vertices[i] - vg.StartCenterPoint).sqrMagnitude;
				if (distance < minDistance) {
					minDistance = distance;
					nearestVg = vg;
				}
			}
			nearestVg.AddVertex(i, Mesh.vertices[i]);
		}
	}

	/// <summary>
	/// Calculation of the percent change in geometry.
	/// The sum of the vertices, the farther from the target point, the less it is deformed.
	/// Max distance is 0% deformed.
	/// Min distance is 100% deformed.
	/// </summary>
	void CalculateProcentDeformation () {
		//Distance to far point
		float maxDistance = 0;

		//Find max distance
		for (int i = 0; i < VerticesGroups.Count; i++) {
			VerticesGroups[i].DistanceToTargetPoint = Vector3.Distance(VerticesGroups[i].StartCenterPoint, TargetPositionInLocalSpace);
			if (maxDistance < VerticesGroups[i].DistanceToTargetPoint) {
				maxDistance = VerticesGroups[i].DistanceToTargetPoint;
			}
		}
		for (int i = 0; i < VerticesGroups.Count; i++) {
			VerticesGroups[i].ProcentDeformstion = 1 - VerticesGroups[i].DistanceToTargetPoint / maxDistance;
		}
	}

	private void LateUpdate () {
		UpdateMesh ();
	}

	/// <summary>
	/// Applying change in geometry of object
	/// </summary>
	void UpdateMesh () {
		for (int i = 0; i < VerticesGroups.Count; i++) {
			var vg = VerticesGroups[i];
			var diff = DiffTargetPosition * vg.ProcentDeformstion;
			vg.CurrentCenterPoint = Vector3.zero;
			foreach (var kv in vg.VerticesCenterOffset) {
				var pos = vg.StartCenterPoint + kv.Value + diff;
				vg.CurrentCenterPoint += pos;
				Vertices[kv.Key] = pos;
			}
			vg.CurrentCenterPoint /= vg.VerticesCenterOffset.Count;
		}
		Mesh.SetVertices(Vertices);
		Mesh.RecalculateNormals();
		Mesh.RecalculateTangents();
	}

	protected virtual void OnDrawGizmosSelected () {

		Gizmos.color = Color.yellow;

		Gizmos.DrawWireSphere(transform.position, 0.03f);

		Gizmos.color = Color.yellow;
		if (TargetTransform) {
			Gizmos.DrawLine(transform.position, TargetPosition);
			Gizmos.DrawWireSphere(TargetPosition, 0.03f);
		}

		for (int i = 0; i < VerticesGroups.Count; i++) {
			Gizmos.DrawWireSphere(transform.TransformPoint(VerticesGroups[i].CurrentCenterPoint), 0.03f);
		}

		if (Vertices != null && Vertices.Count > 0) {
			Gizmos.color = Color.green;
			foreach (var v in Vertices) {
				Gizmos.DrawWireSphere(transform.TransformPoint(v), 0.005f);
			}
		}

		Gizmos.color = Color.white;
	}

	private class VerticesGroup {
		public Dictionary<int, Vector3> VerticesCenterOffset { get; private set; } 
		public Vector3 StartCenterPoint { get; private set; }
		public Vector3 CurrentCenterPoint { get; set; }
		public float DistanceToTargetPoint { get; set; }
		public float ProcentDeformstion { get; set; }

		public VerticesGroup (Mesh deformingMesh, List<Vector3> vertices) {

			StartCenterPoint = Vector3.zero;

			var verticesIndexHashSet = new HashSet<int>();
			VerticesCenterOffset = new Dictionary<int, Vector3>();

			foreach (var vert in vertices) {
				StartCenterPoint += vert;
			}
			StartCenterPoint /= vertices.Count;
		}

		public void AddVertex (int index, Vector3 vertex) {
			VerticesCenterOffset.Add(index, vertex - StartCenterPoint);
		}
	}
}
