using System;
using UnityEngine;

public class CameraController : MonoBehaviour
{
	[Serializable]
	public class AdvancedOptions
	{
		public bool updateCameraInUpdate;
		public bool updateCameraInFixedUpdate = true;
		public bool updateCameraInLateUpdate;
		public KeyCode switchViewKey = KeyCode.C;
	}

	public CarController car;
	public GameObject instrumentCluster;
	public float speedFactor;
	public float basePositionValue;

	public float smoothing = 6f;
	public Transform lookAtTarget;
	public Transform positionTarget;
	public Transform DriverView;
	public AdvancedOptions advancedOptions;

	bool m_ShowingDriverView;

	private void FixedUpdate ()
	{
		if (advancedOptions.updateCameraInFixedUpdate)
			UpdateCamera ();
	}

	private void Update ()
	{
		if (Input.GetKeyDown (advancedOptions.switchViewKey))
			m_ShowingDriverView = !m_ShowingDriverView;

		if (advancedOptions.updateCameraInUpdate)
			UpdateCamera ();

		if (m_ShowingDriverView)
			instrumentCluster.SetActive(true);
		else
			instrumentCluster.SetActive(false);
	}

	private void LateUpdate ()
	{
		if (advancedOptions.updateCameraInLateUpdate)
			UpdateCamera ();
	}

	private void UpdateCamera ()
	{
		Vector3 newPos = positionTarget.localPosition;

		newPos.z = basePositionValue + (car.currSpeed / car.topSpeed) * speedFactor;
		positionTarget.localPosition = newPos;
		if (m_ShowingDriverView) {
			transform.position = DriverView.position;
			transform.rotation = DriverView.rotation;
		} else {
			transform.position = Vector3.Lerp (transform.position, positionTarget.position, Time.deltaTime * smoothing);
			transform.LookAt(lookAtTarget);
		}
	}
}
