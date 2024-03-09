using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// This script is needed to demonstrate this asset.
/// </summary>
[RequireComponent(typeof(Rigidbody))]
public class CarController : MonoBehaviour {

	[SerializeField] float MaxMotorTorque = 1500;
	[SerializeField] float MaxBrakeTorque = 500;
	[SerializeField] float AccelerationTorque = 1f;
	[SerializeField] float AccelerationBrakeTorque = 0.5f;
	[SerializeField] float AccelerationSteer = 10f;
	[SerializeField] GameObject COM;
	[SerializeField] List<WheelPreset> DrivingWheels = new List<WheelPreset>();
	[SerializeField] List<WheelPreset> SteeringWheels = new List<WheelPreset>();

	Rigidbody RB;
	HashSet<WheelPreset> AllWheels = new HashSet<WheelPreset>();
	float CurrentAcceleration;
	float CurrentBrake;
	float CurrentSteer;

	public bool Enable { get; set; }

	private void Awake () {
		Enable = false;
		RB = GetComponent<Rigidbody>();
		RB.centerOfMass = COM.transform.localPosition;
		RB.ResetInertiaTensor();
		foreach (var wheel in SteeringWheels) {
			AllWheels.Add(wheel);
		}
		foreach (var wheel in DrivingWheels) {
			AllWheels.Add(wheel);
		}
		
	}

	private void Update () {
		 
		float targetAcceleration = Input.GetAxis("Vertical");
		float targetSteer = Input.GetAxis("Horizontal");

		if (Input.GetButton("Jump") || !Enable) {
			CurrentAcceleration = 0;
			CurrentBrake = Mathf.MoveTowards(CurrentBrake, 1, AccelerationBrakeTorque * Time.deltaTime);
		} else {
			CurrentAcceleration = Mathf.MoveTowards(CurrentAcceleration, targetAcceleration, AccelerationTorque * Time.deltaTime);
			CurrentBrake = 0;
		}

		if (Enable) {
			CurrentSteer = Mathf.MoveTowards(CurrentSteer, targetSteer, AccelerationSteer * Time.deltaTime);
		}
	}

	private void FixedUpdate () {
		WheelCollider wheelCollider;
		for (int i = 0; i < DrivingWheels.Count; i++) {
			wheelCollider = DrivingWheels[i].WheelCollider;
			wheelCollider.motorTorque = CurrentAcceleration * MaxMotorTorque;
			wheelCollider.brakeTorque = DrivingWheels[i].BrakeTorque * CurrentBrake * MaxBrakeTorque;
		}

		for (int i = 0; i < SteeringWheels.Count; i++) {
			wheelCollider = SteeringWheels[i].WheelCollider;
			wheelCollider.steerAngle = CurrentSteer * SteeringWheels[i].SteerAngle;
			wheelCollider.brakeTorque = DrivingWheels[i].BrakeTorque * CurrentBrake * MaxBrakeTorque;
		}
	}

	[System.Serializable]
	private class WheelPreset {
		public WheelCollider WheelCollider;
		public float BrakeTorque = 1;
		public float SteerAngle = 25;
	}

}
