//--------------------------------------------------------------
//      Vehicle Physics Pro: advanced vehicle physics kit
//          Copyright © 2011-2019 Angel Garcia "Edy"
//        http://vehiclephysics.com | @VehiclePhysics
//--------------------------------------------------------------

using UnityEngine;


namespace VehiclePhysics
{

public class PauseVehicle : VehicleBehaviour
	{
	public bool pause = false;
	public KeyCode key = KeyCode.P;


	bool m_pausedState = false;
	Vector3 m_velocity = Vector3.zero;
	Vector3 m_angularVelocity = Vector3.zero;


	// Must use Update and FixedUpdate because UpdateVehicle and FixedUpdateVehicle
	// aren't called when the vehicle is Paused.


	void FixedUpdate ()
		{
		if (!vehicle.initialized) return;

		if (pause && !m_pausedState)
			{
			m_velocity = vehicle.cachedRigidbody.velocity;
			m_angularVelocity = vehicle.cachedRigidbody.angularVelocity;

			m_pausedState = true;
			vehicle.cachedRigidbody.isKinematic = true;
			vehicle.paused = true;
			}
		else
		if (!pause && m_pausedState)
			{
			vehicle.cachedRigidbody.isKinematic = false;
            vehicle.paused = false;

			vehicle.cachedRigidbody.velocity = m_velocity;
			vehicle.cachedRigidbody.angularVelocity = m_angularVelocity;

			m_pausedState = false;
			}
		}


	void Update ()
		{
		if (Input.GetKeyDown(key)) pause = !pause;
		}
	}

}