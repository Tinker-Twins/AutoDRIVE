//--------------------------------------------------------------
//      Vehicle Physics Pro: advanced vehicle physics kit
//          Copyright © 2011-2019 Angel Garcia "Edy"
//        http://vehiclephysics.com | @VehiclePhysics
//--------------------------------------------------------------

using UnityEngine;


namespace VehiclePhysics.Utility
{

[AddComponentMenu("Vehicle Physics/Utility/Follow Heading", 21)]
public class FollowHeading : VehicleBehaviour
	{
	public float heading = 0.0f;			// Degrees, 0 = 360 = "north" (World +Z)
	public float maxSteerRate = 90.0f; 		// Degrees per second

	public bool zigzag = false;
	public float maxAngle = 45.0f;
	public float transitionTime = 2.0f;
	public float waitTime = 1.0f;

	VPVehicleController m_vehicle;
	VPStandardInput m_input;


	public override void OnEnableVehicle ()
		{
		m_vehicle = vehicle.GetComponent<VPVehicleController>();
		m_input = vehicle.GetComponent<VPStandardInput>();
		}


	public override void OnDisableVehicle ()
		{
		m_input.externalSteer = 0.0f;
		}


	public override void UpdateVehicle ()
		{
		float offset = 0.0f;

		if (zigzag)
			{
			float period = transitionTime*2.0f + waitTime*2.0f;
			float t = Mathf.Repeat(Time.time, period);

			if (t < waitTime)
				{
				offset = -maxAngle;
				}
			else
			if (t < waitTime + transitionTime)
				{
				float pos = (t - waitTime) / transitionTime;
				offset = Mathf.Lerp(-maxAngle, +maxAngle, pos);
				}
			else
			if (t < waitTime*2.0f + transitionTime)
				{
				offset = +maxAngle;
				}
			else
				{
				float pos = (t - waitTime*2 - transitionTime) / transitionTime;
				offset = Mathf.Lerp(+maxAngle, -maxAngle, pos);
				}
			}

		float deltaAngle = Mathf.DeltaAngle(vehicle.cachedTransform.eulerAngles.y, heading + offset);

		float targetSteer = Mathf.Clamp(deltaAngle / m_vehicle.steering.maxSteerAngle, -1.0f, +1.0f);
		float targetRate = maxSteerRate / m_vehicle.steering.maxSteerAngle;

		m_input.externalSteer = Mathf.MoveTowards(m_input.externalSteer, targetSteer, targetRate * Time.deltaTime);
		}
	}

}