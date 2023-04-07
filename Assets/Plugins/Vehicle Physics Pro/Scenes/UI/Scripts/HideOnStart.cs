//--------------------------------------------------------------
//      Vehicle Physics Pro: advanced vehicle physics kit
//          Copyright © 2011-2019 Angel Garcia "Edy"
//        http://vehiclephysics.com | @VehiclePhysics
//--------------------------------------------------------------

// Just hides the GameObject on first run.
// Useful for panels that are visible in the Editor but should start hidden.


using UnityEngine;


namespace VehiclePhysics.UI
{

public class HideOnStart : MonoBehaviour
	{
	void Start ()
		{
		this.gameObject.SetActive(false);
		}
	}
}