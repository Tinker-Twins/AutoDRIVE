//--------------------------------------------------------------
//      Vehicle Physics Pro: advanced vehicle physics kit
//          Copyright © 2011-2019 Angel Garcia "Edy"
//        http://vehiclephysics.com | @VehiclePhysics
//--------------------------------------------------------------

// DevideDebugInfo: displays the raw values received from the device in runtime


using UnityEngine;
using UnityEngine.UI;
using EdyCommonTools;


namespace VehiclePhysics.UI
{

public class DeviceDebugInfo : MonoBehaviour
	{
	public VehicleBase vehicle;

	public Text axis1;
	public Text axis2;
	public Text pov1;
	public Text pov2;
	public Text buttons;
	}
}