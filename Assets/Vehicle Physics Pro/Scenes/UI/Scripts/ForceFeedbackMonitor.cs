//--------------------------------------------------------------
//      Vehicle Physics Pro: advanced vehicle physics kit
//          Copyright © 2011-2019 Angel Garcia "Edy"
//        http://vehiclephysics.com | @VehiclePhysics
//--------------------------------------------------------------

// ForceFeedbackMonitor: displays live force feedback values


using UnityEngine;
using UnityEngine.UI;
using EdyCommonTools;


namespace VehiclePhysics.UI
{

public class ForceFeedbackMonitor : MonoBehaviour
	{
	public VehicleBase vehicle;

	// These bars will be controlled via Image.fillAmount

	public Image steeringForceBar;
	public Image steeringFrictionBar;

	public Color saturationColor = GColor.accentRed;
	}
}