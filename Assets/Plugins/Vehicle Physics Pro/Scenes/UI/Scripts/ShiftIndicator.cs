//--------------------------------------------------------------
//      Vehicle Physics Pro: advanced vehicle physics kit
//          Copyright © 2011-2019 Angel Garcia "Edy"
//        http://vehiclephysics.com | @VehiclePhysics
//--------------------------------------------------------------

// ShiftIndicator: Shows the optimal point for shift up/down


using UnityEngine;
using UnityEngine.UI;
using EdyCommonTools;


namespace VehiclePhysics.UI
{

public class ShiftIndicator : MonoBehaviour
	{
	// Requires a VPVehicleController component, which exposes the method.

	public VPVehicleController vehicleController;

	[Space(5)]
	public float upshiftMinThreshold = 0.7f;
	public float downshiftMinThreshold = 1.0f;

	[Header("UI")]
	public Text text;
	public Color defaultColor = Color.white;


    void Update()
		{
		if (vehicleController == null) return;

		if (text != null)
			{
			float shiftPoint = vehicleController.GetOptimalGearShiftRatio();

			if (shiftPoint >= upshiftMinThreshold)
				{
				text.text = shiftPoint.ToString("0.00");

				if (shiftPoint >= 1.0f)
					text.color = GColor.accentGreen;
				else
					text.color = defaultColor;
				}
			else
			if (shiftPoint <= -downshiftMinThreshold)
				{
				text.text = (-shiftPoint).ToString("0.00");
				text.color = GColor.accentRed;
				}
			else
				{
				text.color = defaultColor;
				text.text = "";
				}
			}
		}
	}

}