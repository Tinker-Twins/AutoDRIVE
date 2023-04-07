//--------------------------------------------------------------
//      Vehicle Physics Pro: advanced vehicle physics kit
//          Copyright © 2011-2019 Angel Garcia "Edy"
//        http://vehiclephysics.com | @VehiclePhysics
//--------------------------------------------------------------

// GearModeSelector: Shows current gear mode and allows selecting it


using UnityEngine;
using UnityEngine.UI	;
using UnityEngine.EventSystems;
using EdyCommonTools;


namespace VehiclePhysics.UI
{

public class GearModeSelector : MonoBehaviour,
		IPointerDownHandler
	{
	public VehicleBase vehicle;

	public Color selectedColor = GColor.ParseColorHex("#E6E6E6");
	public Color unselectedColor = GColor.ParseColorHex("#999999");
	public Transform selector;

	[Header("Gear elements")]
	public Graphic gearM;
	public Graphic gearP;
	public Graphic gearR;
	public Graphic gearN;
	public Graphic gearD;
	public Graphic gearL;


	int m_prevGearMode = -1;
	int m_prevGearInput = -1;


	void OnEnable ()
		{
		m_prevGearMode = -1;
		m_prevGearInput = -1;

		if (vehicle != null)
			vehicle.onBeforeIntegrationStep += UpdateInputState;
		}


	void OnDisable ()
		{
		if (vehicle != null)
			vehicle.onBeforeIntegrationStep -= UpdateInputState;
		}


	void UpdateInputState ()
		{
		if (vehicle == null) return;

		int gearInput = vehicle.data.Get(Channel.Input, InputData.AutomaticGear);
		int gearMode = vehicle.data.Get(Channel.Vehicle, VehicleData.GearboxMode);

		if (gearMode != m_prevGearMode)
			{
			HighlightGear(gearMode);
			m_prevGearMode = gearMode;
			}

		if (gearInput != m_prevGearInput)
			{
			MoveGearSelector(gearInput);
			m_prevGearInput = gearInput;
			}
		}


	public void OnPointerDown (PointerEventData eventData)
		{
		if (vehicle == null || eventData.button != PointerEventData.InputButton.Left) return;

		GameObject pressed = eventData.pointerCurrentRaycast.gameObject;
		Graphic graphic = pressed.GetComponentInChildren<Graphic>();

		if (graphic != null)
			{
			int newGearMode = GraphicToGear(graphic);

			if (newGearMode >= 0)
				vehicle.data.Set(Channel.Input, InputData.AutomaticGear, newGearMode);
			}
		}


	void HighlightGear (int gearMode)
		{
		ClearEngagedGearMode();
		SetGraphicColor(GearToGraphic(gearMode), selectedColor);
		}


	void MoveGearSelector (int gearInput)
		{
		if (selector != null)
			{
			Graphic targetGraphic = GearToGraphic(gearInput);

			if (targetGraphic != null)
				{
				selector.position = targetGraphic.transform.position;
				}
			}
		}


	void ClearEngagedGearMode ()
		{
		SetGraphicColor(gearM, unselectedColor);
		SetGraphicColor(gearP, unselectedColor);
		SetGraphicColor(gearR, unselectedColor);
		SetGraphicColor(gearN, unselectedColor);
		SetGraphicColor(gearD, unselectedColor);
		SetGraphicColor(gearL, unselectedColor);
		}


	Graphic GearToGraphic (int gearMode)
		{
		switch (gearMode)
			{
			case 0: return gearM;
			case 1: return gearP;
			case 2: return gearR;
			case 3: return gearN;
			case 4: return gearD;
			case 5: return gearL;
			}

		return null;
		}


	int GraphicToGear (Graphic graphic)
		{
		if (graphic == gearM) return 0;
		if (graphic == gearP) return 1;
		if (graphic == gearR) return 2;
		if (graphic == gearN) return 3;
		if (graphic == gearD) return 4;
		if (graphic == gearL) return 5;

		return -1;
		}


	void SetGraphicColor (Graphic graphic, Color color)
		{
		if (graphic != null)
			graphic.color = color;
		}
	}

}