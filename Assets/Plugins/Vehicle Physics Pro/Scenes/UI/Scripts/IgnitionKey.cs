//--------------------------------------------------------------
//      Vehicle Physics Pro: advanced vehicle physics kit
//          Copyright © 2011-2019 Angel Garcia "Edy"
//        http://vehiclephysics.com | @VehiclePhysics
//--------------------------------------------------------------

// IgnitionKey: UI for the ignition key


using UnityEngine;
using UnityEngine.UI;
using UnityEngine.EventSystems;
using EdyCommonTools;


namespace VehiclePhysics.UI
{

public class IgnitionKey : MonoBehaviour,
		IPointerDownHandler,
		IPointerUpHandler
	{
	public VehicleBase vehicle;

	public Text start;
	public Text accOn;
	public Text off;

    public Color normalColor = GColor.ParseColorHex("#999999");
	public Color highlightColor = Color.white;



	void OnEnable ()
		{
		if (vehicle != null)
			vehicle.onBeforeIntegrationStep += UpdateKeyInput;
		}


	void OnDisable ()
		{
		if (vehicle != null)
			vehicle.onBeforeIntegrationStep -= UpdateKeyInput;
		}


	// Listeners


	void UpdateKeyInput ()
		{
		if (vehicle == null) return;

		int key = vehicle.data.Get(Channel.Input, InputData.Key);

		SetHighlight(start, key == 1);
		SetHighlight(accOn, key == 0);
		SetHighlight(off, key == -1);
		}


	public void OnPointerDown (PointerEventData eventData)
		{
		if (eventData.button == PointerEventData.InputButton.Left)
			{
			GameObject pressed = eventData.pointerCurrentRaycast.gameObject;

			if (start != null && pressed == start.gameObject)
				// Debug.Log("Start");
				StartPressed();
			else
			if (accOn != null && pressed == accOn.gameObject)
				// Debug.Log("Acc On");
				AccOnPressed();
			else
			if (off != null && pressed == off.gameObject)
				// Debug.Log("Off");
				OffPressed();
			}
		}


	public void OnPointerUp (PointerEventData eventData)
		{
		ReleaseKey();
		}


	// Functionality


	public void StartPressed ()
		{
		if (vehicle == null) return;

		// If key was Off, move to Acc-On.
		// If it was Acc-On, move to Start.

		int key = vehicle.data.Get(Channel.Input, InputData.Key);

		if (key == -1)
			vehicle.data.Set(Channel.Input, InputData.Key, 0);
		else
		if (key == 0)
			vehicle.data.Set(Channel.Input, InputData.Key, 1);
		}


	public void AccOnPressed ()
		{
		if (vehicle == null) return;

		// Move key to Acc-On

		vehicle.data.Set(Channel.Input, InputData.Key, 0);
		}


	public void OffPressed ()
		{
		if (vehicle == null) return;

		// Move key to Off

		vehicle.data.Set(Channel.Input, InputData.Key, -1);
		}


	public void ReleaseKey ()
		{
		if (vehicle == null) return;

		// If Start was pressed, move back to Acc-On.

		int key = vehicle.data.Get(Channel.Input, InputData.Key);

		if (key == 1)
			vehicle.data.Set(Channel.Input, InputData.Key, 0);
		}


	// Utility


	void SetHighlight (Text text, bool highlight)
		{
		if (text != null)
			text.color = highlight? highlightColor : normalColor;
		}
	}

}
