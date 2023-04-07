//--------------------------------------------------------------
//      Vehicle Physics Pro: advanced vehicle physics kit
//          Copyright © 2011-2019 Angel Garcia "Edy"
//        http://vehiclephysics.com | @VehiclePhysics
//--------------------------------------------------------------

// Toggle: an UI switch for a feature that may be disabled.


using System;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.Events;
using UnityEngine.EventSystems;
using EdyCommonTools;


namespace VehiclePhysics.UI
{

public class MultiToggle : MonoBehaviour,
		IPointerEnterHandler,
		IPointerExitHandler,
		IPointerDownHandler
	{
    public enum State { Absent, Disabled, Enabled, Active };
	public State state = State.Absent;

	// The bifferent parts of the toggle button should be in the same level
	// of the hierarchy. Only the "Button" part needs Raycast Enabled.

	[Space(5)]
	public Image toggleButton;
	public Image disabledImage;
	public Image statusIcon;
	public Text statusText;
	public Text hintText;

	[Space(5)]
	public Color disabledColor = GColor.ParseColorHex("#595959");
	public Color enabledColor = GColor.ParseColorHex("#1B5E20");
	public Color activeColor = GColor.ParseColorHex("#E4D235");
	public Color textColor = GColor.ParseColorHex("#E6E6E6");

	[Space(5)]
	public string absentCaption = "N/A";
	public string onCaption = "ON";
	public string offCaption = "OFF";
	public string hint;

	// Event to register to the toggle calls (runtime)

	[NonSerialized, HideInInspector]
	public UnityEvent onClick = new UnityEvent();


	State m_prevState;


	void OnEnable ()
		{
		// Ensure first update is applied

		m_prevState = (State)(-1);
		}


	void Update ()
		{
		// Check for state changes

		if (state == m_prevState) return;
		m_prevState = state;

		// Update state

		switch (state)
			{
			case State.Absent:
				// Icon: gray
				// Caption: N/A, gray
				// Background: hide
				// Button: hide

				SetImageColor(statusIcon, disabledColor);
				SetText(statusText, absentCaption, disabledColor);
				ShowImage(disabledImage, false);
				ShowImage(toggleButton, false);
				break;

			case State.Disabled:
				// Icon: gray
				// Caption: OFF, white
				// Background: show
				// Button: show

				SetImageColor(statusIcon, disabledColor);
				SetText(statusText, offCaption, textColor);
				ShowImage(disabledImage, true);
				ShowImage(toggleButton, true);
				break;

			case State.Enabled:
				// Icon: green
				// Caption: ON, white
				// Background: hide
				// Button: show

				SetImageColor(statusIcon, enabledColor);
				SetText(statusText, onCaption, textColor);
				ShowImage(disabledImage, false);
				ShowImage(toggleButton, true);
				break;

			case State.Active:
				// Icon: yellow
				// Caption: ON, white
				// Background: hide
				// Button: show

				SetImageColor(statusIcon, activeColor);
				SetText(statusText, onCaption, textColor);
				ShowImage(disabledImage, false);
				ShowImage(toggleButton, true);
				break;
			}
		}


	// Here the script receives the events in all the children UI elements
	// that have Raycast Target enabled.


	public void OnPointerDown (PointerEventData eventData)
		{
		// eventData.pointerPress references this script's GameObject.
		// eventData.pointerEnter may contain the target's GameObject or null.
		//		NOTE: pointerEnter will be NULL if a click is detected before an OnEnter,
		//		i.e. just when gaining focus with the click.
		//		Use eventData.pointerCurrentRaycast.gameObject instead.
		//
		// eventData.pointerCurrentRaycast.gameObject is target's GameObject.
		//
		// Anyways, we pass the event along if any UI belonging to this toggle is clicked.

		onClick.Invoke();
		// SetText(hintText, "Clicked: " + this.gameObject.name);
		}


	public void OnPointerEnter (PointerEventData eventData)
		{
		if (!string.IsNullOrEmpty(hint))
			SetText(hintText, hint);
		}


	public void OnPointerExit (PointerEventData eventData)
		{
		if (!string.IsNullOrEmpty(hint))
			SetText(hintText, "");
		}


	// Safe methods for modifying UI images


	void SetText (Text text, string value)
		{
		if (text != null) text.text = value;
		}


	void SetText (Text text, string value, Color color)
		{
		if (text != null)
			{
			text.text = value;
			text.color = color;
			}
		}


	void SetImageColor (Image image, Color color)
		{
		if (image != null) image.color = color;
		}


	void ShowImage (Image image, bool show)
		{
		if (image != null) image.gameObject.SetActive(show);
		}
	}
}