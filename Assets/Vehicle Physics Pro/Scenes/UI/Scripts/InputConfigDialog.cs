//--------------------------------------------------------------
//      Vehicle Physics Pro: advanced vehicle physics kit
//          Copyright © 2011-2019 Angel Garcia "Edy"
//        http://vehiclephysics.com | @VehiclePhysics
//--------------------------------------------------------------

// SettingsDialog: a dialog for configuring vehicle settings
//
// Assumes the VPStandardInput component is the only one active in the vehicle.


using UnityEngine;
using UnityEngine.UI;
using UnityEngine.Events;
using EdyCommonTools;
using System;


namespace VehiclePhysics.UI
{

public class InputConfigDialog : MonoBehaviour
	{
	public VehicleBase vehicle;

	// Monitor toggles only. Panels are shown/hidden via GenericMenu

	[Header("Menu Toggles")]
	public Toggle keyboard;
	public Toggle xbox;
	public Toggle wheel;

	[Header("Xbox")]
	public Toggle xbox1;
	public Toggle xbox2;
	public Toggle xbox3;
	public Toggle xbox4;
	public Slider xboxNonLinearity;
	public Slider xboxDeadZone;

	[Header("Wheel")]
	public Toggle device1;
	public Toggle device2;
	public Toggle device3;
	public Toggle device4;
	public Dropdown deviceModel;
	public Toggle deviceClutchPresent;
	public Toggle deviceClutchAbsent;
	public Slider deviceThrottle;
	public Slider deviceBrake;
	public Slider deviceClutch;
	public Slider deviceIntensity;
	public Slider deviceLinearity;
	public Slider deviceFriction;
	public Toggle deviceUIShow;
	public Toggle deviceUIHide;

	[Header("UI")]
	public Text messageBox;
	public Color defaultMessageColor = Color.white;
	public Color warningMessageColor = GColor.orangeA100;
	public Button deviceDebugInfo;

	[Header("External")]
	public GameObject deviceDebugInfoPanel;


	void Awake ()
		{
		// First time this GameObject is enabled: clear the method options.
		// They will be configured from the vehicle in OnEnable.

		SetEnabled(keyboard, false);
		SetEnabled(xbox, false);
		SetEnabled(wheel, false);
		}


	void Start ()
		{
		// WATCHDOG

		// In some builds the Keyboard panel remains enabled
		// after resetting the scene with Wheel enabled.
		//
		// Looks like an Unity bug. Tracing the execution reads keyboard.isOn = true
		// right after setting keyboard.isOn = false, in both Awake and OnDisable.
		// Maybe something related with the order of initialization in the Unity UI scripts?
		//
		// Never happens on first run, only after resetting the scene with Esc
		// (SceneManager.LoadScene).

		// So we explicitly disable the Keyboard panel here if it should be disabled.

		if (IsEnabled(xbox) || IsEnabled(wheel))
			SetEnabled(keyboard, false);

		// Additionally, when this happens then "SetEnabled(keyboard, true)" in OnEnable
		// won't trigger the EnableKeyboard call, as it's already on.

		// So we explicitly call EnableKeyboard here if Keyboard should be enabled.

		if (IsEnabled(keyboard))
			EnableKeyboard();
		}


	void OnEnable ()
		{
		// Input methods

		AddListener(keyboard, OnKeyboard);
		AddListener(xbox, OnXbox);
		AddListener(wheel, OnWheel);

		// Initialize the UI with the values from the components.
		// Must be done before adding the remaining listeners, otherwise they would get invoked.
		// Method option listeners are enabled so current method is properly initialized.

		InitializeXboxUI();
		InitializeWheelUI();
		if (!IsEnabled(xbox) && !IsEnabled(wheel))
			SetEnabled(keyboard, true);

		// Changing Xbox device requires restart the input

		AddListener(xbox1, OnXbox);
		AddListener(xbox2, OnXbox);
		AddListener(xbox3, OnXbox);
		AddListener(xbox4, OnXbox);

		// Xbox settings don't require a restart

		AddListener(xboxNonLinearity, UpdateXboxSettings);
		AddListener(xboxDeadZone, UpdateXboxSettings);

		// Changing Wheel device requires restart the input

		AddListener(device1, OnWheel);
		AddListener(device2, OnWheel);
		AddListener(device3, OnWheel);
		AddListener(device4, OnWheel);

		// Changing Wheel parameters don't require a restart

		AddListener(deviceModel, UpdateWheelSettings);
		AddListener(deviceClutchPresent, delegate { UpdateWheelSettings(); });
		AddListener(deviceThrottle, UpdateWheelSettings);
		AddListener(deviceBrake, UpdateWheelSettings);
		AddListener(deviceClutch, UpdateWheelSettings);
		AddListener(deviceIntensity, UpdateWheelSettings);
		AddListener(deviceLinearity, UpdateWheelSettings);
		AddListener(deviceFriction, UpdateWheelSettings);
		AddListener(deviceUIShow, delegate { UpdateWheelSettings(); });

		// Toggle debug info button

		if (deviceDebugInfo != null)
			deviceDebugInfo.onClick.AddListener(OnDeviceDebugInfo);
		}


	void OnDisable ()
		{
		RemoveListener(keyboard, OnKeyboard);
		RemoveListener(xbox, OnXbox);
		RemoveListener(wheel, OnWheel);

		RemoveListener(xbox1, OnXbox);
		RemoveListener(xbox2, OnXbox);
		RemoveListener(xbox3, OnXbox);
		RemoveListener(xbox4, OnXbox);

		RemoveListener(xboxNonLinearity, UpdateXboxSettings);
		RemoveListener(xboxDeadZone, UpdateXboxSettings);

		RemoveListener(device1, OnWheel);
		RemoveListener(device2, OnWheel);
		RemoveListener(device3, OnWheel);
		RemoveListener(device4, OnWheel);

		RemoveListener(deviceModel, UpdateWheelSettings);
		RemoveListener(deviceClutchPresent, delegate { UpdateWheelSettings(); });
		RemoveListener(deviceThrottle, UpdateWheelSettings);
		RemoveListener(deviceBrake, UpdateWheelSettings);
		RemoveListener(deviceClutch, UpdateWheelSettings);
		RemoveListener(deviceIntensity, UpdateWheelSettings);
		RemoveListener(deviceLinearity, UpdateWheelSettings);
		RemoveListener(deviceFriction, UpdateWheelSettings);
		RemoveListener(deviceUIShow, delegate { UpdateWheelSettings(); });

		if (deviceDebugInfo != null)
			deviceDebugInfo.onClick.RemoveListener(OnDeviceDebugInfo);
		}


	// Listeners


	void OnKeyboard (bool value)
		{
		if (value)
			EnableKeyboard();
		}


	void OnXbox (bool value)
		{
		if (value)
			EnableXbox();
		}


	void OnWheel (bool value)
		{
		if (value)
			EnableWheel();
		}


	void OnDeviceDebugInfo ()
		{
		ToggleDebugInfoPanel();
		}


	// Keyboard


	void EnableKeyboard ()
		{
		SetEnabled(typeof(VPStandardInput), true);
		HideDebugInfoPanel();

		SetMessage("");
		}


	// Xbox


	void EnableXbox ()
		{
		if (vehicle == null) return;

		EnableKeyboard();
		SetMessage("Xbox Gamepad support not available.\nUsing keyboard.", warningMessageColor);
		}


	void UpdateXboxSettings ()
		{
		if (vehicle == null) return;
		}


	void InitializeXboxUI ()
		{
		if (vehicle == null) return;
		}


	// Wheel


	void EnableWheel ()
		{
		if (vehicle == null) return;

		EnableKeyboard();
		SetMessage("Wheel device support not available.\nUsing keyboard.", warningMessageColor);
		}


	void UpdateWheelSettings ()
		{
		if (vehicle == null) return;
		}


	void InitializeWheelUI ()
		{
		if (vehicle == null) return;
		}


	void ToggleDebugInfoPanel ()
		{
		if (vehicle == null) return;
		}


	void HideDebugInfoPanel ()
		{
		if (deviceDebugInfoPanel != null)
			deviceDebugInfoPanel.SetActive(false);
		}


	// Helper methods


	void SetMessage (string text)
		{
		SetMessage(text, defaultMessageColor);
		}


	void SetMessage (string text, Color color)
		{
		if (messageBox != null)
			{
			messageBox.text = text;
			messageBox.color = color;
			}
		}


	void AddListener (Toggle toggle, UnityAction<bool> call)
		{
		if (toggle != null) toggle.onValueChanged.AddListener(call);
		}


	void RemoveListener (Toggle toggle, UnityAction<bool> call)
		{
		if (toggle != null) toggle.onValueChanged.RemoveListener(call);
		}

	void AddListener (Dropdown dropdown, UnityAction call)
		{
		if (dropdown != null) dropdown.onValueChanged.AddListener(delegate { call(); });
		}


	void RemoveListener (Dropdown dropdown, UnityAction call)
		{
		if (dropdown != null) dropdown.onValueChanged.RemoveListener(delegate { call(); });
		}


	void AddListener (Slider slider, UnityAction call)
		{
		if (slider != null) slider.onValueChanged.AddListener(delegate { call(); });
		}


	void RemoveListener (Slider slider, UnityAction call)
		{
		if (slider != null) slider.onValueChanged.RemoveListener(delegate { call(); });
		}


	void SetEnabled (Toggle toggle, bool enabled)
		{
		if (toggle != null) toggle.isOn = enabled;
		}


	bool IsEnabled (Toggle toggle)
		{
		return toggle != null? toggle.isOn : false;
		}


	void SetEnabled (Type type, bool enabled)
		{
		if (vehicle == null) return;

		MonoBehaviour comp = vehicle.GetComponentInChildren(type) as MonoBehaviour;
		if (comp != null) comp.enabled = enabled;
		}


	bool IsAvailable (Type type)
		{
		if (vehicle == null) return false;

		MonoBehaviour comp = vehicle.GetComponentInChildren(type) as MonoBehaviour;
		return comp != null;
		}
	}
}