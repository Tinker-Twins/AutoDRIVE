//--------------------------------------------------------------
//      Vehicle Physics Pro: advanced vehicle physics kit
//          Copyright © 2011-2019 Angel Garcia "Edy"
//        http://vehiclephysics.com | @VehiclePhysics
//--------------------------------------------------------------

// ConfigDialog: a dialog for configuring the demo settings


using UnityEngine;
using UnityEngine.UI;
using UnityEngine.Events;
using EdyCommonTools;
using System;
using System.IO;


namespace VehiclePhysics.UI
{

public class ConfigDialog : MonoBehaviour
	{
	public VehicleBase vehicle;

	[Header("Camera")]
	public Slider cameraFov;
	public Toggle headMotionOn;
	public Toggle headMotionOff;

	[Header("Motion Platform")]
	public Toggle motionPlatformEnabled;
	public Toggle motionPlatformDisabled;

	public Dropdown motionPlatformProtocol;
	public InputField motionSimtoolsHost;
	public InputField motionSimtoolsPort;
	public Button motionDboxPanelLink;

	[Header("External")]
	public Camera driverCamera;

	// Components that will be disabled and restored when inputfields are focused

	public Behaviour[] inputComponents;


	bool m_isTextFocused = false;
	bool[] m_inputStates = new bool[0];


	void OnEnable ()
		{
		// Camera

		InitializeUI();
		AddListener(cameraFov, UpdateConfiguration);
		}


	void OnDisable ()
		{
		// Listeners

		RemoveListener(cameraFov, UpdateConfiguration);

		// Input

		if (m_isTextFocused)
			RestoreInputComponents();
		}


	void Update ()
		{
		// Disable car input when any input field is focused

		bool textFocused = motionSimtoolsHost != null && motionSimtoolsHost.isFocused
			|| motionSimtoolsPort != null && motionSimtoolsPort.isFocused;

		if (textFocused != m_isTextFocused)
			{
			if (textFocused)
				SaveAndDisableInputComponents();
			else
				RestoreInputComponents();

			m_isTextFocused = textFocused;
			}
		}


	// Camera


	void InitializeUI ()
		{
		if (driverCamera != null)
			{
			if (cameraFov != null) cameraFov.value = driverCamera.fieldOfView;
			}
		}


	void UpdateConfiguration ()
		{
		if (driverCamera != null)
			{
			if (cameraFov != null) driverCamera.fieldOfView = cameraFov.value;
			}
		}


	// States of the input components


	void SaveAndDisableInputComponents ()
		{
		m_inputStates = new bool[inputComponents.Length];

        for (int i = 0; i < inputComponents.Length; i++)
			{
			m_inputStates[i] = inputComponents[i].enabled;
			inputComponents[i].enabled = false;
			}
		}


	void RestoreInputComponents ()
		{
		if (m_inputStates.Length != inputComponents.Length)
			{
			Debug.LogWarning(this.name + ": inconsistency between saved states and input component list. States not restored.");
			return;
			}

		for (int i = 0; i < inputComponents.Length; i++)
			inputComponents[i].enabled = m_inputStates[i];
		}


	// Utility


	void AddListener (Toggle toggle, UnityAction<bool> call)
		{
		if (toggle != null) toggle.onValueChanged.AddListener(call);
		}


	void RemoveListener (Toggle toggle, UnityAction<bool> call)
		{
		if (toggle != null) toggle.onValueChanged.RemoveListener(call);
		}

	void AddListener (Toggle toggle, UnityAction call)
		{
		if (toggle != null) toggle.onValueChanged.AddListener(delegate { call(); });
		}

	void RemoveListener (Toggle toggle, UnityAction call)
		{
		if (toggle != null) toggle.onValueChanged.RemoveListener(delegate { call(); });
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


	void SetInteractable (Selectable field, bool interactable)
		{
		if (field != null) field.interactable = interactable;
		}
	}

}