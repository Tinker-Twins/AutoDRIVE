//--------------------------------------------------------------
//      Vehicle Physics Pro: advanced vehicle physics kit
//          Copyright © 2011-2019 Angel Garcia "Edy"
//        http://vehiclephysics.com | @VehiclePhysics
//--------------------------------------------------------------

// MenuOverlay: controls the main UI menu


using UnityEngine;
using UnityEngine.UI;
using UnityEngine.Events;
using UnityEngine.EventSystems;


namespace VehiclePhysics.UI
{

public class MenuOverlay : MonoBehaviour
	{
	public VehicleBase vehicle;

	[Header("Menu Toggles")]
	public Toggle telemetry;
	public Toggle charts;
	public Toggle setup;
	public Toggle input;
	public Toggle config;
	public Toggle help;

	[Header("UI")]
	public GameObject setupDialog;
	public GameObject inputDialog;
	public GameObject configDialog;
	public GameObject helpDialog;
    public TelemetryChartToolbar chartsToolbar;

	[Header("External UI")]
	public GameObject quickStart;
	public EscapeDialog escapeDialog;


	void OnEnable ()
		{
		AddListener(telemetry, OnTelemetry);
		AddListener(charts, OnCharts);
		AddListener(setup, OnSetup);
		AddListener(input, OnInput);
		AddListener(config, OnConfig);
		AddListener(help, OnHelp);

		OnTelemetry(false);
		OnCharts(false);
		OnSetup(false);
		OnInput(false);
		OnConfig(false);
		OnHelp(false);
		}


	void OnDisable ()
		{
		RemoveListener(telemetry, OnTelemetry);
		RemoveListener(charts, OnCharts);
		RemoveListener(setup, OnSetup);
		RemoveListener(input, OnInput);
		RemoveListener(config, OnConfig);
		RemoveListener(help, OnHelp);
		}


	void Start ()
		{
		// Show Quick Start once on application startup

		if (quickStart != null && vehicle != null && vehicle.isActiveAndEnabled)
			quickStart.SetActive(true);
		}


	void Update ()
		{
		// Some elements may close themselves. Detect it here.

		MonitorAndDisableToggle(charts, chartsToolbar);
		MonitorAndDisableToggle(setup, setupDialog);
		MonitorAndDisableToggle(input, inputDialog);
		MonitorAndDisableToggle(config, configDialog);
		MonitorAndDisableToggle(help, helpDialog);

		// Hide Quick Start

		if (quickStart != null && quickStart.activeSelf)
			{
			if (Input.GetMouseButtonDown(0)
				|| vehicle != null && vehicle.data.Get(Channel.Vehicle, VehicleData.EngineWorking) != 0)
				{
				quickStart.SetActive(false);
				}
			}

		// Show Escape dialog. There should be no other dialog active.

		if (escapeDialog != null && !escapeDialog.gameObject.activeSelf
			&& Input.GetKeyDown(escapeDialog.escapeKey)
			&& !IsEnabled(quickStart)
			&& !IsEnabled(setupDialog)
			&& !IsEnabled(inputDialog)
			&& !IsEnabled(configDialog)
			&& !IsEnabled(helpDialog)
			)
			{
			escapeDialog.gameObject.SetActive(true);
			}
		}


	// Menu methods
	// Clicking any of them hides the escape dialog


	void OnTelemetry (bool value)
		{
		if (vehicle != null)
			{
			VPTelemetry telemetryComp = vehicle.GetComponentInChildren<VPTelemetry>();

			if (telemetryComp != null)
				telemetryComp.showData = value;
			}

		HideEscapeDialog();
		}


	void OnCharts (bool value)
		{
		SetEnabled(chartsToolbar, false);
		HideEscapeDialog();
		}


	void OnSetup (bool value)
		{
		SetEnabled(setupDialog, value);
		HideEscapeDialog();
		}


	void OnInput (bool value)
		{
		SetEnabled(inputDialog, value);
		HideEscapeDialog();
		}


	void OnConfig (bool value)
		{
		SetEnabled(configDialog, value);
		HideEscapeDialog();
		}


	void OnHelp (bool value)
		{
		SetEnabled(helpDialog, value);
		HideEscapeDialog();
		}


	void HideEscapeDialog ()
		{
		if (escapeDialog != null)
			escapeDialog.gameObject.SetActive(false);
		}


	// Helper methods


	void AddListener (Toggle toggle, UnityAction<bool> call)
		{
		if (toggle != null) toggle.onValueChanged.AddListener(call);
		}


	void RemoveListener (Toggle toggle, UnityAction<bool> call)
		{
		if (toggle != null) toggle.onValueChanged.RemoveListener(call);
		}


	bool IsEnabled (Toggle toggle)
		{
		return toggle != null? toggle.isOn : false;
		}


	void MonitorAndDisableToggle (Toggle toggle, GameObject go)
		{
		if (toggle != null)
			{
			if (go != null && toggle.isOn && go.activeSelf == false)
				toggle.isOn = false;
			}
		}


	void MonitorAndDisableToggle (Toggle toggle, Component comp)
		{
		if (toggle != null)
			{
			if (comp != null && toggle.isOn && comp.gameObject.activeSelf == false)
				toggle.isOn = false;
			}
		}


	VPVehicleController GetVPVehicleController ()
		{
		if (vehicle == null) return null;
		return vehicle.GetComponent<VPVehicleController>();
		}


	void SetEnabled (GameObject go, bool enabled)
		{
		if (go != null) go.SetActive(enabled);
		}


	void SetEnabled (Component comp, bool enabled)
		{
		if (comp != null) comp.gameObject.SetActive(enabled);
		}


	bool IsEnabled (GameObject go)
		{
		return go != null && go.activeSelf;
		}
	}
}