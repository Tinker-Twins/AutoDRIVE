//--------------------------------------------------------------
//      Vehicle Physics Pro: advanced vehicle physics kit
//          Copyright © 2011-2019 Angel Garcia "Edy"
//        http://vehiclephysics.com | @VehiclePhysics
//--------------------------------------------------------------

// SettingsDialog: a dialog for configuring vehicle settings


using UnityEngine;
using UnityEngine.UI;
using UnityEngine.Events;
using UnityEngine.EventSystems;


namespace VehiclePhysics.UI
{

public class VehicleSetupDialog : MonoBehaviour,
		IPointerEnterHandler,
		IPointerExitHandler
	{
	public VehicleBase vehicle;
	public Text helpText;
	public string noHelpText;

	[Header("Settings Toggles")]
	public Toggle autoShift;
	public Toggle drivetrain2wd;
	public Toggle differentialLockAxle;
	public Toggle differentialLockCenter;
	public Toggle differentialLockFull;
	public Toggle automaticTransmission;
	public Toggle torqueConverter;
	public Toggle engineStall;


	void OnEnable ()
		{
		AddListener(autoShift, OnAutoShift);
		AddListener(drivetrain2wd, OnDrivetrain2wd);
		AddListener(differentialLockAxle, OnAxleDifferentialLock);
		AddListener(differentialLockCenter, OnCenterDifferentialLock);
		AddListener(differentialLockFull, OnFullDifferentialLock);
		AddListener(automaticTransmission, OnAutomaticTransmission);
		AddListener(torqueConverter, OnTorqueConverter);
		AddListener(engineStall, OnEngineStall);
		if (helpText != null) helpText.text = noHelpText;
		}


	void OnDisable ()
		{
		RemoveListener(autoShift, OnAutoShift);
		RemoveListener(drivetrain2wd, OnDrivetrain2wd);
		RemoveListener(differentialLockAxle, OnAxleDifferentialLock);
		RemoveListener(differentialLockCenter, OnCenterDifferentialLock);
		RemoveListener(differentialLockFull, OnFullDifferentialLock);
		RemoveListener(automaticTransmission, OnAutomaticTransmission);
		RemoveListener(torqueConverter, OnTorqueConverter);
		RemoveListener(engineStall, OnEngineStall);
		}


	// Help text


	public void OnPointerEnter (PointerEventData eventData)
		{
		/*
		Currently handled via local HoverHandler. Some components may intercept this event
		so these don't fall back here.

		NOTE: pointerEnter will be NULL if a click is detected before an OnEnter,
			i.e. just when gaining focus with the click.
			Use eventData.pointerCurrentRaycast.gameObject instead.

		if (helpText != null)
			{
			Hint hint = eventData.pointerEnter.GetComponentInParent<Hint>();

			if (hint != null)
				{
				if (!string.IsNullOrEmpty(hint.shortText))
					helpText.text = hint.shortText + ": " + hint.longDescription;
				else
					helpText.text = hint.longDescription;
				}
			}
		*/
		}


	public void OnPointerExit (PointerEventData eventData)
		{
		if (helpText != null) helpText.text = noHelpText;
		}


	// Settings methods


	void OnAutoShift (bool value)
		{
		// No effect on automatic transmissions

		if (IsEnabled(automaticTransmission)) return;

		if (value)
			SetVehicleSetting(SettingsData.AutoShiftOverride, 1);
		else
			SetVehicleSetting(SettingsData.AutoShiftOverride, 2);
		}


	void OnDrivetrain2wd (bool value)
		{
		if (value)
			{
			SetVehicleSetting(SettingsData.DrivelineLock, 2);

			if (IsEnabled(differentialLockCenter))
				SetVehicleSetting(SettingsData.DifferentialLock, 1);
			}
		else
			{
			if (IsEnabled(differentialLockFull))
				{
				SetVehicleSetting(SettingsData.DrivelineLock, 1);
				}
			else
			if (IsEnabled(differentialLockCenter))
				{
				SetVehicleSetting(SettingsData.DrivelineLock, 1);
				SetVehicleSetting(SettingsData.DifferentialLock, 0);
				}
			else
				{
				SetVehicleSetting(SettingsData.DrivelineLock, 0);
				}
			}
		}


	void OnAxleDifferentialLock (bool value)
		{
		SetVehicleSetting(SettingsData.DifferentialLock, value? 1 : 0);

		if (!IsEnabled(drivetrain2wd))
			SetVehicleSetting(SettingsData.DrivelineLock, 0);
		}


	void OnCenterDifferentialLock (bool value)
		{
		if (IsEnabled(drivetrain2wd))
			{
			SetVehicleSetting(SettingsData.DifferentialLock, value? 1 : 0);
			}
		else
			{
			SetVehicleSetting(SettingsData.DrivelineLock, value? 1 : 0);
			SetVehicleSetting(SettingsData.DifferentialLock, 0);
			}
		}


	void OnFullDifferentialLock (bool value)
		{
		SetVehicleSetting(SettingsData.DifferentialLock, value? 1 : 0);

		if (!IsEnabled(drivetrain2wd))
			SetVehicleSetting(SettingsData.DrivelineLock, value? 1 : 0);
		}


	// VPVehicleController specific


	void OnAutomaticTransmission (bool value)
		{
		VPVehicleController vehicleController = GetVPVehicleController();

		if (vehicleController != null)
			{
			if (value)
				{
				// Automatic: no auto-shift overrides, initial state D

				vehicleController.gearbox.type = Gearbox.Type.Automatic;
				SetVehicleSetting(SettingsData.AutoShiftOverride, 0);
				SetVehicleInput(InputData.AutomaticGear, 4);
				}
			else
				{
				// Manual: auto-shift based on setting

				vehicleController.gearbox.type = Gearbox.Type.Manual;
				OnAutoShift(IsEnabled(autoShift));
				}
			}
		}


	void OnTorqueConverter (bool value)
		{
		VPVehicleController vehicleController = GetVPVehicleController();

		if (vehicleController != null)
			vehicleController.clutch.type = value? Engine.ClutchType.TorqueConverterLimited : Engine.ClutchType.FrictionDisc;
		}


	void OnEngineStall (bool value)
		{
		VPVehicleController vehicleController = GetVPVehicleController();

		if (vehicleController != null)
			vehicleController.engine.canStall = value;
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


	void SetVehicleSetting (int setting, int value)
		{
		if (vehicle != null)
			vehicle.data.Set(Channel.Settings, setting, value);
		}


	void SetVehicleInput (int input, int value)
		{
		if (vehicle != null)
			vehicle.data.Set(Channel.Input, input, value);
		}


	VPVehicleController GetVPVehicleController ()
		{
		if (vehicle == null) return null;
		return vehicle.GetComponent<VPVehicleController>();
		}
	}
}