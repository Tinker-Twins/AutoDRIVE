//--------------------------------------------------------------
//      Vehicle Physics Pro: advanced vehicle physics kit
//          Copyright © 2011-2019 Angel Garcia "Edy"
//        http://vehiclephysics.com | @VehiclePhysics
//--------------------------------------------------------------

// AidsPanel: panel for displaying aids activity and enabling/disabling them


using UnityEngine;


namespace VehiclePhysics.UI
{

public class AidsPanel : MonoBehaviour
	{
	public VehicleBase vehicle;

	public MultiToggle absToggle;
	public MultiToggle escToggle;
	public MultiToggle tcsToggle;
	public MultiToggle asrToggle;
	public MultiToggle steeringToggle;


	bool m_absAvailable = true;
	bool m_escAvailable = true;
	bool m_tcsAvailable = true;
	bool m_asrAvailable = true;
	bool m_steeringAidAvailable = true;


	void OnEnable ()
		{
		if (absToggle != null) absToggle.onClick.AddListener(ToggleAbsEnabled);
		if (escToggle != null) escToggle.onClick.AddListener(ToggleEscEnabled);
		if (tcsToggle != null) tcsToggle.onClick.AddListener(ToggleTcsEnabled);
		if (asrToggle != null) asrToggle.onClick.AddListener(ToggleAsrEnabled);
		if (steeringToggle != null) steeringToggle.onClick.AddListener(ToggleSteeringAidEnabled);
		}


	void OnDisable ()
		{
		if (absToggle != null) absToggle.onClick.RemoveListener(ToggleAbsEnabled);
		if (escToggle != null) escToggle.onClick.RemoveListener(ToggleEscEnabled);
		if (tcsToggle != null) tcsToggle.onClick.RemoveListener(ToggleTcsEnabled);
		if (asrToggle != null) asrToggle.onClick.RemoveListener(ToggleAsrEnabled);
		if (steeringToggle != null) steeringToggle.onClick.RemoveListener(ToggleSteeringAidEnabled);
		}


	void Update ()
		{
		if (vehicle == null) return;

		// Read the settings fromthe vehicle

		int[] settingsData = vehicle.data.Get(Channel.Settings);
		int[] vehicleData = vehicle.data.Get(Channel.Vehicle);

		UpdateToggleState(absToggle, m_absAvailable, vehicleData[VehicleData.AbsEngaged] != 0, settingsData[SettingsData.AbsOverride] == 2);
		UpdateToggleState(escToggle, m_escAvailable, vehicleData[VehicleData.EscEngaged] != 0, settingsData[SettingsData.EscOverride] == 2);
		UpdateToggleState(tcsToggle, m_tcsAvailable, vehicleData[VehicleData.TcsEngaged] != 0, settingsData[SettingsData.TcsOverride] == 2);
		UpdateToggleState(asrToggle, m_asrAvailable, vehicleData[VehicleData.AsrEngaged] != 0, settingsData[SettingsData.AsrOverride] == 2);
		UpdateToggleState(steeringToggle, m_steeringAidAvailable, false, settingsData[SettingsData.SteeringAidsOverride] == 2);
		}


	void UpdateToggleState (MultiToggle toggle, bool available, bool active, bool disabled)
		{
		if (toggle == null) return;

		if (!available)
			toggle.state = MultiToggle.State.Absent;
		else
		if (active)
			toggle.state = MultiToggle.State.Active;
		else
		if (disabled)
			toggle.state = MultiToggle.State.Disabled;
		else
			toggle.state = MultiToggle.State.Enabled;
		}


	// Public API
	// ---------------------------------------------------------------------------------------------


	//  ABS


	public bool absAvailable
		{
		get
			{
			return m_absAvailable;
			}

		set
			{
			if (value != m_absAvailable)
				{
				m_absAvailable = value;
				SetAbsEnabled(value);
				}
			}
		}


	public void SetAbsEnabled (bool enabled)
		{
		if (vehicle == null) return;

		int[] settingsData = vehicle.data.Get(Channel.Settings);

		if (m_absAvailable && enabled)
			settingsData[SettingsData.AbsOverride] = 0;
		else
			settingsData[SettingsData.AbsOverride] = 2;
		}


	public bool IsAbsEnabled ()
		{
		if (vehicle == null) return false;

		int[] settingsData = vehicle.data.Get(Channel.Settings);
		return m_absAvailable && settingsData[SettingsData.AbsOverride] != 2;
		}


	// ESC


	public bool escAvailable
		{
		get
			{
			return m_escAvailable;
			}

		set
			{
			if (value != m_escAvailable)
				{
				m_escAvailable = value;
				SetEscEnabled(value);
				}
			}
		}


	public void SetEscEnabled (bool enabled)
		{
		if (vehicle == null) return;

		int[] settingsData = vehicle.data.Get(Channel.Settings);

		if (m_escAvailable && enabled)
			settingsData[SettingsData.EscOverride] = 0;
		else
			settingsData[SettingsData.EscOverride] = 2;
		}


	public bool IsEscEnabled ()
		{
		if (vehicle == null) return false;

		int[] settingsData = vehicle.data.Get(Channel.Settings);
		return m_escAvailable && settingsData[SettingsData.EscOverride] != 2;
		}


	// TCS


	public bool tcsAvailable
		{
		get
			{
			return m_tcsAvailable;
			}

		set
			{
			if (value != m_tcsAvailable)
				{
				m_tcsAvailable = value;
				SetTcsEnabled(value);
				}
			}
		}


	public void SetTcsEnabled (bool enabled)
		{
		if (vehicle == null) return;

		int[] settingsData = vehicle.data.Get(Channel.Settings);

		if (m_tcsAvailable && enabled)
			settingsData[SettingsData.TcsOverride] = 0;
		else
			settingsData[SettingsData.TcsOverride] = 2;
		}


	public bool IsTcsEnabled ()
		{
		if (vehicle == null) return false;

		int[] settingsData = vehicle.data.Get(Channel.Settings);
		return m_tcsAvailable && settingsData[SettingsData.TcsOverride] != 2;
		}


	// ASR


	public bool asrAvailable
		{
		get
			{
			return m_asrAvailable;
			}

		set
			{
			if (value != m_asrAvailable)
				{
				m_asrAvailable = value;
				SetAsrEnabled(value);
				}
			}
		}


	public void SetAsrEnabled (bool enabled)
		{
		if (vehicle == null) return;

		int[] settingsData = vehicle.data.Get(Channel.Settings);

		if (m_asrAvailable && enabled)
			settingsData[SettingsData.AsrOverride] = 0;
		else
			settingsData[SettingsData.AsrOverride] = 2;
		}


	public bool IsAsrEnabled ()
		{
		if (vehicle == null) return false;

		int[] settingsData = vehicle.data.Get(Channel.Settings);
		return m_asrAvailable && settingsData[SettingsData.AsrOverride] != 2;
		}


	// Steering Aid


	public bool steeringAidAvailable
		{
		get
			{
			return m_steeringAidAvailable;
			}

		set
			{
			if (value != m_steeringAidAvailable)
				{
				m_steeringAidAvailable = value;
				SetSteeringAidEnabled(value);
				}
			}
		}


	public void SetSteeringAidEnabled (bool enabled)
		{
		if (vehicle == null) return;

		int[] settingsData = vehicle.data.Get(Channel.Settings);

		if (m_steeringAidAvailable && enabled)
			settingsData[SettingsData.SteeringAidsOverride] = 0;
		else
			settingsData[SettingsData.SteeringAidsOverride] = 2;
		}


	public bool IsSteeringAidEnabled ()
		{
		if (vehicle == null) return false;

		int[] settingsData = vehicle.data.Get(Channel.Settings);
		return m_steeringAidAvailable && settingsData[SettingsData.SteeringAidsOverride] != 2;
		}


	// Toggle enabled/disabled, also click listeners


	public void ToggleAbsEnabled ()
		{
		SetAbsEnabled(!IsAbsEnabled());
		}


	public void ToggleEscEnabled ()
		{
		SetEscEnabled(!IsEscEnabled());
		}


	public void ToggleTcsEnabled ()
		{
		SetTcsEnabled(!IsTcsEnabled());
		}


	public void ToggleAsrEnabled ()
		{
		SetAsrEnabled(!IsAsrEnabled());
		}


	public void ToggleSteeringAidEnabled ()
		{
		SetSteeringAidEnabled(!IsSteeringAidEnabled());
		}
	}
}