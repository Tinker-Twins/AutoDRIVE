//--------------------------------------------------------------
//      Vehicle Physics Pro: advanced vehicle physics kit
//          Copyright © 2011-2019 Angel Garcia "Edy"
//        http://vehiclephysics.com | @VehiclePhysics
//--------------------------------------------------------------

// Dashboard: handles a dashboard UI with signals and gauges


using UnityEngine;
using UnityEngine.UI;
using System;
using EdyCommonTools;


namespace VehiclePhysics.UI
{

public class Dashboard : MonoBehaviour
	{
	public VehicleBase vehicle;

	[Header("Needles")]
	public Needle speedNeedle = new Needle();
	public Needle rpmNeedle = new Needle();

	[Header("Dashboard elements")]
	public GameObject stalledSignal;
	public GameObject handbrakeSignal;
	public GameObject warningSignal;
	public GameObject retarderSignal;
	public GameObject axleDiffLockSignal;
	public GameObject centerDiffLockSignal;
	public GameObject fullDiffLockSignal;
	public GameObject singleAxleDriveSignal;
	public GameObject singleAxleDiffLockSignal;

	[Header("UI labels")]
	public Text gearLabel;
	public Text speedMphLabel;

	[Serializable]
	public class Needle
		{
		public Transform needle;
		public float minValue = 0.0f;
		public float maxValue = 200.0f;
		public float angleAtMinValue = 135.0f;
		public float angleAtMaxValue = -135.0f;

		public void SetValue (float value)
			{
			if (needle == null) return;

			float x = (value - minValue) / (maxValue - minValue);
			float angle = MathUtility.UnclampedLerp(angleAtMinValue, angleAtMaxValue, x);

			needle.localRotation = Quaternion.Euler(0, 0, angle);
			}
		}


	float m_lastVehicleTime;
	float m_warningTime;


	InterpolatedFloat m_speedMs = new InterpolatedFloat();
	InterpolatedFloat m_engineRpm = new InterpolatedFloat();


	void OnEnable ()
		{
		m_lastVehicleTime = -1.0f;
		m_warningTime = -10.0f;

		if (vehicle == null)
			vehicle = GetComponentInParent<VehicleBase>();
		}


	void FixedUpdate ()
		{
		if (vehicle == null || !vehicle.isActiveAndEnabled) return;

		m_speedMs.Set(vehicle.data.Get(Channel.Vehicle, VehicleData.Speed) / 1000.0f);
		m_engineRpm.Set(vehicle.data.Get(Channel.Vehicle, VehicleData.EngineRpm) / 1000.0f);
		}


	void Update ()
		{
		if (vehicle == null || !vehicle.isActiveAndEnabled)
			{
			speedNeedle.SetValue(0.0f);
			rpmNeedle.SetValue(0.0f);
			m_speedMs.Reset(0.0f);
			m_engineRpm.Reset(0.0f);
			SetEnabled(stalledSignal, false);
			SetEnabled(handbrakeSignal, false);
			SetEnabled(warningSignal, false);
			SetEnabled(retarderSignal, false);
			SetEnabled(fullDiffLockSignal, false);
			SetEnabled(axleDiffLockSignal, false);
			SetEnabled(centerDiffLockSignal, false);
			SetEnabled(singleAxleDriveSignal, false);
			SetEnabled(singleAxleDiffLockSignal, false);
			if (gearLabel != null) gearLabel.text = "-";
			if (speedMphLabel != null) speedMphLabel.text = "-";

			m_lastVehicleTime = -1.0f;
			return;
			}

		// Needles use interpolated values computed at visual frame rate

		float frameRatio = InterpolatedFloat.GetFrameRatio();

		float speedMs = m_speedMs.GetInterpolated(frameRatio);
		float engineRpm = m_engineRpm.GetInterpolated(frameRatio);
		if (speedMs < 0) speedMs = 0.0f;
		if (engineRpm < 0) engineRpm = 0.0f;

		speedNeedle.SetValue(speedMs * 3.6f);
		rpmNeedle.SetValue(engineRpm);

		// Warning signal also at update rate because of its timing

		int[] vehicleData = vehicle.data.Get(Channel.Vehicle);

		bool abs = vehicleData[VehicleData.AbsEngaged] != 0;
		bool tcs = vehicleData[VehicleData.TcsEngaged] != 0;
		bool esc = vehicleData[VehicleData.EscEngaged] != 0;
		bool asr = vehicleData[VehicleData.AsrEngaged] != 0;

		if (abs || tcs || esc || asr)
			m_warningTime = Time.time;

		if (Time.time - m_warningTime < 0.5f)
			SetEnabled(warningSignal, Mathf.Repeat(Time.time, 0.3f) < 0.2f);
		else
			SetEnabled(warningSignal, false);

		// Visual stuff (Update) that is updated at FixedUpdate rate.
		// Ignore frames without updates.

		if (vehicle.time > m_lastVehicleTime)
			{
			// Prepare bus values

			int[] inputData = vehicle.data.Get(Channel.Input);
			int[] settingsData = vehicle.data.Get(Channel.Settings);

			// Stalled, handbrake and retarder signals

			bool vehicleEnabled = inputData[InputData.Key] >= 0;

			bool vehicleStalled = vehicleEnabled && vehicleData[VehicleData.EngineStalled] != 0;
			SetEnabled(stalledSignal, vehicleStalled);

			float handbrakeInput = inputData[InputData.Handbrake] / 10000.0f;
			bool handbrakeEngaged = vehicleEnabled && handbrakeInput > 0.1f;
			SetEnabled(handbrakeSignal, handbrakeEngaged);

			bool retarderEngaged = inputData[InputData.Retarder] > 0;
			SetEnabled(retarderSignal, retarderEngaged);

			// Driveline lock / unlock signals

			bool axleDiffLock = settingsData[SettingsData.DifferentialLock] == 1;
			bool centerDiffLock = settingsData[SettingsData.DrivelineLock] == 1;
			bool centerDiffUnlock = settingsData[SettingsData.DrivelineLock] == 2;

			SetEnabled(fullDiffLockSignal, axleDiffLock && centerDiffLock);
			SetEnabled(axleDiffLockSignal, axleDiffLock && !centerDiffLock && !centerDiffUnlock);
			SetEnabled(centerDiffLockSignal, !axleDiffLock && centerDiffLock);

			SetEnabled(singleAxleDriveSignal, !axleDiffLock && centerDiffUnlock);
			SetEnabled(singleAxleDiffLockSignal, axleDiffLock && centerDiffUnlock);

			// Speed Mph label

			if (speedMphLabel != null)
				speedMphLabel.text = (speedMs * 2.237f).ToString("0") + "\nmph";

			// Gear label

			if (gearLabel != null)
				{
				int gearId = vehicleData[VehicleData.GearboxGear];
				int gearMode = vehicleData[VehicleData.GearboxMode];
				bool switchingGear = vehicleData[VehicleData.GearboxShifting] != 0;

				switch (gearMode)
					{
					case 0:		// M
						if (gearId == 0)
							gearLabel.text = switchingGear? " " : "N";
						else
						if (gearId > 0)
							gearLabel.text = gearId.ToString();
						else
							{
							if (gearId == -1)
								gearLabel.text = "R";
							else
								gearLabel.text = "R" + (-gearId).ToString();
							}
						break;

					case 1:		// P
						gearLabel.text = "P";
						break;

					case 2:		// R
						if (gearId < -1)
							gearLabel.text = "R" + (-gearId).ToString();
						else
							gearLabel.text = "R";
						break;

					case 3:		// N
						gearLabel.text = "N";
						break;

					case 4:		// D
						if (gearId > 0)
							gearLabel.text = "D" + gearId.ToString();
						else
							gearLabel.text = "D";
						break;

					case 5:		// L
						if (gearId > 0)
							gearLabel.text = "L" + gearId.ToString();
						else
							gearLabel.text = "L";
						break;

					default:
						gearLabel.text = "-";
						break;
					}
				}


			m_lastVehicleTime = vehicle.time;
			}
		}


	void SetEnabled (GameObject go, bool active)
		{
		if (go != null) go.SetActive(active);
		}
	}

}
