//--------------------------------------------------------------
//      Vehicle Physics Pro: advanced vehicle physics kit
//          Copyright © 2011-2019 Angel Garcia "Edy"
//        http://vehiclephysics.com | @VehiclePhysics
//--------------------------------------------------------------

// EscapeDialog: controls the Escape dialog and its options


using UnityEngine;
using UnityEngine.UI;
using UnityEngine.Events;


namespace VehiclePhysics.UI
{

public class EscapeDialog : MonoBehaviour
	{
	public VehicleBase vehicle;
	public KeyCode escapeKey = KeyCode.Escape;

	[Header("Buttons")]
	public Button continueButton;
	public Button resetCarButton;
	public Button resetDemoButton;
	public Button quitDemoButton;


	// float m_currentTimeScale;


	void OnEnable ()
		{
		AddListener(continueButton, OnContinue);
		AddListener(resetCarButton, OnResetCar);
		AddListener(resetDemoButton, OnResetDemo);
		AddListener(quitDemoButton, OnQuitDemo);

		// m_currentTimeScale = Time.timeScale;
		// Time.timeScale = 0.0f;
		}


	void OnDisable ()
		{
		RemoveListener(continueButton, OnContinue);
		RemoveListener(resetCarButton, OnResetCar);
		RemoveListener(resetDemoButton, OnResetDemo);
		RemoveListener(quitDemoButton, OnQuitDemo);

		// Time.timeScale = m_currentTimeScale;
		}


	void Update ()
		{
		if (Input.GetKeyDown(escapeKey))
			this.gameObject.SetActive(false);
		}


	// Listeners


	void OnContinue ()
		{
		this.gameObject.SetActive(false);
		}


	void OnResetCar ()
		{
		if (vehicle != null)
			{
			VPResetVehicle resetScript = vehicle.GetComponent<VPResetVehicle>();
			if (resetScript != null)
				{
				resetScript.DoReset();
				this.gameObject.SetActive(false);
				}
			}
		}


	void OnResetDemo ()
		{
		EdyCommonTools.SceneReload.Reload();
		}


	void OnQuitDemo ()
		{
		Application.Quit();
		}


	void AddListener (Button button, UnityAction method)
		{
		if (button != null) button.onClick.AddListener(method);
		}


	void RemoveListener (Button button, UnityAction method)
		{
		if (button != null) button.onClick.RemoveListener(method);
		}




	}
}
