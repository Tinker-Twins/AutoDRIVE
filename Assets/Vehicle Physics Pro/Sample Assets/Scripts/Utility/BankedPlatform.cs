//--------------------------------------------------------------
//      Vehicle Physics Pro: advanced vehicle physics kit
//          Copyright © 2011-2019 Angel Garcia "Edy"
//        http://vehiclephysics.com | @VehiclePhysics
//--------------------------------------------------------------

using UnityEngine;


namespace VehiclePhysics.Utility
{

[RequireComponent(typeof(Rigidbody))]
public class BankedPlatform : MonoBehaviour
	{
	public float minAngle = 2.0f;
	public float maxAngle = 45.0f;
	public float eulerAngleVelocity = 1.0f;
	public float multiplerOnCtrl = 5.0f;
	public float pauseAtLimits = 10.0f;

	public bool manual = false;
	public bool showAngle = false;

	public GUIStyle labelStyle = new GUIStyle();
	public Vector2 labelPosition = new Vector2(10, 320);


	int m_state = 0;
	int m_lastState = -1;
	float m_wakeUpTime = 0.0f;
	bool m_uiPressed = false;
	Rigidbody m_rigidbody;


	void Start ()
		{
		m_state = 3;
		m_wakeUpTime = Time.time + pauseAtLimits;
		}


	void OnEnable ()
		{
		m_rigidbody = GetComponent<Rigidbody>();
		}


	void Update ()
		{
		if (Input.GetKeyDown(KeyCode.KeypadEnter))
			{
			manual = !manual;
			showAngle = manual;
			}
		}


	void FixedUpdate ()
		{
		Vector3 eulerVelocity = new Vector3(eulerAngleVelocity, 0, 0);
		Quaternion deltaRotation = Quaternion.identity;

		if (Input.GetKey(KeyCode.LeftControl) || Input.GetKey(KeyCode.RightControl))
			eulerVelocity *= multiplerOnCtrl;

		float currentAngle = transform.eulerAngles.x;
		if (currentAngle > 180.0f) currentAngle -= 360.0f;

		if (manual)
			{
			if (Input.GetKey(KeyCode.Keypad0))
				m_state = 0;
			else
			if (Input.GetKey(KeyCode.KeypadPeriod))
				m_state = 2;
			else
			if (!m_uiPressed)
				m_state = -1;
			}
		else
			if (m_state < 0)
				{
				m_state = m_lastState == 2? 1 : 3;
				m_wakeUpTime = Time.time + pauseAtLimits;
				}

		switch (m_state)
			{
			case 0: // Increasing angle up to upper limit
				if (currentAngle < maxAngle)
					{
					deltaRotation = Quaternion.Euler(eulerVelocity * Time.deltaTime);
					}
				else
					{
					m_wakeUpTime = Time.time + pauseAtLimits;
					m_state = 1;
					}

				m_lastState = 0;
				break;

			case 1:	// Pause before decreasing
				if (!manual && Time.time >= m_wakeUpTime)
					m_state = 2;
				break;

			case 2:	// Decreasing angle down to lower limit
				if (currentAngle > minAngle)
					{
					deltaRotation = Quaternion.Euler(-eulerVelocity * Time.deltaTime);
					}
				else
					{
					m_wakeUpTime = Time.time + pauseAtLimits;
					m_state = 3;
					}

				m_lastState = 2;
				break;

			case 3:	// Pause before increasing
				if (!manual && Time.time >= m_wakeUpTime)
					m_state = 0;
				break;
			}

		m_rigidbody.MoveRotation(m_rigidbody.rotation * deltaRotation);
		}


	void OnGUI ()
		{
		if (showAngle)
			GUI.Label(new Rect(labelPosition.x, labelPosition.y, 300, 100), string.Format("Platform angle: {0,4:0.0}°", transform.localEulerAngles.x), labelStyle);
		}



	// UI interaction


	public void OnControlPress (GameObject value)
		{
		switch (value.name)
			{
			case "Button Bank Down": m_state = 2; break;
			case "Button Bank Up": m_state = 0; break;
			}

		m_uiPressed = true;
		}


	public void OnControlRelease (GameObject value)
		{
		if (manual) m_state = -1;
		m_uiPressed = false;
		}


	public void OnCheckboxActivate (bool value)
		{
		manual = !value;

		if (!manual && m_state < 0)
			{
			m_state = m_lastState == 2? 1 : 3;
			m_wakeUpTime = Time.time;
			}
		}


	public void OnSliderChange (float value)
		{
		eulerAngleVelocity = value * 10.0f;
		}

	}

}