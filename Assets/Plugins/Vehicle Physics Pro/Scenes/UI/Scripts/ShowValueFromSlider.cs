//--------------------------------------------------------------
//      Vehicle Physics Pro: advanced vehicle physics kit
//          Copyright © 2011-2019 Angel Garcia "Edy"
//        http://vehiclephysics.com | @VehiclePhysics
//--------------------------------------------------------------

// CloseOnKeyOrClick: Disables this GameObject when pressing a key or clicking an UI item


using UnityEngine;
using UnityEngine.UI;


namespace VehiclePhysics.UI
{

[RequireComponent(typeof(Text))]
public class ShowValueFromSlider : MonoBehaviour
	{
	public Slider slider;
	public string format = "0.0";


	Text m_text;


	void OnEnable ()
		{
		m_text = GetComponent<Text>();

		if (slider != null)
			{
			slider.onValueChanged.AddListener(SliderChanged);
			SliderChanged(slider.value);
			}
		else
			{
			SliderChanged(0.0f);
			}
		}


	void OnDisable ()
		{
		if (slider != null)
			slider.onValueChanged.RemoveListener(SliderChanged);
		}


	void SliderChanged (float value)
		{
		m_text.text = value.ToString(format);
		}
	}
}