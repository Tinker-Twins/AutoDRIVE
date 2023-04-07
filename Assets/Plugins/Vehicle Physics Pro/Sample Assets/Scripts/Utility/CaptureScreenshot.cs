//--------------------------------------------------------------
//      Vehicle Physics Pro: advanced vehicle physics kit
//          Copyright © 2011-2019 Angel Garcia "Edy"
//        http://vehiclephysics.com | @VehiclePhysics
//--------------------------------------------------------------

// A simple screenshot capture script


using UnityEngine;
using EdyCommonTools;
using System.IO;


namespace VehiclePhysics.Utility
{

public class CaptureScreenshot : MonoBehaviour
	{
	public KeyCode key = KeyCode.F12;
	public string fileName = "Screenshot{0}.png";
	public int sizeMultiplier = 1;

	int m_sequence = 0;

    void Update()
		{
		if (Input.GetKeyDown(key))
			{
			string fileNameToUse = fileName;

			if (fileName.Contains("{0}"))
				{
				fileNameToUse = GetNextFileName();
				while (File.Exists(fileNameToUse))
					{
					fileNameToUse = GetNextFileName();
					}
				}

			ScreenCapture.CaptureScreenshot(fileNameToUse, sizeMultiplier);
			Debug.Log("Screenshot saved: " + fileNameToUse);
			}
		}


	string GetNextFileName()
		{
		string result = string.Format(fileName, m_sequence.ToString("00"));
		m_sequence++;

		return result;
		}
	}

}