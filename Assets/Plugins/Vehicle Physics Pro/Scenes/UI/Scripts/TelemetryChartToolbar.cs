//--------------------------------------------------------------
//      Vehicle Physics Pro: advanced vehicle physics kit
//          Copyright © 2011-2019 Angel Garcia "Edy"
//        http://vehiclephysics.com | @VehiclePhysics
//--------------------------------------------------------------

// TelemetryChartToolbar: reads toolbar buttons and applies the functions


using UnityEngine;
using UnityEngine.UI	;
using UnityEngine.EventSystems;
using EdyCommonTools;


namespace VehiclePhysics.UI
{

public class TelemetryChartToolbar : MonoBehaviour
	{
	public Color onPressColor = GColor.ParseColorHex("#838383");

	[Header("Toolbar elements")]
	public GameObject program;
	public GameObject startStop;
	public GameObject panLeft;
	public GameObject panRight;
	public GameObject panUp;
	public GameObject panDown;
	public GameObject zoomHorizIn;
	public GameObject zoomHorizOut;
	public GameObject zoomVertIn;
	public GameObject zoomVertOut;
	public GameObject reset;
	public GameObject windowSize;
	public GameObject close;

	[Header("UI Texts")]
	public Text startStopText;
	public Text windowSizeText;
	public string startText = "START";
	public string stopText = "STOP";
	public string maximizeText = "\uF065";	// FontAwesome glyphs
	public string minimizeText = "\uF066";
	}

}