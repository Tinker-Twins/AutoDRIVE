//--------------------------------------------------------------
//      Vehicle Physics Pro: advanced vehicle physics kit
//          Copyright © 2011-2019 Angel Garcia "Edy"
//        http://vehiclephysics.com | @VehiclePhysics
//--------------------------------------------------------------


using UnityEngine;
using UnityEditor;
using EdyCommonTools.EditorTools;

namespace VehiclePhysics.EditorTools
{

[CustomEditor(typeof(UI.Gauge)), CanEditMultipleObjects]
public class GaugeInspector : VPInspector
	{
	static bool s_autoRegenerate = false;

	public override void DrawInspectorGUI ()
		{
		SetMinLabelWidth(180);
		DrawDefaultInspector();

		Space(10);
		EditorGUILayout.BeginHorizontal();
		Space(50);

		// Handle auto-regenerate

		s_autoRegenerate = GUILayout.Toggle(s_autoRegenerate, "Auto Generate");

		if (s_autoRegenerate && CommonEditorTools.GUIChanged())
			(target as UI.Gauge).Regenerate();

		// Handle Regenerate button.
		// Must be done separately as GUIChanged doesn't respond to Undo within GUI.enabled = false.

		if (s_autoRegenerate)
			GUI.enabled = false;

		if (GUILayout.Button("Generate Gauge"))
			(target as UI.Gauge).Regenerate();

		GUI.enabled = true;

		EditorGUILayout.EndHorizontal();
		}
	}
}