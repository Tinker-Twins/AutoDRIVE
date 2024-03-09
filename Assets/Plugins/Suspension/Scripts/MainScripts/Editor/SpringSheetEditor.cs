using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

[CustomEditor(typeof(SpringSheet))]

public class SpringSheetEditor : Editor {

	public override void OnInspectorGUI()
	{
		EditorGUI.BeginChangeCheck();

		base.OnInspectorGUI();

		if (EditorGUI.EndChangeCheck()) {
			var target = serializedObject.targetObject as SpringSheet;
			target.UpdateStartPosition();
        }

	}

}
