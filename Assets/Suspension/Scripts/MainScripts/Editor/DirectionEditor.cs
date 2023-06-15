using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

[CustomPropertyDrawer(typeof(Direction))]

public class DirectionEditor : PropertyDrawer {

    public override void OnGUI(Rect position, SerializedProperty property, GUIContent label)
    {

        Rect contentPosition = EditorGUI.PrefixLabel(position, new GUIContent(label));

        EditorGUIUtility.labelWidth = 14f;

		Direction.EnumDirection enumDirection;
		var selsectedDirection = property.FindPropertyRelative("direction");
		enumDirection = (Direction.EnumDirection)selsectedDirection.enumValueIndex;

        EditorGUI.BeginProperty(contentPosition, label, property);
        {
            EditorGUI.BeginChangeCheck();
            var menu = EditorGUI.EnumPopup(contentPosition, enumDirection);
			if (EditorGUI.EndChangeCheck()) {
				enumDirection = (Direction.EnumDirection)System.Convert.ToInt16(menu);
				var propertyVector3 = property.FindPropertyRelative("vector3");
				switch (enumDirection) {
					case Direction.EnumDirection.Forward: propertyVector3.vector3Value = Vector3.forward; break;
					case Direction.EnumDirection.Back: propertyVector3.vector3Value = Vector3.back; break;
					case Direction.EnumDirection.Left: propertyVector3.vector3Value = Vector3.left; break;
					case Direction.EnumDirection.Right: propertyVector3.vector3Value = Vector3.right; break;
					case Direction.EnumDirection.Up: propertyVector3.vector3Value = Vector3.up; break;
					case Direction.EnumDirection.Down: propertyVector3.vector3Value = Vector3.down; break;
					default: propertyVector3.vector3Value = Vector3.zero; break;
				}
				selsectedDirection.enumValueIndex = (int)enumDirection;
				
			}
               
        }
        EditorGUI.EndProperty();
    }

    public override float GetPropertyHeight(SerializedProperty property, GUIContent label)
    {
        return Screen.width < 333 ? (16f + 18f) : 16f;
    }
}
