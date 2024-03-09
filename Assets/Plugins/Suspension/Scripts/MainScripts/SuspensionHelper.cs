using UnityEngine;

/// <summary>
/// For convenient selection of Axis in the editor inspector
/// </summary>
public enum Axis {
	X,
	Y,
	Z
}

/// <summary>
/// For convenient selection of directions in the editor inspector
/// field "direction" shown in inspector
/// field "vector3" calculated after changing "direction"
/// </summary>
[System.Serializable]
public struct Direction {

	public EnumDirection direction;
	public Vector3 vector3;

	public enum EnumDirection {
		None,
		Forward,
		Back,
		Left,
		Right,
		Up,
		Down
	}
}
