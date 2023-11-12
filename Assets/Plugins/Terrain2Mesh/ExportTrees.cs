using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using UnityEditor;

[ExecuteInEditMode]
public class ExportTrees : EditorWindow
{
	
	[Header("Settings")]
	public GameObject _tree;
	
	[Header("References")]
	public Terrain _terrain;
	
	//============================================
	
	[MenuItem("AutoDRIVE/Terrain Tools/Convert Trees to Gameobjects")]
	static void Init()
	{
		ExportTrees window = (ExportTrees)GetWindow(typeof(ExportTrees));
	}
	
	void OnGUI()
	{
		_terrain = (Terrain)EditorGUILayout.ObjectField(_terrain, typeof(Terrain), true);
	
		_tree = (GameObject)EditorGUILayout.ObjectField(_tree, typeof(GameObject), true);
	
		if (GUILayout.Button("Convert to objects"))
		{
			Convert();
		}
	
		if (GUILayout.Button("Clear generated trees"))
		{
			Clear();
		}
	}
	
	//============================================
	
	public void Convert()
	{
		TerrainData data = _terrain.terrainData;
		float width = data.size.x;
		float height = data.size.z;
		float y = data.size.y;
	
		// Create parent
		GameObject parent = GameObject.Find("TREES_GENERATED");
	
		if (parent == null)
		{
			parent = new GameObject("TREES_GENERATED");
		}
	
		// Create trees
		foreach (TreeInstance tree in data.treeInstances)
		{
			//Vector3 position = new Vector3(tree.position.x * width, tree.position.y * y, tree.position.z * height);
			Vector3 position = new Vector3(tree.position.x * data.detailWidth - (data.size.x / 2), tree.position.y * y - (data.size.y / 2), tree.position.z * data.detailHeight - (data.size.z / 2));
	
			//Instantiate(_trees[tree.prototypeIndex], position, Quaternion.identity, parent.transform);
			Instantiate(_tree, position, Quaternion.identity, parent.transform);
		}
	}
	
	public void Clear()
	{
		DestroyImmediate(GameObject.Find("TREES_GENERATED"));
	}

}