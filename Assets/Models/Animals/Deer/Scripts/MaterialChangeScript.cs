using UnityEngine;
using System.Collections;

public class MaterialChangeScript : MonoBehaviour {
	public Renderer rend;
	public Material mat;

	// Use this for initialization
	void Start () {
		Debug.Log (rend.materials [1].name);
		rend.materials[1]=mat;
		Debug.Log (rend.materials [1].name);
	}
	
	// Update is called once per frame
	void Update () {
	
	}
}
