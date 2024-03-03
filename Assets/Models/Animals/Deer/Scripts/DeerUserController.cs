using UnityEngine;
using System.Collections;

public class DeerUserController : MonoBehaviour {
	DeerCharacter deerCharacter;
	
	void Start () {
		deerCharacter = GetComponent < DeerCharacter> ();
	}
	
	void Update () {	
		if (Input.GetButtonDown ("Fire1")) {
			deerCharacter.Attack();
		}
		if (Input.GetButtonDown ("Jump")) {
			deerCharacter.Jump();
		}
		if (Input.GetKeyDown (KeyCode.H)) {
			deerCharacter.Hit();
		}
		if (Input.GetKeyDown (KeyCode.E)) {
			deerCharacter.EatStart();
		}
		if (Input.GetKeyUp (KeyCode.E)) {
			deerCharacter.EatEnd();
		}
		if (Input.GetKeyDown (KeyCode.F)) {
			deerCharacter.SideStepL(true);
		}
		if (Input.GetKeyUp (KeyCode.F)) {
			deerCharacter.SideStepL(false);
		}
		if (Input.GetKeyDown (KeyCode.G)) {
			deerCharacter.SideStepR(true);
		}
		if (Input.GetKeyUp (KeyCode.G)) {
			deerCharacter.SideStepR(false);
		}
		if (Input.GetKeyDown (KeyCode.K)) {
			deerCharacter.Death();
		}
		if (Input.GetKeyDown (KeyCode.L)) {
			deerCharacter.Rebirth();
		}		
		if (Input.GetKeyDown (KeyCode.R)) {
			deerCharacter.Roar();
		}		
		if (Input.GetKeyDown (KeyCode.J)) {
			deerCharacter.SitDown();
		}		
		if (Input.GetKeyDown (KeyCode.U)) {
			deerCharacter.WakeUp();
		}	
		if (Input.GetKeyDown (KeyCode.I)) {
			deerCharacter.StandUp();
		}		
		if (Input.GetKeyDown (KeyCode.M)) {
			deerCharacter.Sleep();
		}	
		if (Input.GetKeyDown (KeyCode.N)) {
			deerCharacter.NeckControll(true);
		}		
		if (Input.GetKeyUp (KeyCode.N)) {
			deerCharacter.NeckControll(false);
		}		

		deerCharacter.upDown= Input.GetAxis ("Vertical");
		deerCharacter.leftRight= Input.GetAxis ("Horizontal");
	}
}
