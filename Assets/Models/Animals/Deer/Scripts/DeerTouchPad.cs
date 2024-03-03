using UnityEngine;
using UnityEngine.UI;
using UnityEngine.EventSystems;
using System.Collections;

public class DeerTouchPad : MonoBehaviour, IPointerDownHandler, IDragHandler, IPointerUpHandler  {
	private Vector2 smoothDirection;
	private Vector2 touchPadgOrigin;
	private Vector2 touchedPosition;
	private bool touched;
	private int pointerID;
	float imageWidthHalf;
	private float directionMultiplier;
	public  DeerCharacter deerCharacter;
	public GameObject joyStick;
	
	void Awake () {	
		touched = false;	
		imageWidthHalf = GetComponent<RectTransform> ().rect.width/2;
		touchPadgOrigin= new Vector2(imageWidthHalf,imageWidthHalf);
		touchedPosition = new Vector2(imageWidthHalf,imageWidthHalf);
		directionMultiplier = 1.0f / imageWidthHalf;	
	}

	void FixedUpdate(){
		if(!touched){
			touchedPosition=Vector2.Lerp(touchedPosition,touchPadgOrigin,Time.deltaTime);
			smoothDirection = (touchedPosition - touchPadgOrigin)*directionMultiplier;
			deerCharacter.forwardSpeed=smoothDirection.y;
			deerCharacter.turnSpeed=smoothDirection.x;
			joyStick.gameObject.transform.position=touchedPosition;
		}
	}

	public void OnPointerDown (PointerEventData data) {
		touched = true;
		pointerID = data.pointerId;
		touchedPosition= data.position;
		joyStick.gameObject.transform.position=touchedPosition;
		smoothDirection = (touchedPosition - touchPadgOrigin)*directionMultiplier;
	}
	
	public void OnDrag (PointerEventData data) {
		if (data.pointerId == pointerID & data.position.x<imageWidthHalf*2f  & data.position.y<imageWidthHalf*2f & data.position.x>0f  & data.position.y>0f ) {
			touchedPosition= data.position;
			joyStick.gameObject.transform.position=touchedPosition;			
			smoothDirection = (touchedPosition - touchPadgOrigin)*directionMultiplier;
			deerCharacter.forwardSpeed=smoothDirection.y;
			deerCharacter.turnSpeed=smoothDirection.x;
		}
	}
	
	public void OnPointerUp (PointerEventData data) {
		if (data.pointerId == pointerID) {
			touched = false;
		}
	}
	
	public void SetDeerCharacter(DeerCharacter dc){
		deerCharacter = dc;
	}
}
