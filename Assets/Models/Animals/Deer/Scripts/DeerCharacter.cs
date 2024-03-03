using UnityEngine;
using System.Collections;

public class DeerCharacter : MonoBehaviour {
	Animator deerAnimator;
	public bool jumpStart=false;
	public float groundCheckDistance = 0.6f;
	public float groundCheckOffset=0.01f;
	public bool isGrounded=true;
	public float jumpSpeed=1f;
	Rigidbody deerRigid;
	public float forwardSpeed;
	public float turnSpeed;
	public float upDown;
	public float leftRight;
	public float jumpStartTime=0f;
	
	void Start () {
		deerAnimator = GetComponent<Animator> ();
		deerRigid=GetComponent<Rigidbody>();
	}
	
	void FixedUpdate(){
		CheckGroundStatus ();
		Move ();
		jumpStartTime+=Time.deltaTime;
	}
	
	public void Attack(){
		deerAnimator.SetTrigger("Attack");
	}
	
	public void NeckControll(bool isControlled){
		if(isControlled){
			deerAnimator.SetLayerWeight(1,1f);
		}else{
			deerAnimator.SetLayerWeight(1,0f);
		}
	}

	public void SideStepL(bool tf){
		deerAnimator.SetBool ("SideStepL",tf);
	}

	public void SideStepR(bool tf){
		deerAnimator.SetBool ("SideStepR",tf);
	}

	public void Hit(){
		deerAnimator.SetTrigger("Hit");
	}
	
	public void EatStart(){
		deerAnimator.SetBool("Eat",true);
	}
	public void EatEnd(){
		deerAnimator.SetBool("Eat",false);
	}

	public void Death(){
		deerAnimator.SetTrigger("Death");
	}
	
	public void Rebirth(){
		deerAnimator.SetTrigger("Rebirth");
	}
	
	public void Roar(){
		deerAnimator.SetTrigger("Roar");
	}
	
	public void SitDown(){
		deerAnimator.SetTrigger("SitDown");
	}

	public void WakeUp(){
		deerAnimator.SetTrigger("WakeUp");
	}


	public void Sleep(){
		deerAnimator.SetTrigger("Sleep");
	}
	
	public void StandUp(){
		deerAnimator.SetTrigger("StandUp");
	}
	
	public void Jump(){
		if (isGrounded) {
			deerAnimator.SetTrigger ("Jump");
			jumpStart = true;
			jumpStartTime=0f;
			isGrounded=false;
			deerAnimator.SetBool("IsGrounded",false);
		}
	}
	
	void CheckGroundStatus()
	{
		RaycastHit hitInfo;
		isGrounded = Physics.Raycast (transform.position + (transform.up * groundCheckOffset), Vector3.down, out hitInfo, groundCheckDistance);

		if (jumpStart) {
			if(jumpStartTime>.25f){
				jumpStart=false;
				deerRigid.AddForce((transform.up+transform.forward*forwardSpeed)*jumpSpeed,ForceMode.Impulse);
				deerAnimator.applyRootMotion = false;
				deerAnimator.SetBool("IsGrounded",false);
			}
		}
		
		if (isGrounded && !jumpStart && jumpStartTime>.5f) {
			deerAnimator.applyRootMotion = true;
			deerAnimator.SetBool ("IsGrounded", true);
		} else {
			if(!jumpStart){
				deerAnimator.applyRootMotion = false;
				deerAnimator.SetBool ("IsGrounded", false);
			}
		}
	}
	
	public void Move(){
		deerAnimator.SetFloat ("Forward", forwardSpeed);
		deerAnimator.SetFloat ("Turn", turnSpeed);
		deerAnimator.SetFloat ("UpDown", upDown);
		deerAnimator.SetFloat ("LeftRight", leftRight);
	}
}
