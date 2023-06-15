using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;

public class InteractionStand : MonoBehaviour {

	[SerializeField] Animator Animator;
	[SerializeField] int RegimesCount;

	int CurrentRegimeIndex = 0;

	public void SetNextRegime () {
		CurrentRegimeIndex++;
		if (CurrentRegimeIndex >= RegimesCount) {
			CurrentRegimeIndex = 0;
		}
		Animator.SetInteger("CurrentRegimeIndex", CurrentRegimeIndex);
	}
}
