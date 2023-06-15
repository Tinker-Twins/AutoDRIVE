using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class GameController : MonoBehaviour {

	[SerializeField] List<CarController> Cars = new List<CarController>();
	[SerializeField] CameraController CameraController;
	[SerializeField] InteractionStand InteractionStand;
	

	[SerializeField] float MaxTimeScale = 2f;
	[SerializeField] float MinTimeScale = 0f;
	[SerializeField] float TimeStepScale = 0.2f;
	[SerializeField] Text TimeScaleText;
	[SerializeField] GameObject HelpText;

	int CurrentCarIndex;
	CarController CurrentCar;
	ViewPoints CurrebtViewPoints;
	float FixedDeltaTimeStart;

	KeyCode ShowHideHelp = KeyCode.F1;
	KeyCode SetNextCar = KeyCode.C;
	KeyCode SetNextView = KeyCode.V;
	KeyCode InteractionKey = KeyCode.E;
	KeyCode PlusTimeScale = KeyCode.KeypadPlus;
	KeyCode MinusTimeScale = KeyCode.KeypadMinus;

	public static GameController Instance;

	void Start () {
		CurrentCarIndex = 0;
		SelectCar(CurrentCarIndex);
		FixedDeltaTimeStart = Time.fixedDeltaTime;
		SetTimeScale(1f);
		Instance = this;
	}

	private void Update () {
		if (Input.GetKeyDown(ShowHideHelp)) {
			HelpText.SetActive(!HelpText.activeInHierarchy);
		}

		if (Input.GetKeyDown(SetNextCar)) {
			SelectCar(CurrentCarIndex + 1);
		}

		if (Input.GetKeyDown(SetNextView)) {
			CurrebtViewPoints.SettNextPoint();
		}

		if (Input.GetKeyDown(PlusTimeScale)) {
			var newTime =  Mathf.MoveTowards(Time.timeScale, MaxTimeScale, TimeStepScale);
			SetTimeScale(newTime);
		}

		if (Input.GetKeyDown(MinusTimeScale)) {
			var newTime = Mathf.MoveTowards(Time.timeScale, MinTimeScale, TimeStepScale);
			SetTimeScale(newTime);
		}

		if (Input.GetKeyDown(InteractionKey)) {
			InteractionStand.SetNextRegime();
		}
	}

	void SelectCar (int index) {
		if (CurrentCar) {
			CurrentCar.Enable = false;
		}

		CurrentCarIndex = index;
		if (CurrentCarIndex >= Cars.Count) {
			CurrentCarIndex = 0;
		}

		CurrentCar = Cars[CurrentCarIndex];
		CurrentCar.Enable = true;

		CurrebtViewPoints = CurrentCar.GetComponent<ViewPoints>();
		CameraController.SetViewPoints(CurrebtViewPoints);
	}

	void SetTimeScale (float newScale) {
		Time.timeScale = newScale;
		TimeScaleText.text = "Time scale: " + Time.timeScale.ToString("#0.#");
		if (newScale > 0) {
			Time.fixedDeltaTime = FixedDeltaTimeStart * newScale;
		}
	}
}
