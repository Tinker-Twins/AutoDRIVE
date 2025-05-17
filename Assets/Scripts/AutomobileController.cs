using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using TMPro;

public class AutomobileController : MonoBehaviour
{
    public enum InputType { Default, F710, Xbox, G29 };
    public InputType HMIType = InputType.Default; // Set input type
    internal enum driveType { FrontWheelDrive, RearWheelDrive,	AllWheelDrive }
	public TextMeshProUGUI speedText;
	public TextMeshProUGUI gearText;
	public float Wheelbase = 3.09f; // m
	public float TrackWidth = 1.734f; // m
	public Vector3 COM;
    public float ThrottleLimit = 1.0f; // norm%
    public float topSpeed;
	public float accleration;
	public float brakingPower;
	public float driftPower;
	public bool driftAllowed = false;
	[SerializeField] private driveType carDriveType;
	public AnimationCurve gearRatios;
	public AnimationCurve torqueCurve;
	public bool checkSpeeds = true;
	public float[] gearSpeeds;
	public float idleRPM, maxGearChangeRPM, minGearChangeRPM;
	public float smoothTime = 0.3f;
	public float finalDriveRatio1, finalDriveRatio2;
	public float speedMultiplier = 2.23694f;
	public float topSpeedDrag = 0.032f, idleDrag = 0.05f, runningDrag = 0.01f;
	public float forwardFrictionSpeedFactor;
	public float baseFwdExtremum = 1, baseFwdAsymptote = 0.5f;
	public float baseSideAsymptote, baseSideExtremum;
	public float driftVelocityFactor;
	public float defaultForwardStiffness;
	public float maxSidewaysStiffness;
	public float maxSidewaysFrictionValue;
	public float turnPower;
	public float revTorquePower;
	public float differentialTorqueDrop;
	public float turnRange = 4f;
	public float autoStraight = 1;
	public float turnCheckSense = 10000;
	public float acc;
	public float throttle;
	public Transform[] wheelMesh;
	public WheelCollider[] wheelColliders;
	public GameObject steeringWheel;
	public float maxSteerAngle;
	public float steerRatio;
	public float steerSensitivity;
	public float speedDependencyFactor;
	public float steerAngleLimitingFactor;
	public float engineRPM;
	public int gearNum;
	[Range (0, 1)] public float _steerHelper;
	public float currSpeed;
	public float fwdInput, revInput, steerInput, brakeInput, handbrakeInput;
	public float traction;
	public float lockRPM;
	public float longitudinalSlipLimit;
	public float lateralSlipLimit;
	public float longitudinalSlipAudioThreshold;
	public float lateralSlipAudioThreshold;
	public float downForce;
	public float criticalDonutSpeed;
	public int collisionCount = 0;

	public float driftX { get; private set; }

	private Rigidbody car;
	private float totalTorque;
	private float outputTorque;
	private float wheelRPM;

	public float steerAngle{ get; private set; }

	private float turnAngle;
	private float maxReverseSpeed = 30;
	private float reverseDrag = 0.1f;
	private float local_finalDrive;
	private WheelFrictionCurve fwf, swf;
	private float iRPM;
	private float thrAgg = 0.8f;
	private float currentTorque;
	private float oldRotation;
	private float localSteerHelper;
	private bool drifting;
	private float upClamp = 0;
	private float prevAngularVelocity;
	private float angularAcclY;
	private bool MouseHold;
	private float MouseStart;

	[SerializeField] private WheelEffects[] m_WheelEffects = new WheelEffects[4];

	private float currentT = 0;
    private float currentS = 0;
	private float currentB = 0;
    private float currentH = 0;

	[Range(-1,1)] public float AutonomousThrottle = 0;
    [Range(-1,1)] public float AutonomousSteering = 0;
	[Range(-1,1)] public float AutonomousBrake = 0;
    [Range(-1,1)] public float AutonomousHandbrake = 0;
    public int DrivingMode = 0; // Driving mode: 0 is manual, 1 is autonomous

	public int CurrentDrivingMode
    {
        get { return DrivingMode; }
    }

    public float CurrentThrottle
    {
        get { return currentT; }
        set { AutonomousThrottle = Mathf.Clamp(value, -1.0f, 1.0f); }
    }

    public float CurrentSteeringAngle
    {
        get { return currentS; }
        set { AutonomousSteering = Mathf.Clamp(value, -1.0f, 1.0f); }
    }

	public float CurrentBrake
    {
        get { return currentB; }
        set { AutonomousBrake = Mathf.Clamp(value, 0.0f, 1.0f); }
    }

    public float CurrentHandbrake
    {
        get { return currentH; }
        set { AutonomousHandbrake = Mathf.Clamp(value, 0.0f, 1.0f); }
    }

	void OnCollisionEnter(Collision collision)
    {
        collisionCount += 1;
    }

	void Awake ()
	{
		car = GetComponent<Rigidbody> ();
		car.centerOfMass = COM; // Set COM
	}

	void FixedUpdate ()
	{
		getInput ();
		adjustFinalDrive ();
		moveCar ();
		steerCar ();
		brakeCar ();
		animateWheels ();
		animateSteeringWheel ();
		getCarSpeed ();
		calcTurnAngle ();
		adjustDrag ();
		adjustForwardFriction ();
		adjustSidewaysFriction ();
		rotationalStabilizer ();
		steerHelper ();
		tractionControl ();
		driftCar ();
		checkForWheelSpin();
		updateText();
	}

	void updateText ()
	{
		speedText.text = ((int)Mathf.Abs(currSpeed)).ToString();
		if (currSpeed != 0 && gearNum > 0) gearText.text = "D";
		if (currSpeed == 0 && !(brakeInput > 0)) gearText.text = "N";
		if (currSpeed == 0 && handbrakeInput > 0) gearText.text = "P";
		if (currSpeed != 0 && gearNum < 0) gearText.text = "R";
	}

	void getInput ()
	{
		// THROTTLE INPUT
		if(DrivingMode == 0) // Manual Driving
		{
            if (HMIType == InputType.Default)
			{
                fwdInput = (Input.GetAxis("Vertical") > 0) ? ThrottleLimit * Input.GetAxis("Vertical") : 0;
                revInput = (Input.GetAxis("Vertical") < 0) ? ThrottleLimit * Input.GetAxis("Vertical") : 0;
                currentT = ThrottleLimit * Input.GetAxis("Vertical");
            }
            if (HMIType == InputType.F710)
            {
                fwdInput = (Input.GetAxis("Vertical F710") > 0) ? ThrottleLimit * Input.GetAxis("Vertical F710") : 0;
                revInput = (Input.GetAxis("Vertical F710") < 0) ? ThrottleLimit * Input.GetAxis("Vertical F710") : 0;
                currentT = ThrottleLimit * Input.GetAxis("Vertical F710");
            }
            if (HMIType == InputType.Xbox)
            {
                fwdInput = (Input.GetAxis("Vertical Xbox") > 0) ? ThrottleLimit * Input.GetAxis("Vertical Xbox") : 0;
                revInput = (Input.GetAxis("Vertical Xbox") < 0) ? ThrottleLimit * Input.GetAxis("Vertical Xbox") : 0;
                currentT = ThrottleLimit * Input.GetAxis("Vertical Xbox");
            }
            if (HMIType == InputType.G29)
            {
                fwdInput = (Input.GetAxis("Vertical G29") > 0) ? ThrottleLimit * Input.GetAxis("Vertical G29") : 0;
                revInput = (Input.GetAxis("Vertical G29") > 0) ? -ThrottleLimit * Input.GetAxis("Vertical G29") : 0;
				if (Input.GetAxis("Vertical G29") > 0)
				{
					currentT = (gearNum >= 0) ? ThrottleLimit * Input.GetAxis("Vertical G29") : -ThrottleLimit * Input.GetAxis("Vertical G29");
				}
				else currentT = 0;
            }

        }
        else // Autonomous Driving
		{
			fwdInput = (AutonomousThrottle > 0) ? AutonomousThrottle : 0;
			revInput = (AutonomousThrottle < 0) ? AutonomousThrottle : 0;
			currentT = ThrottleLimit * AutonomousThrottle;
		}

		// STEERING INPUT
		if(DrivingMode == 0) // Manual Driving
		{
			// Continuous control using mouse
			if (HMIType == InputType.Default && Input.GetMouseButton(0))
			{
				float MousePosition = Input.mousePosition.x; // Get the mouse position
				// Check if its the first time pressing down on mouse button
				if (!MouseHold)
				{
					MouseHold = true; // The mouse button is held down
					MouseStart = MousePosition;   // Set the reference position for tracking mouse movement
				}
				steerInput = -Mathf.Clamp((MousePosition - MouseStart)/(Screen.width/6), -1, 1); // Clamp the steering command in range of [-1, 1]
			}

			// Discrete control using keyboard
			else
			{
				MouseHold = false; // The mouse button is released
                if (HMIType == InputType.Default) steerInput = -Input.GetAxis("Horizontal");
                if (HMIType == InputType.F710) steerInput = -Input.GetAxis("Horizontal F710");
                if (HMIType == InputType.Xbox) steerInput = -Input.GetAxis("Horizontal Xbox");
                if (HMIType == InputType.G29) steerInput = -Input.GetAxis("Horizontal G29");
            }

			currentS = steerInput*maxSteerAngle*(Mathf.PI/180);
		}
		else // Autonomous Driving
		{
			steerInput = AutonomousSteering;
			currentS = AutonomousSteering*maxSteerAngle*(Mathf.PI/180);
		}

		// BRAKE INPUT
		if(DrivingMode == 0) // Manual Driving
		{
			if (HMIType == InputType.G29)
			{
                brakeInput = (Input.GetAxis("Vertical G29") < 0) ? -Input.GetAxis("Vertical G29") : 0;
				currentB = (Input.GetAxis("Vertical G29") < 0) ? -Input.GetAxis("Vertical G29") : 0;
			}
			else
			{
				brakeInput = Input.GetKey(KeyCode.X) ? 1.0f : 0.0f;
				currentB = Input.GetKey(KeyCode.X) ? 1.0f : 0.0f;
			}
		}
        else // Autonomous Driving
		{
			brakeInput = AutonomousBrake;
			currentB = AutonomousBrake;
		}

		// HANDBRAKE INPUT
		if(DrivingMode == 0) // Manual Driving
		{
			handbrakeInput = Input.GetKey(KeyCode.Space) ? 1.0f: 0.0f;
			currentH = Input.GetKey(KeyCode.Space) ? 1.0f: 0.0f;
		}
        else // Autonomous Driving
		{
			handbrakeInput = AutonomousHandbrake;
			currentH = AutonomousHandbrake;
		}
	}

	void moveCar ()
	{
		float leftWheelTorque = 0;
		float rightWheelTorque = 0;
		calcTorque ();
		if (carDriveType == driveType.AllWheelDrive) {
			outputTorque = totalTorque / 4;
			leftWheelTorque = outputTorque * (1 - Mathf.Clamp (differentialTorqueDrop * ((steerAngle < 0) ? -steerAngle : 0), 0, 0.9f));
			rightWheelTorque = outputTorque * (1 - Mathf.Clamp (differentialTorqueDrop * ((steerAngle > 0) ? steerAngle : 0), 0, 0.9f));
			wheelColliders [0].motorTorque = wheelColliders [2].motorTorque = leftWheelTorque;
			wheelColliders [1].motorTorque = wheelColliders [3].motorTorque = rightWheelTorque;
		} else if (carDriveType == driveType.FrontWheelDrive) {
			outputTorque = totalTorque / 2;
			for (int i = 0; i < 2; i++) {
				wheelColliders [i].motorTorque = outputTorque;
			}
		} else {
			outputTorque = totalTorque / 2;
			for (int i = 2; i < 4; i++) {
				wheelColliders [i].motorTorque = outputTorque;
			}
		}
	}

	void steerCar ()
	{
		float x = -steerInput * (maxSteerAngle - (currSpeed / topSpeed) * steerAngleLimitingFactor);
		float steerSpeed = steerSensitivity + (currSpeed / topSpeed) * speedDependencyFactor;

		steerAngle = Mathf.SmoothStep (steerAngle, x, steerSpeed);

		wheelColliders[0].steerAngle = Mathf.Rad2Deg*(Mathf.Atan((2*Wheelbase*Mathf.Tan(Mathf.Deg2Rad*(steerAngle)))/((2*Wheelbase)+(TrackWidth*Mathf.Tan(Mathf.Deg2Rad*(steerAngle))))));;
		wheelColliders[1].steerAngle = Mathf.Rad2Deg*(Mathf.Atan((2*Wheelbase*Mathf.Tan(Mathf.Deg2Rad*(steerAngle)))/((2*Wheelbase)-(TrackWidth*Mathf.Tan(Mathf.Deg2Rad*(steerAngle))))));;

		if (!isFlying ())
			car.AddRelativeTorque (transform.up * turnPower * currSpeed * -steerInput);
	}

	void brakeCar ()
	{
		// Hydraulic Disc Brake
		for (int i = 0; i < 4; i++) {
			wheelColliders [i].brakeTorque = brakeInput*brakingPower;
			// if (Input.GetKey (KeyCode.X)) {
			// 	wheelColliders [i].brakeTorque = brakingPower;
			// } else
			// 	wheelColliders [i].brakeTorque = 0;
		}

		// Emergency Brakes
		wheelColliders [2].brakeTorque = wheelColliders [3].brakeTorque = handbrakeInput*brakingPower;
		// if (Input.GetKey (KeyCode.Space))
		// 		wheelColliders [2].brakeTorque = wheelColliders [3].brakeTorque = brakingPower;
	}

	void driftCar ()
	{
		if (currSpeed > 0 && Mathf.Abs (-steerInput) > 0 && (brakeInput > 0 || handbrakeInput > 0) && (!isFlying ())) {
			float localDriftPower = handbrakeInput > 0 ? driftPower : 0.8f * driftPower;
			float torque = Mathf.Clamp (localDriftPower * -steerInput * currSpeed, -15000, 15000);
			car.AddRelativeTorque (transform.up * torque);
		}
	}

	void adjustFinalDrive ()
	{
		if (gearNum == 1 || gearNum == 4 || gearNum == 5) {
			local_finalDrive = finalDriveRatio1;
		} else {
			local_finalDrive = finalDriveRatio2;
		}
	}

	void calcTorque ()
	{
		acc = (gearNum == 1) ? Mathf.MoveTowards (0, 1 * fwdInput, thrAgg) : accleration;
		throttle = (gearNum == -1) ? revInput : fwdInput;
		shiftGear ();
		getEngineRPM ();
		totalTorque = torqueCurve.Evaluate (engineRPM) * (gearRatios.Evaluate (gearNum)) * local_finalDrive * throttle * acc;
		if (engineRPM >= maxGearChangeRPM)
			totalTorque = 0;
		tractionControl ();
	}

	void shiftGear ()
	{
		if (gearNum == 0 && ((HMIType != InputType.G29 && fwdInput > 0) || (HMIType == InputType.G29 && Input.GetKey(KeyCode.T))))
			gearNum = 1;
		if ((gearNum < gearRatios.length - 1 && engineRPM >= maxGearChangeRPM) && !isFlying () && checkGearSpeed ())
			gearNum++;
		if (gearNum > 1 && engineRPM <= minGearChangeRPM)
			gearNum--;
		if (checkStandStill () && ((HMIType != InputType.G29 && revInput < 0) || (HMIType == InputType.G29 && Input.GetKey(KeyCode.G))))
			gearNum = -1;
		if (gearNum == -1 && checkStandStill () && ((HMIType != InputType.G29 && fwdInput > 0) || (HMIType == InputType.G29 && Input.GetKey(KeyCode.T))))
			gearNum = 1;
	}

	bool checkGearSpeed ()
	{
		if (gearNum != -1) {
			if (checkSpeeds) {
				return currSpeed >= gearSpeeds [gearNum - 1];
			} else
				return true;
		} else
			return false;
	}

	void idlingRPM ()
	{
		iRPM = (gearNum > 1) ? 0 : idleRPM;
	}

	void getEngineRPM ()
	{
		idlingRPM ();
		getWheelRPM ();
		float velocity = currSpeed;
		engineRPM = Mathf.SmoothDamp (engineRPM, iRPM + (Mathf.Abs (wheelRPM) * local_finalDrive * gearRatios.Evaluate (gearNum)), ref velocity, smoothTime);
	}

	void getWheelRPM ()
	{
		float sum = 0;
		int c = 0;
		for (int i = 0; i < 4; i++) {
			if (wheelColliders [i].isGrounded) {
				sum += wheelColliders [i].rpm;
				c++;
			}
		}
		wheelRPM = (c != 0) ? sum / c : 0;
	}

	void getCarSpeed ()
	{
		currSpeed = Vector3.Dot (transform.forward.normalized, car.velocity);
		currSpeed *= speedMultiplier;
		currSpeed = Mathf.Round (currSpeed);
	}

	void animateWheels ()
	{
		Vector3 wheelPosition = Vector3.zero;
		Quaternion wheelRotation = Quaternion.identity;

		for (int i = 0; i < 4; i++) {
			wheelColliders [i].GetWorldPose (out wheelPosition, out wheelRotation);
			wheelMesh [i].position = wheelPosition;
			wheelMesh [i].rotation = wheelRotation;
		}
	}

	void animateSteeringWheel ()
	{
		steeringWheel.transform.localEulerAngles = new Vector3(steeringWheel.transform.localEulerAngles.x, steeringWheel.transform.localEulerAngles.y, steerAngle*steerRatio);
	}

	void adjustDrag ()
	{
		if (currSpeed >= topSpeed)
			car.drag = topSpeedDrag;
		else if (outputTorque == 0)
			car.drag = idleDrag;
		else if (currSpeed >= maxReverseSpeed && gearNum == -1 && wheelRPM <= 0)
			car.drag = reverseDrag;
		else {
			car.drag = runningDrag;
		}
	}

	bool isFlying ()
	{
		if (!wheelColliders [0].isGrounded && !wheelColliders [1].isGrounded && !wheelColliders [2].isGrounded && !wheelColliders [3].isGrounded) {
			return true;
		} else
			return false;
	}

	bool checkStandStill ()
	{
		if (currSpeed == 0) {
			return true;
		} else {
			return false;
		}
	}

	void calcTurnAngle ()
	{
		Vector3 flatForward = transform.forward;
		flatForward.y = 0;
		if (flatForward.sqrMagnitude > 0) {
			flatForward.Normalize ();
			Vector3 localFlatForward = transform.InverseTransformDirection (flatForward);
			turnAngle = Mathf.Atan2 (localFlatForward.x, localFlatForward.z) * turnCheckSense;
		}
	}

	bool isTurning ()
	{
		if (turnAngle > -turnRange && turnAngle < turnRange)
			return false;
		else
			return true;
	}

	void adjustForwardFriction ()
	{
		fwf = wheelColliders [0].forwardFriction;
		fwf.extremumValue = baseFwdExtremum + ((currSpeed <= 0) ? 0 : currSpeed / topSpeed) * forwardFrictionSpeedFactor;
		fwf.asymptoteValue = baseFwdAsymptote + ((currSpeed <= 0) ? 0 : currSpeed / topSpeed) * forwardFrictionSpeedFactor;

		fwf.extremumValue = Mathf.Clamp (fwf.extremumValue, baseFwdExtremum, 5);
		fwf.asymptoteValue = Mathf.Clamp (fwf.asymptoteValue, baseFwdAsymptote, 5);

		for (int i = 0; i < 4; i++) {
			wheelColliders [i].forwardFriction = fwf;
		}
	}

	void adjustSidewaysFriction ()
	{
		upClamp = Mathf.SmoothStep (upClamp, maxSidewaysFrictionValue, 0.2f);
		driftX = Mathf.Abs (transform.InverseTransformVector (car.velocity).x);
		float driftFactor = driftX * driftVelocityFactor;

		swf = wheelColliders [0].sidewaysFriction;

		float x = baseSideAsymptote + driftFactor;
		float y = baseSideExtremum + driftFactor;

		if (Mathf.Abs (currSpeed) < criticalDonutSpeed) {
			swf.stiffness = 0.8f;
		} else
			swf.stiffness = maxSidewaysStiffness;

		swf.asymptoteValue = Mathf.Clamp (x, baseSideAsymptote, maxSidewaysFrictionValue);
		swf.extremumValue = Mathf.Clamp (y, baseSideAsymptote, maxSidewaysFrictionValue);

		for (int i = 0; i < 4; i++) {
			wheelColliders [i].sidewaysFriction = swf;
		}
	}

	void steerHelper ()
	{
		localSteerHelper = Mathf.SmoothStep (localSteerHelper, _steerHelper * Mathf.Abs (-steerInput), 0.1f);
		if (brakeInput > 0) {
			_steerHelper *= -1;
		}

		foreach (WheelCollider wc in wheelColliders) {
			WheelHit wheelHit;
			wc.GetGroundHit (out wheelHit);
			if (wheelHit.normal == Vector3.zero)
				return;
		}

		if (Mathf.Abs (oldRotation - transform.eulerAngles.y) < 10) {
			float turnAdjust = (transform.eulerAngles.y - oldRotation) * _steerHelper;
			Quaternion velRotation = Quaternion.AngleAxis (turnAdjust, Vector3.up);
			car.velocity = velRotation * car.velocity;
		}

		oldRotation = transform.eulerAngles.y;
	}

	void addDownForce ()
	{
		car.AddForce (-transform.up * downForce * car.velocity.magnitude);
	}

	void adjustTorque (float forwardSlip)
	{
		if (forwardSlip >= longitudinalSlipLimit && currentTorque >= 0) {
			currentTorque -= 1000 * traction;
		} else {
			currentTorque += 1000 * traction;
			if (currentTorque >= totalTorque) {
				currentTorque = totalTorque;
			}
		}
	}

	void rotationalStabilizer ()
	{
		calcAngularAccl ();

		float reverseTorque = -1 * Mathf.Abs (angularAcclY) * revTorquePower * Mathf.Sign (car.angularVelocity.y) * (currSpeed / topSpeed);

		car.AddRelativeTorque (transform.up * reverseTorque);
	}

	void calcAngularAccl ()
	{
		angularAcclY = (prevAngularVelocity - car.angularVelocity.y) / Time.deltaTime;
		prevAngularVelocity = car.angularVelocity.y;
	}

	void tractionControl ()
	{
		WheelHit wheelHit;

		switch (carDriveType) {
		case driveType.AllWheelDrive:
			foreach (WheelCollider wc in wheelColliders) {
				wc.GetGroundHit (out wheelHit);
				adjustTorque (wheelHit.forwardSlip);
			}

			break;
		case driveType.RearWheelDrive:
			wheelColliders [2].GetGroundHit (out wheelHit);
			adjustTorque (wheelHit.forwardSlip);
			wheelColliders [3].GetGroundHit (out wheelHit);
			adjustTorque (wheelHit.forwardSlip);

			break;
		case driveType.FrontWheelDrive:
			wheelColliders [0].GetGroundHit (out wheelHit);
			adjustTorque (wheelHit.forwardSlip);
			wheelColliders [1].GetGroundHit (out wheelHit);
			adjustTorque (wheelHit.forwardSlip);

			break;
		}
	}

	// checks if the wheels are spinning and if so does three things
	// 1) emits particles
	// 2) plays tire skidding sounds
	// 3) leaves skidmarks on the ground
	// these effects are controlled through the WheelEffects class
	private void checkForWheelSpin()
	{
			// loop through all wheels
			for (int i = 0; i < 4; i++)
			{
					WheelHit wheelHit;
					wheelColliders[i].GetGroundHit(out wheelHit);

					// is the tire slipping above the given threshhold
					if (Mathf.Abs(wheelHit.forwardSlip) >= longitudinalSlipLimit || Mathf.Abs(wheelHit.sidewaysSlip) >= lateralSlipLimit)
					{
							if (wheelColliders[i].rpm <= lockRPM && Mathf.Abs(wheelHit.forwardSlip) >= longitudinalSlipAudioThreshold*longitudinalSlipLimit || Mathf.Abs(wheelHit.sidewaysSlip) >= lateralSlipAudioThreshold*lateralSlipLimit)
							{
									// avoiding all four tires screeching at the same time
									// if they do it can lead to some strange audio artefacts
									if (!AnySkidSoundPlaying())
									{
											m_WheelEffects[i].PlayAudio();
									}
									m_WheelEffects[i].EmitTyreSmoke();
							}
							continue;
					}

					// if it wasnt slipping stop all the audio
					if (m_WheelEffects[i].PlayingAudio)
					{
							m_WheelEffects[i].StopAudio();
					}
					// end the trail generation
					m_WheelEffects[i].EndSkidTrail();
			}
	}

	private bool AnySkidSoundPlaying()
	{
			for (int i = 0; i < 4; i++)
			{
					if (m_WheelEffects[i].PlayingAudio)
					{
							return true;
					}
			}
			return false;
	}
}
