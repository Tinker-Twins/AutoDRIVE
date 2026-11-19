using System.Collections.Generic;
using UnityEngine;

public class NeoRacerController : VehicleController
{
    /*
    The NeoRacer as neoracer-mujoco models it (Neobotics-Foundation-Inc/neoracer-mujoco, which
    describes the car once, in its spec neoracer_spec), in place of VehicleController's
    generic drive and steering:
    - The firmware's speed loop. The throttle command is a speed: MaxSpeed at 1, as the driver's
      /motor speed is. Every LoopPeriod the car's ESP32 measures the speed from the drivetrain's
      angle (the motor encoder), with a filter of its own, and sets the ESC's pulse: a
      feed-forward on the commanded speed plus a PID on the error. Under StopSpeed it sends
      neutral. (neoracer_mujoco/speed_loop.py and encoder.py)
    - The ESC and the motor. The pulse over FullThrottlePulse is the share of the battery's
      voltage that the motor gets. A DC motor, its torque the stall torque x (voltage / rated
      voltage - speed / free speed), turns all four wheels through its gear and open
      differentials: each wheel gets a quarter of the torque, and the motor turns at the wheels'
      mean speed. At neutral it brakes.
    - The steering servo. A DC motor with its own position loop (voltage = ServoKp x error -
      ServoKd x speed, clamped to its supply) turns the steering's inertia against its damping
      and dry friction, each command held back ServoLatency s. Full lock is SteerLock rad of the
      virtual (bicycle) wheel; the Ackermann geometry turns it into the two front wheels' angles.
    NeoRacerPhysics (menu AutoDRIVE > NeoRacer: Physics from the Spec) sets all of these from the
    spec, with the body's mass, center of mass and inertia, the wheels and the springs.
    Not modelled: the tires' scrub, which slows the steering of a car at rest.
    */

    [Header("Firmware speed loop")]
    public float MaxSpeed = 6f; // m/s that a throttle of 1 commands
    public float LoopPeriod = 0.02f; // s
    public float SpeedFilter = 0.15f; // Weight of a new window's speed in the measured speed
    public float LoopKp = 447f; // us of pulse per m/s of error
    public float LoopKi = 4.7f; // us per m of the error's integral
    public float LoopKd = 4.7f; // us per m/s^2 of the error's change
    public float IntegralLimit = 1000f; // m
    public float ErrorDeadband = 0.05f; // m/s
    public float StopSpeed = 0.05f; // m/s
    public float FeedForward = 68f; // us per m/s
    public float FeedForwardDeadband = 90f; // us
    public float FeedForwardLimit = 500f; // us
    public float PulseLimit = 500f; // us
    public float FullThrottlePulse = 500f; // us

    [Header("Motor")]
    public float BatteryVoltage = 11.1f; // V
    public float MotorRatedVoltage = 12f; // V
    public float MotorStallTorque = 0.0937957f; // N-m at the rated voltage
    public float MotorFreeSpeed = 1151.92f; // rad/s at the rated voltage
    public float GearRatio = 10.55f; // Motor turns per wheel turn
    // Each wheel's drivetrain losses, N-m per rad/s. Unity holds a wheel collider's damping at
    // 0.001 or more, so each wheel is given back what its damping takes beyond this
    public float DriveDamping = 0.0005f;

    [Header("Steering servo")]
    public float ServoVoltage = 6f; // V, its supply: its rating's and its loop's clamp
    public float ServoStallTorque = 1.99442f; // N-m at the steering
    public float ServoFreeSpeed = 7.99757f; // rad/s at the steering
    public float ServoKp = 50f; // V per rad
    public float ServoKd = 1f; // V per rad/s
    public float ServoLatency = 0.045f; // s
    public float SteerLock = 0.440491f; // rad of the virtual wheel at full lock
    public float SteerInertia = 0.00607779f; // kg-m^2
    public float SteerDamping = 0.003f; // N-m per rad/s
    public float SteerFriction = 0.03f; // N-m

    [Header("Body")]
    public Vector3 Inertia = new Vector3(0.0152808f, 0.0200685f, 0.00721822f); // kg-m^2 about x, y, z (right, up, forward)

    // Tire force sub-steps per physics step. A wheel is light and turns freely, so its spin and
    // its tire's grip answer each other faster than one 1 ms physics step resolves: in one step
    // the wheels chatter (at 1 m/s the car's acceleration swung by 17 m/s^2), in 10 they do not
    // (0.14 m/s^2). MuJoCo, which solves contacts implicitly, has no such limit.
    public int TireSubsteps = 10;

    // The tires' solid colliders, there so a wheel hits what stands beside it (a wheel collider
    // only feels the ground below). They do not touch the ground the wheel colliders stand on:
    // the wheel colliders are the tires there, as MuJoCo's wheels are, and a second, rigid tire
    // rubbing the floor 2.5 mm under it dragged the car and shook it
    public Collider[] TireColliders = new Collider[0];
    private readonly HashSet<Collider> grounds = new HashSet<Collider>();

    private float simTime; // s
    private readonly Queue<(float time, float target)> steerCommands = new Queue<(float time, float target)>();
    private float steerTarget; // rad, + = left: the command the servo acts on now
    private float steer; // rad of the virtual wheel, + = left
    private float steerRate; // rad/s

    private float drivetrainAngle; // rad: the wheels' mean angle, which the motor encoder counts
    private float windowAngle; // rad, when the encoder's window opened
    private float windowSpan; // s since it opened
    private float windowDue; // s until the firmware's next tick
    private float measuredSpeed; // m/s, the speed loop's
    private float loopIntegral; // m
    private float loopError; // m/s at the last tick
    private float escThrottle; // Share of the battery's voltage the motor gets, in [-1, 1]

    protected override void Start()
    {
        base.Start();
        VehicleRigidBody.inertiaTensor = Inertia;
        VehicleRigidBody.inertiaTensorRotation = Quaternion.identity;
        FrontLeftWheelCollider.ConfigureVehicleSubsteps(1000f, TireSubsteps, TireSubsteps); // the same at every speed
        windowDue = LoopPeriod;
    }

    protected override void Steer()
    {
        float dt = Time.fixedDeltaTime;
        simTime += dt;
        float share = DrivingMode == 0 ? SteeringInput : AutonomousSteering; // + = left
        steerCommands.Enqueue((simTime, Mathf.Clamp(share, -1f, 1f) * SteerLock));
        while (steerCommands.Count > 0 && simTime - steerCommands.Peek().time >= ServoLatency - 1e-6f)
            steerTarget = steerCommands.Dequeue().target;

        float volts = Mathf.Clamp(ServoKp * (steerTarget - steer) - ServoKd * steerRate, -ServoVoltage, ServoVoltage);
        float torque = ServoStallTorque * (volts / ServoVoltage - steerRate / ServoFreeSpeed) - SteerDamping * steerRate;
        if (steerRate == 0f && Mathf.Abs(torque) <= SteerFriction)
        {
            // Dry friction holds the steering still
        }
        else
        {
            float before = steerRate;
            float friction = SteerFriction * Mathf.Sign(before != 0f ? before : torque);
            steerRate += (torque - friction) / SteerInertia * dt;
            // Friction stops the steering; it never turns it back
            if (before != 0f && Mathf.Sign(steerRate) != Mathf.Sign(before)) steerRate = 0f;
        }
        steer += steerRate * dt;

        SteeringAngle = -steer * Mathf.Rad2Deg; // Unity's wheels: + = right
        float tan = Mathf.Tan(Mathf.Deg2Rad * SteeringAngle);
        FrontLeftWheelCollider.steerAngle = Mathf.Rad2Deg * Mathf.Atan((2 * Wheelbase * tan) / ((2 * Wheelbase) + (TrackWidth * tan)));
        FrontRightWheelCollider.steerAngle = Mathf.Rad2Deg * Mathf.Atan((2 * Wheelbase * tan) / ((2 * Wheelbase) - (TrackWidth * tan)));
        RearLeftWheelCollider.steerAngle = 0f;
        RearRightWheelCollider.steerAngle = 0f;
    }

    protected override void Drive()
    {
        float dt = Time.fixedDeltaTime;
        float wheelSpeed = (FrontLeftWheelCollider.rpm + FrontRightWheelCollider.rpm + RearLeftWheelCollider.rpm +
                            RearRightWheelCollider.rpm) / 4f * 2f * Mathf.PI / 60f; // rad/s, the drivetrain's
        drivetrainAngle += wheelSpeed * dt;
        SpeedLoop(dt);

        float volts = escThrottle * BatteryVoltage;
        float motorTorque = MotorStallTorque * (volts / MotorRatedVoltage - GearRatio * wheelSpeed / MotorFreeSpeed);
        float perWheel = GearRatio * motorTorque / 4f;
        // Stopped at neutral, the wheels are held, with as much torque as the motor can give them.
        // Unity's wheel colliders do not hold a car still on the flat (it crept 9 mm/s); MuJoCo's
        // contacts, solved exactly, do, as a real drivetrain's own friction does
        bool held = escThrottle == 0f && Mathf.Abs(wheelSpeed * WheelRadius) < StopSpeed;
        float hold = GearRatio * MotorStallTorque * BatteryVoltage / MotorRatedVoltage / 4f;
        foreach (WheelCollider wheel in new[] { FrontLeftWheelCollider, FrontRightWheelCollider,
                                                RearLeftWheelCollider, RearRightWheelCollider })
        {
            float extraDamping = (wheel.wheelDampingRate - DriveDamping) * wheel.rpm * 2f * Mathf.PI / 60f;
            wheel.motorTorque = held ? 0f : perWheel + extraDamping;
            wheel.brakeTorque = held ? hold : 0f;
            if (wheel.GetGroundHit(out WheelHit hit) && grounds.Add(hit.collider))
                foreach (Collider tire in TireColliders) Physics.IgnoreCollision(tire, hit.collider);
        }
        DriveTorque = escThrottle * MotorTorque; // The telemetry's throttle is the ESC's
    }

    // The firmware's speed loop after a physics step of dt (neoracer_mujoco/speed_loop.py)
    private void SpeedLoop(float dt)
    {
        windowSpan += dt;
        windowDue -= dt;
        if (windowDue >= 1e-6f) return; // The encoder's window is open: no tick
        float raw = (drivetrainAngle - windowAngle) * WheelRadius / windowSpan;
        measuredSpeed = SpeedFilter * raw + (1f - SpeedFilter) * measuredSpeed;
        windowAngle = drivetrainAngle;
        windowSpan = 0f;
        windowDue += LoopPeriod;

        float commanded = (DrivingMode == 0 ? ThrottleInput : AutonomousThrottle) * MaxSpeed;
        if (Mathf.Abs(commanded) < StopSpeed)
        {
            loopIntegral = 0f;
            loopError = 0f;
            escThrottle = 0f;
            return;
        }
        float error = commanded - measuredSpeed;
        if (Mathf.Abs(error) < ErrorDeadband) error = 0f;
        loopIntegral = Mathf.Clamp(loopIntegral + error * LoopPeriod, -IntegralLimit, IntegralLimit);
        float derivative = (error - loopError) / LoopPeriod;
        loopError = error;
        float pid = LoopKp * error + LoopKi * loopIntegral + LoopKd * derivative;
        float pulse = Mathf.Clamp(FeedForwardPulse(commanded) + (float)System.Math.Truncate(pid), -PulseLimit, PulseLimit);
        escThrottle = Mathf.Clamp(pulse / FullThrottlePulse, -1f, 1f);
    }

    // The feed-forward's pulse (us) for a commanded speed at or over StopSpeed
    private float FeedForwardPulse(float commanded)
    {
        float pulse = Mathf.Floor(FeedForwardDeadband + Mathf.Abs(commanded) * FeedForward + 0.5f);
        pulse = Mathf.Clamp(pulse, FeedForwardDeadband, FeedForwardLimit);
        return commanded > 0f ? pulse : -pulse;
    }
}
