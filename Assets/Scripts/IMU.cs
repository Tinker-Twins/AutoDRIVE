using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class IMU : MonoBehaviour
{
    /*
    This script attaches an inertial measurement unit (IMU) to a specified component.
    The simulated 9-DOF IMU measures the following inertial data:

    - Local orientation of the specified `Transform` represented as a Quternion [x,y,z,w]
      and Euler Angle triplet [x,y,z]. The unit for Euler Angle representation is rad.

    - Angular velocity of the specified `Rigidbody` about its local axes [x,y,z] in rad/s.

    - Linear acceleration of the specified `Rigidbody` along its local axes [x,y,z] in m/s^2.

    Note: In order to measure inertial data of a single object, it is recommended that the
    `Rigidbody` and `Transform` components both belong to the said parent object.
    */

    public Transform VehicleTransform;
    public Rigidbody VehicleRigidBody;

    private Quaternion OrientationQuaternion = new Quaternion (0,0,0,0);
    private Vector3 EulerAngles = new Vector3 (0,0,0);
    private Vector3 OrientationEulerAngles = new Vector3 (0,0,0);
    private Vector3 AngularVelocity = new Vector3 (0,0,0);
    private Vector3 CurrentVelocity = new Vector3 (0,0,0);
    private Vector3 PreviousVelocity = new Vector3 (0,0,0);
    private Vector3 LinearAcceleration = new Vector3 (0,0,0);

    private float[] OrientationQuaternionArray = new float[4];
    private float[] OrientationEulerAnglesArray = new float[3];
    private float[] LinearVelocityArray = new float[3];
    private float[] AngularVelocityArray = new float[3];
    private float[] LinearAccelerationArray = new float[3];

    public float[] CurrentOrientationQuaternion{get{return OrientationQuaternionArray;}}
    public float[] CurrentOrientationEulerAngles{get{return OrientationEulerAnglesArray;}}
    public float[] CurrentLinearVelocity{get {return LinearVelocityArray;}}
    public float[] CurrentAngularVelocity{get{return AngularVelocityArray;}}
    public float[] CurrentLinearAcceleration{get{return LinearAccelerationArray;}}

    void FixedUpdate()
    {
        // Orientation (Quaternion)
        OrientationQuaternion = VehicleTransform.localRotation;
        OrientationQuaternionArray[0] = -OrientationQuaternion.z;
        OrientationQuaternionArray[1] = OrientationQuaternion.x;
        OrientationQuaternionArray[2] = -OrientationQuaternion.y;
        OrientationQuaternionArray[3] = OrientationQuaternion.w;
        //Debug.Log("Quaternion [x: " + OrientationQuaternionArray[0] + " y: " + OrientationQuaternionArray[1] + " z: " + OrientationQuaternionArray[2] + " w: " + OrientationQuaternionArray[3] + "]");

        // Orientation (Euler Angles)
        EulerAngles = VehicleTransform.localRotation.eulerAngles;
        if(System.Math.Round(EulerAngles.z,2)==0) OrientationEulerAngles.x = EulerAngles.z;
        else OrientationEulerAngles.x = (360f-EulerAngles.z)*(Mathf.PI/180);
        OrientationEulerAnglesArray[0] = OrientationEulerAngles.x;
        if(System.Math.Round(EulerAngles.x,2)==0) OrientationEulerAngles.y = EulerAngles.x;
        else OrientationEulerAngles.y = (EulerAngles.x)*(Mathf.PI/180);
        OrientationEulerAnglesArray[1] = OrientationEulerAngles.y;
        if(System.Math.Round(EulerAngles.y,2)==0) OrientationEulerAngles.z = EulerAngles.y;
        else OrientationEulerAngles.z = (360f-EulerAngles.y)*(Mathf.PI/180);
        OrientationEulerAnglesArray[2] = OrientationEulerAngles.z;
        //Debug.Log("Euler Angles [x: " + OrientationEulerAnglesArray[0] + " y: " + OrientationEulerAnglesArray[1] + " z: " + OrientationEulerAnglesArray[2] + "]");

        // Angular Velocity (rad/s)
        AngularVelocity = VehicleRigidBody.transform.InverseTransformDirection(VehicleRigidBody.angularVelocity);
        AngularVelocityArray[0] = -AngularVelocity.z;
        AngularVelocityArray[1] = AngularVelocity.x;
        AngularVelocityArray[2] = -AngularVelocity.y;
        //Debug.Log("Angular Velocity [x: " + AngularVelocityArray[0] + " y: " + AngularVelocityArray[1] + " z: " + AngularVelocityArray[2] + "]");

        // Linear Velocity (m/s) & Acceleration (m/s^2)
        CurrentVelocity = VehicleRigidBody.transform.InverseTransformDirection(VehicleRigidBody.velocity);
        LinearVelocityArray[0] = CurrentVelocity.z;
        LinearVelocityArray[1] = -CurrentVelocity.x;
        LinearVelocityArray[2] = CurrentVelocity.y;
        LinearAcceleration = (CurrentVelocity - PreviousVelocity)/(Time.deltaTime);
        PreviousVelocity = CurrentVelocity;
        LinearAccelerationArray[0] = LinearAcceleration.z;
        LinearAccelerationArray[1] = -LinearAcceleration.x;
        LinearAccelerationArray[2] = LinearAcceleration.y;
        //Debug.Log("Linear Velocity [x: " + LinearVelocityArray[0] + " y: " + LinearVelocityArray[1] + " z: " + LinearVelocityArray[2] + "]");
        //Debug.Log("Linear Acceleration [x: " + LinearAccelerationArray[0] + " y: " + LinearAccelerationArray[1] + " z: " + LinearAccelerationArray[2] + "]");
    }
}
