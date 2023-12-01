using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GPS : MonoBehaviour
{
    /*
    This script attaches a GNSS receiver to a specified component. The simulated
    sensor returns the global position of the specified component's `Transform`.
    */

    public Transform VehicleTransform;
    private float[] VehiclePosition = new float[3];

    public float[] CurrentPosition{get{return VehiclePosition;}}

    void FixedUpdate()
    {
        VehiclePosition[0] = VehicleTransform.position.z;
        VehiclePosition[1] = -VehicleTransform.position.x;
        VehiclePosition[2] = VehicleTransform.position.y;
        //Debug.Log("Position [x: " + VehiclePosition[0] + " y: " + VehiclePosition[1] + " z: " + VehiclePosition[2] + "]");
    }
}
