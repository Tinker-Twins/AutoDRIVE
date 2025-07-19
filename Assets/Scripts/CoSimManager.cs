using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CoSimManager : MonoBehaviour
{
    /*
    This script smoothly updates the vehicle states based on targets
    provided by an external co-simulation application. This script 
    should be attached to the senior-most parent of the `Vehicle`
    game object, whose states are to be updated.
    */

    public Rigidbody VehicleRigidBody;
    public bool Teleport = false;
    public float CoSimTimer = 0.0f;
    public float CoSimSmoothness = 3f;
    public Vector3 CoSimPosition;
    public Quaternion CoSimRotation;

    private Vector3 position;
    private Quaternion rotation; 

    void Start()
    {

    }

    void Update()
    {   if (Teleport)
        {
            VehicleRigidBody.position = CoSimPosition;
            VehicleRigidBody.rotation = CoSimRotation;
        }
        else
        {
            position = Vector3.Slerp(transform.position, CoSimPosition, CoSimTimer/CoSimSmoothness);
            rotation = Quaternion.Slerp(transform.rotation, CoSimRotation, CoSimTimer/CoSimSmoothness);
            VehicleRigidBody.MovePosition(position);
            VehicleRigidBody.MoveRotation(rotation);
            CoSimTimer += Time.deltaTime;
            if(CoSimTimer >= CoSimSmoothness) CoSimTimer = CoSimSmoothness;
        }
    }
}
