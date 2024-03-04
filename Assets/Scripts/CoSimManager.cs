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

    public float CoSimTimer = 0.0f;
    public float CoSimSmoothness = 3f;
    public Vector3 CoSimPosition;
    public Quaternion CoSimRotation;

    void Start()
    {

    }

    void Update()
    {
        transform.position = Vector3.Slerp(transform.position, CoSimPosition, CoSimTimer/CoSimSmoothness);
        transform.rotation = Quaternion.Slerp (transform.rotation, CoSimRotation, CoSimTimer/CoSimSmoothness);
        CoSimTimer += Time.deltaTime;
        if(CoSimTimer >= CoSimSmoothness) CoSimTimer = CoSimSmoothness;
    }
}
