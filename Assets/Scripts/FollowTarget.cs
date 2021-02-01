using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FollowTarget : MonoBehaviour
{
    /*
    This script makes one object `Follower` to follow the other `Target` from above.
    The X and Z position of the `Follower` is updated according to that of the `Target`.
    The Y position is updated w.r.t the mouse scroll-wheel input, which can be used as
    a zooming control. None of the orientations are updated, as they feel really
    discomfortable to the viewer.
    */

    public GameObject Target;
    public GameObject Follower;
    public float ZoomOutLimit = 5.0f;
    public float ZoomInLimit = 0.5f;

    // Update is called once per frame
    void Update()
    {
        var zoom = Input.GetAxis("Mouse ScrollWheel"); // Get input from mouse scroll-wheel

        if(zoom < 0)
        {
            Follower.transform.position = new Vector3(Target.transform.position.x, Follower.transform.position.y+0.1f, Target.transform.position.z); // Follow the target while zooming in
        }
        else if(zoom > 0)
        {
            Follower.transform.position = new Vector3(Target.transform.position.x, Follower.transform.position.y-0.1f, Target.transform.position.z); // Follow the target while zooming out
        }
        else
        {
            Follower.transform.position = new Vector3(Target.transform.position.x, Follower.transform.position.y, Target.transform.position.z); // Follow the target from a constant height
        }

        if(Follower.transform.position.y < ZoomInLimit)
        {
            Follower.transform.position = new Vector3(Target.transform.position.x, ZoomInLimit, Target.transform.position.z); // Follow the target from a constant height
        }
        if(Follower.transform.position.y > ZoomOutLimit)
        {
            Follower.transform.position = new Vector3(Target.transform.position.x, ZoomOutLimit, Target.transform.position.z); // Follow the target from a constant height
        }
    }
}
