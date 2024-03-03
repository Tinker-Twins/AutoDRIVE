using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

public class NavmeshAgent : MonoBehaviour
{
    NavMeshAgent Agent; // `NavMeshAgent` instance
    
    [Header("Click Destination")]
    public bool ClickDestination = false;
    public float ClickRaycastRange = 100;

    [Header("Random Destination")]
    public bool RandomDestination = false;
    public GameObject RandomRaycaster;
    public float RandomRaycastRange = 2000;
    public float xMinLimit = 0f;
    public float xMaxLimit = 0f;
    public float yMinLimit = 0f;
    public float yMaxLimit = 0f;
    public float zMinLimit = 0f;
    public float zMaxLimit = 0f;
    public float rollMinLimit = 0f;
    public float rollMaxLimit = 0f;
    public float pitchMinLimit = 0f;
    public float pitchMaxLimit = 0f;
    public float yawMinLimit = 0f;
    public float yawMaxLimit = 0f;
    private int navmeshMask;
    private Vector3 position;
    private Vector3 orientation;

    void Start()
    {
        Agent = GetComponent<NavMeshAgent>();
    }

    void Update()
    {
        if(ClickDestination && Input.GetMouseButtonDown(0))
        {
            RaycastHit physicsHit; // Instantiate a raycast hit object
            
            if(Physics.Raycast(Camera.main.ScreenPointToRay(Input.mousePosition), out physicsHit, ClickRaycastRange))
            {
                // Debug.Log(physicsHit.point);
                Agent.destination = physicsHit.point;
                // Debug.Log("Updated agent's destination!");
            }
        }
        else if(RandomDestination)
        {
            if(Agent.hasPath && Agent.remainingDistance >= 10)
            {
                // Debug.Log("Agent is busy traversing the current path!");
                return; // Do not do anything if the agent is already moving to a destination and not close to its destination
            }
            else
            {
                Debug.Log("Updating agent's destination randomly!");

                RandomizePose(RandomRaycaster); // Randomize raycaster position

                RaycastHit physicsHit; // Instantiate a raycast hit object
                
                if(Physics.Raycast(RandomRaycaster.transform.position, Vector3.down, out physicsHit, RandomRaycastRange))
                {
                    NavMeshHit navmeshHit;
                    int navmeshMask = 1 << NavMesh.GetAreaFromName("Not Walkable");
                    if(!NavMesh.SamplePosition(physicsHit.point, out navmeshHit, 1.0f, navmeshMask))
                    {
                        // Debug.Log(physicsHit.point);
                        Agent.destination = physicsHit.point;
                        // Debug.Log("Updated agent's destination!");
                    }
                    else
                    {
                        Debug.Log("Raycast returned a position that is not reachable by the agent!");
                    }
                }
                else
                {
                    Debug.Log("Raycast did not return any hit!");
                }
            }
        }
    }

    void RandomizePose(GameObject gameobject)
    {
        position = new Vector3(Random.Range(xMinLimit, xMaxLimit), Random.Range(yMinLimit, yMaxLimit), Random.Range(zMinLimit, zMaxLimit));
        orientation = new Vector3(Random.Range(rollMinLimit, rollMaxLimit), Random.Range(pitchMinLimit, pitchMaxLimit), Random.Range(yawMinLimit, yawMaxLimit));
        gameobject.transform.SetPositionAndRotation(position, Quaternion.Euler(orientation));
    }
}
