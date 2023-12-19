using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AutoMeshDeformer : MonoBehaviour
{
    // public GameObject GeometricModel;
    public Rigidbody[] RigidBodies;
    public CollisionDetector[] Colliders;
    
    public float radius = 0.5f;
    public float deformationStrength = 5f;
    public float smoothingFactor = 250f;

    private float mass = 0f;
    private Mesh mesh;
    private Vector3[] orgVertices, modVertices;

    // Start is called before the first frame update
    void Start()
    {
        mesh = GetComponentInChildren<MeshFilter>().mesh;
        orgVertices = mesh.vertices;
        modVertices = mesh.vertices;
        // RigidBody = GeometricModel.GetComponent<Rigidbody>();
    }

    void RecalculateMesh()
    {
        mesh.vertices = modVertices;
        GetComponentInChildren<MeshCollider>().sharedMesh = mesh;
        mesh.RecalculateNormals();
    }

    // Update is called once per frame
    void Update()
    {
        mass = 0f;
        // RaycastHit hit;
        // Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);

        // if(Physics.Raycast(ray, out hit, Mathf.Infinity))
        // {
            foreach(var Collision in Colliders)
            {
                Vector3 contactPoint = Collision.contactPoint;
                // Debug.Log("Contact Point: " + contactPoint);

                for(int v=0; v<modVertices.Length; v++)
                {
                    Vector3 distance = modVertices[v] - contactPoint;
                    // Debug.Log("Distance: " + contactPoint + " from vertex: " + v);

                    for(int i=0; i<RigidBodies.Length; i++)
                    {
                        mass += RigidBodies[i].mass;
                    }
                    mass = mass/RigidBodies.Length;

                    float force = deformationStrength*mass;
                    // Debug.Log("Force: " + force + " at vertex: " + v);

                    if(distance.sqrMagnitude < radius)
                    {
                        // if(Input.GetMouseButton(0))
                        // {
                        //     modVertices[v] = modVertices[v] + (Vector3.up * force) / smoothingFactor;
                        // }
                        // else if(Input.GetMouseButton(1))
                        // {
                        //     modVertices[v] = modVertices[v] + (Vector3.down * force) / smoothingFactor;
                        // }
                        Debug.Log("Force: " + force + " at vertex: " + v);
                        
                        modVertices[v] = modVertices[v] + (Vector3.down * force) / smoothingFactor;
                    }
                }
            }
        // }
        RecalculateMesh();
    }
}