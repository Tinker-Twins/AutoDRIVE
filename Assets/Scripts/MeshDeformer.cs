using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MeshDeformer : MonoBehaviour
{
    public float radius = 0.25f;
    public float deformationStrength = 5f;
    public float smoothingFactor = 1f;
    private Mesh mesh;
    private Vector3[] orgVertices, modVertices;

    // Start is called before the first frame update
    void Start()
    {
        mesh = GetComponentInChildren<MeshFilter>().mesh;
        orgVertices = mesh.vertices;
        modVertices = mesh.vertices;
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
        RaycastHit hit;
        Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);

        if(Physics.Raycast(ray, out hit, Mathf.Infinity))
        {
            for(int v=0; v<modVertices.Length; v++)
            {
                Vector3 distance = modVertices[v] - hit.point;
                Debug.Log(hit.point);

                float force = deformationStrength / (1f+ hit.point.sqrMagnitude);

                if(distance.sqrMagnitude < radius)
                {
                    if(Input.GetMouseButton(0))
                    {
                        modVertices[v] = modVertices[v] + (Vector3.up * force) / smoothingFactor;
                    }
                    else if(Input.GetMouseButton(1))
                    {
                        modVertices[v] = modVertices[v] + (Vector3.down * force) / smoothingFactor;
                    }
                }
            }
        }
        RecalculateMesh();
    }
}