using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CollisionDetector : MonoBehaviour
{

    public Vector3 contactPoint;
    
    void OnCollisionEnter(Collision other)
    {
        // Print how many points are colliding with this transform
        // Debug.Log("Points colliding: " + other.contacts.Length);

        // Print the normal of the first point in the collision.
        // Debug.Log("Normal of the first point: " + other.contacts[0].normal);

        // Print the normal of the first point in the collision.
        // Debug.Log("First point: " + other.contacts[0].point);
        contactPoint = other.contacts[0].point;

        // Draw a different colored ray for every normal in the collision
        foreach (var item in other.contacts)
        {
            Debug.DrawRay(item.point, item.normal * 100, Random.ColorHSV(0f, 1f, 1f, 1f, 0.5f, 1f), 10f);
        }
    }
    
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
