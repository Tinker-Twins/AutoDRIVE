using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FrictionMap : MonoBehaviour
{
    public Texture2D frictionMap; // Friction map (loaded as 2D image, e.g. BMP, JPG, PNG, etc.)

    public int mapWidth = 256; // Width of the friction map in image coordinates (px)
    public int mapHeight = 256; // Width of the friction map in image coordinates (px)
    public float mapScale = 10.0f; // Scale to convert world coordinates to map coordinates
    public float worldOffsetX = 0.0f; // Offset between map and world origins, along X-axis (m)
    public float worldOffsetY = 0.0f; // Offset between map and world origins, along Y-axis (m)
    
    void Start()
    {
        // Optionally, load the friction map from a file at runtime
        // string filePath = "path_to_your_friction_map.png";
        // LoadFrictionMap(filePath);
    }

    public float FrictionLookup(Vector3 position)
    {
        // Convert the world position to map coordinates
        int xIndex = Mathf.FloorToInt(position.x * mapScale);
        int zIndex = Mathf.FloorToInt(position.z * mapScale);

        // Make sure the indices are within the bounds of the friction map
        xIndex = Mathf.Clamp(xIndex, 0, frictionMap.width - 1);
        zIndex = Mathf.Clamp(zIndex, 0, frictionMap.height - 1);

        // Get the pixel color at the (xIndex, zIndex) from the friction map
        Color pixelColor = frictionMap.GetPixel(xIndex, zIndex);

        // Assuming grayscale friction values, convert the pixel color to a friction value (0 to 1 range)
        float friction = pixelColor.grayscale;  // Get the grayscale value, assuming the friction is stored in grayscale

        // Log the friction value
        // Debug.Log($"Friction: {friction} at position ({xIndex}, {zIndex})");
        // Debug.Log($"Friction: {friction} at position ({position.z}, {-position.x})");

        return friction;
    }

    // Function to load the friction map from a file
    void LoadFrictionMap(string filePath)
    {
        byte[] fileData = System.IO.File.ReadAllBytes(filePath);
        frictionMap = new Texture2D(2, 2); // Create a new texture of the appropriate size
        frictionMap.LoadImage(fileData);   // Load the image into the texture
    }
}
