using System;
using System.Collections.Generic;
using UnityEngine;

public class PointcloudMapper : MonoBehaviour
{

    [SerializeField]
    [Tooltip("Gameobject containing 3D LIDAR to capture pointcloud.")]
    private GameObject vehicleGameObject;

    [SerializeField]
    [Tooltip("Result PCD file name. On Editor/Windows, it will be saved in Assets/")]
    private string outputPCDFilePath = "Map.pcd";

    [SerializeField]
    [Tooltip("World origin in RHS coordinate system, will be added to every point coordinates")]
    private Vector3 mapOrigin; // Right-handed frame (X=Z_Unity, Y=-X_Unity, Z=Y_Unity)

    private PointcloudMappingAdapter mappingSensor;

    private void Start()
    {
        // TODO: Support multiple sensors
        mappingSensor = vehicleGameObject.GetComponentInChildren<PointcloudMappingAdapter>();
        if (mappingSensor == null)
        {
            Debug.LogError($"Could not find mapping sensor in {vehicleGameObject.name}. Disabling PointCloudMapper!");
            enabled = false;
            return;
        }

        Debug.Log($"Found mapping sensor in {vehicleGameObject.name}: {mappingSensor.GetSensorName()}");

        mappingSensor.Initialize(mapOrigin, $"{Application.dataPath}/{outputPCDFilePath}");
    }

    private void Update()
    {
        if (Input.GetKeyDown(KeyCode.Q))
        {
            SavePCD();
            enabled = false;
            return;
        }

        mappingSensor.Capture();
    }

    public void OnDestroy()
    {
        if (enabled)
        {
            SavePCD();
        }
    }

    private void SavePCD()
    {
        Debug.Log($"Writing PCD to {Application.dataPath}/{outputPCDFilePath}");
        mappingSensor.SavePCD();
        Debug.Log("PCL data saved successfully!");
    }
}