using System;
using RGLUnityPlugin;
using UnityEngine;

public class PointcloudMappingAdapter : MonoBehaviour
{
    [SerializeField]
    [Tooltip("Resolution to sub-sample point cloud data. Set leaf size to 0 if you don't want to sub-sample.")]
    private float leafSize;

    private bool isInitialized = false;

    private LidarSensor lidarSensor;

    private RGLNodeSequence rglSubgraphMapping;

    // Note: The matrix here is written as-if on paper,
    // but Unity's Matrix4x4 is constructed from column-vectors, hence the transpose.
    private Matrix4x4 WorldTF = new Matrix4x4(
            new Vector4( 0.0f, 0.0f, 1.0f, 0.0f),
            new Vector4(-1.0f, 0.0f, 0.0f, 0.0f),
            new Vector4( 0.0f, 1.0f, 0.0f, 0.0f),
            new Vector4( 0.0f, 0.0f, 0.0f, 1.0f)
        ).transpose;

    public void Awake()
    {
        lidarSensor = GetComponent<LidarSensor>();
        // Make sure automatic capture in RGL Lidar Sensor is disabled.
        // We want to perform captures only on demand (after warping).
        lidarSensor.AutomaticCaptureHz = 0;
    }

    public void Initialize(Vector3 mapOrigin, string outputPcdFilePath)
    {
        if (isInitialized)
        {
            throw new Exception("Attempted to initialize PointcloudMappingAdapter more than once!");
        }

        // Create and connect subgraph
        Matrix4x4 worldTransform = WorldTF;
        worldTransform.SetColumn(3, worldTransform.GetColumn(3) + (Vector4) mapOrigin);
        rglSubgraphMapping = new RGLNodeSequence()
            .AddNodePointsTransform("WORLD_TF", worldTransform);

        if (leafSize > 0.0f)
        {
            rglSubgraphMapping.AddNodePointsDownsample("DOWNSAMPLE", new Vector3(leafSize, leafSize, leafSize));
        }

        rglSubgraphMapping.AddNodePointsWritePCDFile("WRITE_PCD", outputPcdFilePath);

        lidarSensor.ConnectToWorldFrame(rglSubgraphMapping);

        isInitialized = true;
    }

    public string GetSensorName()
    {
        return gameObject.name;
    }

    public void SavePCD()
    {
        if (rglSubgraphMapping == null)
        {
            Debug.LogWarning("Skipped saving PCD file - empty point cloud!");
            return;
        }
        rglSubgraphMapping.Clear();
    }

    public void Capture()
    {
        if (!isInitialized)
        {
            throw new Exception("Attempted to run PointcloudMappingAdapter without initialization!");
        }

        lidarSensor.Capture();
    }
}