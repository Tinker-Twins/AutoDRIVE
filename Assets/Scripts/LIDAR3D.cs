using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RGLUnityPlugin;
using UnityEngine.Profiling;

public class LIDAR3D : MonoBehaviour
{

    public bool publishPCL24 = true; // PCL24 message format
    public bool publishPCL48 = true; // PCL48 message format

    public MeshRenderer[] TerrainMeshes; // Terrain mesh gameobject references

    public byte[] PointcloudData;
    // public uint Width;
    // public uint RowStep;

    public byte[] CurrentPointcloud{get{return PointcloudData;}}
    // public uint CurrentPCWidth{get{return Width;}}
    // public uint CurrentPCRowStep{get{return RowStep;}}

        
    private LidarSensor lidarSensor;
    private RGLNodeSequence rglSubgraphLidar;
    private RGLNodeSequence rglSubgraphPcl24;
    private RGLNodeSequence rglSubgraphPcl48;
    private byte[] pcl24Data; // Point cloud data in PCL24 format
    private byte[] pcl48Data; // Point cloud data in PCL48 format

    // Note: The matrix here is written as-if on paper,
    // but Unity's Matrix4x4 is constructed from column-vectors, hence the transpose.
    private Matrix4x4 LidarTF = new Matrix4x4(
            new Vector4( 0.0f, 0.0f, 1.0f, 0.0f),
            new Vector4(-1.0f, 0.0f, 0.0f, 0.0f),
            new Vector4( 0.0f, 1.0f, 0.0f, 0.0f),
            new Vector4( 0.0f, 0.0f, 0.0f, 1.0f)
        ).transpose;


    void Start()
    {

        if (!publishPCL24 && !publishPCL48)
        {
            Debug.LogWarning("All LIDAR message formats are disabled!");
        }

        lidarSensor = GetComponent<LidarSensor>();
        lidarSensor.onNewData += OnNewLidarData;

        rglSubgraphLidar = new RGLNodeSequence()
                .AddNodePointsTransform("LIDAR", LidarTF);
        lidarSensor.ConnectToLidarFrame(rglSubgraphLidar);

        if (publishPCL24)
        {
            pcl24Data = new byte[0];
            rglSubgraphPcl24 = new RGLNodeSequence()
                .AddNodePointsFormat("PCL24", FormatPCL24.GetRGLFields());
            RGLNodeSequence.Connect(rglSubgraphLidar, rglSubgraphPcl24);
        }

        if (publishPCL48)
        {
            pcl48Data = new byte[0];
            rglSubgraphPcl48 = new RGLNodeSequence()
                .AddNodePointsFormat("PCL48", FormatPCL48.GetRGLFields());
            RGLNodeSequence.Connect(rglSubgraphLidar, rglSubgraphPcl48);
        }

    }

    private void OnNewLidarData()
    {
        UnityEngine.Profiling.Profiler.BeginSample("Publish Pointclouds");
        if (publishPCL24)
        {
            int hitCount = rglSubgraphPcl24.GetResultDataRaw(ref pcl24Data, 24);
            PointcloudData = pcl24Data;
            // Width = (uint) hitCount;
            // RowStep = 24 * Width;
        }

        if (publishPCL48)
        {
            int hitCount = rglSubgraphPcl48.GetResultDataRaw(ref pcl48Data, 48);
            PointcloudData = pcl48Data;
            // Width = (uint) hitCount;
            // RowStep = 48 * Width;
        }
        UnityEngine.Profiling.Profiler.EndSample();

        if(TerrainMeshes.Length !=0 && TerrainMeshes[0].enabled == true)
        {
            for(int i=0;i<TerrainMeshes.Length;i++)
            {
                TerrainMeshes[i].enabled = false;
                // Debug.Log("Disabled Terrain Mesh " + i.ToString());

            }
        }
    }
}

public static class FormatPCL24
    { 
        public static RGLField[] GetRGLFields()
        {
            return new[]
            {
                RGLField.XYZ_F32,
                RGLField.PADDING_32,
                RGLField.INTENSITY_F32,
                RGLField.RING_ID_U16,
                RGLField.PADDING_16
            };
        }
    }

    public static class FormatPCL48
    {
        public static RGLField[] GetRGLFields()
        {
            return new[]
            {
                RGLField.XYZ_F32,
                RGLField.PADDING_32,
                RGLField.INTENSITY_F32,
                RGLField.RING_ID_U16,
                RGLField.PADDING_16,
                RGLField.AZIMUTH_F32,
                RGLField.DISTANCE_F32,
                RGLField.RETURN_TYPE_U8,
                RGLField.PADDING_8,
                RGLField.PADDING_16,
                RGLField.PADDING_32,
                RGLField.TIME_STAMP_F64
            };
        }
    }