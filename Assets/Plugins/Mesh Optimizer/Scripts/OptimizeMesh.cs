using System.Collections;
using System.Collections.Generic;
using UnityEngine;

#if UNITY_EDITOR
using UnityEditor;
[CustomEditor(typeof(OptimizeMesh))]
public class LevelScriptEditor : Editor
{
    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();
        OptimizeMesh myTarget = (OptimizeMesh)target;

        if (GUILayout.Button("Optimize Mesh!"))
        {
            myTarget.DecimateMesh();
        }

        if (GUILayout.Button("Save Mesh!"))
        {
            myTarget.SaveMesh();
        }

    }
}
#endif
[ExecuteInEditMode]
public class OptimizeMesh : MonoBehaviour
{
    [Range(0.0f, 1.0f)]
    [SerializeField] float _quality = 0.5f;
    MeshFilter _renderer;
    Mesh _mesh;
    void Start()
    {
        _renderer = GetComponent<MeshFilter>();
        _mesh = _renderer.sharedMesh;
    }
#if UNITY_EDITOR
    // Update is called once per frame
    void Update()
    {
        if (Input.GetKeyDown(KeyCode.O))
        {
            DecimateMesh();
        }
    }

    public void DecimateMesh()
    {
        if (!EditorApplication.isPlaying)
        {
            var meshSimplifier = new UnityMeshSimplifier.MeshSimplifier();
            meshSimplifier.Initialize(_mesh);
            meshSimplifier.SimplifyMesh(_quality);
            var destMesh = meshSimplifier.ToMesh();
            _renderer.sharedMesh = destMesh;
        }
    }

    public void SaveMesh()
    {
        if (!EditorApplication.isPlaying)
        {
            MeshSaverEditor.SaveMesh(_renderer.sharedMesh, "Optimized__" + gameObject.name, false, true);
        }
    }
#endif
}
