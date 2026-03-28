#if (UNITY_EDITOR) 
using UnityEngine;
using UnityEditor;
using kawetofe.randomPrefabPlacer;
using System.Collections.Generic;
using System.Reflection;

namespace kawetofe.randomPrefabPlacer
{



    [CustomEditor(typeof(RandomPrefabPlacer),true)]
    public class RandomObjectPlacerEditor : Editor
    {
        RandomPrefabPlacer placer;
        bool multiplePlacementObjects = true;
        bool showStandardButtons = true;        
        float brushFlowRate = .2f;

        private Texture icon_brush, icon_options, icon_flat, icon_spherical, icon_bin, icon_sync,icon_lock;
        private GUIContent icon_brush_con, icon_options_con, icon_flat_con, icon_spherical_con, icon_bin_con, icon_sync_con,icon_lock_con, icon_static_con;

        private int selectionInt = 0;
        private int surfaceMode = 0;
        int selectedBrush = 0;

        Texture2D brushImg;

        public void OnEnable()
        {
            placer = (RandomPrefabPlacer)target;
            selectionInt = placer.lastBrushSetting;

            if (placer.ignoreSlopeAngle)
            {
                surfaceMode = 1;
            }
            else
            {
                surfaceMode = 0;
            }
            selectedBrush = placer.selectedBrushId;
            brushImg = (Texture2D)this.icon_brush;
            //DrawMainGUI();
             this.icon_brush = (Texture)AssetDatabase.LoadAssetAtPath("Assets/Plugins/RandomPrefabPlacer/RandomPrefabPlacerV2/scripts/editorScripts/Images/kw_icon_brush.png",typeof(Texture));
            this.icon_brush_con = new GUIContent(icon_brush);
            this.icon_options = (Texture)AssetDatabase.LoadAssetAtPath("Assets/Plugins/RandomPrefabPlacer/RandomPrefabPlacerV2/scripts/editorScripts/Images/kw_icon_options.png", typeof(Texture));
            this.icon_options_con = new GUIContent(icon_options);
            this.icon_flat = (Texture)AssetDatabase.LoadAssetAtPath("Assets/Plugins/RandomPrefabPlacer/RandomPrefabPlacerV2/scripts/editorScripts/Images/kw_icon_flat.png", typeof(Texture));
            this.icon_flat_con = new GUIContent(icon_flat);
            this.icon_spherical = (Texture)AssetDatabase.LoadAssetAtPath("Assets/Plugins/RandomPrefabPlacer/RandomPrefabPlacerV2/scripts/editorScripts/Images/kw_icon_spherical.png", typeof(Texture));
            this.icon_spherical_con = new GUIContent(icon_spherical);
            this.icon_bin = (Texture)AssetDatabase.LoadAssetAtPath("Assets/Plugins/RandomPrefabPlacer/RandomPrefabPlacerV2/scripts/editorScripts/Images/kw_icon_bin.png", typeof(Texture));
            this.icon_bin_con = new GUIContent(icon_bin);
            this.icon_sync = (Texture)AssetDatabase.LoadAssetAtPath("Assets/Plugins/RandomPrefabPlacer/RandomPrefabPlacerV2/scripts/editorScripts/Images/kw_icon_sync.png", typeof(Texture));
            this.icon_sync_con = new GUIContent(icon_sync);
            this.icon_lock = (Texture)AssetDatabase.LoadAssetAtPath("Assets/Plugins/RandomPrefabPlacer/RandomPrefabPlacerV2/scripts/editorScripts/Images/kw_icon_lock.png", typeof(Texture));
            this.icon_lock_con = new GUIContent(icon_lock);
            this.icon_static_con = new GUIContent(icon_lock);
            icon_options_con.tooltip = "Change the brush settings";
            icon_brush_con.tooltip = "Draw with the selected RPPBrush";
            icon_brush_con.text = "Draw Mode";
            icon_options_con.text = "Edit Brush";
            icon_flat_con.tooltip = "Draw on Landscape like surfaces";
            icon_spherical_con.tooltip = "Draw on surfaces with any angle";
            icon_bin_con.text = "Remove All";
            icon_bin_con.tooltip = "Remove all objects set in RPPBrush";
            icon_sync_con.text = "Sync with prefab";
            icon_sync_con.tooltip = "Synchronize objects with prefabs set in RPPBrush";
            icon_lock_con.text = "lock placed objects";
            icon_lock_con.tooltip = "locked objects will no longer have a connection to this placer tool";
            icon_static_con.text = "Static Mode";
            icon_static_con.tooltip = "Use this for edit the brush or when you don't want to use the brush for placing objects";
            DestroyImmediate(GameObject.Find("unassigned"));
        }


        public override void OnInspectorGUI()
        {
           
            
            

            GUILayout.Label("Random Prefab Placer v2.0");
            GUILayout.Label("for help visit");
            EditorGUILayout.BeginHorizontal();
            if (GUILayout.Button("www.kawetofe.com"))
            {
                Application.OpenURL("http://www.kawetofe.com/wordpress/random-prefab-placer/");
            }
            if (GUILayout.Button(" rate it \u2730\u2730\u2730\u2730\u2730"))
            {
                Application.OpenURL("https://assetstore.unity.com/packages/tools/terrain/random-prefab-placer-104765");
            }
            EditorGUILayout.EndHorizontal();
           
            EditorGUILayout.Space();
          
            


            /// Brush Selection Toggle
            GUIContent[] icon_content = { icon_brush_con, icon_static_con };//, icon_options_con};
           

            selectionInt = GUI.SelectionGrid(EditorGUILayout.GetControlRect(false, 30), selectionInt, icon_content, 2);
           


            if (selectionInt == 0)
            {
                placer.brushMode = true;
            }
            else
            {
                placer.brushMode = false;
            }

            if(surfaceMode == 0)
            {
                placer.ignoreSlopeAngle = false;
            } else if(surfaceMode == 1)
            {
                placer.ignoreSlopeAngle = true;
            }

           


            /// Different Menus
            if (!placer.brushMode)
            {
                if(placer.rppBrush != null)
                EditorGUILayout.HelpBox("- > Use 'LMB' to place objects\n - > use right mouse button 'RMB' to move placement sphere\n - > press 'left control' to remove all objects inside sphere \n - > switch to draw mode to draw with the selected prefab brush", MessageType.Info);
                DrawMainGUI();


            } else if(placer.brushMode)
            {
                EditorGUILayout.HelpBox(" - > use left mouse button (LMB) to place objects \n - > use CTRL+LMB to delete objects \n - > use RMB to change placement object", MessageType.Info);
                DrawMainGUI();

                
            }

            EditorGUILayout.Separator();
            EditorGUILayout.Separator();
            if (showStandardButtons)
            {
                EditorGUILayout.BeginHorizontal();
            
                if (!placer.brushMode)
                {

                    if (GUILayout.Button("Place Objects"))
                    {
                        placer.PlaceObjects();
                    }
                }

               
                
                if (GUILayout.Button(icon_lock_con,GUILayout.Height(20), GUILayout.Width(150)))
                {
                    if (EditorUtility.DisplayDialog("Make objects permanent?", "This will make all placed objects on this surface permanent.\n The brush tool will no longer have access to them, are you sure you want to do that?", "Yes", "No"))
                    {
                        //placer.ReconnectToPrefabs();
                        placer.MakeObjectsPermanent();
                    }
                }

                if(GUILayout.Button("Clear Cache", GUILayout.Height(20), GUILayout.Width(150)))
                {
                    placer.RefreshObjList();
                    
                }
            
                EditorGUILayout.EndHorizontal();
            

                /***
                 * DEPRECATED**
                 * 
                 * */
                //EditorGUILayout.BeginHorizontal();

                //if (GUILayout.Button(icon_sync_con,GUILayout.Height(30)))
                //{
                //    if (EditorUtility.DisplayDialog("Warning", "All placed objects which are present in the RPPBrush will by synced to the original prefab (except transform values), Are you sure you want to do this?", "Yes", "No"))
                //    {
                //        //placer.SynchronizeWithPrefabChanges();
                //        placer.ReconnectToPrefabs();
                        
                        
                        
                        
                //    }
                //}

                //EditorGUILayout.EndHorizontal();
                
                
            }


        }

        Vector2 scrollPos;
        
        private void DrawMainGUI()
        {
           

            // Choose RPPBrush 
            //EditorGUILayout.BeginHorizontal();
            //Texture2D brushImg = (Texture2D)this.icon_brush;
           // EditorGUILayout.EndHorizontal();
            
            if (placer.rppBrush != null)
            {
                if (placer.rppBrush.prefabs.Count > 0)
                {
                    EditorGUILayout.BeginHorizontal();
                    int j = 0;
                    foreach (PlacementPrefab o in placer.rppBrush.prefabs)
                    {
                        brushImg = AssetPreview.GetAssetPreview(o.prefab);
                        if (brushImg != null)
                        {
                            GUI.DrawTexture(EditorGUILayout.GetControlRect(false, 50, GUILayout.Width(50)), brushImg);
                            j++;
                        }
                        if (j % 6 ==0)
                        {
                            EditorGUILayout.EndHorizontal();
                            EditorGUILayout.BeginHorizontal();
                        }

                    }
                    EditorGUILayout.EndHorizontal();
                    
                }
                else
                {
                    EditorGUILayout.LabelField(placer.rppBrush.name);
                }
                EditorGUILayout.LabelField(placer.rppBrush.name);
            }
            else
            {
                EditorGUI.HelpBox(EditorGUILayout.GetControlRect(false, 100), "There are no prefab brushes present \n Please create a prefab brush asset to start.", MessageType.Error);
            }
            
            EditorGUILayout.BeginHorizontal();
            EditorGUI.LabelField(EditorGUILayout.GetControlRect(), "Brush Selection");
            EditorGUILayout.EndHorizontal();
            if (!placer.brushMode) {
                EditorGUILayout.BeginHorizontal();
                if (GUILayout.Button(icon_options_con, GUILayout.Height(30), GUILayout.Width(100)))
                {
                    InspectBrush(placer.rppBrush);
                }
                EditorGUILayout.EndHorizontal();
            }

            EditorGUILayout.Separator();
            GUIStyle style1 = new GUIStyle();
            style1.padding.left = 10;
            style1.padding.right = 10;
            

            List<RPPBrush> brushes = new List<RPPBrush>();
            List<GUIContent> brushesGuiContent = new List<GUIContent>();
            brushes.Clear();
            brushesGuiContent.Clear();
            string[] brushesDir = AssetDatabase.FindAssets("t:RPPBrush", null);
            if(brushesDir.Length > 0){
            foreach (string s in brushesDir)
            {
                RPPBrush brush = AssetDatabase.LoadAssetAtPath(AssetDatabase.GUIDToAssetPath(s), typeof(RPPBrush)) as RPPBrush;
                brushes.Add(brush);
                GUIContent brushGuiCont = new GUIContent();
                if (brush.prefabs.Count > 0){
                    brushGuiCont.image = AssetPreview.GetAssetPreview(brush.prefabs[0].prefab);}
                    else {
                        brushGuiCont.image = (Texture2D)this.icon_brush;
                    }
                brushGuiCont.tooltip = brush.name;
                if(brushGuiCont.image == null)
                    brushGuiCont.text = brush.name;
                brushesGuiContent.Add(brushGuiCont);
            }
            EditorGUILayout.BeginHorizontal(style1);
            scrollPos = EditorGUILayout.BeginScrollView(scrollPos, GUILayout.Height(150));//brushes.Count / 4 * 50
            int h = 100;
            if(brushes.Count > 3){
                h = brushes.Count/3 * 100;
            }
            selectedBrush = GUI.SelectionGrid(EditorGUILayout.GetControlRect(false, h), selectedBrush, brushesGuiContent.ToArray(), 4);
            placer.selectedBrushId = selectedBrush;
            placer.rppBrush = brushes[selectedBrush];
            EditorGUILayout.EndScrollView();
            EditorGUILayout.EndHorizontal();
            EditorGUILayout.Space();
            EditorGUILayout.Space();
            // END Choose RPPBrush
            GUIContent[] surface_content = { icon_flat_con, icon_spherical_con };
            // Spheric or Flat Mode
            EditorGUILayout.BeginHorizontal();
            EditorGUILayout.Separator();
            EditorGUILayout.LabelField("Surface Type");
            surfaceMode = GUI.SelectionGrid(EditorGUILayout.GetControlRect(false, 30), surfaceMode, surface_content, 2);
            EditorGUILayout.EndHorizontal();
            // END Spheric or Flat Mode
            EditorGUILayout.Space();
            EditorGUILayout.Space();
            

            EditorGUILayout.BeginHorizontal();
            multiplePlacementObjects = EditorGUILayout.Toggle("Draw on multiple objects", multiplePlacementObjects);
           
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.BeginHorizontal();
            float sliderValue = EditorGUILayout.Slider(new GUIContent("Brush Size"), placer.placementRadius, .1f, 80f);
            placer.placementRadius = sliderValue;
            EditorGUILayout.EndHorizontal();
            EditorGUILayout.BeginHorizontal();
            int numberOfObjects = (int)EditorGUILayout.Slider(new GUIContent("Number of objects"), placer.objectsToPlace, 1, 100);
            placer.objectsToPlace = numberOfObjects;
            EditorGUILayout.EndHorizontal();
            EditorGUILayout.BeginHorizontal();
            float remAlpha = EditorGUILayout.Slider(new GUIContent("Alpha for removing"), placer.removalAlpha, 1f, 100f);
            placer.removalAlpha = remAlpha;
            EditorGUILayout.EndHorizontal();
            EditorGUILayout.BeginHorizontal();
            placer.eraseAll = EditorGUILayout.Toggle("Ignore brush when removing", placer.eraseAll);
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.BeginHorizontal();

            if (placer.rppBrush != null)
            {

                placer.prefabs.Clear();
                foreach (PlacementPrefab p in placer.rppBrush.prefabs)
                {
                    placer.prefabs.Add(p);
                }

            }
            else
            {
                EditorGUILayout.LabelField("Please insert RPPBrush to start");
            }

            EditorGUILayout.EndHorizontal();
            }
        }


        /// <summary>
        /// Pick active placement object to place prefabs to.
        /// </summary>
        public void PickObjectToPlacePrefabs()
        {
            // Shoot a ray from the mouse position into the world
            Ray worldRay = HandleUtility.GUIPointToWorldRay(Event.current.mousePosition);
            RaycastHit hitInfo;
            // Shoot this ray. check in a distance of 10000.
            if (Physics.Raycast(worldRay, out hitInfo, 10000))
            {
                if (!hitInfo.collider.gameObject.name.Contains("(RPPClone)"))
                {
                    placer.objectToPlacePrefabs = hitInfo.collider.transform;
                    Vector3 worldPoint = hitInfo.point;
                    placer.placementCircleCenter.transform.position = worldPoint;
                    placer.tempHitNormal = hitInfo.normal;
                }
            }
        }

        /// <summary>
        /// Find Object Position()
        /// </summary>
       private void FindObjectPosition(){
            if (multiplePlacementObjects)
                        {
                            PickObjectToPlacePrefabs();
                        }

                        RaycastHit[] hits;

                        // Shoot a ray from the mouse position into the world
                        Ray worldRay = HandleUtility.GUIPointToWorldRay(Event.current.mousePosition);
                         hits = Physics.RaycastAll(worldRay,10000);
                         foreach(RaycastHit h in hits){
                             if(h.collider.transform == placer.objectToPlacePrefabs){
                                Vector3 worldPoint = h.point;
                                placer.placementCircleCenter.transform.position = worldPoint;
                                break;
                             }
                         }
       }

        /// <summary>
        /// OnSceneGUI 
        /// </summary>
        public void OnSceneGUI()
        {
           

            if (placer != null)
            {
                HandleUtility.AddDefaultControl(GUIUtility.GetControlID(FocusType.Passive));
                
                // if circle center GameObject is missing
                if (placer.placementCircleCenter == null || placer.placementCircleCenter == placer.objectToPlacePrefabs)
                {
                    Transform circleCenter = (Transform)Instantiate(new GameObject(), placer.objectToPlacePrefabs.position, Quaternion.identity).transform;
                    circleCenter.name = "RPPCircleCenter";
                    circleCenter.SetParent(placer.transform);
                    placer.placementCircleCenter = circleCenter;
                }

                // if placer is not in brush mode
                if (!placer.brushMode)

                {
                    placer.lastBrushSetting = 1;
                    if (Event.current.control)
                    {
                        HandleUtility.AddDefaultControl(GUIUtility.GetControlID(FocusType.Passive));
                    }
                    if (Event.current.type == EventType.MouseDown && Event.current.button == 1 )//&& Event.current.control)
                    {
                       
                        PickObjectToPlacePrefabs();
                    }

                    if (Event.current.type == EventType.MouseDown && Event.current.button == 0 && Event.current.keyCode != KeyCode.LeftControl)
                    {
                        float tempAlpha = placer.removalAlpha;
                        float tempRadius = placer.placementRadius;
                        placer.placementRadius = tempRadius*1.2f;
                        placer.removalAlpha = 100f;
                        placer.RemovePlacedObjectsInCircle();
                        placer.removalAlpha = tempAlpha;
                        placer.placementRadius = tempRadius;
                        placer.PlaceObjects();
                    }

                    if (Event.current.keyCode == KeyCode.LeftControl)
                    {
                        float tempRadius = placer.placementRadius;
                        placer.placementRadius = tempRadius * 1.2f;
                        float tempAlpha = placer.removalAlpha;
                        placer.removalAlpha = 100f;
                        placer.RemovePlacedObjectsInCircle();
                        placer.removalAlpha = tempAlpha;
                        placer.placementRadius = tempRadius;

                    }
                }

                // if placer is in Brush mode
                if (placer.brushMode)
                {
                    placer.lastBrushSetting = 0;
                    HandleUtility.AddDefaultControl(GUIUtility.GetControlID(FocusType.Passive));

                   
                    
                    
                    // Event on mouse move
                    if (Event.current.type == EventType.MouseMove)
                    {
                        FindObjectPosition();

                         
                    }

                    if (Event.current.type == EventType.MouseDrag  && Event.current.button == 0 && !Event.current.alt && !Event.current.control)
                        {
                                FindObjectPosition();
                                placer.PlaceObjectWithFlowRate(flowRate:brushFlowRate);
                        }


                    // Place Objects on Mouse Button down
                    if (Event.current.type == EventType.MouseDown && Event.current.button == 0 && !Event.current.alt && !Event.current.control)
                    {
                       
                        placer.PlaceObjects();
                    }

                    

                    // Draw differen Gizmo Color when in removal mode
                    if (Event.current.control)
                    {
                        placer.removeMode = true;
                    } else
                    {
                        placer.removeMode = false;
                    }

                    // Remove Objects inside Circle
                    if (Event.current.type == EventType.MouseDown && Event.current.button == 0 && Event.current.control)
                    {
                        placer.RemovePlacedObjectsInCircle();
                    } 

                     if (Event.current.type == EventType.MouseDrag && Event.current.button == 0 && Event.current.control)
                    {
                       FindObjectPosition();
                        placer.RemovePlacedObjectsInCircleWithFlowRate(flowRate:brushFlowRate);
                    }

                    if (Event.current.type == EventType.MouseDown && Event.current.button == 1)
                    {
                        PickObjectToPlacePrefabs();
                    }

                    if(Event.current.type == EventType.MouseUp && Event.current.button == 0 && !Event.current.control)
                    {
                        //placer.ReconnectToPrefabs();
                        
                    }


                  
                    SceneView.RepaintAll();
                } // End if __editMode
            }
        }

        /// <summary>
        /// Menu Item
        /// </summary>
        [MenuItem("AutoDRIVE/Terrain Tools/Random Prefab Placer")]
        public static void InstantiateRPPBrushTool()
        {
            GameObject rppBrush = (GameObject)Resources.Load("RPPBrushTool");
            GameObject cloneObj = Instantiate(rppBrush, Vector3.zero, Quaternion.identity);
            cloneObj.name = cloneObj.name.Replace("(Clone)", "");
        }


        /// <summary>
        /// Creates a new inspector window instance and locks it to inspect the specified target
        /// </summary>
        public static void InspectBrush(RPPBrush target)
        {
            // Get a reference to the `InspectorWindow` type object
            var inspectorType = typeof(Editor).Assembly.GetType("UnityEditor.InspectorWindow");
            // Create an InspectorWindow instance
            var inspectorInstance = ScriptableObject.CreateInstance(inspectorType) as EditorWindow;
            // We display it - currently, it will inspect whatever gameObject is currently selected
            // So we need to find a way to let it inspect/aim at our target GO that we passed
            // For that we do a simple trick:
            // 1- Cache the current selected gameObject
            // 2- Set the current selection to our target GO (so now all inspectors are targeting it)
            // 3- Lock our created inspector to that target
            // 4- Fallback to our previous selection
            inspectorInstance.Show();
            // Cache previous selected gameObject
            var prevSelection = Selection.activeGameObject;
            // Set the selection to GO we want to inspect
            Selection.activeObject = target;
            // Get a ref to the "locked" property, which will lock the state of the inspector to the current inspected target
            var isLocked = inspectorType.GetProperty("isLocked", BindingFlags.Instance | BindingFlags.Public);
            // Invoke `isLocked` setter method passing 'true' to lock the inspector
            isLocked.GetSetMethod().Invoke(inspectorInstance, new object[] { true });
            // Finally revert back to the previous selection so that other inspectors continue to inspect whatever they were inspecting...
            Selection.activeGameObject = prevSelection;

            // Now you just:
            //InspectTarget(myGO);
        }

    }

}
#endif
