using Chisel.Components;
using NUnit.Framework.Constraints;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq.Expressions;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnitySceneExtensions;
using Grid = UnitySceneExtensions.Grid;

namespace Chisel.Editors
{

    public class UVWindow : EditorWindow
    {
        [MenuItem("Chisel DEBUG/UVWindow")]
        static void Create()
        {
            var window = (UVWindow)EditorWindow.GetWindow(typeof(UVWindow), false, "UV Window");
            window.autoRepaintOnSceneChange = true;
        }

        struct ChannelInfo
        {
            public Vector2 min;
            public Vector2 max;

            public void Reset()
            {
                min = Vector2.zero;
                max = Vector2.one;
            }
        }

        const int kChannelCount = 8;

        //List<Vector2> uvs;
        Grid grid;
        MeshFilter[]    selectedModels;
        ChannelInfo[]   channels        = new ChannelInfo[kChannelCount];
        static readonly GUIContent[]  channelNames    = new GUIContent[kChannelCount]
        {
            new GUIContent("0"), 
            new GUIContent("1"),
            new GUIContent("2"),
            new GUIContent("3"),
            new GUIContent("4"),
            new GUIContent("5"),
            new GUIContent("6"),
            new GUIContent("7")
        };
        
        class Styles
        {
            public GUIStyle[] leftButton    = new GUIStyle[2];
            public GUIStyle[] midButton     = new GUIStyle[2];
            public GUIStyle[] rightButton   = new GUIStyle[2];

            public Styles()
            {
                leftButton[0] = new GUIStyle(EditorStyles.miniButtonLeft) { stretchWidth = false, stretchHeight = true };
                leftButton[0].padding.top += 3;
                leftButton[0].padding.bottom += 3;
                leftButton[0].padding.left += 3;
                leftButton[0].padding.right += 4;
                leftButton[0].fixedHeight += 5;

                leftButton[1] = new GUIStyle(leftButton[0]);
                leftButton[1].normal = leftButton[0].active;

                {
                    var color = leftButton[0].normal.textColor;
                    color.a *= 0.5f;
                    leftButton[0].normal.textColor = color;
                }


                midButton[0] = new GUIStyle(EditorStyles.miniButtonMid) { stretchWidth = false, stretchHeight = true };
                midButton[0].padding.top += 3;
                midButton[0].padding.bottom += 3;
                midButton[0].padding.left += 3;
                midButton[0].padding.right += 4;
                midButton[0].fixedHeight += 5;

                midButton[1] = new GUIStyle(midButton[0]);
                midButton[1].normal = midButton[0].active;

                {
                    var color = midButton[0].normal.textColor;
                    color.a *= 0.5f;
                    midButton[0].normal.textColor = color;
                }


                rightButton[0] = new GUIStyle(EditorStyles.miniButtonRight) { stretchWidth = false, stretchHeight = true };
                rightButton[0].padding.top += 3;
                rightButton[0].padding.bottom += 3;
                rightButton[0].padding.left += 3;
                rightButton[0].padding.right += 4;
                rightButton[0].fixedHeight += 5;

                rightButton[1] = new GUIStyle(rightButton[0]);
                rightButton[1].normal = rightButton[0].active;

                {
                    var color = rightButton[0].normal.textColor;
                    color.a *= 0.5f;
                    rightButton[0].normal.textColor = color;
                }
            }
        };

        static Styles styles;
        //static private Scene m_Scene;
        //static private Camera m_Camera;


        private void OnEnable()
        {
            OnSelectionChange();
            if (grid == null)
                grid = new Grid();

            /*
            if (!m_Camera ||
                m_Scene == null || !m_Scene.IsValid())
            {
                m_Scene = EditorSceneManager.NewPreviewScene();
            }
            if (!m_Camera)
            { 
                var camGO = EditorUtility.CreateGameObjectWithHideFlags("Preview Scene Camera", HideFlags.HideAndDontSave, typeof(Camera));
                SceneManager.MoveGameObjectToScene(camGO, m_Scene);
                m_Camera = camGO.GetComponent<Camera>();
                m_Camera.cameraType = CameraType.Preview;
                m_Camera.enabled = false;
                m_Camera.clearFlags = CameraClearFlags.Depth | CameraClearFlags.Color;
                m_Camera.orthographic = true;
                m_Camera.fieldOfView = 15;
                m_Camera.farClipPlane = 10.0f;
                m_Camera.nearClipPlane = 2.0f;
                m_Camera.renderingPath = RenderingPath.Forward;
                m_Camera.transform.rotation = Quaternion.LookRotation(Vector3.up);
                m_Camera.useOcclusionCulling = false;
                m_Camera.scene = m_Scene;
            }*/
        }

        private void OnSelectionChange()
        {
            selectedModels = Selection.GetFiltered<MeshFilter>(SelectionMode.Deep);
            //uvs = new List<Vector2>();
            for (int i = 0; i < channels.Length; i++)
            {
                channels[i].Reset();                
                FindBounds(selectedModels, ref channels[i], i);
            }

            //foreach(var selectedModel in selectedModels)
            //    uvs.AddRange(Unwrapping.GeneratePerTriangleUV(selectedModel.sharedMesh));

            this.Repaint();
        }

        private void FindBounds(MeshFilter[] selectedModels, ref ChannelInfo info, int channel)
        {
            if (selectedModels == null || selectedModels.Length == 0)
                return;

            var min = Vector2.zero;
            var max = Vector2.one;
            foreach (var model in selectedModels)
            {
                var mesh = model.sharedMesh;
                if (!mesh)
                    continue;

                mesh.GetUVs(channel, sVertices);
                if (sVertices.Count == 0)
                    continue;

                for (int i = 0; i < sVertices.Count; i++)
                {
                    var vert = sVertices[i];
                    min.x = Math.Min(min.x, vert.x);
                    min.y = Math.Min(min.y, vert.y);
                    max.x = Math.Max(max.x, vert.x);
                    max.y = Math.Max(max.y, vert.y);
                }

                min.x = (float)Math.Floor(min.x);
                min.y = (float)Math.Floor(min.y);
                max.x = (float)Math.Ceiling(max.x);
                max.y = (float)Math.Ceiling(max.y);
            }
            info.min = min;
            info.max = max;
        }

        void DrawGrid(ChannelInfo info)
        {
            Handles.color = Color.gray;

            int minX    = (int)info.min.x;
            int minY    = (int)info.min.y;
            int maxX    = (int)info.max.x;
            int maxY    = (int)info.max.y;

            for (int x = minX; x <= maxX; x++)
                Handles.DrawLine(new Vector3(x, minY, 0), new Vector3(x, maxY, 0));
            for (int y = minY; y <= maxY; y++)
                Handles.DrawLine(new Vector3(minX, y, 0), new Vector3(maxX, y, 0));
        }

        int currentChannelIndex = 0;

        static bool Toggle(bool selected, GUIContent content, GUIStyle[] style)
        {
            var selectedStyle = selected ? style[1] : style[0];

            return GUILayout.Button(content, selectedStyle);
        }

        private void OnGUI()
        {
            if (styles == null)
                styles = new Styles();


            if (selectedModels != null &&
                selectedModels.Length > 0)
            {
                foreach (var model in selectedModels)
                {
                    if (!model)
                    {
                        OnSelectionChange();
                        break;
                    }
                }
            }

            GUILayout.BeginHorizontal();
            EditorGUILayout.PrefixLabel("Channel ");
            for (int i = 0; i < kChannelCount; i++)
            {
                var style = (i == 0) ? styles.leftButton :
                            (i == kChannelCount - 1) ? styles.rightButton :
                            styles.midButton;
                if (Toggle(currentChannelIndex == i, channelNames[i], style))
                {
                    currentChannelIndex = i;
                }
            }
            GUILayout.FlexibleSpace();
            GUILayout.EndHorizontal();
            var gridArea = EditorGUILayout.GetControlRect(GUILayout.ExpandHeight(true), GUILayout.ExpandWidth(true));
            gridArea.xMin += 4;
            gridArea.yMin += 4;
            gridArea.xMax -= 2;
            gridArea.yMax -= 2;
            GUI.BeginClip(gridArea);
            if (Event.current.type == EventType.Repaint)
            {
                var info            = channels[currentChannelIndex];
                var min             = info.min;
                var max             = info.max;
                var size            = max - min;
                var scaleX          = (gridArea.width  - 2) / Math.Max(1.0f, size.x);
                var scaleY          = (gridArea.height - 2) / Math.Max(1.0f, size.y);
                var scale           = Math.Min(scaleX, scaleY);
                var scaleVector     = new Vector3(scale, scale, 1);

                var positionVector  = (-min * (Vector2)scaleVector) + Vector2.one;
                var matrix = Matrix4x4.TRS(positionVector, Quaternion.identity, scaleVector);

                grid.WorldToGridSpace = matrix;

                var prevMatrix = Handles.matrix;
                var prevColor = Handles.color;

                Handles.matrix  = Matrix4x4.identity;
                Handles.color   = Color.black;
                Handles.DrawAAConvexPolygon(Vector3.zero, new Vector2(8000, 0), new Vector2(0, 8000));


                Handles.color = prevColor;
                //Handles.SetCamera(m_Camera);
                //Handles.ClearCamera(gridArea, m_Camera);
                //grid.Render(m_Camera, gridArea.size.magnitude);

                Handles.matrix = matrix;


                DrawGrid(info);

                if (selectedModels != null &&
                    selectedModels.Length > 0)
                {
                    foreach (var model in selectedModels)
                    {
                        DrawUVLines(model, currentChannelIndex);
                    }
                }

                //if (uvs != null)
                //    DrawUVLines(uvs);

                Handles.color = prevColor;
                Handles.matrix = prevMatrix;
            }
            GUI.EndClip();
        }

        static readonly List<Vector2> sVertices = new List<Vector2>();

        private void DrawUVLines(MeshFilter model, int channel)
        {
            var mesh = model.sharedMesh;
            if (!mesh)
                return;

            mesh.GetUVs(channel, sVertices); 
            if (sVertices.Count == 0)
                return;

            Handles.color = Color.white;
            for (int i = 0; i < mesh.subMeshCount; i++)
                DrawUVLines(mesh, i, sVertices);
        }

        static readonly List<int> sTriangles = new List<int>();

        private void DrawUVLines(Mesh mesh, int subMeshIndex, List<Vector2> vertices)
        {
            mesh.GetTriangles(sTriangles, subMeshIndex);
            for (int i = 0; i < sTriangles.Count; i+=3)
            {
                var i0 = sTriangles[i + 0];
                var i1 = sTriangles[i + 1];
                var i2 = sTriangles[i + 2];

                var v0 = vertices[i0];
                var v1 = vertices[i1];
                var v2 = vertices[i2];

                Handles.DrawLine(v0, v1);
                Handles.DrawLine(v1, v2);
                Handles.DrawLine(v2, v0);
            }
        }


        private void DrawUVLines(List<Vector2> vertices)
        {
            for (int i = 0; i < vertices.Count; i += 3)
            {
                var v0 = vertices[i + 0];
                var v1 = vertices[i + 1];
                var v2 = vertices[i + 2];

                Handles.DrawLine(v0, v1);
                Handles.DrawLine(v1, v2);
                Handles.DrawLine(v2, v0);
            }
        }
    }

}