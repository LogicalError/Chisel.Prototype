using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEditor;
using UnityEngine;
using Chisel.Core;
using Chisel.Components;
using UnitySceneExtensions;

namespace Chisel.Editors
{
    // TODO: add tooltips
    public static class ChiselOptionsOverlay
    {
        const int kPrimaryOrder = int.MaxValue;

        static ChiselOverlay overlay = new ChiselOverlay(EditorGUIUtility.TrTextContent("Chisel"), DisplayControls, kPrimaryOrder);

        static GUIContent rebuildButton = EditorGUIUtility.TrTextContent("Rebuild");

        public static void Rebuild()
        {
            var startTime = EditorApplication.timeSinceStartup;
            ChiselNodeHierarchyManager.Rebuild();
            var csg_endTime = EditorApplication.timeSinceStartup;
            Debug.Log($"Full CSG rebuild done in {((csg_endTime - startTime) * 1000)} ms. ");
        }

        static void DisplayControls(System.Object target, SceneView sceneView)
        {
            if (!sceneView)
                return;

            EditorGUI.BeginChangeCheck();
            GUILayout.BeginHorizontal(ChiselOverlay.kMinWidthLayout);

            GUILayout.FlexibleSpace();

            // TODO: assign hotkey to rebuild, and possibly move it elsewhere to avoid it seemingly like a necessary action.
            if (GUILayout.Button(rebuildButton))
            {
                Rebuild();
            }

            GUILayout.FlexibleSpace();

            GUILayout.EndHorizontal();
            if (EditorGUI.EndChangeCheck())
                ChiselEditorSettings.Save();
        }

        public static void Show()
        {
            overlay.Show();
        }
    }
}
