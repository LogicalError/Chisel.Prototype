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
    public static class ChiselGridOptionsOverlay
    {
        const int kPrimaryOrder = 99;

        static ChiselOverlay overlay = new ChiselOverlay(EditorGUIUtility.TrTextContent("Grid"), DisplayControls, kPrimaryOrder);

        static GUIContent doubleSnapDistanceButton = EditorGUIUtility.TrTextContent("+", "Double the snapping distance.\nHotkey: ]");
        static GUIContent halveSnapDistanceButton = EditorGUIUtility.TrTextContent("-", "Halve the snapping distance.\nHotkey: [");

        static GUILayoutOption sizeButtonWidth = GUILayout.Width(16);

        static void DisplayControls(System.Object target, SceneView sceneView)
        {
            if (!sceneView)
                return;

            EditorGUI.BeginChangeCheck();
            GUILayout.BeginHorizontal(ChiselOverlay.kMinWidthLayout);

            ChiselEditorSettings.ShowGrid = GUILayout.Toggle(ChiselEditorSettings.ShowGrid, "Show Grid", GUI.skin.button);

            ChiselEditorSettings.UniformSnapSize = EditorGUILayout.FloatField(ChiselEditorSettings.UniformSnapSize);
            if (GUILayout.Button(halveSnapDistanceButton, EditorStyles.miniButtonLeft, sizeButtonWidth))
            {
                SnappingKeyboard.HalfGridSize();
            }
            if (GUILayout.Button(doubleSnapDistanceButton, EditorStyles.miniButtonRight, sizeButtonWidth))
            {
                SnappingKeyboard.DoubleGridSize();
            }

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
