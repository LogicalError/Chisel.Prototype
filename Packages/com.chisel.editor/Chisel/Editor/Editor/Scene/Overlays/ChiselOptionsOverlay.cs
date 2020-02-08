using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEditor;
using UnityEngine;
using Chisel.Core;
using Chisel.Components;
using UnitySceneExtensions;
using UnityEditor.EditorTools;

namespace Chisel.Editors
{
    // TODO: add tooltips
    public static class ChiselOptionsOverlay
    {
        const string kRebuildIconName   = "rebuild";
        const string kRebuildTooltip    = "Rebuild all generated meshes";
        public static void Rebuild()
        {
            var startTime = EditorApplication.timeSinceStartup;
            ChiselNodeHierarchyManager.Rebuild();
            var csg_endTime = EditorApplication.timeSinceStartup;
            Debug.Log($"Full CSG rebuild done in {((csg_endTime - startTime) * 1000)} ms. ");
        }

        public static ChiselOverlay.WindowFunction AdditionalSettings;


        const int kPrimaryOrder = int.MaxValue;
        
        static readonly GUIContent      kOverlayTitle   = new GUIContent("Chisel");
        static readonly ChiselOverlay   kOverlay        = new ChiselOverlay(kOverlayTitle, DisplayControls, kPrimaryOrder);

        static GUIContent kRebuildButton;

        static SortedList<string, ChiselEditToolBase> editModes = new SortedList<string, ChiselEditToolBase>();

        [InitializeOnLoadMethod]
        static void Initialize()
        {
            kRebuildButton = ChiselEditorResources.GetIconContent(kRebuildIconName, kRebuildTooltip)[0];
        }

        internal static void Register(ChiselEditToolBase editMode)
        {
            editModes[editMode.ToolName] = editMode;
        }

        static float kButtonSize = 32;
        static GUILayoutOption[] buttonOptions = new[]
        {
            GUILayout.Width(kButtonSize),
            GUILayout.Height(kButtonSize)
        };


        static bool Toggle(ChiselEditToolBase editMode, Type editModeType)
        {
            var content = editMode.m_ToolIcon;
            var selected = EditorTools.activeToolType == editModeType;
            return GUILayout.Toggle(selected, content, GUI.skin.button, buttonOptions);
        }

        static void EditModeButton(ChiselEditToolBase editMode)
        {
            var editModeType = editMode.GetType();
            var disabled = false;
            if (editModeType != typeof(ChiselCreateTool))
            {
                disabled = Selection.GetFiltered<ChiselNode>(SelectionMode.OnlyUserModifiable).Length == 0;
            }
            using (new EditorGUI.DisabledScope(disabled))
            {
                EditorGUI.BeginChangeCheck();
                var value = Toggle(editMode, editModeType);
                if (EditorGUI.EndChangeCheck() && value)
                {
                    EditorTools.SetActiveTool(editModeType);
                    ChiselEditorSettings.Save();
                }
            }
        }

        static void DisplayControls(System.Object target, SceneView sceneView)
        {
            if (!sceneView)
                return;

            EditorGUI.BeginChangeCheck();
            {
                if (AdditionalSettings != null)
                {
                    GUILayout.BeginVertical(ChiselOverlay.kMinWidthLayout);
                    AdditionalSettings(target, sceneView);
                    GUILayout.EndVertical();
                    GUILayout.Space(10);
                }

                GUILayout.BeginHorizontal(ChiselOverlay.kMinWidthLayout);

                foreach (var editMode in editModes.Values)
                    EditModeButton(editMode);
                GUILayout.FlexibleSpace();

                // TODO: assign hotkey to rebuild, and possibly move it elsewhere to avoid it seemingly like a necessary action.

                if (GUILayout.Toggle(false, kRebuildButton, GUI.skin.button, buttonOptions))
                {
                    Rebuild();
                }
                GUILayout.EndHorizontal();
            }
            if (EditorGUI.EndChangeCheck())
                ChiselEditorSettings.Save();
        }

        public static void Show()
        {
            kOverlay.Show();
        }
    }
}
