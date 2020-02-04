using Chisel.Components;
using Chisel.Core;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEditor;
using UnityEditor.EditorTools;
using UnityEditor.ShortcutManagement;
using UnityEngine;

namespace Chisel.Editors
{
    [EditorTool("Chisel " + kToolName + " Tool", typeof(ChiselNode))]
    class ChiselShapeEditTool : ChiselEditToolBase
    {
        const string kToolName = "Shape Edit";
        public override string ToolName => kToolName;

        public static bool IsActive() { return EditorTools.activeToolType == typeof(ChiselShapeEditTool); }


        #region Keyboard Shortcut
        const string kEditModeShotcutName = ChiselKeyboardDefaults.ShortCutEditModeBase + kToolName + " Mode";
        [Shortcut(kEditModeShotcutName, ChiselKeyboardDefaults.SwitchToShapeEditMode, displayName = kEditModeShotcutName)]
        public static void ActivateTool() { EditorTools.SetActiveTool<ChiselShapeEditTool>(); }
        #endregion

        public override void OnActivate()
        {
            ChiselOutlineRenderer.VisualizationMode = VisualizationMode.None;
        }

        public override void OnSceneGUI(SceneView sceneView, Rect dragArea)
        {
            // NOTE: Actual work is done by Editor classes
        }
    }
}
