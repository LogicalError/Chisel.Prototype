using Chisel.Core;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEditor;
using UnityEditor.EditorTools;
using UnityEditor.ShortcutManagement;
using UnityEngine;
using UnityObject = UnityEngine.Object;
 
namespace Chisel.Editors
{
    [EditorTool("Chisel " + kToolName + " Tool")]
    class ChiselCreateTool : ChiselEditToolBase
    {
        const string kToolName = "Create";
        public override string ToolName => kToolName;
        public static bool IsActive() { return EditorTools.activeToolType == typeof(ChiselCreateTool); }
        
        #region Keyboard Shortcut
        const string kEditModeShotcutName = ChiselKeyboardDefaults.ShortCutEditModeBase + kToolName + " Mode";
        [Shortcut(kEditModeShotcutName, ChiselKeyboardDefaults.SwitchToCreateEditMode, displayName = kEditModeShotcutName)]
        public static void ActivateTool() { EditorTools.SetActiveTool<ChiselCreateTool>(); }
        #endregion

        public override void OnActivate()
        {
            ChiselGeneratorManager.GeneratorMode.OnActivate();
        }

        public override void OnDeactivate()
        {
            ChiselGeneratorManager.GeneratorMode.OnDeactivate();
        }

        public override void OnSceneGUI(SceneView sceneView, Rect dragArea)
        {
            ChiselOptionsOverlay.Show();
            ChiselGridOptionsOverlay.Show();
            ChiselGeneratorManager.GeneratorMode.OnSceneGUI(sceneView, dragArea);
        }
    }
}
