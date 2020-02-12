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
        public const string kToolName = "Create";
        public override string ToolName => kToolName;

        public override GUIContent Content
        {
            get 
            {
                return ChiselGeneratorManager.GeneratorMode.Content;
            } 
        }

        public static bool IsActive() { return EditorTools.activeToolType == typeof(ChiselCreateTool); }
        
        #region Keyboard Shortcut
        const string kEditModeShotcutName = kToolName + " Mode";
        [Shortcut(ChiselKeyboardDefaults.ShortCutEditModeBase + kEditModeShotcutName, ChiselKeyboardDefaults.SwitchToCreateEditMode, displayName = kEditModeShotcutName)]
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

        public override void OnSceneSettingsGUI(UnityEngine.Object target, SceneView sceneView)
        {
            ChiselGeneratorManager.GeneratorMode.OnSceneSettingsGUI(target, sceneView);
        }

        public override void OnSceneGUI(SceneView sceneView, Rect dragArea)
        {
            var generatorMode = ChiselGeneratorManager.GeneratorMode;
            if (generatorMode == null)
                return;

            ChiselOptionsOverlay.AdditionalSettings = OnSceneSettingsGUI;
            ChiselOptionsOverlay.SetTitle($"Create {generatorMode.ToolName}");
            generatorMode.ShowSceneGUI(sceneView, dragArea);

            /// TODO: pressing escape when not in the middle of creation something, should cancel this edit mode instead
        }
    }
}
