using System;
using System.Collections.Generic;
using System.Linq;
using UnityEditor;
using UnityEngine;
using Chisel.Core;
using Chisel.Components;
using UnitySceneExtensions;
using UnityEditor.ShortcutManagement;

namespace Chisel.Editors
{
    public sealed class ChiselLinearStairsGeneratorMode : ChiselGeneratorMode
    {
        const string kToolName = ChiselLinearStairs.kNodeTypeName;
        public override string ToolName => kToolName;
        public override string Group => "Stairs";

        #region Keyboard Shortcut
        const string kToolShotcutName = ChiselKeyboardDefaults.ShortCutCreateBase + kToolName;
        [Shortcut(kToolShotcutName, ChiselKeyboardDefaults.LinearStairsBuilderModeKey, ChiselKeyboardDefaults.LinearStairsBuilderModeModifiers, displayName = kToolShotcutName)]
        public static void StartGeneratorMode() { ChiselGeneratorManager.GeneratorType = typeof(ChiselLinearStairsGeneratorMode); }
        #endregion

        public override void Reset()
        {
            BoxExtrusionHandle.Reset();
            linearStairs = null;
        }
        
        ChiselLinearStairs linearStairs;
        // TODO: Handle forcing operation types
        CSGOperationType? forceOperation = null; // TODO: WHY???
        public override void OnSceneSettingsGUI()
        {
            // TODO: implement
            GUILayout.BeginVertical();
            ChiselOperationGUI.ChooseGeneratorOperation(ref forceOperation);
            GUILayout.EndVertical();
        }

        public override void OnSceneGUI(SceneView sceneView, Rect dragArea)
        {
            var flags = BoxExtrusionFlags.AlwaysFaceUp | BoxExtrusionFlags.AlwaysFaceCameraXZ;

            switch (BoxExtrusionHandle.Do(dragArea, out Bounds bounds, out float height, out ChiselModel modelBeneathCursor, out Matrix4x4 transformation, flags, Axis.Y))
            {
                case BoxExtrusionState.Create:
                {
                    linearStairs = ChiselComponentFactory.Create<ChiselLinearStairs>(ChiselLinearStairs.kNodeTypeName,
                                                                        ChiselModelManager.GetActiveModelOrCreate(modelBeneathCursor),
                                                                        transformation);
                    linearStairs.definition.Reset();
                    linearStairs.Operation  = forceOperation ?? CSGOperationType.Additive;
                    linearStairs.Bounds     = bounds;
                    linearStairs.UpdateGenerator();
                    break;
                }

                case BoxExtrusionState.Modified:
                {
                    linearStairs.definition.Reset();
                    linearStairs.Operation  = forceOperation ?? 
                                              ((height < 0 && modelBeneathCursor) ? 
                                                CSGOperationType.Subtractive : 
                                                CSGOperationType.Additive);
                    linearStairs.Bounds     = bounds;
                    break;
                }
                
                case BoxExtrusionState.Commit:      { Commit(linearStairs.gameObject); break; }
                case BoxExtrusionState.Cancel:      { Cancel(); break; }
                case BoxExtrusionState.BoxMode:
                case BoxExtrusionState.SquareMode:	{ ChiselOutlineRenderer.VisualizationMode = VisualizationMode.SimpleOutline; break; }
                case BoxExtrusionState.HoverMode:	{ ChiselOutlineRenderer.VisualizationMode = VisualizationMode.Outline; break; }
            }

            HandleRendering.RenderBox(transformation, bounds);
        }
    }
}
