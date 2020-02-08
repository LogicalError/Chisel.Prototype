using Chisel.Core;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEditor;
using UnityEditor.EditorTools;
using UnityEngine;
using UnitySceneExtensions;

namespace Chisel.Editors
{
    public abstract class ChiselGeneratorMode : IChiselToolMode
    {
        public abstract string  ToolName        { get; }

        public virtual void     OnActivate()    { Reset(); }

        public virtual void     OnDeactivate()  { Reset(); }

        public virtual void     Reset() { }

        public void Commit(GameObject newGameObject)
        {
            if (!newGameObject)
            {
                Cancel();
                return;
            }
            UnityEditor.Selection.activeGameObject = newGameObject;
            Reset();
            EditorTools.SetActiveTool(typeof(ChiselShapeEditTool));
        }

        public void Cancel()
        { 
            Reset();
            Undo.RevertAllInCurrentGroup();
            EditorGUIUtility.ExitGUI();
        }

        public void OnSettingsGUI(System.Object target, SceneView sceneView) { OnSceneSettingsGUI(); }
        public virtual void     OnSceneSettingsGUI() {}

        public virtual void     OnSceneGUI(SceneView sceneView, Rect dragArea)
        {
            ChiselOptionsOverlay.AdditionalSettings = OnSettingsGUI;            

            var evt = Event.current;
            switch (evt.type)
            {
                case EventType.KeyDown:
                case EventType.ValidateCommand:
                {
                    if (SceneHandles.InCameraOrbitMode ||
                        (evt.modifiers & (EventModifiers.Shift | EventModifiers.Control | EventModifiers.Alt | EventModifiers.Command)) != EventModifiers.None ||
                        GUIUtility.hotControl != 0)
                        break;

                    if (evt.keyCode == KeyCode.Escape)
                    {
                        evt.Use();
                        break;
                    }
                    break;
                }
                case EventType.KeyUp:
                {
                    if (SceneHandles.InCameraOrbitMode ||
                        (evt.modifiers & (EventModifiers.Shift | EventModifiers.Control | EventModifiers.Alt | EventModifiers.Command)) != EventModifiers.None ||
                        GUIUtility.hotControl != 0)
                        break;

                    if (evt.keyCode == KeyCode.Escape)
                    {
                        evt.Use();
                        GUIUtility.ExitGUI();
                    }
                    break;
                }
            }
        }
    }

    public class ChiselGeneratorSelectionWindow : EditorWindow
    {
        const float kSingleLineHeight   = 20f;
        const float kSingleSpacing      = 0.0f;


        [MenuItem("Window/Chisel/Generator Panel")]
        public static void Create()
        {
            var window = (ChiselGeneratorSelectionWindow)GetWindow(typeof(ChiselGeneratorSelectionWindow), false, "Generators");
            window.Initialize();
        }

        void Initialize()
        {
            this.minSize = new Vector2(100, 100);
        }

        private void OnEnable()
        {
            ChiselGeneratorManager.GeneratorSelectionChanged -= GeneratorSelectionChanged;
            ChiselGeneratorManager.GeneratorSelectionChanged += GeneratorSelectionChanged;
            EditorTools.activeToolChanged -= EditModeSelectionChanged;
            EditorTools.activeToolChanged += EditModeSelectionChanged;
        }

        private void OnDisable()
        {
            ChiselGeneratorManager.GeneratorSelectionChanged -= GeneratorSelectionChanged;
            EditorTools.activeToolChanged -= EditModeSelectionChanged;
        }

        public void EditModeSelectionChanged()
        {
            Repaint();
        }

        public void GeneratorSelectionChanged(IChiselToolMode prevGenerator, IChiselToolMode nextGenerator)
        {
            Repaint();
        }

        static bool Toggle(Rect togglePosition, ChiselGeneratorManager.ChiselEditModeItem generator, GUIStyle style, bool isActive)
        {
            var content     = ChiselEditorResources.GetIconContent(generator.instance.ToolName, generator.instance.ToolName);
            var selected    = ChiselGeneratorManager.GeneratorMode == generator.instance;
            var prevBackgroundColor = GUI.backgroundColor;
            if (selected && !isActive)
            {
                var color = Color.white;
                color.a = 0.25f;
                GUI.backgroundColor = color;
            }
            var result = GUI.Toggle(togglePosition, selected, content[0], style);
            GUI.backgroundColor = prevBackgroundColor;
            return result;
        }

        static void GeneratorButton(ChiselGeneratorManager.ChiselEditModeItem generator, Rect togglePosition, GUIStyle style, bool isActive)
        {
            EditorGUI.BeginChangeCheck();
            var value   = Toggle(togglePosition, generator, style, isActive);
            if (EditorGUI.EndChangeCheck())
            {
                ChiselCreateTool.ActivateTool();
                Selection.activeObject = null;
                ChiselGeneratorManager.GeneratorMode = generator.instance;
                if (value)
                    ChiselEditorSettings.Save();
                SceneView.RepaintAll();
            }
        }

        public void OnGUI()
        {
            // TODO: have scrollbar when window is too small
            // TODO: if window is wider than higher, place everything horizontal vs vertical
            var generatorModes  = ChiselGeneratorManager.generatorModes;
            var isActive        = ChiselCreateTool.IsActive();

            var togglePosition = new Rect(0,0, ChiselEditorUtility.ContextWidth, kSingleLineHeight);
            var style = GUI.skin.button;
            for (int i = 0; i < generatorModes.Length; i++)
            {
                GeneratorButton(generatorModes[i], togglePosition, style, isActive);
                togglePosition.y += kSingleLineHeight + kSingleSpacing;
            }
        }
    }
}
