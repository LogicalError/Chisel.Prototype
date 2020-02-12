using Chisel.Components;
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
    public abstract class ChiselGeneratorMode
    {
        public abstract string  ToolName        { get; }

        public GUIContent       Content         { get { return ChiselEditorResources.GetIconContent(ToolName, ToolName)[0]; } }

        public bool             InToolBox 
        {
            get { return ChiselEditorSettings.IsInToolBox(ToolName, true); }
            set { ChiselEditorSettings.SetInToolBox(ToolName, value);  }
        }

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
            
            // Throws an error because selection apparently hasn't arrived to EditorTools API yet??
            EditorTools.SetActiveTool(typeof(ChiselEditGeneratorTool));
        }

        public void Cancel()
        { 
            Reset();
            Undo.RevertAllInCurrentGroup();
            EditorGUIUtility.ExitGUI();
        }

        public void             OnSceneSettingsGUI(UnityEngine.Object target, SceneView sceneView) { OnSceneSettingsGUI(); }
        public virtual void     OnSceneSettingsGUI() {}

        public abstract void OnSceneGUI(SceneView sceneView, Rect dragArea);

        public virtual void ShowSceneGUI(SceneView sceneView, Rect dragArea)
        {
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
            OnSceneGUI(sceneView, dragArea);
        }
    }
}
