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
    public class ChiselPlacementToolsSelectionWindow : EditorWindow
    {
        const float kSingleLineHeight   = 32f;
        const float kSingleSpacing      = 0.0f;


        [MenuItem("Window/Chisel/Placement Tools")]
        public static void Create()
        {
            var window = (ChiselPlacementToolsSelectionWindow)GetWindow(typeof(ChiselPlacementToolsSelectionWindow), false, "Chisel Placement Tools");
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

        public void GeneratorSelectionChanged(ChiselGeneratorMode prevGenerator, ChiselGeneratorMode nextGenerator)
        {
            Repaint();
        }

        static bool Toggle(Rect togglePosition, ChiselGeneratorMode generator, GUIStyle style, bool isActive)
        {
            var content     = ChiselEditorResources.GetIconContentWithName(generator.ToolName, generator.ToolName);
            var selected    = ChiselGeneratorManager.GeneratorMode == generator;
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

        static void NamedGeneratorButton(ChiselGeneratorMode generator, Rect togglePosition, GUIStyle style, bool isActive)
        {
            var temp = togglePosition;
            temp.xMin += 5;
            temp.width = 20;
            {
                EditorGUI.BeginChangeCheck();
                var value = GUI.Toggle(temp, generator.InToolBox, GUIContent.none);
                if (EditorGUI.EndChangeCheck())
                {
                    // TODO: make undoable
                    generator.InToolBox = value;
                    if (ChiselGeneratorManager.GeneratorMode == generator)
                    {
                        if (!DeselectGeneratorMode())
                            generator.InToolBox = true;
                    }
                    ChiselEditorSettings.Save();
                    SceneView.RepaintAll();
                }
            }
            temp = togglePosition;
            temp.xMin += 25; 
            {
                EditorGUI.BeginChangeCheck();
                var value = Toggle(temp, generator, style, isActive);
                if (EditorGUI.EndChangeCheck() && value)
                {
                    // TODO: make undoable
                    generator.InToolBox = true;
                    ChiselCreateTool.ActivateTool();
                    Selection.activeObject = null;
                    ChiselGeneratorManager.GeneratorMode = generator;
                    ChiselEditorSettings.Save();
                    SceneView.RepaintAll();
                }
            }
        }

        static bool DeselectGeneratorMode()
        {
            if (PrevGeneratorMode())
                return true;
            if (NextGeneratorMode())
                return true;
            return false;
        }

        public static bool PrevGeneratorMode()
        {
            var currentGeneratorMode = ChiselGeneratorManager.GeneratorMode;
            var generatorModes = ChiselGeneratorManager.generatorModes;
            var index = currentGeneratorMode == null ? 1 : ArrayUtility.IndexOf(generatorModes, currentGeneratorMode);
            do { index--; } while (index >= 0 && !generatorModes[index].InToolBox);
            if (index < 0)
                return false;
            ChiselGeneratorManager.GeneratorMode = generatorModes[index];
            return true;
        }

        public static bool NextGeneratorMode()
        {
            var currentGeneratorMode = ChiselGeneratorManager.GeneratorMode;
            var generatorModes = ChiselGeneratorManager.generatorModes;
            var index = currentGeneratorMode == null ? generatorModes.Length - 1 : ArrayUtility.IndexOf(generatorModes, currentGeneratorMode);
            do { index++; } while (index < generatorModes.Length && !generatorModes[index].InToolBox);
            if (index >= generatorModes.Length)
                return false;
            ChiselGeneratorManager.GeneratorMode = generatorModes[index];
            return true;
        }

        static bool Toggle(ChiselGeneratorMode generator, GUIStyle style, bool isActive)
        {
            var content = generator.Content;
            var selected = ChiselGeneratorManager.GeneratorMode == generator;
            var prevBackgroundColor = GUI.backgroundColor;
            if (selected && !isActive)
            {
                var color = Color.white;
                color.a = 0.25f;
                GUI.backgroundColor = color;
            }
            var result = GUILayout.Toggle(selected, content, style);
            GUI.backgroundColor = prevBackgroundColor;
            return result;
        }

        static void GeneratorButton(ChiselGeneratorMode generator, GUIStyle style, bool isActive)
        {
            EditorGUI.BeginChangeCheck();
            var value = Toggle(generator, style, isActive);
            if (EditorGUI.EndChangeCheck())
            {
                ChiselCreateTool.ActivateTool();
                Selection.activeObject = null;
                ChiselGeneratorManager.GeneratorMode = generator;
                if (value)
                    ChiselEditorSettings.Save();
                SceneView.RepaintAll();
            }
        }

        class Styles
        {
            public GUIStyle namedToggleStyle;
            public GUIStyle toggleStyle;
        }

        static Styles styles = null;
        static void InitStyles()
        {
            if (styles == null)
            {
                ChiselEditorSettings.Load();
                styles = new Styles
                {
                    namedToggleStyle = new GUIStyle(GUI.skin.button)
                    {
                        alignment   = TextAnchor.MiddleLeft,
                        fixedHeight = kSingleLineHeight - 2
                    },
                    toggleStyle = new GUIStyle(GUI.skin.button)
                    {
                        fixedWidth  = kSingleLineHeight,
                        fixedHeight = kSingleLineHeight,
                        padding     = new RectOffset(2, 2, 2, 2)
                    }
                };
            }
        }

        public void OnGUI()
        {
            InitStyles();
            // TODO: have scrollbar when window is too small
            // TODO: if window is wider than higher, place everything horizontal vs vertical
            var generatorModes  = ChiselGeneratorManager.generatorModes;
            var isActive        = ChiselCreateTool.IsActive();

            var togglePosition = new Rect(0,0, ChiselEditorUtility.ContextWidth, kSingleLineHeight);
            var style = styles.namedToggleStyle;
            for (int i = 0; i < generatorModes.Length; i++)
            {
                NamedGeneratorButton(generatorModes[i], togglePosition, style, isActive);
                togglePosition.y += kSingleLineHeight + kSingleSpacing;
            }
        }

        public static void RenderCreationTools()
        {
            InitStyles();

            var generatorModes = ChiselGeneratorManager.generatorModes;
            var isActive = ChiselCreateTool.IsActive();

            var style = styles.toggleStyle;

            // TODO: add search functionality
            // TODO: add groupings
            // TODO: add automatic finding node based generators in project
            GUILayout.BeginVertical(GUI.skin.box, ChiselOverlay.kMinWidthLayout);
            GUILayout.BeginHorizontal();
            int index = 0;
            for (int i = 0; i < generatorModes.Length; i++)
            {
                if (!generatorModes[i].InToolBox && 
                    ChiselGeneratorManager.GeneratorMode != generatorModes[i])
                    continue;
                if (index > 0 && (index % 7) == 0)
                {
                    GUILayout.EndHorizontal();
                    GUILayout.BeginHorizontal();
                }
                GeneratorButton(generatorModes[i], style, isActive);
                index++;
            }
            GUILayout.EndHorizontal();
            GUILayout.EndVertical();
        }
    }
}
