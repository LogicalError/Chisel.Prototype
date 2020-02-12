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
    public class ChiselGeneratorSelectionWindow : EditorWindow
    {
        const float kSingleLineHeight   = 32f;
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
            var content     = ChiselEditorResources.GetIconContentWithName(generator.instance.ToolName, generator.instance.ToolName);
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

        static void NamedGeneratorButton(ChiselGeneratorManager.ChiselEditModeItem generator, Rect togglePosition, GUIStyle style, bool isActive)
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


        static bool Toggle(ChiselGeneratorManager.ChiselEditModeItem generator, GUIStyle style, bool isActive)
        {
            var content = ChiselEditorResources.GetIconContent(generator.instance.ToolName, generator.instance.ToolName);
            var selected = ChiselGeneratorManager.GeneratorMode == generator.instance;
            var prevBackgroundColor = GUI.backgroundColor;
            if (selected && !isActive)
            {
                var color = Color.white;
                color.a = 0.25f;
                GUI.backgroundColor = color;
            }
            var result = GUILayout.Toggle(selected, content[0], style);
            GUI.backgroundColor = prevBackgroundColor;
            return result;
        }

        static void GeneratorButton(ChiselGeneratorManager.ChiselEditModeItem generator, GUIStyle style, bool isActive)
        {
            EditorGUI.BeginChangeCheck();
            var value = Toggle(generator, style, isActive);
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
                styles = new Styles();
                styles.namedToggleStyle = new GUIStyle(GUI.skin.button);
                styles.namedToggleStyle.alignment = TextAnchor.MiddleLeft;
                styles.namedToggleStyle.fixedHeight = kSingleLineHeight - 2;

                styles.toggleStyle = new GUIStyle(styles.namedToggleStyle);
                styles.toggleStyle.fixedHeight = kSingleLineHeight;
                styles.toggleStyle.fixedWidth = styles.toggleStyle.fixedHeight;
                styles.toggleStyle.padding = new RectOffset(2, 2, 2, 2);
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

            GUILayout.BeginHorizontal(ChiselOverlay.kMinWidthLayout);
            for (int i = 0; i < generatorModes.Length; i++)
            {
                if (i >0 && (i%7) == 0)
                {
                    GUILayout.EndHorizontal();
                    GUILayout.BeginHorizontal(ChiselOverlay.kMinWidthLayout);
                }
                GeneratorButton(generatorModes[i], style, isActive);
            }
            GUILayout.EndHorizontal();
        }
    }
}
