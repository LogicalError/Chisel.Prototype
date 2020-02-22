using System;
using System.Collections.Generic;
using Chisel.Components;
using Chisel.Core;
using UnityEditor;
using UnityEngine;

namespace Chisel.Editors
{
    internal partial class SurfaceEditor : EditorWindow
    {
        private static bool                      m_CloseOnLostFocusPref = false;
        private static HashSet<SurfaceReference> m_CurrentSelection     = null;

        private static HashSet<SurfaceReference> CurrentSelection
        {
            get
            {
                if( m_CurrentSelection == null )
                    m_CurrentSelection = ChiselSurfaceSelectionManager.Selection;

                return m_CurrentSelection;
            }
        }

        [MenuItem( "Window/Chisel/Surface Editor" )]
        private static void ShowWindow()
        {
            SurfaceEditor window = GetWindow<SurfaceEditor>( true, "Surface Properties" );

            window.minSize = new Vector2( 400, 230 );
            window.maxSize = window.minSize;
            window.Show();
        }

        private void OnGUI()
        {
            InitStyles();

            DrawTabBar();
            DrawWindowContents();

            GUILayout.FlexibleSpace();
            // $TODO: do we need one?
            DrawStatusBar();
        }

        private void OnLostFocus()
        {
            if( m_CloseOnLostFocusPref )
                Close();
        }

        private void OnEnable()
        {
            m_CloseOnLostFocusPref = EditorPrefs.GetBool( "Chisel.SurfaceEditor.CloseOnLostFocus", false );
        }

        private void OnDisable()
        {
            EditorPrefs.SetBool( "Chisel.SurfaceEditor.CloseOnLostFocus", m_CloseOnLostFocusPref );
        }
    }
}
