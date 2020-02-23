using System;
using System.Collections.Generic;
using System.Linq;
using Chisel.Components;
using Chisel.Core;
using UnityEditor;
using UnityEngine;

namespace Chisel.Editors
{
    internal partial class SurfaceEditor : EditorWindow
    {
        private static bool       m_CloseOnLostFocusPref = false;
        private        UVMatrix[] m_SelectedUVMatrices   = null;

        private SurfaceReference[] CurrentSurfaceSelection
        {
            get { return ChiselSurfaceSelectionManager.Selection.ToArray(); }
        }

        private UVMatrix[] SelectedUVMatrices
        {
            get
            {
                m_SelectedUVMatrices = new UVMatrix[CurrentSurfaceSelection.Length];

                for( int i = 0; i < CurrentSurfaceSelection.Length; i++ )
                {
                    if( CurrentSurfaceSelection[i].Polygon.surface == null )
                        m_SelectedUVMatrices[i] = UVMatrix.identity;
                    else
                        m_SelectedUVMatrices[i] = CurrentSurfaceSelection[i].Polygon.surface.surfaceDescription.UV0;
                }

                return m_SelectedUVMatrices;
            }
        }

        private static ChiselBrushContainerAsset[] CurrentBrushSelection
        {
            get { return ChiselSurfaceSelectionManager.SelectedBrushMeshes.ToArray(); }
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
