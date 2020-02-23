/*
 * $TODO: clean up layout code - headers can be put into their own methods, etc.
 */

using Chisel.Components;
using Chisel.Core;
using UnityEditor;
using UnityEngine;
using EditorGUILayout = UnityEditor.EditorGUILayout;

namespace Chisel.Editors
{
    internal partial class SurfaceEditor
    {
        private float           m_PanIncrementU   = 1.0f;
        private float           m_PanIncrementV   = 1.0f;
        private float           m_RotIncrement    = 15.0f;
        private float           m_ScaleIncrementU = 1.0f;
        private float           m_ScaleIncrementV = 1.0f;
        private int             m_CurrentTab      = 0;
        private LayerUsageFlags m_LayerUsage      = LayerUsageFlags.All;

        // $TODO: find a way to edit surface flags outside of model... its serialized by unity and not visible to SurfaceReference or ChiselBrushMaterial.
        // ^ this can be done, reference new EditorTools code for this
        //private SurfaceFlags    m_WorldSpaceTex = SurfaceFlags.TextureIsInWorldSpace;

        private void DrawTabBar()
        {
            GUILayout.BeginHorizontal();

            m_CurrentTab = GUILayout.Toolbar
            (
                m_CurrentTab, new string[] { "Flags", "UVs", "Custom", "Other" }, m_TabStyle
            );

            GUILayout.EndHorizontal();
        }

        private void DrawWindowContents()
        {
            switch( m_CurrentTab )
            {
                case 0: // Flags
                {
                    DrawFlagsTabContent();
                    break;
                }

                case 1: // UVs
                {
                    DrawUVTabContent();
                    break;
                }

                case 2: // Custom
                {
                    DrawCustomTabContent();
                    break;
                }

                case 3: // Other
                {
                    DrawOtherTabContent();
                    break;
                }
            }
        }

        // Surface flags - visible, receive shadows, collidable, cast shadows, lock texture to object
        private void DrawFlagsTabContent()
        {
            m_LayerUsage = GetFlagsUI( m_LayerUsage );

            if( CurrentSurfaceSelection != null )
            {
                foreach( SurfaceReference surfaceReference in CurrentSurfaceSelection )
                {
                    ChiselBrushMaterial bm = surfaceReference.BrushMaterial;

                    if( bm.LayerUsage != m_LayerUsage )
                        bm.LayerUsage = m_LayerUsage;
                }
            }
        }

        // UV Editing - translation, scale, rotation, anchor/pivot, auto fit
        private void DrawUVTabContent()
        {
            GUILayout.BeginHorizontal(); // pan/rotation/scale
            {
                GUILayout.Space( 4 );

                GUILayout.BeginVertical( "Tooltip", GUILayout.Width( 194 ) ); // Pan/Nudge
                {
                    GUILayout.BeginHorizontal();
                    {
                        GUILayout.Label( "Pan",       "OL Title", GUILayout.Width( 64 ) );
                        GUILayout.Label( "Increment", "OL Title" );
                    }
                    GUILayout.EndHorizontal();

                    GUILayout.Space( 2 );
                    GUILayout.BeginHorizontal(); // U
                    {
                        GUILayout.Label( "U", GUILayout.Width( 16 ) );
                        if( GUILayout.Button( "<", "minibuttonleft", GUILayout.Width( 20 ) ) )
                        {
                            TranslateUVSurfaceWorld( new Vector3( m_PanIncrementU, 0, 0 ) );
                        }

                        if( GUILayout.Button( ">", "minibuttonright", GUILayout.Width( 20 ) ) )
                        {
                            TranslateUVSurfaceWorld( new Vector3( -m_PanIncrementU, 0, 0 ) );
                        }

                        GUILayout.Space( 2 );

                        m_PanIncrementU = EditorGUILayout.FloatField( m_PanIncrementU, "MiniTextField" );
                    }
                    GUILayout.EndHorizontal();

                    GUILayout.BeginHorizontal(); // V
                    {
                        GUILayout.Label( "V", GUILayout.Width( 16 ) );

                        if( GUILayout.Button( "<", "minibuttonleft", GUILayout.Width( 20 ) ) )
                        {
                            TranslateUVSurfaceWorld( new Vector3( 0, 0, m_PanIncrementV ) );
                        }

                        if( GUILayout.Button( ">", "minibuttonright", GUILayout.Width( 20 ) ) )
                        {
                            TranslateUVSurfaceWorld( new Vector3( 0, 0, -m_PanIncrementV ) );
                        }

                        GUILayout.Space( 2 );

                        m_PanIncrementV = EditorGUILayout.FloatField( m_PanIncrementV, "MiniTextField" );
                    }
                    GUILayout.EndHorizontal();

                    GUILayout.Space( 4 );
                    GUILayout.BeginHorizontal();
                    {
                        GUILayout.Label( "Scale", "OL Title", GUILayout.Width( 64 ) );
                        GUILayout.Label( "",      "OL Title" );
                    }
                    GUILayout.EndHorizontal();

                    GUILayout.Space( 2 );
                    GUILayout.BeginHorizontal(); // scale
                    {
                        GUILayout.Label( "U", GUILayout.Width( 16 ) );
                        if( GUILayout.Button( "-", "minibuttonleft", GUILayout.Width( 20 ) ) )
                        {
                            ScaleUVSurfaceWorld_U( -m_ScaleIncrementU );
                        }

                        if( GUILayout.Button( "+", "minibuttonright", GUILayout.Width( 20 ) ) )
                        {
                            ScaleUVSurfaceWorld_U( m_ScaleIncrementU );
                        }

                        GUILayout.Space( 2 );
                        m_ScaleIncrementU = EditorGUILayout.FloatField( m_ScaleIncrementU, "MiniTextField" );
                    }
                    GUILayout.EndHorizontal();

                    GUILayout.BeginHorizontal();
                    {
                        GUILayout.Label( "V", GUILayout.Width( 16 ) );
                        if( GUILayout.Button( "-", "minibuttonleft", GUILayout.Width( 20 ) ) )
                        {
                            // this does nothing right now and is just a stub
                            ScaleUVSurfaceWorld_V( -m_ScaleIncrementV );
                        }

                        if( GUILayout.Button( "+", "minibuttonright", GUILayout.Width( 20 ) ) )
                        {
                            // this does nothing right now and is just a stub
                            ScaleUVSurfaceWorld_V( m_ScaleIncrementV );
                        }

                        GUILayout.Space( 2 );
                        m_ScaleIncrementV = EditorGUILayout.FloatField( m_ScaleIncrementV, "MiniTextField" );
                    }
                    GUILayout.EndHorizontal();

                    GUILayout.Space( 2 );

                    GUILayout.BeginHorizontal();
                    {
                        if( GUILayout.Button( "Fit To Face", "minibutton" ) )
                        {
                            // $TODO: implement fit to face functionality
                        }
                    }
                    GUILayout.EndHorizontal();

                    GUILayout.Space( 4 );
                }
                GUILayout.EndVertical();

                GUILayout.Space( 4 );

                GUILayout.BeginVertical( "Tooltip", GUILayout.Width( 194 ) ); // rotation
                {
                    GUILayout.BeginHorizontal();
                    {
                        GUILayout.Label( "Rotation",  "OL Title", GUILayout.Width( 64 ) );
                        GUILayout.Label( "Increment", "OL Title" );
                    }
                    GUILayout.EndHorizontal();

                    GUILayout.Space( 2 );
                    GUILayout.BeginHorizontal(); // rotation
                    {
                        if( GUILayout.Button( "<", "minibuttonleft", GUILayout.Width( 32 ) ) )
                        {
                            // $TODO: can we do this without using UVTool?
                            RotateUVSurfaceWorld( ChiselUVToolCommon.worldStartPosition,
                                                  ChiselUVToolCommon.worldDragPlane.normal, m_RotIncrement );
                        }

                        if( GUILayout.Button( ">", "minibuttonright", GUILayout.Width( 32 ) ) )
                        {
                            // $TODO: can we do this without using UVTool?
                            RotateUVSurfaceWorld( ChiselUVToolCommon.worldStartPosition,
                                                  ChiselUVToolCommon.worldDragPlane.normal, -m_RotIncrement );
                        }

                        m_RotIncrement = EditorGUILayout.FloatField( m_RotIncrement, "MiniTextField" );
                    }
                    GUILayout.EndHorizontal();

                    GUILayout.BeginHorizontal();
                    {
                        if( GUILayout.Button( "Flip U", "minibuttonleft" ) )
                        {
                            // $TODO: Implement flip UV functionality
                        }

                        if( GUILayout.Button( "Flip V", "minibuttonright" ) )
                        {
                        }
                    }
                    GUILayout.EndHorizontal();

                    GUILayout.Space( 4 );

                    GUILayout.BeginHorizontal();
                    {
                        GUILayout.Label( "Other", "OL Title" );
                    }
                    GUILayout.EndHorizontal();

                    GUILayout.Space( 2 );
                    GUILayout.BeginHorizontal();
                    {
                        if( GUILayout.Button( "Open UV Editor", "minibutton" ) )
                        {
                            // $TODO: add a UV editor window
                        }
                    }
                    GUILayout.EndHorizontal();
                }
                GUILayout.EndVertical();
            }
            GUILayout.EndHorizontal();
        }


        // Custom user surface properties
        // $TODO: get custom/user surface properties
        private void DrawCustomTabContent()
        {
            GUILayout.Label( "Custom" );
        }

        // Other content not fitting in other tabs - physic material, material, smoothing groups, etc.
        // $TODO: Implement material/physic material, etc. properties
        private void DrawOtherTabContent()
        {
            GUILayout.Label( "Other" );
        }

        private void DrawStatusBar()
        {
            GUILayout.BeginHorizontal( m_FooterStyle, GUILayout.Height( 20 ) );

            GUILayout.Label( $"Surfaces: {CurrentSurfaceSelection.Length.ToString()}", m_FooterTextStyle );

            GUILayout.FlexibleSpace();

            GUILayout.Label( "Close Automatically", m_FooterTextStyle, GUILayout.Width( 100 ) );
            m_CloseOnLostFocusPref =
                EditorGUILayout.Toggle( m_CloseOnLostFocusPref, m_MiniToggleStyle, GUILayout.Width( 16 ) );

            GUILayout.EndHorizontal();
        }
    }
}
