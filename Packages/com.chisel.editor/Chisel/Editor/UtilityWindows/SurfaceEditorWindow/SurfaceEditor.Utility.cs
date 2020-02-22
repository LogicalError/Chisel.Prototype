/* * * * * * * * * * * * * * * * * * * * * *
Chisel.Editor.UVEditorUtility.cs

License: 
Author: Daniel Cornelius

Description:

* * * * * * * * * * * * * * * * * * * * * */

using Chisel.Core;
using UnityEditor;
using UnityEngine;

namespace Chisel.Editors
{
    internal partial class SurfaceEditor
    {
        private LayerUsageFlags GetFlagsUI( LayerUsageFlags flags )
        {
            EditorGUI.BeginChangeCheck();

            bool isRenderable     = ( flags & LayerUsageFlags.Renderable )     != 0;
            bool isCastShadows    = ( flags & LayerUsageFlags.CastShadows )    != 0;
            bool isReceiveShadows = ( flags & LayerUsageFlags.ReceiveShadows ) != 0;
            bool isCollidable     = ( flags & LayerUsageFlags.Collidable )     != 0;

            GUILayout.Space( 6 );

            GUILayout.BeginHorizontal();
            {
                GUILayout.Space( 12 );
                GUILayout.BeginVertical();
                {
                    isRenderable = EditorGUILayout.ToggleLeft( "Visible",    isRenderable );
                    isCollidable = EditorGUILayout.ToggleLeft( "Collidable", isCollidable );
                }
                GUILayout.EndVertical();

                GUILayout.BeginVertical();
                {
                    EditorGUI.BeginDisabledGroup( !ChiselEditorUtility.IsUsingDeferredRenderingPath() ||
                                                  !isRenderable );
                    {
                        if( !isRenderable )
                        {
                            //EditorGUILayout.ToggleLeft( "Receive Shadows", false );
                        }
                        else if( ChiselEditorUtility.IsUsingDeferredRenderingPath() )
                        {
                            //EditorGUILayout.ToggleLeft( "Receive Shadows", true );
                        }
                        else
                        {
                            isReceiveShadows = EditorGUILayout.ToggleLeft( "Receive Shadows", isReceiveShadows );
                        }
                    }
                    EditorGUI.EndDisabledGroup();
                    isCastShadows = EditorGUILayout.ToggleLeft( "Cast Shadows", isCastShadows );

                    GUILayout.FlexibleSpace();
                }
                GUILayout.EndVertical();
            }
            GUILayout.EndHorizontal();

            if( EditorGUI.EndChangeCheck() )
            {
                if( isRenderable )
                    flags |= LayerUsageFlags.Renderable;
                else
                    flags &= ~LayerUsageFlags.Renderable;

                if( isCastShadows )
                    flags |= LayerUsageFlags.CastShadows;
                else
                    flags &= ~LayerUsageFlags.CastShadows;

                if( isReceiveShadows )
                    flags |= LayerUsageFlags.ReceiveShadows;
                else
                    flags &= ~LayerUsageFlags.ReceiveShadows;

                if( isCollidable )
                    flags |= LayerUsageFlags.Collidable;
                else
                    flags &= ~LayerUsageFlags.Collidable;
            }

            return flags;
        }
    }
}
