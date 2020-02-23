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

        private void TranslateUVSurfaceWorld( Vector3 translation )
        {
            Matrix4x4 movement = Matrix4x4.TRS( translation, Quaternion.identity, Vector3.one );
            Undo.RecordObjects( CurrentBrushSelection, "Moved UV coordinates" );

            for( int i = 0; i < CurrentSurfaceSelection.Length; i++ )
            {
                CurrentSurfaceSelection[i].WorldSpaceTransformUV( in movement, in SelectedUVMatrices[i] );
            }
        }

        private void RotateUVSurfaceWorld( Vector3 center, Vector3 normal, float angle )
        {
            Matrix4x4 wsRotation = MathExtensions.RotateAroundAxis( center, normal, angle );

            Undo.RecordObjects( CurrentBrushSelection, "Rotate UV coordinates" );

            for( int i = 0; i < CurrentSurfaceSelection.Length; i++ )
            {
                Matrix4x4 psRotation = CurrentSurfaceSelection[i].WorldSpaceToPlaneSpace( in wsRotation );

                Quaternion rotateToPlane = Quaternion.FromToRotation( psRotation.GetColumn( 2 ), Vector3.forward );
                Matrix4x4  fixedRotation = Matrix4x4.TRS( Vector3.zero, rotateToPlane, Vector3.one ) * psRotation;

                CurrentSurfaceSelection[i].PlaneSpaceTransformUV( in fixedRotation, in SelectedUVMatrices[i] );
            }
        }

        private void ScaleUVSurfaceWorld_U( float scale )
        {
            // $TODO: implement scale UV U
        }

        private void ScaleUVSurfaceWorld_V( float scale )
        {
            // $TODO: implement scale UV V
        }
    }
}
