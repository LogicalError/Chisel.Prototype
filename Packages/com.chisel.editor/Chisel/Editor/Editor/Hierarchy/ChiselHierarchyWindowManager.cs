using UnityEngine;
using UnityEditor;
using Chisel.Components;

namespace Chisel.Editors
{
    public sealed class GUIView
    {
        static ReflectedProperty<object>        currentProperty     = ReflectionExtensions.GetStaticProperty<object>("UnityEditor.GUIView", "current");
        static ReflectedInstanceProperty<bool>  hasFocusProperty    = ReflectionExtensions.GetProperty<bool>("UnityEditor.GUIView", "hasFocus");

        public static bool HasFocus
        {
            get
            {
                var currentGuiView = currentProperty.Value;
                if (currentGuiView == null)
                    return false;
                
                return hasFocusProperty.GetValue(currentGuiView);
            }
        }
    }

    public static class ChiselHierarchyWindowManager
    {
        public static void RenderIcon(Rect selectionRect, GUIContent icon)
        {
            const float iconSize = 16;
            const float indent   = 0;
            var max = selectionRect.xMax;
            selectionRect.width = iconSize;
            selectionRect.height = iconSize;
            selectionRect.x = max - (iconSize + indent);
            selectionRect.y--;
            GUI.Label(selectionRect, icon);
        }
        
        static void RenderHierarchyItem(int instanceID, ChiselNode node, Rect selectionRect)
        {
            if (!ChiselSceneGUIStyle.isInitialized)
                return;
            var model = node as ChiselModel;
            if (!ReferenceEquals(model, null))
            {
                if (model == ChiselModelManager.ActiveModel)
                {
                    var content = EditorGUIUtility.TrTempContent(node.name + " (active)");

                    bool selected = GUIView.HasFocus && Selection.Contains(instanceID);
                    GUI.Label(selectionRect, content, selected ? ChiselSceneGUIStyle.inspectorSelectedLabel : ChiselSceneGUIStyle.inspectorLabel);
                }
            }

            var icon = ChiselNodeDetailsManager.GetHierarchyIcon(node);
            if (icon != null)
                RenderIcon(selectionRect, icon);
        }

        internal static void OnHierarchyWindowItemGUI(int instanceID, ChiselNode node, Rect selectionRect)
        {
            // TODO: implement material drag & drop support on hierarchy items

            if (Event.current.type == EventType.Repaint)
                RenderHierarchyItem(instanceID, node, selectionRect);
        }
    }
}
