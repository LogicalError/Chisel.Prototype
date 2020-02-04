using UnityEngine;
using UnityEditor;
using UnityEditor.SceneManagement;
using System;
using System.Linq;
using System.Collections.Generic;
using Chisel;
using Chisel.Core;
using Chisel.Components;
using UnityEditor.EditorTools;

namespace Chisel.Editors
{
    public static class ChiselOperationGUI
    {
        class Styles
        {
            public GUIStyle[] leftButton    = new GUIStyle[2];
            public GUIStyle[] midButton     = new GUIStyle[2];
            public GUIStyle[] rightButton   = new GUIStyle[2];

            public Styles()
            {
                leftButton[0] = new GUIStyle(EditorStyles.miniButtonLeft) { stretchWidth = true, stretchHeight = true };
                leftButton[0].padding.top += 3;
                leftButton[0].padding.bottom += 3;
                leftButton[0].fixedHeight += 5;
                leftButton[1] = new GUIStyle(leftButton[0]);
                leftButton[1].normal.background = leftButton[0].active.background;

                midButton[0] = new GUIStyle(EditorStyles.miniButtonMid) { stretchWidth = true, stretchHeight = true };
                midButton[0].padding.top += 3;
                midButton[0].padding.bottom += 3;
                midButton[0].fixedHeight += 5;
                midButton[1] = new GUIStyle(midButton[0]);
                midButton[1].normal.background = midButton[0].active.background;

                rightButton[0] = new GUIStyle(EditorStyles.miniButtonRight) { stretchWidth = true, stretchHeight = true };
                rightButton[0].padding.top += 3;
                rightButton[0].padding.bottom += 3;
                rightButton[0].fixedHeight += 5;
                rightButton[1] = new GUIStyle(rightButton[0]);
                rightButton[1].normal.background = rightButton[0].active.background;
            }
        };

        static Styles styles;

        static bool Toggle(bool selected, GUIContent[] content, GUIStyle[] style)
        {
            var selectedContent = selected ? content[1] : content[0];
            var selectedStyle = selected ? style[1] : style[0];

            return GUILayout.Button(selectedContent, selectedStyle);
        }

        // TODO: put somewhere else
        public static void ShowOperationChoicesInternal(SerializedProperty operationProp)
        {
            if (operationProp == null)
                return;

            EditorGUI.BeginChangeCheck();
            var result = ShowOperationChoicesInternal(operationProp.hasMultipleDifferentValues ? (CSGOperationType?)null : (CSGOperationType)operationProp.enumValueIndex);
            if (EditorGUI.EndChangeCheck())
            {
                operationProp.enumValueIndex = (int)result;
            }
        }

        // TODO: put somewhere else
        public static CSGOperationType ShowOperationChoicesInternal(CSGOperationType? operation)
        {
            if (styles == null)
                styles = new Styles();

            const string AdditiveIconName           = "csg_addition";
            const string SubtractiveIconName        = "csg_subtraction";
            const string IntersectingIconName       = "csg_intersection";

            const string AdditiveIconTooltip        = "Additive Boolean Operation";
            const string SubtractiveIconTooltip     = "Subtractive Boolean Operation";
            const string IntersectingIconTooltip    = "Intersecting Boolean Operation";

            var additiveIcon        = ChiselEditorResources.GetIconContent(AdditiveIconName,        AdditiveIconTooltip);
            var subtractiveIcon     = ChiselEditorResources.GetIconContent(SubtractiveIconName,     SubtractiveIconTooltip);
            var intersectingIcon    = ChiselEditorResources.GetIconContent(IntersectingIconName,    IntersectingIconTooltip);

            using (new EditorGUIUtility.IconSizeScope(new Vector2(16, 16)))     // This ensures that the icons will be the same size on regular displays and HDPI displays
                                                                                // Note that the loaded images are different sizes on different displays
            {
                var operationType = !operation.HasValue ? ((CSGOperationType)255) : (operation.Value);
                if (Toggle((operationType == CSGOperationType.Additive), additiveIcon, styles.leftButton))
                    return CSGOperationType.Additive;
                if (Toggle((operationType == CSGOperationType.Subtractive), subtractiveIcon, styles.midButton))
                    return CSGOperationType.Subtractive;
                if (Toggle((operationType == CSGOperationType.Intersecting), intersectingIcon, styles.rightButton))
                    return CSGOperationType.Intersecting;
                return operationType;
            }
        }

    }
}
