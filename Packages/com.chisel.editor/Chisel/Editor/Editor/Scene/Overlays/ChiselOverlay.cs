using Chisel.Core;
using Chisel.Components;
using UnitySceneExtensions;
using System;
using System.Collections.Generic;
using System.Reflection;
using UnityEditor;
using UnityEditor.EditorTools;
using UnityEngine;
using UnityEngine.SceneManagement;
using System.Linq;

namespace Chisel.Editors
{
    public class ChiselOverlay
    {
        public const int kMinWidth = 244;
        public static readonly GUILayoutOption kMinWidthLayout = GUILayout.MinWidth(kMinWidth);

        public delegate void WindowFunction(UnityEngine.Object target, SceneView sceneView);

        public ChiselOverlay(GUIContent title, WindowFunction sceneViewFunc, int primaryOrder)
        {
            InitializeTypes();

            var sceneViewFuncDelegate = Delegate.CreateDelegate(s_WindowFunctionType, sceneViewFunc.Method);

#if UNITY_2019_3
            windowMethod_parameters = new object[]
            { 
                title, sceneViewFuncDelegate, primaryOrder, s_WindowMethod_parameter_overlay
            };
#elif UNITY_2020_1_OR_NEWER
            //public OverlayWindow(GUIContent title, SceneViewOverlay.WindowFunction guiFunction, int primaryOrder, Object target, SceneViewOverlay.WindowDisplayOption option)
            var overlayWindow = Activator.CreateInstance(overlayWindowType, title, sceneViewFuncDelegate, primaryOrder, null, windowMethod_parameter_overlay);
            windowMethod_parameters = new object[] 
            { 
                overlayWindow 
            };
#endif
        }

        public void Show()
        {
            if (windowMethod != null)
                windowMethod.Invoke(null, windowMethod_parameters);
        }

        static Type s_SceneViewOverlayType;
        static Type s_WindowFunctionType;
        static object s_WindowMethod_parameter_overlay;
        static System.Reflection.MethodInfo windowMethod;
        static Type s_OverlayWindowType;

        object[] windowMethod_parameters;

        internal void InitializeTypes()
        {
            if (windowMethod != null)
                return;
            
            s_OverlayWindowType       = ReflectionExtensions.GetTypeByName("UnityEditor.SceneViewOverlay+OverlayWindow");
            s_WindowFunctionType      = ReflectionExtensions.GetTypeByName("UnityEditor.SceneViewOverlay+WindowFunction");
            s_SceneViewOverlayType    = ReflectionExtensions.GetTypeByName("UnityEditor.SceneViewOverlay");
                
            var windowDisplayOptionType = ReflectionExtensions.GetTypeByName("UnityEditor.SceneViewOverlay+WindowDisplayOption");
            s_WindowMethod_parameter_overlay = Enum.Parse(windowDisplayOptionType, "OneWindowPerTarget");

#if UNITY_2019_3
            windowMethod = s_SceneViewOverlayType.GetMethods(System.Reflection.BindingFlags.Static | System.Reflection.BindingFlags.Public).FirstOrDefault(t => t.Name == "Window" && t.GetParameters().Length == 4);
#elif UNITY_2020_1_OR_NEWER
            //public static void ShowWindow(OverlayWindow window)
            windowMethod = sceneViewOverlayType.GetMethod("ShowWindow", System.Reflection.BindingFlags.Static | System.Reflection.BindingFlags.Public);
#endif
        }

    }
}
