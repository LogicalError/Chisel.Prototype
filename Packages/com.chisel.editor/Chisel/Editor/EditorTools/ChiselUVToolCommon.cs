using Chisel.Core;
using Chisel.Components;
using System;
using System.Collections.Generic;
using System.Linq;
using UnityEditor;
using UnityEngine;
using UnitySceneExtensions;
using Chisel.Utilities;
using UnityEditor.ShortcutManagement;
using Snapping = UnitySceneExtensions.Snapping;
using UnityEditor.EditorTools;

namespace Chisel.Editors
{
    sealed class ChiselSurfaceContainer : ScriptableObject
    {
        public const string kBrushSurfaceName = nameof(BrushSurface);
        
        public ChiselSurface BrushSurface;
        [NonSerialized] public SurfaceReference SurfaceReference;

        public static ChiselSurfaceContainer Create(SurfaceReference surfaceReference)
        {
            var container = ScriptableObject.CreateInstance<ChiselSurfaceContainer>();
            container.SurfaceReference  = surfaceReference;
            container.BrushSurface      = surfaceReference.BrushSurface;
            container.hideFlags         = HideFlags.DontSave;
            return container;
        }

        public void Update()
        {
            SurfaceReference.BrushSurface.surfaceDescription.smoothingGroup     = BrushSurface.surfaceDescription.smoothingGroup;
            SurfaceReference.BrushSurface.surfaceDescription.surfaceFlags       = BrushSurface.surfaceDescription.surfaceFlags;
            SurfaceReference.BrushSurface.surfaceDescription.UV0                = BrushSurface.surfaceDescription.UV0;
            BrushSurface.brushMaterial.SetDirty();
            SurfaceReference.SetDirty();
        }
    }

    sealed class ChiselUVToolCommon : ScriptableObject
    {
        #region Instance
        static ChiselUVToolCommon _instance;
        public static ChiselUVToolCommon Instance
        {
            get
            {
                if (_instance)
                    return _instance;

                _instance = ScriptableObject.CreateInstance<ChiselUVToolCommon>();
                _instance.hideFlags = HideFlags.HideAndDontSave;
                return _instance;
            }
        }
        #endregion


        public void OnActivate()
        {
            ChiselSurfaceSelectionManager.selectionChanged -= UpdateSurfaceSelection;
            ChiselSurfaceSelectionManager.selectionChanged += UpdateSurfaceSelection;

            Reset();
            UpdateSurfaceSelection();
        }

        public void OnDeactivate()
        {
            ChiselSurfaceSelectionManager.selectionChanged -= UpdateSurfaceSelection;
            Reset();
            DestroyOldSurfaces();
        }

        void Reset()
        {
            if (serializedObject != null)
            {
                serializedObject.Dispose();
                serializedObject = null;
            }
            layerUsageProp = null;
            renderMaterialProp = null;
            physicsMaterialProp = null;
            initialized = false;
        }

        void DestroyOldSurfaces()
        {
            if (surfaces != null &&
                surfaces.Length > 0)
            {
                foreach (var surface in surfaces)
                    UnityEngine.Object.DestroyImmediate(surface);
            }
            surfaces = Array.Empty<ChiselSurfaceContainer>();
        }

        void UpdateSurfaceSelection()
        {
            DestroyOldSurfaces();
            surfaces        = (from reference in ChiselSurfaceSelectionManager.Selection select ChiselSurfaceContainer.Create(reference)).ToArray();
            var undoableObjectList = (from surface in surfaces select (UnityEngine.Object)surface.SurfaceReference.brushContainerAsset).ToList();
            undoableObjectList.AddRange(from surface in surfaces select (UnityEngine.Object)surface.SurfaceReference.node);
            undoableObjects = undoableObjectList.ToArray();

            if (surfaces.Length == 0)
            {
                Reset();
                return;
            }

            serializedObject = new SerializedObject(surfaces);
            var brushSurfaceProperty = serializedObject.FindProperty(ChiselSurfaceContainer.kBrushSurfaceName);
            if (brushSurfaceProperty != null)
            { 
                var materialProperty = brushSurfaceProperty.FindPropertyRelative(ChiselSurface.kBrushMaterialName);
                if (materialProperty != null)
                { 
                    layerUsageProp      = materialProperty.FindPropertyRelative(ChiselBrushMaterial.kLayerUsageFieldName);
                    renderMaterialProp  = materialProperty.FindPropertyRelative(ChiselBrushMaterial.kRenderMaterialFieldName);
                    physicsMaterialProp = materialProperty.FindPropertyRelative(ChiselBrushMaterial.kPhysicsMaterialFieldName);
                }
                var surfaceDescriptionProp  = brushSurfaceProperty.FindPropertyRelative(ChiselSurface.kSurfaceDescriptionName);
                if (surfaceDescriptionProp != null)
                {
                    smoothingGroupProp  = surfaceDescriptionProp.FindPropertyRelative(SurfaceDescription.kSmoothingGroupName);
                    surfaceFlagsProp    = surfaceDescriptionProp.FindPropertyRelative(SurfaceDescription.kSurfaceFlagsName);
                    UV0Prop             = surfaceDescriptionProp.FindPropertyRelative(SurfaceDescription.kUV0Name);
                }
            }

            initialized = true;
        }

        bool initialized = false;
        SerializedObject serializedObject;
        SerializedProperty layerUsageProp;
        SerializedProperty renderMaterialProp;
        SerializedProperty physicsMaterialProp;

        SerializedProperty smoothingGroupProp;
        SerializedProperty surfaceFlagsProp;
        SerializedProperty UV0Prop;
        [SerializeField] UnityEngine.Object[]       undoableObjects = Array.Empty<UnityEngine.Object>();
        [SerializeField] ChiselSurfaceContainer[]   surfaces        = Array.Empty<ChiselSurfaceContainer>();

        public void OnSceneSettingsGUI()
        {
            if (!initialized)
                UpdateSurfaceSelection();

            if (!initialized)
                return;

            serializedObject.UpdateIfRequiredOrScript();
            EditorGUI.BeginChangeCheck();
            {
                var desiredHeight   = ChiselBrushMaterialPropertyDrawer.DefaultMaterialLayerUsageHeight;
                var position        = EditorGUILayout.GetControlRect(false, desiredHeight);
                ChiselBrushMaterialPropertyDrawer.ShowMaterialLayerUsage(position, renderMaterialProp, layerUsageProp);

                var prevLabelWidth = EditorGUIUtility.labelWidth;
                EditorGUIUtility.labelWidth = 85;
                position = EditorGUILayout.GetControlRect(true, SurfaceFlagsPropertyDrawer.DefaultHeight);
                if (surfaceFlagsProp != null)
                    EditorGUI.PropertyField(position, surfaceFlagsProp, SurfaceDescriptionPropertyDrawer.kSurfaceFlagsContents, true);

                position = EditorGUILayout.GetControlRect(true, UVMatrixPropertyDrawer.DefaultHeight);
                if (UV0Prop != null)
                    EditorGUI.PropertyField(position, UV0Prop, SurfaceDescriptionPropertyDrawer.kUV0Contents, true);

                EditorGUIUtility.labelWidth = prevLabelWidth;
            }
            if (EditorGUI.EndChangeCheck() && undoableObjects.Length > 0)
            {
                Undo.RegisterCompleteObjectUndo(undoableObjects, undoableObjects.Length > 1 ? "Modified materials" : "Modified material");
                serializedObject.ApplyModifiedProperties();
                foreach (var item in surfaces)
                    item.Update();
            }
        }
    }
}
