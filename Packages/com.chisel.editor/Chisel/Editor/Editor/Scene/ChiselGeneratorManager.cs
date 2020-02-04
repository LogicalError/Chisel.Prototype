using Chisel.Core;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEditor;
using UnityEditor.EditorTools;
using UnityEditor.ShortcutManagement;
using UnityEngine;

namespace Chisel.Editors
{
    [Serializable]
    public class ChiselEditModeData : ISingletonData
    {
        public IChiselToolMode currentEditMode;
        public IChiselToolMode currentGenerator;
        public IChiselToolMode previousTool;

        public void OnAfterDeserialize() {}
        public void OnBeforeSerialize() {}
    }

    public class ChiselGeneratorManager : SingletonManager<ChiselEditModeData, ChiselGeneratorManager>
    {
        internal sealed class ChiselEditModeItem
        {
            public ChiselEditModeItem(IChiselToolMode value, Type type)
            {
                this.instance   = value;
                this.type       = type;
            }
            public IChiselToolMode  instance;
            public Type             type;
        }

        internal static ChiselEditModeItem[]    generatorModes;


        [InitializeOnLoadMethod]
        static void InitializeEditModes()
        {
            var generatorModeList   = new List<ChiselEditModeItem>();
            foreach (var type in ReflectionExtensions.AllNonAbstractClasses)
            {
                if (!type.GetInterfaces().Contains(typeof(IChiselToolMode)))
                    continue;

                if (type.BaseType == typeof(ChiselGeneratorMode))
                {
                    var instance = (IChiselToolMode)Activator.CreateInstance(type);
                    generatorModeList.Add(new ChiselEditModeItem(instance, type));
                }
            }
            generatorModes  = generatorModeList.ToArray();
        }


        // TODO: create proper delegate for this, with named parameters for clarity
        public static event Action<IChiselToolMode, IChiselToolMode> GeneratorSelectionChanged;


        public static IChiselToolMode GeneratorMode
        {
            get
            {
                var currentTool = Instance.data.currentGenerator;
                if (currentTool == null)
                    GeneratorIndex = 1;
                return Instance.data.currentGenerator;
            }
            set
            {
                if (Instance.data.currentGenerator == value)
                    return;

                RecordUndo("Generator selection changed");

                var prevMode = Instance.data.currentGenerator;
                Instance.data.currentGenerator = value;

                GeneratorSelectionChanged?.Invoke(prevMode, value);
            }
        }

        internal static void ActivateTool(IChiselToolMode currentTool)
        {
            if (currentTool == Instance.data.previousTool)
                return;
            if (Instance.data.previousTool != null)
                Instance.data.previousTool.OnDeactivate();
            if (currentTool != null)
                currentTool.OnActivate();
            Instance.data.previousTool = currentTool;
        }

        internal static int GeneratorIndex
        {
            get
            {
                var currentGenerator = Instance.data.currentGenerator;

                for (int j = 0; j < generatorModes.Length; j++)
                {
                    if (generatorModes[j].instance == currentGenerator)
                        return (j + 1);
                }

                return 0;
            }
            set
            {
                if (value > 0)
                {
                    if (generatorModes == null)
                        InitializeEditModes();
                    var index = (value - 1);
                    if (index >= generatorModes.Length)
                    {
                        GeneratorMode = null;
                        return;
                    }
                    Instance.data.currentGenerator = generatorModes[index].instance;
                    return;
                }

                Instance.data.currentGenerator = generatorModes[0].instance;
            }
        }

        public static Type GeneratorType
        {
            set
            {
                if (value == null)
                    return;

                foreach (var generatorMode in generatorModes)
                {
                    if (generatorMode.type != value)
                        continue;
                    GeneratorMode = generatorMode.instance;
                }
            }
        }

        public void OnSelectionChanged()
        {
            // Make sure we're currently in a non-generator, otherwise this makes no sense
            // We might actually be currently restoring a selection
            if (!(GeneratorMode is ChiselGeneratorMode))
                return;

            var activeObject = Selection.activeObject;
            // This event is fired when we select or deselect something.
            // We only care if we select something
            if (activeObject == null)
                return;

            // We just selected something in the editor, so we want to get rid of our 
            // stored selection to avoid restoring an old selection for no reason later on.
            //ClearStoredEditModeState();

            var is_generator = activeObject is Components.ChiselGeneratorComponent;
            if (!is_generator)
            {
                var gameObject = activeObject as GameObject;
                if (gameObject != null)
                    is_generator = gameObject.GetComponent<Components.ChiselGeneratorComponent>() != null;
            }

            if (is_generator)
                EditorTools.SetActiveTool(typeof(ChiselShapeEditTool));
            else
                EditorTools.RestorePreviousPersistentTool();
        }

    }
}
