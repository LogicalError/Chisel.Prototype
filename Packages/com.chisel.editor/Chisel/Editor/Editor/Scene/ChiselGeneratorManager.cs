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
    }
}
