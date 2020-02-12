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
        public ChiselGeneratorMode currentGenerator;
        public ChiselGeneratorMode previousGenerator;

        public void OnAfterDeserialize() {}
        public void OnBeforeSerialize() {}
    }

    public class ChiselGeneratorManager : SingletonManager<ChiselEditModeData, ChiselGeneratorManager>
    {
        internal static ChiselGeneratorMode[] generatorModes;


        [InitializeOnLoadMethod]
        static void InitializeEditModes()
        {
            var generatorModeList = new List<ChiselGeneratorMode>();
            foreach (var type in ReflectionExtensions.AllNonAbstractClasses)
            {
                if (type.BaseType != typeof(ChiselGeneratorMode))
                    continue;

                var instance = (ChiselGeneratorMode)Activator.CreateInstance(type); 
                generatorModeList.Add(instance);
            }
            generatorModes  = generatorModeList.ToArray();
        }


        // TODO: create proper delegate for this, with named parameters for clarity
        public static event Action<ChiselGeneratorMode, ChiselGeneratorMode> GeneratorSelectionChanged;


        public static ChiselGeneratorMode GeneratorMode
        {
            get
            {
                if (generatorModes == null ||
                    generatorModes.Length == 0)
                    InitializeEditModes();
                var currentTool = Instance.data.currentGenerator;
                if (currentTool == null)
                    GeneratorIndex = 1;
                return Instance.data.currentGenerator;
            }
            set
            {
                if (generatorModes == null ||
                    generatorModes.Length == 0)
                    InitializeEditModes();
                if (Instance.data.currentGenerator == value)
                    return;

                RecordUndo("Generator selection changed");

                var prevMode = Instance.data.currentGenerator;
                Instance.data.currentGenerator = value;

                ChiselOptionsOverlay.UpdateCreateToolIcon();

                GeneratorSelectionChanged?.Invoke(prevMode, value);
            }
        }

        internal static void ActivateTool(ChiselGeneratorMode currentTool)
        {
            if (currentTool == Instance.data.previousGenerator)
                return;
            if (Instance.data.previousGenerator != null)
                Instance.data.previousGenerator.OnDeactivate();
            if (currentTool != null)
                currentTool.OnActivate();
            Instance.data.previousGenerator = currentTool;
        }

        internal static int GeneratorIndex
        {
            get
            {
                if (generatorModes == null ||
                    generatorModes.Length == 0)
                    InitializeEditModes();
                var currentGenerator = Instance.data.currentGenerator;
                for (int j = 0; j < generatorModes.Length; j++)
                {
                    if (generatorModes[j] == currentGenerator)
                        return (j + 1);
                }
                return 0;
            }
            set
            {
                if (generatorModes == null ||
                    generatorModes.Length == 0)
                    InitializeEditModes();
                if (value > 0)
                {
                    var index = (value - 1);
                    if (index >= generatorModes.Length)
                    {
                        GeneratorMode = null;
                        return;
                    }
                    Instance.data.currentGenerator = generatorModes[index];
                    return;
                }

                Instance.data.currentGenerator = generatorModes[0];
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
                    if (generatorMode.GetType() != value)
                        continue;
                    GeneratorMode = generatorMode;
                }
            }
        }
    }
}
