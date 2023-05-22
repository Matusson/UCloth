using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;
using UnityEngine.UI;

namespace UCloth.Editor
{
    [InitializeOnLoad]
    public class OnComponentAddedUtil
    {
        static OnComponentAddedUtil()
        {
            ObjectFactory.componentWasAdded -= ComponentAdded;
            EditorApplication.quitting -= Cleanup;

            ObjectFactory.componentWasAdded += ComponentAdded;
            EditorApplication.quitting += Cleanup;
        }
        private static void ComponentAdded(Component component)
        {
            if (component is UCDebugVisualizer visualizer)
            {
                visualizer.Reset();
            }
        }

        private static void Cleanup()
        {
            ObjectFactory.componentWasAdded -= ComponentAdded;
            EditorApplication.quitting -= Cleanup;
        }
    }
}
