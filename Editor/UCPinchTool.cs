using Unity.Mathematics;
using UnityEditor;
using UnityEditor.EditorTools;
using UnityEditor.ShortcutManagement;
using UnityEngine;

namespace UCloth.Editor
{

    [EditorTool("Pinch Tool", typeof(UCCloth))]
    public class UCPinchTool : EditorTool
    {
        private bool nodeSelected;
        private ushort selectedNodeIndex;
        private float3 handlePos;

        UCCloth cloth;

        // The second "context" argument accepts an EditorWindow type.
        [Shortcut("Activate Pinch Tool", typeof(SceneView), KeyCode.P)]
        public static void PinchToolShortcut()
        {
            if (Selection.GetFiltered<UCCloth>(SelectionMode.TopLevel).Length > 0)
                ToolManager.SetActiveTool<UCPinchTool>();
            else
                Debug.Log("No cloth objects selected!");
        }

        // Called when the active tool is set to this tool instance. Global tools are persisted by the ToolManager,
        // so usually you would use OnEnable and OnDisable to manage native resources, and OnActivated/OnWillBeDeactivated
        // to set up state. See also `EditorTools.{ activeToolChanged, activeToolChanged }` events.
        public override void OnActivated()
        {
            SceneView.lastActiveSceneView.ShowNotification(new GUIContent("Pinch Tool"), .1f);
        }

        // Called before the active tool is changed, or destroyed. The exception to this rule is if you have manually
        // destroyed this tool (ex, calling `Destroy(this)` will skip the OnWillBeDeactivated invocation).
        public override void OnWillBeDeactivated()
        {
            if (cloth != null && nodeSelected)
            {
                cloth.simData.reciprocalWeight[selectedNodeIndex] = 1f;
                cloth.simData.pinnedPositions.Remove(selectedNodeIndex);
            }
            nodeSelected = false;
            selectedNodeIndex = 0;
        }

        // Equivalent to Editor.OnSceneGUI.
        public override void OnToolGUI(EditorWindow window)
        {
            if (window is not SceneView)
                return;

            if (!Application.isPlaying)
                return;

            if (target == null)
                return;

            cloth = target as UCCloth;

            // Select a node first
            if (!nodeSelected)
            {
                if (Event.current.type != EventType.MouseDown || Event.current.button != 0)
                    return;

                Vector3 mousePosition = Event.current.mousePosition;
                var ray = HandleUtility.GUIPointToWorldRay(mousePosition);

                // If nothing hit
                if (!Physics.Raycast(ray, out RaycastHit hit, 100f))
                    return;

                float3 hitPoint = new(hit.point.x, hit.point.y, hit.point.z);

                ushort lowestIndex = 0;
                float lowestDistance = 100000;
                for (ushort i = 0; i < cloth.simData.positionsReadOnly.Length; i++)
                {
                    float dist = math.distancesq(hitPoint, cloth.simData.positionsReadOnly[i]);

                    if (dist < lowestDistance)
                    {
                        lowestDistance = dist;
                        lowestIndex = i;
                    }
                }
                selectedNodeIndex = lowestIndex;
                nodeSelected = true;

                float3 pos = cloth.simData.positionsReadOnly[selectedNodeIndex];
                handlePos = pos;
                cloth.simData.reciprocalWeight[selectedNodeIndex] = 0;
                cloth.simData.pinnedPositions.Add(selectedNodeIndex, pos);

                cloth.simData.ApplyModifiedData();
            }
            else
            {
                handlePos = Handles.PositionHandle(handlePos, Quaternion.identity);
                cloth.simData.pinnedPositions[selectedNodeIndex] = handlePos;
                cloth.simData.ApplyModifiedData();

            }
        }
    }
}