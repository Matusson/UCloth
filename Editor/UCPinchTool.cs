using Unity.Mathematics;
using UnityEditor;
using UnityEditor.EditorTools;
using UnityEditor.ShortcutManagement;
using UnityEngine;

namespace UCloth.Editor
{

    [EditorTool("Pinch Tool", typeof(UCloth))]
    public class UCPinchTool : EditorTool
    {
        private bool nodeSelected;
        private ushort selectedNodeIndex;

        UCloth cloth;

        // The second "context" argument accepts an EditorWindow type.
        [Shortcut("Activate Pinch Tool", typeof(SceneView), KeyCode.P)]
        public static void PinchToolShortcut()
        {
            if (Selection.GetFiltered<UCloth>(SelectionMode.TopLevel).Length > 0)
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
                cloth.simData.cReciprocalWeight[selectedNodeIndex] = 1f;
                cloth.simData.cPinnedLocalPos.Remove(selectedNodeIndex);
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

            cloth = target as UCloth;

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
                for (ushort i = 0; i < cloth.simData.cPositions.Length; i++)
                {
                    float dist = math.distancesq(hitPoint, cloth.simData.cPositions[i]);

                    if (dist < lowestDistance)
                    {
                        lowestDistance = dist;
                        lowestIndex = i;
                    }
                }
                selectedNodeIndex = lowestIndex;
                nodeSelected = true;

                float3 localPos = cloth.transform.InverseTransformPoint(cloth.simData.cPositions[selectedNodeIndex]);
                cloth.simData.cReciprocalWeight[selectedNodeIndex] = 0;
                cloth.simData.cPinnedLocalPos.Add(selectedNodeIndex, localPos);
            }
            else
            {
                float3 pos = Handles.PositionHandle(cloth.simData.cPositions[selectedNodeIndex], Quaternion.identity);
                cloth.simData.cPositions[selectedNodeIndex] = pos;
                cloth.simData.cPinnedLocalPos[selectedNodeIndex] = cloth.transform.InverseTransformPoint(pos);
            }
        }
    }
}