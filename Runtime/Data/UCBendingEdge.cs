using System.Diagnostics;

namespace UCloth
{
    /// <summary>
    /// Represents a bending element.
    /// </summary>
    [DebuggerDisplay("{bendingNode1}, {bendingNode2}")]
    public struct UCBendingEdge
    {
        public int bendingNode1;
        public int bendingNode2;

        public UCBendingEdge(int nodeIndex1, int nodeIndex2)
        {
            bendingNode1 = nodeIndex1;
            bendingNode2 = nodeIndex2;
        }
    }
}