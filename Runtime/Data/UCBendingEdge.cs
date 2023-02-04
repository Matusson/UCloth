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
        public int bendingTriangle1;
        public int bendingTriangle2;

        public UCBendingEdge(int nodeIndex1, int nodeIndex2, int triangleIndex1, int triangleIndex2)
        {
            bendingNode1 = nodeIndex1;
            bendingNode2 = nodeIndex2;
            bendingTriangle1 = triangleIndex1;
            bendingTriangle2 = triangleIndex2;
        }
    }
}