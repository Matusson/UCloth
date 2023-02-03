using System.Diagnostics;

namespace UCloth
{
    /// <summary>
    /// Represents an edge.
    /// </summary>
    [DebuggerDisplay("{nodeIndex1}, {nodeIndex2}")]
    public struct UCEdge
    {
        public ushort nodeIndex1;
        public ushort nodeIndex2;

        public UCEdge(ushort nodeIndex1, ushort nodeIndex2)
        {
            this.nodeIndex1 = nodeIndex1;
            this.nodeIndex2 = nodeIndex2;
        }

        public override bool Equals(object obj)
        {
            if (obj == null)
            {
                return false;
            }
            if (obj is not UCEdge)
            {
                return false;
            }

            return (this.nodeIndex1 == ((UCEdge)obj).nodeIndex1 && this.nodeIndex2 == ((UCEdge)obj).nodeIndex2)
                || (this.nodeIndex1 == ((UCEdge)obj).nodeIndex2 && this.nodeIndex2 == ((UCEdge)obj).nodeIndex1);
        }

        public override int GetHashCode()
        {
            unchecked { return nodeIndex1.GetHashCode() ^ nodeIndex2.GetHashCode(); }
        }
    }
}