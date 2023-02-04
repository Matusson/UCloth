using System.Diagnostics;

namespace UCloth
{
    /// <summary>
    /// Represents a triangle.
    /// </summary>
    [DebuggerDisplay("{index1}, {index2}, {index3}")]
    internal struct UCTriangle
    {
        internal ushort index1;
        internal ushort index2;
        internal ushort index3;

        internal UCTriangle(ushort index1, ushort index2, ushort index3)
        {
            this.index1 = index1;
            this.index2 = index2;
            this.index3 = index3;
        }

        public override bool Equals(object obj)
        {
            UCTriangle tri = (UCTriangle)obj;

            return
                (tri.index1 == index1 && tri.index2 == index2 && tri.index3 == index3) ||
                (tri.index1 == index1 && tri.index2 == index3 && tri.index3 == index2) ||

                (tri.index1 == index2 && tri.index2 == index1 && tri.index3 == index3) ||
                (tri.index1 == index2 && tri.index2 == index3 && tri.index3 == index1) ||

                (tri.index1 == index3 && tri.index2 == index1 && tri.index3 == index2) ||
                (tri.index1 == index3 && tri.index2 == index2 && tri.index3 == index1);

        }

        public override int GetHashCode()
        {
            unchecked { return index1.GetHashCode() ^ index2.GetHashCode() ^ index3.GetHashCode(); }
        }
    }
}
