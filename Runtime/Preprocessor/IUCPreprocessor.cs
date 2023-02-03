namespace UCloth
{
    /// <summary>
    /// Creates simulation data.
    /// </summary>
    public interface IUCPreprocessor
    {
        /// <summary>
        /// Converts an object to simulation data.
        /// </summary>
        /// <param name="input"></param>
        /// <param name="positions"></param>
        /// <param name="edges"></param>
        /// <returns> Was the data created successfully? </returns>
        public bool ConvertMeshData(object input, out UCMeshData data);
    }
}