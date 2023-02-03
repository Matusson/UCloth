using System;

namespace UCloth
{
    /// <summary>
    /// Can be run after simulation to add extra effects.
    /// </summary>
    public interface IUCPostprocessor : IDisposable
    {
        /// <summary>
        /// If set, the postprocessor will cleanup next run.
        /// </summary>
        public bool ScheduledToCleanup { get; set; }

        /// <summary>
        /// False if cleaned up and needs to be disposed.
        /// </summary>
        public bool Deactivated { get; }


        /// <summary>
        /// Processes vertices.
        /// </summary>
        /// <param name="vertices"> Mesh data to process. </param>
        /// <returns> Processed mesh data. </returns>
        public UCRenderingMeshData Process(UCRenderingMeshData data);

        /// <summary>
        /// Runs after <see cref="ScheduledToCleanup"/> has been set.
        /// </summary>
        /// <param name="data"> Mesh data to clean up. </param>
        /// <returns> Cleaned up mesh data. </returns>
        public UCRenderingMeshData Cleanup(UCRenderingMeshData data);
    }
}