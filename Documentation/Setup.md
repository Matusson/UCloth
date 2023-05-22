This document details basic setup of a UC Cloth component.

1. Import a mesh into Unity. The format should not matter, but it needs to be a triangle mesh (Unity can triangulate it automatically). For optimal results, use meshes with roughly even vertex density. Experiment with density - high density provides best results with collision, but will be more expensive to simulate.

2. Enable Read/Write in the import settings. The component will not be able to generate simulation data without it.

3. Add the mesh to the scene and add the "UC Cloth" component to it. If you wish, you can add pin colliders to it (more info in the [reference](/Documentation/UCClothReference.md)).

4. Run the application. Depending on the mesh, you might see a lot of stretching, instabilities, or even explosions. The tool does not estimate default parameters, and if you experience issues, you will need to adjust it yourself.
Start with adjusting stiffness coefficient (and lightly damping coefficient alongside it) until little or no stretching occurs. If you're using a very high density mesh, try increasing iterations or constraint iterations, particularly if you're seeing "explosions". Setting constraint iterations to 0 can help with the initial parameters as well.  

5. (Optional) Add the "UC Debug Visualizer" component. This can help visualizing various parameters.

6. Once basic default parameters are set, refine the parameters using the [reference](/Documentation/UCClothReference.md).