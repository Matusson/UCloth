This file contains a list of known issues and possible potential improvements you should keep in mind.

## Issues:
- The friction model is approximated. It's recommended that you keep friction values below 0.95 for plausible results.
- Simulation data is generated on Start(). This creates stutter, particularly with a lot of cloth objects and on low-end devices. Ideally, this would be computed in Editor and simply loaded on Start(). 
- With the way scheduling is currently implemented (InvokeRepeating), simulation can slow down if simulation rate is a lot higher than render refresh rate. This should be decoupled, but you can use higher iteration counts instead for better quality.
- The "Direction" property on capsule colliders is not supported and will default to Y-Axis. The height property might be wrong to some degree. 


## Potential improvements:
- Currently, the simulation mesh is directly coupled to the render mesh. Use tessellation for smoother meshes. For more flexibility, you would need to implement a retargeting system to decouple these two.
- Currently, the pinning system is always active. Giving the option to disable pins when the pinning gameobject is disabled could add some extra flexibility to the simulation.
- Primitive collision with spheres, capsules, and boxes is available. Technically, SDF collision should also be possible.

Other limitations, issues, and potential improvement areas are marked with // TODO and // NOTE in code.