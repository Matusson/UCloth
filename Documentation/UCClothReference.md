The UC Cloth component has a lot of parameters that allow you to heavily customize the simulation. However, configuring them can be difficult, especially initially. This documents hopefully shines some light on what each parameters does, and how to adjust it effectively.


- Preprocessor Type <br>
Only the Mesh option is available. You can implement your own preprocessor to allow generating simulation data from procedural sources.

- Pin Colliders <br>
Here you can specify colliders that will act as pins for the simulation. Every cloth vertex that is within the collider will be pinned to this object. Pin colliders can also move, and the vertices will follow them. Sometimes the placement of the pinning colliders is obvious (for example, for a flag it would be the point of contact with the pole), but you can also use them in places where you need to make the simulation predictable. For example, pinning part of character's clothing to their hands, waist or neck will reduce the possibility of collision issues. You can use triggers to avoid colliding with other physics objects.


## Material Properties
- Stiffness Coefficient <br>
Controls how strong the vertices are pulled together with spring forces to their target distances. If it's too low, cloth will stretch. Increasing it might require you to increase the iteration count or sim frequency, as large movements in a single iteration can collapse the simulation. This parameter highly depends on mesh density.

- Damping Coefficient <br>
If the stiffness coefficient is too strong, it might create oscillation in the system. Damping coefficient can help prevent that. It should be increased alongside the stiffness coefficient, but the optimal ratio differs. Default ratio should provide acceptable results.

- Max Stretch <br>
This parameter affects the edge length constraints. Countering all forces with just the spring forces (controlled by the two parameters above) might not be enough, so extra edge length constraints are added. This parameters affects how much this step will try to compensate. A setting of 1 will converge to the expected stretch amount, but might require a lot of iterations. You can set it below 1 with a lower "Constraint iterations" setting, but with higher iteration count it will overshoot.

- Energy Conservation <br>
Adds some energy back to the simulation after applying edge length constraints. Default of 1 should provide decent results. Higher values can be more "energetic", but less stable.

- Bending Coefficient <br>
Affects the strength of bending forces, useful in simulating materials like leather. High values can be very unstable, particularly if the cloth is not hanging. Reasonably expensive, set to 0 to skip calculations for materials that don't need it.

- Vertex Mass <br>
Affects the overall mass of the cloth. Unstable, only small adjustments recommended if any.

## Simulation Properties
- Gravity Multiplier <br>
Affects how strong the gravity is. This is a multiplier for the scene gravity value.

- Air Resistance Multiplier <br>
Multiplier for air resistance. This is only an approximation, as the simulator does not account for aerodynamics. Changing it affects the perceived "viscousity" of the air.


## Quality Properties
- Sim Frequency <br>
How many times the simulation will try to run per second. Low values will lead to a high time step, which can decrease simulation stability and cause a collapse. Highly depends on the cloth mesh.

- Time Scale Multiplier <br>
Multiplier for game time scale, can be used for slow-motion effects.

- Max Timestep <br>
The highest allowed timestep for the simulation. Used as a fail-safe for the Sim Frequency setting. If the requested timestep is higher than the max, it will be clamped. This can lead to the simulation looking slow. Adjust it responsibly, setting it too high will cause a simulation collapse.

- Iterations <br>
How many times, per simulation frame (as decided by Sim Frequency setting), forces are applied. If you're seeing unstable cloth behaviour, you might need to increase it. Highly depends on the mesh. Does not affect how often collisions are processed (If you need better collision handling, you can uncomment a line in the Execute method of UCJob.cs). Expensive, try to avoid setting it higher than it needs to.

- Constraint Iterations <br>
Controls how many times, per simulation frame, edge length constraints are applied. This can help with the cloth stretching without needing to set the above parameter too high (as it's more expensive). Affected by the Max Stretch setting. Can be set to 0 to skip it.

- Minimize Latency <br>
By default, the simulation can have around 1 frame of latency due to scheduling considerations, which can take better advantage of the background threads. If minimizing latency is important, this setting can slightly improve it at the cost of performance.


## Collision Properties
- Collision Friction <br>
Multiplier for object friction. Setting it too high will lead to weird results. Base friction is derived from the "Dynamic Friction" property of collider's physics material.

- Collision Velocity Correction <br>
Multiplies the collider's velocity by this factor and adds it to colliding vertices. Can improve collisions with moving colliders, but high values lead to unexpected amount of energy from the movement.

- Collision Contact Offset <br>
Extra buffer between colliders and the cloth, can be used to prevent clipping.

- Enable Self Collision <br>
Allows vertices of the cloth to collide with each other. Very expensive setting. All of the self-collision settings have an effect only if this is enabled.

- Self Collision Distance <br>
Distance at which the vertices will start to collide with each other. Low values can lead to missed collisions, high values will cause jitter and instability. Highly dependant on vertex density.

- Self Collision Stiffness <br>
Multiplier for the force that pushes the colliders apart. Reducing it below 1 can lead to smoother-looking result, at the cost of less robust self-collision. Higher values than 1 not recommended.

- Self Collision Friction <br>
Changes how velocity is affected during self-collision.

- Self Collision Velocity Conservation <br>
The way that self-collision is handled can cause velocity bleed - colliding vertices might appear to lose a lot more velocity than would be expected. However, adding this velocity back can decrease the accuracy of collisions. This value is a tradeoff between the two - 0 will provide the best accuracy, but at possible velocity loss, while a setting of 1 will more accurately conserve velocity, but with more clipping.

- Self Collision Accuracy <br>
Affects the accuracy of self-collisions. In technical terms, the lowest setting will only check for collisions between vertices in the same cell (which might be acceptable for just rough collisions), the middle one checks surrounding non-corner cells, and the highest checks all surrounding cells. The default setting is usually fine.

- Auto Adjust Grid Density<br>
If the setting is not None, the self-collision grid size is adjusted automatically. The Performance option might occasionally miss collisions. Affected by the setting above.
If set to None, manually specify the grid density using the parameter below.


## Colliders
All colliders have to be specified in those lists. Note that there are performance differences between the types - spheres are the fastest, capsules are slightly slower, and boxes/cubes are a lot more expensive. Note that you can use triggers as colliders, which will still collide with the cloth, but not with other objects. Non-uniformly scaled colliders will not work well. 


## Post Processing
- Thickness<br>
Adds procedural thickness to the material by extruding vertices. This allows simulating cloth with thickness without including both sides in the simulation mesh, which prevents the cloth clipping into itself.

- Offset Front<br>
If thickness is enabled, enabling this setting will offset the front vertices by half of the thickness. This effectively makes the simulated mesh be in the "center" of the post-processed mesh. Otherwise, the simulated mesh is in the front, and thickness is added in the back, which can cause clipping with colliders on one side, but not the other.

- Smoothing<br>
Smoothes out the movement of clothing using an exponential moving average. Reduces responsiveness, but increases smoothness.