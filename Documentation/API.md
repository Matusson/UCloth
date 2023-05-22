This documents basic APIs that the UC Cloth component exposes. They allow basic interaction with the simulation, such as grabbing, as well as reading its data. 

## UCInternalSimData
This class can be accessed under `UCCloth.simData`. It contains various `NativeArray`s that give access to the simulation data. Most are read-only and cannot be safely modified.

You can currently modify two properties - reciprocal of the weight, and a hashmap of pinned positions (note that a vertex is "pinned" if its weight reciprocal is 0), with the key being the vertex ID. This allows grabbing - if you set the weight reciprocal to 0, you can then freely update the pinned position and it will be updated in the simulation accordingly. Performing one-off modification in this way is also possible, if you set the weight reciprocal back to 1 after submitting the changes.

If you modify data, you have to apply your changes using `UCCloth.simData.ApplyModifiedData()`.

## QueryClosestPoints
This function asynchronously fetches the vertex ID that is the closest to a given position in world space. It's async as the query is performed inside of Burst code, utilizing the spatial partitioning system that is also used for self-collision. As such, you should call it in a Coroutine. You can also specify a radius in your query, which can return multiple points.

<br>
The result is a list of indices that you can utilize to access the sim data in the `UCInternalSimData` class.

