# Prototype Burst-based cloth simulation for Unity

## WARNING: This project is not production ready and no further work will be done. No PRs will be accepted. It is primarily intented to be a learning resource.


---
This package implements custom cloth physics. It runs on the CPU, doing most of the simulation work on a background thread (some main thread cost still applies). It utilizes the Jobs system for partial multithreading - a single cloth object can't be split into multiple threads, but two cloth objects can be simulated at the same time on different threads.

The project contains many issues that would need to be fixed if you want to use it in a production environment. A list of known ones is included in [Issues.md](/Documentation/Issues.md). I do not take responsibility for any, and I will not provide support. You are free to use this project as a starting point or a basic learning resource for your own simulation.

See [Setup.md](/Documentation/Setup.md) for information on how to set up the simulation, and [UCClothReference.md](/Documentation/UCClothReference.md) for a brief explanation of all parameters.