Autonomous Car
==============

This project is the simulation implemented for my master's thesis: Path
Planning and Path Following for an Autonomous Car. It is written in C# with the
XNA game engine using Farseer Physics. This project was developed using Visual
Studio 2010 with Service Pack 1. You must also have [XNA Game Studio 4.0]
(http://www.microsoft.com/en-us/download/details.aspx?id=23714) in order to
compile and run the simulation.

Getting Started with the Simulator
----------------------------------

The simulator runs a mission described by a JSON mission file located in the
Missions folder, which already includes some example mission files. The mission
file defines a start pose, a goal pose, and an environment of obstacles.

Once the simulator is fired up, a mission needs to be loaded. Hit `Escape` to
open the simulator console, then type:

    load <mission file>

This mission file will be searched for in the Missions directory. Leave off the
`.json` extension. For example, use the following console command to load the
`default.json` mission file:

    load default

Hit `Escape` again to close the simulator console. Once a mission is loaded,
the path planner will automatically start searching for a path. Once a path is
found, it will be drawn in the environment. The smoother will then iteratively
smooth the discovered path, updating the scene at each iteration.

Once a path has been discovered and smoothed, hit `Return` to start the path
follower. The path follower will then drive the vehicle along the path to the
goal pose, where it will stop and complete the mission.

There are many options used to control the camera, the objects drawn in the
scene, and other parameters. These options are set using keys on the keyboard
and are described in the following section.

Key Bindings
------------

### Objects Drawn in the Scene

- C: Draw 3D car
- G: Draw distance field
- V: Draw Voronoi field
- H: Draw heuristic value grid
- B: Draw Hybrid A* search tree
- Shift+B: Draw search tree while searching
- P: Draw discovered path
- L: Draw smoothed path
- O: Draw velocity profile
- F: Draw controller's front path
- Home: Draw start pose
- End: Draw goal pose
- Comma: Slow down searching (when Shift+B option is active)
- Period: Slow down smoothing

### Camera

These keys are on the number row, not the numpad.

- 1: Choose birds-eye camera
- 2: Choose chase camera
- 3: Choose side-view camera
- 4: Choose goal camera
- 5: Choose top-down camera

The birds-eye camera can be controlled by the following keys:
- Up/Down/Left/Right: Pan the camera
- Shift+Up/Shift+Down: Zoom in/out

### Car Control
- W: Forward gas
- S: Reverse gas
- A: Turn left
- D: Turn right
- Space: Brake

Using any of the car control keys will deactivate the path follower until the
mission is restarted or a new mission is loaded.

### Simulator
- F11: Toggle fullscreen
- F12: Toggle dark/light background
- Enter: Start the path follower
- 0: Restart the current mission
- Escape: Open or close the simulator console
- Shift-Escape: Exit the simulator
