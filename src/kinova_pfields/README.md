# POTENTIAL FIELD VELOCITY GENERATOR FOR KINOVA ROBOTIC ARMS:

What it contains:

1. This package contains scripts which sets up a potential field for n objects: 1 attractor and n-1 repellers. The scene can be set up in different ways using different scripts.
  a. In the first scenario we assume there are n possible goal locations simultaneously present. For each possible goal there will be n-1 repellers. A total of n potential field nodes will be instantiated where each potential field will take care of each of the goal-repeller combinations.
  b. The flag -n for the potential field node is for all the objects that is relevant to that particular potential field.
  c. For multi-segment tasks, there will be a potential field for each segment, with single goal and no repellers.

2. A sample launch file is included which launches a potential field and spawns one object in the scene.

3. The node which will make use of the potential fields needs to implement the QueryVel service. When needed the service will query the potential field with the current robot pose as its argument and the service will return the 6D velocity to the attractor specified by that particular potential field. For example, refer to ...

4. The potential field so far takes care of:
  a. Collision avoidance with the base of the ROBOT
  b. Collision avoidance with the table

5. Currently for ease of use: 3 pre-specified orientations namely: flat(1), over-the-top (2), and door knob (3) are used. If more orientations are needed you can add them to the available list then index them accordingly.

THINGS TO DO:
1. Computing the shortest angle to the goal, so that the potential field does not make the robot rotate in the wrong direction. Making the goal orientation to be more general.
2. Minor fixes to avoid the robot going in the wrong y-side.
