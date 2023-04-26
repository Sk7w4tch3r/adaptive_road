# Adaptive road

Add sketch to input mesh.


# Usage:
`./build/adaptive_road <mesh> <sketch> <level> <vis>`

mesh: name of the input OFF file without the OFF format. i.e., blender_terrain
sketch: name of the input .png sketch file without the formart, i.e., shaky
level: levels of subdivision to use, i.e., 2
vis: 1 to show visualization at the end, 0 to disable.

# Screenshots


## TODO
- add user interface for drawing sketch.
- use DEM and gps data.
- convert python code to cpp.
