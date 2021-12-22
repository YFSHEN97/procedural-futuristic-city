# Voronoi Textures

## Introduction
This folder contains the main code for Voronoi/Worley noise textures.

voronoi.py and test.py are both Python scripts for generating and evaluating a 
2D Voronoi texture. See F1.jpg, F2.jpg, F3.jpg for examples.

voronoi.osl is the OSL source file for our Voronoi shader. To use it, you need 
to go into Blender and follow these steps:

## Instructions
1. Make sure you have the correct render properties. The renderer should be 
Cycles (not Blender's default Eevee). You also need to check the box for "Open 
Shading Language", otherwise the OSL shader won't be rendered.
2. Create any simple geometry and assign a new material to it.
3. Go into the node editor and show the shader graph for that material.
4. Create a new Script node.
5. In the Script node, link voronoi.osl as an external file.
6. The shader should automatically compile. If it doesn't, click the refresh 
button in the Script node to ask Blender to compile it.
7. Connect one of the Script node's outputs to a material output node.
8. If you want to use a 2D Voronoi texture, you need to create a Texture 
Coordinate node, and then connect its "UV" output to the "UV" input slot of the
Script node. You also need to make sure that your geometry has already been UV 
unwrapped properly. This is because the Voronoi shader doesn't know the UV 
coordinates of the point to be rendered, so you need to explicitly tell it what
the UV values are.
9. Turn on Viewport Shading.
10. You should see the object being rendered interactively with the texture 
applied to it. You can tweak the parameters in the Script node. The meaning of 
all the parameters are explained in the comments for the Voronoi shader's 
signature.

Note: when you load up a .blend file that already contains a Script node and 
the texture isn't rendering correctly, make sure that you re-enter the correct 
path to the .osl source file in the Script node and recompile!!

Have fun playing with it!
