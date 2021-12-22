# Futuristic City Generator

## Instructions:
You can install this plugin, activate it in Preferences, and find it in the 
viewport sidebar. If you can't find it, make sure that you are in "Object 
Mode".

## Usage:
There are two modes available. You can either generate a single building (you 
may specify all the parameters) or you can create a full city. Our program 
makes a full city by first creating a road network using L-systems and defining
empty plots of land that are subsequently filled with randomly produced 
buildings.

## Boid Animation
Clicking 'Generate full city' will create the roads, buildings, and boids which fly around the city. The camera will be scripted to follow one of the boids. Once the city is generated, clicking the animation play button will make the boids fly around the city. You may want to click View->Frame All before doing this in order to see the boids better. This is one way to watch the animation, looking at the city as a whole.

A more fun way of watching the boid animation is from the point of view of the camera. By clicking the 'Toggle the camera view' button on the right side of your viewport (the standard Blender button with shortcat Numpad 0), you can set your viewport to follow the camera view. Since the camera follows one of the boids, pressing play will play a much more fun animation, in which you will get a flying view of the city.

## Parameters:
The meaning and effect of all the parameters are explained in the tooltip that 
appears if you hover over them. You are also welcome to generate the model as 
often as you like and see for yourself what it looks like!

## Videos:
While a complete video of our project is unfortunately too computationally expensive, here are some intermediate videos. These do not have the Voronoi textures we wrote, since they take too long to render for the full city.

Firstly, the [partial demo](https://youtu.be/80W_cSkcwmo) shows the flying motion of the boids, as seen from the camera.

Secondly, the [Blender demonstration](https://www.youtube.com/watch?v=p91UskqbARY) shows a global view of the boids flying, seen in the Blender viewport.

Finally, [here](https://www.youtube.com/watch?v=LjG8hPaMy_M) is a very fun animation clip.
