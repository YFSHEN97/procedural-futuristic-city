bl_info = {
    "name":"Futuristic Building Generator",
    "author":"Yifei Shen",
    "version":(1,0),
    "blender":(2,93,0),
    "location":"3D View > Tools",
    "description":"Creating futuristic buildings procedurally",
    "warning":"",
    "wiki_url":"",
    "tracker_url":"",
    "category":"Add Mesh"
    }

import bpy
import bmesh
from math import pow, sqrt
from random import uniform
from mathutils import Vector
import numpy as np

RANGE = 20.0
SPEED = 20.0
VELOCITY_STEADY_FRAMES = 5 # Boid changes its velocity every n frames
GOALS = {0: (45, 195, 40), 60: (20, 20, 10), 150: (175, 45, 0), 300: (205, 65, 55)} # Points boids aim towards at certain frames
CAMERA = None

# Returns length of a vector
def length(vector):
    return np.sqrt(np.sum(vector ** 2))

# Normalizes a vector
# If length negligible, just returns 0
def normalize(vector):
    norm = length(vector)
    if norm < 0.001:
        return np.array((0, 0, 0))
    
    return vector / norm

# Controls the camera object so we can place it on a boid
class Camera:
    def __init__(self, boidToFollow):
        self.body = bpy.data.objects['Camera']
        self.boid = boidToFollow

    # Updates the position and direction of the camera to match the boid
    def updateToMatchBoid(self):
        # Place camera slightly in front of the boid
        boidRadius = 0.5
        boidVelocity = normalize(self.boid.velocity)
        position = self.boid.position + boidVelocity * (boidRadius + 0.1)

        # Point camera in direction of boid velocity
        direction = Vector(boidVelocity)
        rotation = direction.to_track_quat('-Z', 'Y')
        self.body.rotation_euler = rotation.to_euler()

        '''
        x, y, z = position.tolist()
        rotX = atan(z / y)
        rotY = atan(x / z)
        rotZ = atan(y / x)
        '''

        # Save this key frame (requires global frame to have been set)
        self.body.location = position.tolist()
        self.body.keyframe_insert(data_path='location')
        self.body.keyframe_insert(data_path='rotation_euler')

# A brain that takes a boid and calculates its next velocity
class BoidNavigationSystem:
    def __init__(self, boids, obstacles):  
        self.boids = boids
        self.obstacles = obstacles  
        self.initialiseObstacleStructures(obstacles)

    # Setup the data structures which will allow boids to easily determine
    #   whether they are about to collide with a building
    # obstacles: list of (coords, (x_width, y_width), height) for each building
    def initialiseObstacleStructures(self, obstacles):
        self.buildingsByX = [(obs[0][0], obs) for obs in obstacles]
        self.buildingsByY = [(obs[0][1], obs) for obs in obstacles]
        self.buildingsByX.sort(key = lambda obs: obs[0])
        self.buildingsByY.sort(key = lambda obs: obs[1])
    
    def calculateAcceleration(self, boid):
        goal = self.goalFollowing(boid)
        ca, imminentCollision = self.collisionAvoidance(boid)
        vm = self.velocityMatching(boid)
        fc = self.flockCentering(boid)
        return self.combineBehaviours(goal, ca, vm, fc, imminentCollision)
    
    # Try fly towards the goal
    def goalFollowing(self, boid):
        goal = boid.getGoal()
        accel = goal - boid.position
        #print("Goal: " + str(goal))
        return normalize(accel)
    
    # Try fly away from all neighbours
    def collisionAvoidance(self, boid):
        boidAvoidance = self.boidAvoidance(boid)
        buildingAvoidance = self.cityAvoidance(boid)

        # Prioritise city avoidance if it exists 
        if buildingAvoidance is not None:                           # CHANGE WEIGHTS TO PRIORITISE CITY AVOIDANCE
            return buildingAvoidance, True
        return boidAvoidance, False

    # Return vector for avoiding other boids
    def boidAvoidance(self, boid):
        result = np.array((0.0, 0.0, 0.0))
        for other in self.boids:
            if other == boid:
                continue
            
            dist = length(other.position - boid.position)
            
            if dist > RANGE:
                continue
            
            result += normalize(boid.position - other.position) / (dist ** 2)
        
        return normalize(result)

    # Return vector for avoiding buildings and floor
    def cityAvoidance(self, boid):
        nearbyBuildings = []
        xPos, yPox = boid.position[0], boid.position[1]
        alertDistance = 10.0
        # Search for nearby buildings by x and y coordinate
        for building in self.buildingsByX:
            if abs(building[0] - xPos) < alertDistance:
                nearbyBuildings.append(building[1])
        for building in self.buildingsByY:
            if abs(building[0] - xPos) < alertDistance:
                nearbyBuildings.append(building[1])

        # For each nearby building, check if we're going to hit it
        buildingAhead, buildingDist = None, float('inf')
        for building in nearbyBuildings:
            willHit, dist = self.willHitBuilding(boid, building)
            if not willHit:
                continue

            if dist < buildingDist and building != buildingAhead:
                buildingAhead = building
                buildingDist = dist

        if buildingAhead is None:
            return None

        # If we're going to hit a building, turn right or left
        return normalize(np.cross(boid.velocity, np.array((1, 0, 0))))   # THIS FAILS IF A COMPONENT IS 0
        # Calculate all city squares we're hitting in the near future
        # Extract all buildings in all those squares
        # For each buildling
        #   Check if we're colliding soon
        #   Set nearest collision if that's the case

        # For the nearest collision:
        #   Return normal at that point

    # Calculates whether the boid will hit the building, and if so, the distance to collision
    #   Building is (center, (widthx, widthy), height)
    #   Treats building as a cylinder
    #   Returns whether it will hit and the distance to the hit
    def willHitBuilding(self, boid, building):
        ALERT_RANGE = 10.0

        r = max(building[1])    # Radius of cylinder
        o = boid.position   # Ray origin
        c = building[0]     # Building center
        d = boid.velocity   # Ray direction

        fx = o[0] - c[0]
        fy = o[1] - c[1]

        # Quadratic equation
        a = pow(d[0], 2) + pow(d[1], 1)
        b = 2*fx*d[0] + 2*fy*d[1]
        c = pow(fx, 2) + pow(fy, 2) - pow(r, 2)

        # Solve
        discr = pow(b, 2) - 4*a*c
        if discr < 0:
            return False, None
        discr = sqrt(discr)
        t1 = (-b + discr) / (2*a)
        t2 = (-b - discr) / (2*a)
        solnSmall = min(t1, t2)
        solnBig = max(t1, t2)
        soln = None
        if solnBig < 0:
            soln = None
        elif solnSmall < 0:
            soln = solnBig
        else:
            soln = solnSmall

        # Check hits in reasonable distance
        if soln is None or soln > ALERT_RANGE:
            return False, None

        # Check hits within bounds of cylinder
        height = building[2]
        hitPoint = o + d * soln
        if hitPoint[2] < 0 or hitPoint[2] > height:
            return False, None
        #print(soln)
        return True, soln
     
    # Try match velocity with neighbours       
    def velocityMatching(self, boid):
        totalVel = np.array((0.0, 0.0, 0.0))
        numFriends = 0 # Number of neighbours counted
        for other in self.boids:
            if other == boid:
                continue
            
            dist = length(other.position - boid.position)
            
            if dist > RANGE:
                continue
            
            totalVel += other.velocity
            numFriends += 1
        
        if numFriends == 0:
            return totalVel
        
        #if boid.Number == 1:
        #print(normalize(totalVel / float(numFriends)))
        return normalize((totalVel / float(numFriends)) - boid.velocity)
    
    # Try go to center nearby flockmates
    def flockCentering(self, boid):
        totalPos = np.array((0.0, 0.0, 0.0))
        numFriends = 0 # Number of neighbours counted
        for other in self.boids:
            if other == boid:
                continue
            
            dist = length(other.position - boid.position)
            
            if dist > RANGE:
                continue
            
            totalPos += other.position
            numFriends += 1
            
        if numFriends == 0:
            return totalPos
        
        return normalize((totalPos / float(numFriends)) - boid.position)
    
    # Combines independent boid desires into one acceleration
    def combineBehaviours(self, goal, ca, vm, fc, imminentCollision):
        #kCa, kVm, kFc = 0.2, 0.4, 0.95
        kGoal, kCa, kVm, kFc = 0.8, 0.4, 0.1, 0.2
        if imminentCollision:
            kCa = 1.5

        combined = (kGoal*goal + kCa*ca + kVm*vm + kFc*fc)
        
        return normalize(combined)
    
class Boid:
    Number = 1
    
    def __init__(self, startPos, startVelocity):
        self.velocity = startVelocity
        self.position = startPos
        self.acceleration = None
        self.body = self.createBody()
        self.id = Boid.Number
        Boid.Number += 1
        
    def createBody(self):
        bpy.ops.mesh.primitive_ico_sphere_add(subdivisions=4, radius=0.5)
        newObject = bpy.context.active_object
        newObject.name = 'Boid ' + str(self.Number)
        return newObject

    # Sets the boid's acceleration for the next x frames.
    #   Probably shouldn't be called every frame if your 
    #   acceleration computation is expensive
    def setNewAcceleration(self, newAcceleration):
        self.acceleration = newAcceleration / 2.0
    
    # Takes accel, acceleration over the next dt seconds
    # Calculates and saves next pos and vel so they can be used messing
    #   The results must be stored in this intermediate way to avoid
    #   up other boids' calculations during the same frame. The caller 
    #   will decide when to actually enact the new position and velocity
    def saveNextState(self, dt):
        # Compute next position
        # suvat: s = ut + (at^2)/2
        disp = self.velocity * dt + (self.acceleration * (dt ** 2)) / 2
        newPos = self.position + disp
        self.nextPos = newPos
        
        # Compute next velocity
        newVel = self.velocity + self.acceleration * dt

        

        # In theory, capping the maximum speed in this way should be better
        # Draw a diagram to see why. But in practice, it doesn't make that
        # much of a difference.
        maxSpeed = dt * SPEED
        if length(newVel) > maxSpeed:
            newVel = normalize(newVel) * dt * SPEED

        #newVel *= dt * SPEED / length(newVel)
        self.nextVel = newVel
    
    # Set the boid's current position as a keyframe
    #   Relies on the correct global frame having been set
    def setCurFrame(self):
        self.body.location = self.position.tolist()
        self.body.keyframe_insert(data_path='location')
        
    # Sets the goal of the boid from the current frame onwards
    # goal is a point towards which the boid will navigate
    def setNewGoal(self, goal):
        self.goal = goal
        
    def getGoal(self):
        return self.goal
        
    # Jumps to the next state
    def setNextState(self):
        self.position = self.nextPos
        self.velocity = self.nextVel
  
def deleteAllBoids():
    for obj in bpy.context.scene.objects:
        if obj.name[:4] == 'Boid':
            obj.select_set(True)
        
    bpy.ops.object.delete()
      
# buildingSpaces is list of spaces in which buildings are (for boid building avoidance)
def createBoids(numBoids, startPoint, frames, dt, buildingSpaces):
    boids = []
    for boid in range(numBoids):
        startX = uniform(-10.0, 10.0) + startPoint[0]
        startY = uniform(-10.0, 10.0) + startPoint[1]
        startZ = uniform(-3.0, 3.0) + startPoint[2]  # Start them all at a similar height
        startPos = np.array((startX, startY, startZ))
        startVel = normalize(GOALS[0] - startPos) * SPEED * dt

        #startPos = 2 * np.random.rand(3) - np.array((1, 1, 1)) + startPoint     # MOVE THIS IF IT'S INSIDE A BUILDING!!
        newBoid = Boid(startPos, startVel) 
        boids.append(newBoid)
        
    # Setup animation system
    navSystem = BoidNavigationSystem(boids, buildingSpaces)
    bpy.context.scene.frame_start = 0
    bpy.context.scene.frame_end = frames

    # Create camera attached to boid 1
    CAMERA = Camera(boids[0])
    
    for frame in range(frames):
        #print('Frame ' + str(frame))
        bpy.context.scene.frame_set(frame)
        # Calculate next pos and vel for each boid
        for boid in boids:
            boid.setCurFrame()
            
            newGoal = GOALS.get(frame)
            if newGoal:
                boid.setNewGoal(newGoal)
            
            # Update boid
            if frame % VELOCITY_STEADY_FRAMES == 0:
                # Calculate new acceleration if we've reached assessment frame
                accel = navSystem.calculateAcceleration(boid)
                boid.setNewAcceleration(accel)
            # Update boid position and velocity 
            boid.saveNextState(dt)
            '''
            if boid.body.name == 'Boid 1':
                print(boid.nextPos)
                print(boid.nextVel)
            '''
            
        # Set new pos and vel for each boid
        for boid in boids:
            #print(boid.velocity)
            boid.setNextState()

        # Set new pos and rot for camera
        CAMERA.updateToMatchBoid()

        #print('')
            
# Takes a list of all building spaces so the boids can fly around them
# cityWidth is the width of the entire city, something like 250m
def generate_boids(buildingSpaces, cityWidth):
    deleteAllBoids()
    maxBuildingHeight = max(buildingSpaces, key=lambda x: x[2])[2]
    cityCenter = np.array((cityWidth/2, cityWidth/2, maxBuildingHeight/2))
    createBoids(30, cityCenter, 480, 1.0/6, buildingSpaces)

'''
if __name__ == '__main__':
    deleteAllBoids()
    createBoids(30, 480, 1.0/6) 
'''