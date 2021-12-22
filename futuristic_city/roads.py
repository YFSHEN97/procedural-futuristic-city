import bpy
import numpy as np
import random
from math import cos, sin, pi, acos, atan
from mathutils import Vector

from . fbsweep_gen import sweep_skyscraper
from . boids import generate_boids
from bpy.props import PointerProperty
from bpy.types import PropertyGroup, Operator
    
def to_radians(degrees):
    return degrees * pi / 180
    
# Creates a physical road from a turtle path string
class TurtlePathBuilder:
    def __init__(self, collection, step, angle, width):
        self.collection = collection
        self.step = step
        self.angle = angle
        self.width = width
        self.startPoint = (0.0, 0.0, 0.0)
        
    # Sets the point at which the turtle starts
    #   startPoint is tuple (x, y, z)
    def setStartPoint(self, startPoint):
        self.startPoint = startPoint
        
    # Takes a string describing a turtle's path and creates a physical road
    #   path can contain F, -, +
    #   returns points of turtle path
    def buildRoadFromPath(self, path):
        turtleV, turtleE = self._createTurtlePointsFromString(path)
        roadV, roadE, roadF = self._createRoadFromTurtlePoints(turtleV)
        #create_object_from_locations(turtleV, turtleE, [], 'turtle', collection)
        self._createPolygon(roadV, roadE, roadF)
        return turtleV
        
    # Creates a list of turtle points from the turtle path_string
    # path_string can contain F, +, and -. All angles are 90 degrees
    # step: distance turtle advances each F
    # angle: angle change each + or -  
    def _createTurtlePointsFromString(self, path_string):
        # Create left and right vertices at each turtle F
        pos = np.array([*self.startPoint]) # Current turtle position
        points = [pos.tolist()]
        edges = []
        orientation = 0.0
        for char in path_string:
            if char == 'F':
                pos += [self.step*sin(to_radians(orientation)), self.step*cos(to_radians(orientation)), 0.0]                       # TRY IT OUT!
                points.append(pos.tolist())
                edges.append((len(points)-2, len(points)-1))
            elif char == '+':
                orientation -= self.angle                        # CHECK THIS GOES IN CORRECT DIRECTION!!
            elif char == '-':
                orientation += self.angle
        return points, edges
    
    # Creates a road centred around the turtle path
    # turtlePath is list of points defining path of turtle
    # So the turtle can walk down the road.
    def _createRoadFromTurtlePoints(self, turtlePath):
        vertices, edges, faces = [[self.startPoint[0]+self.width, self.startPoint[1], self.startPoint[2]], [self.startPoint[0]-self.width, self.startPoint[1], self.startPoint[2]]], [(0, 1)], []
        
        for turtleI in range(len(turtlePath) - 2):
            # Get current three turtle points
            a = np.array(turtlePath[turtleI])
            b = np.array(turtlePath[turtleI+1])
            c = np.array(turtlePath[turtleI+2])
            v1 = b - a
            v2 = c - b
            
            # Calculate angle road is going in now
            bcGradient = float('inf')*(c[1]-b[1]) if c[0]-b[0] == 0 else (c[1]-b[1])/(c[0]-b[0])
            abGradient = float('inf')*(b[1]-a[1]) if b[0]-a[0] == 0 else (b[1]-a[1])/(b[0]-a[0])
            bcDir = atan(bcGradient)
            abDir = atan(abGradient)
            
            # Correct in case tan doesn't reach far enough
            # Orientations will be -pi to pi, with 0 along positive x axis
            if c[0]-b[0] < 0:
                bcDir += pi
            if b[0]-a[0]<0:
                abDir += pi
            bisectAngle = (abDir - bcDir) / 2
            
            # Calculate left and right points
            rightDir = bcDir + bisectAngle
            leftDir = bcDir + bisectAngle - pi
            
            turnWidth = self.width / cos(bisectAngle) # Width of road at the turning point
            right = b + [turnWidth*sin(rightDir), -turnWidth*cos(rightDir), 0]
            left = b + [turnWidth*sin(leftDir), -turnWidth*cos(leftDir), 0]
            
            # Add vertices to list
            vertices.append(right.tolist())
            vertices.append(left.tolist())
            
            oldR, newR = 2*turtleI, 2*turtleI+2
            oldL, newL = 2*turtleI+1, 2*turtleI+3
            
            # Connect left and right to old
            edges.append((oldR, newR))
            edges.append((oldL, newL))
            
            # Create edges crossing through road
            edges.append((oldR, newL))
            edges.append((newL, newR))
            
            # Create two triangular faces
            faces.append((oldR, newL, oldL))
            faces.append((newR, newL, oldR))

        return vertices, edges, faces
            
    # Creates an object in the world from points
    def _createPolygon(self, vertices, edges, faces):
        mesh = bpy.data.meshes.new('roadSurface')
        mesh.from_pydata(vertices, edges, faces)
        mesh.update()
        
        road = bpy.data.objects.new('road', mesh)
        self.collection.objects.link(road)
            
            
# An L System that computes a valid turtle path
class TurtleLSystem:
    def __init__(self, axiom, rules, n):
        self.axiom = axiom
        self.rules = rules
        self.n = n
        
    def computeTurtlePath(self):
        rawResult = self._computeLSystem()
        # Replace temporary characters to allow for extended systems
        final = ('F' if x in ['L', 'R'] else x for x in rawResult)
        return final
    
    def _computeLSystem(self):
        return self._iterateLSystem(self.axiom, self.rules, self.n)
    
    # Iterates the L-System n times
    # axiom is starting string, rules is dictionary of tuples for replacement
    # e.g. rules = {'a': 'ab', 'b': 'ba'}
    def _iterateLSystem(self, axiom, rules, n):
        # Check if finished
        if n == 0:
            return axiom
        
        # Replace all characters in string in parallel
        newString = ''
        for char in axiom:
            newString += self._createCharReplacement(rules, char)
            
        return self._iterateLSystem(newString, rules, n-1)
    
    # Takes a character and replaces it using the rules
    def _createCharReplacement(self, rules, char):
        replacement = rules.get(char, None)
        return replacement if replacement is not None else char
    
# L System that allows randomness
class StochasticTurtleLSystem(TurtleLSystem):
    # Rules have built in probabilities
    #   e.g. rules = { 'a': [('ab', 0.3), ('aa', 0.4), ('a', 0.3)]}
    def __init__(self, axiom, rules, n):
        super().__init__(axiom, rules, n)
    
    def _createCharReplacement(self, rules, char):
        allRuleSets = rules.get(char, None)
        
        # Default to char if no rules available
        if allRuleSets is None:
            return char
        
        # Choose a replacements at random
        replacements = [r[0] for r in allRuleSets]
        weights = [r[1] for r in allRuleSets]
        chosenReplacement = random.choices(replacements, weights=weights)
        return chosenReplacement[0]
    
# Creates a square graph from a list of turtle points
#   Splits the turtle's domain (a big square) into small squares
#   turtlePoints = [(0,0,0), (1,0,0), (2,1,0), ...] 
#   f is length of one turtle movement
# hackyCenterDelete: hacky way to remove the center of the outer grid, 
# which is just in the way
def createGraphFromTurtlePoints(turtlePoints, f, hackyCenterDelete=False):
    bottomLeft, topRight = getTurtleDomain(turtlePoints)
    gridGraph = SquareGraph(f, bottomLeft, topRight)
    gridGraph.joinAllTouchingNodes(turtlePoints)
    
    if hackyCenterDelete:
        gridGraph.hackyDeleteCenterNodes()
    
    #gridGraph.debugBuild()
    return gridGraph


# Returns the bottom left and top right corners of the turtle's domain
#   i.e. the square containing all the turtle's positions
def getTurtleDomain(turtlePoints):    
    minX = min((p[0] for p in turtlePoints))
    maxX = max((p[0] for p in turtlePoints))
    minY = min((p[1] for p in turtlePoints))
    maxY = max((p[1] for p in turtlePoints))
    
    return (minX, minY), (maxX, maxY)
  
# Graph representing a grid of points  
class SquareGraph:
    # Takes side lenth of small squares, as well as 
    #   outer corners of large grid
    def __init__(self, smallSide, bottomLeft, topRight):
        self._createAllNodes(smallSide, bottomLeft, topRight)
    
    # Creates all nodes by splitting up big square into
    #   lots of small square of length smallSide
    def _createAllNodes(self, smallSide, bottomLeft, topRight):
        self.nodes = []
        y = bottomLeft[1] + smallSide/2
        # Create all nodes
        while y < topRight[1]:
            x = bottomLeft[0] + smallSide/2
            curRow = []
            while x < topRight[0]:
                self.nodes.append(Node((x, y, 0)))
                x += smallSide
            y += smallSide
            
        # Create lists of x and y coordinates of nodes
        self.xValues = []
        x = bottomLeft[0] + smallSide/2
        while x < topRight[0]:
            self.xValues.append(x)
            x += smallSide
        self.colNum = len(self.xValues)

        self.yValues = []
        y = bottomLeft[1] + smallSide/2
        while y < topRight[1]:
            self.yValues.append(y)
            y += smallSide
        self.rowNum = len(self.yValues)
    
    # Join all nodes which touch in the turtlePath
    #   i.e. join two nodes iff there is no road separating them    
    def joinAllTouchingNodes(self, turtlePath):
        # Store all horizontal and vertical neighbour edges
        # Every edge is directed smaller_index -> bigger_index
        tentativeEdges = {}
        for i in range(self.colNum*(self.rowNum)):
            # Create right edge
            if (i+1) % self.colNum != 0:
                tentativeEdges[(i, i+1)] = True
                
            # Create up edge
            if i < self.colNum*(self.rowNum-1):
                tentativeEdges[(i, i+self.colNum)] = True
        
        # Iterate through turtle path, removing edges it breaks
        for i,curPoint in enumerate(turtlePath[1:],1):
            prevPoint = turtlePath[i-1]
            
            isVertical = round(curPoint[0], 8) == round(prevPoint[0], 8)    # WATCH OUT! IS IT EXACT?
            
            # Find relative position of edge in grid
            xIndex = self._findIndexInSortedList(self.xValues, max(curPoint[0], prevPoint[0]))
            yIndex = self._findIndexInSortedList(self.yValues, max(curPoint[1], prevPoint[1]))
            
            # If turtle edge is outside of graph, ignore
            if xIndex == None or yIndex == None:
                continue
            
            # Get nodes on either side of edge by indexing into grid
            nodeA = self.colNum*yIndex + xIndex
            nodeB = None
            if isVertical:
                nodeB = self.colNum*yIndex + xIndex+1
            else:
                nodeB = self.colNum*(yIndex+1) + xIndex
                
            # Break edge
            tentativeEdges[(nodeA, nodeB)] = False
            
        # Create all remaining edges in graph
        self.edges = {}
        for edge,exists in tentativeEdges.items():
            if exists:
                nodeA, nodeB = self.nodes[edge[0]], self.nodes[edge[1]]
                nodeA.addEdge(nodeB)
                self.edges[edge] = True
                
    # Finds the index at which to insert in a sorted list
    #   Returns index of largest value smaller than mid
    def _findIndexInSortedList(self, values, mid):
        if values[0] > mid:
            return None
        
        for i, point in enumerate(values):
            if point > mid:
                return i-1
            
        return len(values) - 1
    
    # Creates the graph in real space for visualisation
    def debugBuild(self):
        print("Creating build")
        collec = bpy.data.collections.new('debugGraph')
        bpy.context.scene.collection.children.link(collec) 
        
        mesh = bpy.data.meshes.new('wireGraph')
        
        points = [n.point for n in self.nodes]
                
        mesh.from_pydata(points, self.edges, [])
        mesh.update()
        
        graphObj = bpy.data.objects.new('graph', mesh)
        collec.objects.link(graphObj)  
        print("Finished build") 
        
    # Hacky way of deleting the center nodes of the outer grid
    #   They're in the way, since the inner grid fills that space
    def hackyDeleteCenterNodes(self):
        # Delete all middle nodes
        for x in range(5, 20):
            for y in range(5, 20):
                nodeNum = self.colNum * x + y
                self.nodes[nodeNum] = None
                
        # Remove all None values in nodes list
        oldNodes = self.nodes
        self.nodes = []
        nodeIndices = {}
        counter = 0
        for node in oldNodes:
            if node is None:
                continue
            self.nodes.append(node)
            nodeIndices[node] = counter
            counter += 1
                
        # Recreate correct list of edges
        self.edges = {}
        for i,node in enumerate(self.nodes):
            for other in node.edges:
                self.edges[(i, nodeIndices[other])] = True

class Node:
    NODE_NUM = 0
    
    # point is coordinate (center of small square)
    def __init__(self, point):
        self.point = point
        self.edges = [] # List of nodes connected to it
        self.id = Node.getNodeNum()
        
    @staticmethod
    def getNodeNum():
        Node.NODE_NUM += 1
        return Node.NODE_NUM
        
    # Takes a node object and creates an edge
    def addEdge(self, otherNode):
        self.edges.append(otherNode)
        
# Traverses a graph to find and analyze connected components
class GraphTraverser:
    def __init__(self, edgeLength):
        self.edgeLength = edgeLength
    
    # Returns a list of tuples. Each tuple contains all the
    #   nodes in one connected component of the graph
    def getConnectedComponents(self, graph):
        components = [] # List of connected components
        visited = {} # Contains every node that has been visited
        for i,baseNode in enumerate(graph.nodes):
            # Ignore if node already visited
            if visited.get(baseNode.id, False):
                continue
            
            # Visit all nodes in component
            component = [baseNode]
            toVisit = [baseNode]
            while toVisit:
                curNode = toVisit.pop()
                visited[curNode.id] = True
                
                for otherNode in curNode.edges:
                    if visited.get(otherNode.id, False):
                        continue
                        
                    toVisit.append(otherNode)
                    visited[otherNode.id] = True
                    
                component.append(curNode)
            components.append(component)
                
        return components  
    
    # From a list of connected components (lists of graph elements),
    #   Returns all the spaces where buildings can be, in format:
    #   (centerCoords, width)
    def getAllBuildingSpaces(self, components):
        buildingSpaces = []
        for component in components:
            buildingSpaces += self.getComponentBuildingSpaces(component)
            
        return buildingSpaces
    
    # For a graph element (a list of conencted nodes),
    #   Returns all the spaces where buildings can be.
    def getComponentBuildingSpaces(self, component):
        # Store all individual nodes (smallest building spaces)
        spaces = []
        '''
        for node in component:
            # Create a small building space
            center = (node.point[0], node.point[1], node.point[2])
            #spaces.append((center, self.edgeLength, 5, 15))
        '''
            
        dontFindSquares = False
        for i in range(len(component)):
            baseNode = component[i] # Bottom left corner of our potential square
            
            '''
            # Stop looking if there's no hope
            if i > 3:
                dontFindSquares = True
            '''
                
            # Fill small square
            center = (baseNode.point[0], baseNode.point[1], baseNode.point[2])
            spaces.append((center, self.edgeLength, 10, 50))
        
            # Find big squares
            squareSize = float(self.getSquareSizeBasedAtNode(component, baseNode))
            if squareSize == 1:
                continue
            
            width = self.edgeLength * squareSize
            center = (baseNode.point[0] + self.edgeLength*(squareSize-1)/2, baseNode.point[1] + self.edgeLength*(squareSize-1)/2, baseNode.point[2])
            #center = (node.point[0], node.point[1], node.point[2])
            
            spaces = []
            spaces.append((center, width, 5, 15))
            break
        
        '''
        # Try first three nodes, then give up (since nodes are in order)
        for i in range(min(3, len(component)):
            baseNode = component[i] # Bottom left corner of our potential square
            
            # Find the biggest square based at this node
            squareSize = GraphTraverser.getSquareSizeBasedAtNode(baseNode)
            if squareSize == 0:
                continue
            
            # Extract top right corner of square
            # Get center and width of square
        '''
        
        return spaces
    
    # Takes a connected graph and finds the biggest square grid
    #   possible that starts at the baseNode (bottom left corner)
    def getSquareSizeBasedAtNode(self, component, baseNode):
        squares = ['ur--ru', 'uurr---rr---rr', 'uuurrr----rrr----rrr----rrr', 'uuuurrrr-----rrrr-----rrrr-----rrrr-----rrrr']
        for size,map in enumerate(squares, 1):
            # Check if component has this square
            stack = []
            curNode = baseNode
            for symbol in map:
                if symbol == 'u':
                    # Check node has edges
                    if not curNode.edges:
                        return size
                    
                    # Check one of these edges is the up node
                    upNode = None
                    for other in curNode.edges:
                        if round(other.point[1], 5) > round(curNode.point[1], 5):
                            upNode = other
                            break
                    
                    if upNode is None:
                        return size
                    
                    # Move to up node
                    stack.append(curNode)
                    curNode = upNode
                    
                elif symbol == 'r':
                    # Check right node exists
                    if not curNode.edges or round(curNode.edges[0].point[1], 5) > round(curNode.point[1], 5):
                        return size
                    
                    # Move to right node
                    stack.append(curNode)
                    curNode = curNode.edges[0]
                    
                else:
                    # Backtrack
                    curNode = stack.pop()
        return 5
            
# Cretaes roads for a city
#   Returns all the spaces where buildings can be.
def createCityRoads():
    turtles = bpy.data.collections.new('roads')
    bpy.context.scene.collection.children.link(turtles)
    
    # Set grid parameters
    outerGridEdge = 10
    roadWidth = 1.0
    outer_grid = ('F-F-F-F-F', {'F': 'FF-F-F-F-FF-F-F-F-FF-F-F-F-FF'}, 2)
    #inner_grid = ('F-F-F-F-F', {'F': [('FF-FF-F-FF-FF-FF-F-FF-FF-FF-F-FF-FF', 0.3), ('FF-F-F-F-FF-F-F-F-FF-F-F-F-FF', 0.3)], }, 2)
    #inner_grid = ('F-F-F-F-F', {'F': [('FF-FF-F-FF-FF-FF-F-FF-FF-FF-F-FF-FF', 0.3), ('FF-FF-F-FF-FF-F-F-F-FF-FF-F-FF-FF', 0.3), ('FF-F-F-F-FF-F-F-F-FF-F-F-F-FF', 0.3)], }, 2)
    inner_grid = ('F-F-F-F-F', {'F': [('FF-FF-F-FF-FF-FF-F-FF-FF-FF-F-FF-FF', 0.3), ('FF-FF-F-FF-FF-F-F-F-FF-FF-F-FF-FF', 0.3), ('FF-F-F-F-FF-FFF-F-FFF-FF-F-F-F-FF', 0.3)], }, 2)
    
    # Scale and align grids properly
    innerGridEdge = outerGridEdge * (15*outerGridEdge-2*roadWidth)/(25*outerGridEdge);
    startCorner = 5*outerGridEdge+roadWidth
    innerGridStartPoint = (startCorner, startCorner, 0) 
    
    # Create outer grid
    system = TurtleLSystem(*outer_grid)
    builder = TurtlePathBuilder(turtles, outerGridEdge, 90, roadWidth/2)
    outerGridPoints = builder.buildRoadFromPath(system.computeTurtlePath())
    outerGraph = createGraphFromTurtlePoints(outerGridPoints, outerGridEdge, hackyCenterDelete=True)
    outerGridTraverser = GraphTraverser(outerGridEdge)
    outerComponents = outerGridTraverser.getConnectedComponents(outerGraph)
    buildingSpaces = outerGridTraverser.getAllBuildingSpaces(outerComponents)
    #print(*outerComponents, sep="\n")
    
    # Create inner grid
    system = StochasticTurtleLSystem(*inner_grid)
    builder = TurtlePathBuilder(turtles, innerGridEdge, 90, roadWidth/2)
    builder.setStartPoint(innerGridStartPoint)
    innerGridPoints = builder.buildRoadFromPath(system.computeTurtlePath())
    innerGraph = createGraphFromTurtlePoints(innerGridPoints, innerGridEdge)
    innerGridTraverser = GraphTraverser(innerGridEdge)
    innerComponents = innerGridTraverser.getConnectedComponents(innerGraph)
    buildingSpaces += innerGridTraverser.getAllBuildingSpaces(innerComponents)
    
    cityWidth = 25*outerGridEdge
    return buildingSpaces, cityWidth
    #print("\n\n\n")
    #print(*innerComponents, sep="\n")

# Create some example roads
# Note that these are from The Algorithmic Beauty of Plants ('ABoP', Lindenmayer & Prusinkiewicz)
def createDebugExampleRoads(turtles):
    simple_example = ('FFF-FF-F-F+F+FF-F-FFF', {}, 0, 1, 90, 0.2)
    quadratic_koch_island = ('F-F-F-F', {'F': 'F+FF-FF-F-F+F+FF-F-F+F+FF+FF-F'}, 3, 1, 90, 0.2) # ABoP, pg. 9
    quadratic_koch_modified = ('F-F-F-F', {'F': 'F+FF-FF-F-F+F+F+FF-F-F+FF+FFFF-F-F+F+FF+FF-F'}, 3, 1, 90, 0.2) # ABoP, pg. 9
    
    # SIMPLE SYSTEMS
    koch_grid = ('F-F-F-F', {'F': 'FF-F+F-F-FF'}, 5, 3, 90, 0.2) # ABoP, pg. 10
    koch_blocky = ('F-F-F-F', {'F': 'F-FF---F-F'}, 4, 3, 90, 0.2)
    koch_fractal = ('F-F-F-F', {'F': 'FF-F--F-F'}, 4, 1, 90, 0.2)
    koch_square = ('F-F-F-F', {'F': 'FF-F-F-F-FF'}, 3, 1, 90, 0.2)
    koch_rose = ('F-F-F-F', {'F': 'FF-F-F-F-F-F+F'}, 3, 1, 90, 0.2)
    
    # EXTENDED SYSTEMS
    hexagonal_gosper_curve = ('L', {'L': 'L+R++R-L--LL-R+', 'R': '-L+RR++R+L--L-R'}, 4, 3, 60, 0.2) # ABoP, pg. 12
    quadratic_gosper_curve = ('-R', {'L': 'LL-R-R+L+L-R-RL+R+LLR-L+R+LL+R-LR-R-L+L+RR-', 'R': '+LL-R-R+L+LR+L-RR-L-R+LRR-L-RL+L+R-R-L+L+RR'}, 2, 3, 90, 0.4) # ABoP, pg. 12
    
    # CUSTOM SYSTEMS
    city_map = ('F-F-F-F-F', {'F': 'FF-F-F-F-FF-F-F-F-FF-F-F-F-FF'}, 2, 3, 90, 0.2)
    cool_city_map = ('F-F-F-F-F', {'F': 'FF-FF-F-FF-FF-FF-F-FF-FF-FF-F-FF-FF'}, 2, 3, 90, 0.2)
    
    createLSystemRoad(turtles, *koch_square)
    
# Creates cubes representing buildings at each location given
#   buildingSpaces has format: [(center0, width0), (center1, width1), etc]
def createDebugBuildingCubes(buildingSpaces):   
    counter = 0 
    for i in range(0, len(buildingSpaces)):
        space = buildingSpaces[i]
        center, width = space[0], space[1]
        corner = (center[0], center[1], center[2] + width*0.7/2)
        
        if width < 10:
            counter = (counter + 1) % 10
            if counter < 10:
                continue
        
        bpy.ops.mesh.primitive_cube_add(size=width*0.7, location=corner)
        

def createLSystemRoad(collection, axiom, rules, iterations, step, angle, width):
    system = TurtleLSystem(axiom, rules, iterations)
    builder = TurtlePathBuilder(collection, step, angle, width)
    builder.buildRoadFromPath(system.computeTurtlePath())

'''   
if __name__ == '__main__':
    #turtles = bpy.data.collections.new('roads')
    #bpy.context.scene.collection.children.link(turtles)
    
    buildingSpaces = createCityRoads()
    #print(buildingSpaces)
    createDebugBuildingCubes(buildingSpaces)
    #createDebugExampleRoads(turtles)
'''
    

######################## DEBUG



# Tests a function with given inputs
# inputs has tuple for each set of parameters
# expected has all solutions in order
def unit_test(function, inputs, expected):
    for num,input in enumerate(inputs):
        result = function(*input)
        solution = expected[num]
        if result != solution:
            print('FAILED: Test {0} with parameters {1}.\n\t Expected {2} but got {3}\n'.format(num, inputs, solution, result))
        else:
            print('Test {0}: OK'.format(num))
            
# Test from Algorithmic Beauty of Plants, pg. 4
#unit_test(lambda a,b,c: TurtleLSystem(a,b,c).computeTurtlePath(), [('b', {'a': 'ab', 'b': 'a'}, 5)], ['abaababa'])

# Ideas:
#   Randomly remove chunks of a good L-System
#   Layer/combine two L-Systems, perhaps with different angles
#   Change the angle halfway/have more symbols for different steps and angle changes

# TRY A TURTLE PATH AND SEE IF IT IS CORRECT!
    # TURN IT INTO A SIMPLE WIRE MESH TO VISUALISE!
   

# Find positions for buildings
# Input: turtlePath = 'F-FF-FF-F-F-FF...'
#   turtlePoints = [(0,0,0), (1,0,0), (2,1,0), ...]

# Figure out limits of grid (just take biggest/smallest x/y)
# Split grid into small squares of length F
# Create graph
    # For each small square, create node representing middle of small square
    # For each small square, create edges
        # Edge if small square is connected to its neighbour
        # Check if a vertex exists that separates the two neighbours
# Find maximal cliques in graph
#   Fill the biggest ones with big buildings
#   Randomly fill smallest ones (i.e. small squares)


# density 
# min height
# max height
# potentially some regions could have different height ranges?
# level of randomness of roads

# passes randomized parameters to sweep_skyscraper function
# a little bit hacky, but works
class DummyProps:
    def __init__(self):
        pass

# a wrapper for creating road network and populating plots with buildings
class WM_OT_GenWholeCity(Operator):
    bl_idname = 'wm.gen_wholecity'
    bl_label = 'Create an entire futuristic city'
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        # first create road network and get a list of empty plots for buildings
        buildingSpaces, cityWidth = createCityRoads()
        actualBuildingPlots = []

        # for each plot, randomize the sweep parameters and create building stochastically
        for i in range(0, len(buildingSpaces), 3):
            shift, width, minh, maxh = buildingSpaces[i]
            props = DummyProps()
            props.x_width = 0.66 * width
            props.y_width = 0.66 * width
            props.curve = random.choice(['SQUARE', 'ELLIPSE', 'HYPOTROCHOID', 'EPITROCHOID'])
            props.revolution = random.choice(range(3, 13))
            props.ellipse_ratio = 2 * random.random() + 1.0
            props.height = (maxh - minh) * random.random() + minh
            props.floors = int(props.height)
            props.model_floors = random.choice([True, False, False])
            props.max_rotation = (random.random() - 0.5) * 6.28
            props.rotate_func = random.choice(['LINEAR', 'EXPONENTIAL', 'LOGARITHMIC', 'SIGMOID'])
            props.top_scale_x = 0.9 * random.random() + 0.1
            props.top_scale_y = 0.9 * random.random() + 0.1
            props.scale_func_x = random.choice(['LINEAR', 'EXPONENTIAL', 'LOGARITHMIC', 'SIGMOID'])
            props.scale_func_y = random.choice(['LINEAR', 'EXPONENTIAL', 'LOGARITHMIC', 'SIGMOID'])
            props.support = False if props.curve == 'SQUARE' else random.choice([True, False])
            props.n_columns = random.choice(range(1, 21))
            props.column_size = 0.067 * width
            props.height_ratio = 0.9 * random.random() + 0.1
            props.column_shape = random.choice(['SQUARE', 'CIRCLE'])
            props.bidirectional = random.choice([True, False])
            props.capped = True
            sweep_skyscraper(props, shift=Vector(shift))

            # Save building dimensions for boid collision purposes
            buildingPlot = (shift, (props.x_width, props.y_width), props.height)
            actualBuildingPlots.append(buildingPlot)

        # Generate boids flying around buildings
        generate_boids(actualBuildingPlots, cityWidth)

        return {'FINISHED'}