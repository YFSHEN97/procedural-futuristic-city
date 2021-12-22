bl_info = {
    "name":"Futuristic Building Generator",
    "author":"Yifei Shen",
    "version":(1,0),
    "blender":(3,0,0),
    "location":"3D View > Tools",
    "description":"Creating futuristic buildings procedurally",
    "warning":"",
    "wiki_url":"",
    "tracker_url":"",
    "category":"Add Mesh"
    }

import bpy
from bpy.types import Operator

from mathutils import Vector, Matrix
from math import *


def ellipse(t, k):
    return Vector((k * cos(t), sin(t), 0))

def hypotrochoid(t, k):
    x = (k-1) * cos(t) + cos((k-1) * t)
    y = (k-1) * sin(t) - sin((k-1) * t)
    return Vector((x, y, 0))

def epitrochoid(t, k):
    x = (k-1) * cos(t) - cos((k-1) * t)
    y = (k-1) * sin(t) - sin((k-1) * t)
    return Vector((x, y, 0))

# a dictionary storing functions to compute discrete points on parametric curves
ParametricCurveLibrary = {
    'CIRCLE': ellipse,
    'HYPOTROCHOID': hypotrochoid,
    'EPITROCHOID': epitrochoid
}

# sweep out the skyscraper's geometry and add mesh to scene
# props contain all parameters specified by the user
# shift is optional, specifying a location to put the newly created building
def sweep_skyscraper(props, shift=Vector((0,0,0))):
    # A few basic things are necessary for the sweeping operation
    # xsection: a list of points in XY plane (CCW) that specify the shape of the cross section curve
    # frotate: a function that takes in parameter t and returns a rotation matrix around Z
    # fscale: a function that takes in parameter t and returns a scaling matrix (only scales XY)
    # ftranslate: a function that takes in parameter t and returns a translation vector (only moves XY)
    # the parameter t has value range [0, 1]
    # and frotation, fscale, ftranslate should all be defined w.r.t. this scale

    # first we compute the cross section curve
    xsection = list()
    if props.curve in ['HYPOTROCHOID', 'EPITROCHOID']:
        for i in range(60):
            k = props.revolution
            t = i / 60 * 2 * pi
            xsection.append(ParametricCurveLibrary[props.curve](t, k))
    elif props.curve == 'ELLIPSE':
        for i in range(60):
            t = i / 60 * 2 * pi
            k = props.ellipse_ratio
            xsection.append(ellipse(t, k))
    elif props.curve == 'SQUARE':
        xsection.append(Vector((0.5, 0.5, 0)))
        xsection.append(Vector((-0.5, 0.5, 0)))
        xsection.append(Vector((-0.5, -0.5, 0)))
        xsection.append(Vector((0.5, -0.5, 0)))
    # scale the curve so that they roughly occupy a bounding region in [-0.5, 0.5]
    max_x = float("-inf")
    max_y = float("-inf")
    min_x = float("inf")
    min_y = float("inf")
    for v in xsection:
        max_x = max(v[0], max_x)
        max_y = max(v[1], max_y)
        min_x = min(v[0], min_x)
        min_y = min(v[1], min_y)
    ssx = 1 / (max_x - min_x)
    ssy = 1 / (max_y - min_y)
    ss = Matrix([
        [min(ssx, ssy), 0, 0],
        [0, min(ssx, ssy), 0],
        [0, 0, 1]
    ])
    xsection = list(map(lambda v: ss @ v, xsection))

    # rotation at each time point t is defined as Rota(t, m), a function of t and m (max_rotation)
    # t is in range [0, 1] and max_rotation is given by user input
    # returns a 3D rotation matrix that is later used to transform vertices in a cross section
    def frotate(t):
        # compute Rota(t, m) at time t with max_rotation
        if props.rotate_func == 'LINEAR':
            theta = t
        elif props.rotate_func == 'EXPONENTIAL':
            theta = 1 - (1 - exp(t - 1)) / (1 - exp(-1))
        elif props.rotate_func == 'LOGARITHMIC':
            theta = (1 - exp(-t)) / (1 - exp(-1))
        elif props.rotate_func == 'SIGMOID':
            theta = 1 / (1 + exp((0.5 - t) * 10))
        theta *= props.max_rotation
        # compute rotation matrix (only rotates around Z axis)
        return Matrix([
            [cos(theta), -sin(theta), 0],
            [sin(theta), cos(theta), 0],
            [0, 0, 1]
        ])

    # scaling at each time point t is defined as Scal(t, s), a function of t and s (top_scale)
    # t is in range [0, 1] and top_scale is given by user input
    # returns a 3D scaling matrix that is later used to transform vertices in a cross section
    def fscale(t):
        # compute Scal(t, s) at time t with top_scale
        if props.scale_func_x == 'LINEAR':
            sx = 1 - t * (1 - props.top_scale_x)
        elif props.scale_func_x == 'EXPONENTIAL':
            sx = 1 / ((1/props.top_scale_x) ** t)
        elif props.scale_func_x == 'LOGARITHMIC':
            sx = props.top_scale_x + (1 - 1 / ((1/props.top_scale_x) ** (1 - t)))
        elif props.scale_func_x == 'SIGMOID':
            sx = 1 - 1 / (1 + exp((0.5 - t) * 10)) * (1 - props.top_scale_x)
        if props.scale_func_y == 'LINEAR':
            sy = 1 - t * (1 - props.top_scale_y)
        elif props.scale_func_y == 'EXPONENTIAL':
            sy = 1 / ((1/props.top_scale_y) ** t)
        elif props.scale_func_y == 'LOGARITHMIC':
            sy = props.top_scale_y + (1 - 1 / ((1/props.top_scale_y) ** (1 - t)))
        elif props.scale_func_y == 'SIGMOID':
            sy = 1 - 1 / (1 + exp((0.5 - t) * 10)) * (1 - props.top_scale_y)
        # compute the scaling matrix (only scales in the XY-plane)
        return Matrix([
            [sx, 0, 0],
            [0, sy, 0],
            [0, 0, 1]
        ])

    # translation at each time point t is defined as Tran(t), a function of t
    # t is in range [0, 1]
    # returns a 3D vector that is later used to transform vertices in a cross section
    def ftranslate(t):
        # compute Tran(t) and return a 3D vector
        return Vector((0, 0, 0))

    # this function uses the functions defined above to produce geometry
    # returns vertices, faces, and uvs for the mesh
    def sweepExterior(xsection, xwidth=1, ywidth=1, height=1, tsamples=20):
        vertices = []
        faces = []
        # uvs is a list of entries, each entry is a 2-tuple (U, V)
        # len(uvs) is equal to exactly 4 * len(faces)
        # for each face, uvs specifies 4 entries, one entry for each vertex of the quad face
        uvs = []
        # Sxy is the overall scaling matrix, used to scale the entire building
        Sxy = Matrix([
            [xwidth, 0, 0],
            [0, ywidth, 0],
            [0, 0, 1]
        ])
        # sample TSAMPLE values uniformly in the range [0, 1]
        for n in range(tsamples+1):
            t = n / tsamples
            Rota = frotate(t)
            Scal = fscale(t)
            Tran = ftranslate(t)
            V_offset = Vector((0, 0, t * height)) # vertical offset for this floor
            # produce a new row of vertices
            row = []
            for v in xsection:
                # apply scaling first, then rotation, then translation
                row.append(Sxy @ (V_offset + Tran + Rota @ Scal @ v))
            # store new vertices and produce new faces
            vertices.extend(row)
            if len(vertices) == len(xsection):
                continue
            for i in range(len(vertices) - 2*len(xsection), len(vertices) - len(xsection) - 1):
                faces.append((i, i+1, i+1+len(xsection), i+len(xsection)))
            faces.append((
                len(vertices)-len(xsection)-1,
                len(vertices)-2*len(xsection),
                len(vertices)-len(xsection),
                len(vertices)-1
            ))
            for i in range(len(xsection)):
                uvs.append((i / len(xsection), (n-1) / tsamples))
                uvs.append(((i+1) / len(xsection), (n-1) / tsamples))
                uvs.append(((i+1) / len(xsection), t))
                uvs.append((i / len(xsection), t))
        # add ngon caps at the top and the bottom
        faces.append(tuple([x for x in range(len(xsection))]))
        uvs.extend([(0, 0) for _ in range(len(xsection))])
        faces.append(tuple([x for x in range(len(vertices)-len(xsection), len(vertices))]))
        uvs.extend([(1, 1) for _ in range(len(xsection))])
        return vertices, faces, uvs

    # this function creates a separate mesh object to represent the floors
    def sweepFloors(xsection, vertices, height=1, tsamples=20):
        floor_vertices = []
        floor_faces = []
        floor_scale = Matrix([[1.05, 0, 0], [0, 1.05, 0], [0, 0, 1]])
        floor_offset = Vector((0, 0, height / tsamples / 20))
        for f in range(0, len(vertices), len(xsection)):
            floor = vertices[f:f+len(xsection)]
            floor = list(map(lambda v: floor_scale @ v, floor))
            for v in floor:
                floor_vertices.append(v - floor_offset)
            for v in floor:
                floor_vertices.append(v + floor_offset)
            for i in range(len(floor_vertices) - 2*len(xsection), len(floor_vertices) - len(xsection) - 1):
                floor_faces.append((i, i+1, i+1+len(xsection), i+len(xsection)))
            floor_faces.append((
                len(floor_vertices)-len(xsection)-1,
                len(floor_vertices)-2*len(xsection),
                len(floor_vertices)-len(xsection),
                len(floor_vertices)-1
            ))
            floor_faces.append(tuple([x for x in range(len(floor_vertices) - 2*len(xsection), len(floor_vertices) - len(xsection))]))
            floor_faces.append(tuple([x for x in range(len(floor_vertices) - len(xsection), len(floor_vertices))]))
        return floor_vertices, floor_faces

    # this function sweeps the exterior support structures defined by the user
    # parameters:
    # xsection: cross section curve for each column in the support structure
    # n_columns: number of columns
    # bidirectional: Boolean indicating if all columns sweep in same direction
    # xwidth, ywidth: dimensions of the imaginary circle on which we plant support columns
    # height_ratio: height of support structure divided by height of exterior
    # capped: Boolean indicating if we put a ring on top to cap all the columns
    def sweepSupport(xsection, n_columns, bidirectional, xwidth, ywidth, height_ratio, capped):
        vertices = []
        faces = []
        # we produce the sweep column by column
        for col in range(n_columns):
            column_vertices = []
            column_faces = []
            # compute the location of the column's base
            theta = col / n_columns * 2 * pi
            base_x = 0.5 * xwidth * cos(theta)
            base_y = 0.5 * ywidth * sin(theta)
            # rotate the cross section curve and move it to the correct location
            R = Matrix([[cos(theta), -sin(theta), 0],[sin(theta), cos(theta), 0],[0, 0, 1]])
            T = Vector((base_x, base_y, 0))
            base = list(map(lambda v: T + R @ v, xsection))
            # build up the sweep similar to how we do it for exterior geometry
            tsamples = int(height_ratio * props.floors)
            for n in range(tsamples+1):
                t = n / props.floors
                Rota = frotate(t)
                if bidirectional and col % 2 == 1:
                    Rota.invert()
                # scaling matrix need to be applied after rotation in this case
                # to avoid bidirectional supports causing weird artifacts
                # need to scale along the "rotated" cardinal axes
                # first compute a scaling matrix along the rotated x-direction
                Scal = fscale(t)
                kx = Scal[0][0]
                ky = Scal[1][1]
                R_basic = frotate(t)
                x_transformed = R_basic @ Vector((1, 0, 0))
                y_transformed = R_basic @ Vector((0, 1, 0))
                nx1 = x_transformed[0]
                ny1 = x_transformed[1]
                Sx = Matrix([
                    [1+(kx-1)*nx1*nx1, (kx-1)*nx1*ny1, 0],
                    [(kx-1)*nx1*ny1, 1+(kx-1)*ny1*ny1, 0],
                    [0, 0, 1],
                ])
                nx2 = y_transformed[0]
                ny2 = y_transformed[1]
                Sy = Matrix([
                    [1+(ky-1)*nx2*nx2, (ky-1)*nx2*ny2, 0],
                    [(ky-1)*nx2*ny2, 1+(ky-1)*ny2*ny2, 0],
                    [0, 0, 1],
                ])
                Scal = Sy @ Sx
                # translation and vertical offsets are the same as before
                Tran = ftranslate(t)
                V_offset = Vector((0, 0, t * props.height)) # vertical offset for this floor
                # produce a new row of vertices
                row = []
                for v in base:
                    # apply scaling first, then rotation, then translation
                    row.append(V_offset + Tran + Scal @ Rota @ v)
                # store new vertices and produce new faces
                column_vertices.extend(row)
                if len(column_vertices) == len(base):
                    continue
                for i in range(len(column_vertices) - 2*len(base), len(column_vertices) - len(base) - 1):
                    column_faces.append((
                        i+len(vertices),
                        i+1+len(vertices),
                        i+1+len(base)+len(vertices),
                        i+len(base)+len(vertices)
                    ))
                column_faces.append((
                    len(column_vertices)-len(base)-1+len(vertices),
                    len(column_vertices)-2*len(base)+len(vertices),
                    len(column_vertices)-len(base)+len(vertices),
                    len(column_vertices)-1+len(vertices)
                ))
            # add ngon caps at the top and bottom of this column
            column_faces.append(tuple([x+len(vertices) for x in range(len(base))]))
            column_faces.append(tuple([x+len(vertices) for x in range(len(column_vertices)-len(base), len(column_vertices))]))
            # add this column's vertices and faces to the total list
            vertices.extend(column_vertices)
            faces.extend(column_faces)
        # sweep the cap
        if capped:
            cap_vertices = []
            cap_faces = []
            # compute a "base loop", situated at angle = 0 (on the positive x-axis)
            # we will sweep this base loop to create our cap
            R = Matrix([[1, 0, 0],[0, cos(-pi/2), -sin(-pi/2)],[0, sin(-pi/2), cos(-pi/2)]])
            T = Vector((0.5 * xwidth, 0, 0))
            base = list(map(lambda v: T + R @ v, xsection))
            # compute all the required transforms for the cap
            Rota = frotate(int(height_ratio * props.floors) / props.floors)
            Scal = fscale(int(height_ratio * props.floors) / props.floors)
            Tran = ftranslate(int(height_ratio * props.floors) / props.floors)
            V_offset = Vector((0, 0, int(height_ratio * props.floors) / props.floors * props.height))
            # sweep out a torus shape with 60 subdivisions
            for n in range(60):
                a = n / 60 * 2 * pi
                RR = Matrix([[cos(a), -sin(a), 0],[sin(a), cos(a), 0],[0, 0, 1]])
                loop = list(map(lambda v: V_offset + Tran + Rota @ Scal @ RR @ v, base))
                cap_vertices.extend(loop)
                if len(cap_vertices) == len(base):
                    continue
                for i in range(len(cap_vertices) - 2*len(base), len(cap_vertices) - len(base) - 1):
                    cap_faces.append((
                        i+len(vertices),
                        i+1+len(vertices),
                        i+1+len(base)+len(vertices),
                        i+len(base)+len(vertices)
                    ))
                cap_faces.append((
                    len(cap_vertices)-len(base)-1+len(vertices),
                    len(cap_vertices)-2*len(base)+len(vertices),
                    len(cap_vertices)-len(base)+len(vertices),
                    len(cap_vertices)-1+len(vertices)
                ))
            # connect loop 59 to loop 0
            for i in range(len(base)-1):
                cap_faces.append((
                    59*len(base)+i+len(vertices),
                    59*len(base)+i+1+len(vertices),
                    i+1+len(vertices),
                    i+len(vertices)
                ))
            cap_faces.append((
                60*len(base)-1+len(vertices),
                59*len(base)+len(vertices),
                0+len(vertices),
                len(base)-1+len(vertices)
            ))
            vertices.extend(cap_vertices)
            faces.extend(cap_faces)
        return vertices, faces


    # define exterior sweep geometry
    vertices, faces, uvs = sweepExterior(xsection, 
        xwidth=props.x_width, ywidth=props.y_width, height=props.height, tsamples=props.floors)

    # render the exterior
    building_exterior = bpy.data.meshes.new('building_exterior')
    building_exterior.from_pydata(vertices, [], faces)
    building_exterior.update()
    building_exterior.uv_layers.new(name='building_exterior_uv')
    building_exterior_uvs = building_exterior.uv_layers.active.data
    assert(len(building_exterior_uvs) == len(uvs))
    for i in range(len(building_exterior_uvs)):
        building_exterior_uvs[i].uv = Vector(uvs[i])
    building_exterior.update()
    building_exterior.validate()
    building_exterior_obj = bpy.data.objects.new('building_exterior_obj', building_exterior)
    building_exterior_obj.location += shift
    bpy.data.collections['Collection'].objects.link(building_exterior_obj)

    # render floors (optional)
    if props.model_floors:
        floor_vertices, floor_faces = sweepFloors(xsection, vertices, height=props.height, tsamples=props.floors)
        building_floors = bpy.data.meshes.new('building_floors')
        building_floors.from_pydata(floor_vertices, [], floor_faces)
        building_floors.update()
        building_floors_obj = bpy.data.objects.new('building_floors_obj', building_floors)
        building_floors_obj.location += shift
        bpy.data.collections['Collection'].objects.link(building_floors_obj)

    # render exterior support (optional)
    if props.support:
        column_xsection = list()
        if props.column_shape == 'SQUARE':
            column_xsection = [
                Vector((0.5 * props.column_size, 0.5 * props.column_size, 0)),
                Vector((-0.5 * props.column_size, 0.5 * props.column_size, 0)),
                Vector((-0.5 * props.column_size, -0.5 * props.column_size, 0)),
                Vector((0.5 * props.column_size, -0.5 * props.column_size, 0))
            ]
        if props.column_shape == 'CIRCLE':
            for i in range(20):
                a = i / 20 * 2 * pi
                column_xsection.append(Vector((0.5 * cos(a) * props.column_size, 0.5 * sin(a) * props.column_size, 0)))
        support_vertices, support_faces = sweepSupport(column_xsection, props.n_columns, props.bidirectional,
            props.x_width+props.column_size, props.y_width+props.column_size, props.height_ratio, props.capped)
        building_support = bpy.data.meshes.new('building_support')
        building_support.from_pydata(support_vertices, [], support_faces)
        building_support.update()
        building_support_obj = bpy.data.objects.new('building_support_obj', building_support)
        building_support_obj.location += shift
        bpy.data.collections['Collection'].objects.link(building_support_obj)

# a wrapper for the sweeping operator
class WM_OT_GenFBSweep(Operator):
    bl_idname = 'wm.gen_fbsweep'
    bl_label = 'Sweep out Futuristic Buildings'
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        scene = bpy.context.scene
        props = scene.fbsweep_properties
        sweep_skyscraper(props)
        return {'FINISHED'}
