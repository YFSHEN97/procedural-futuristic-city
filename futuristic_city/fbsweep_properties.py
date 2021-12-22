import bpy

from bpy.props import (
    BoolProperty,
    FloatProperty,
    IntProperty,
    EnumProperty,
    PointerProperty
)

from bpy.types import PropertyGroup

# Properties class to hold required parameters
class FBSweepProperties(PropertyGroup):

    x_width: FloatProperty(
        name='X Width',
        description='Width of building in X-direction',
        default=3,
        min=1,
        max=100,
        unit='LENGTH'
    )

    y_width: FloatProperty(
        name='Y Width',
        description='Width of building in Y-direction',
        default=3,
        min=1,
        max=100,
        unit='LENGTH'
    )

    curve: EnumProperty(
        name='Curve',
        description='Shape of cross section curve to be swept',
        items=[
            ('SQUARE', 'Square', 'Square-shaped cross section'),
            ('ELLIPSE', 'Ellipse', 'Elliptical cross section (need to specify semi-major / semi-minor ratio'),
            ('HYPOTROCHOID', 'Hypotrochoid', 'Hypotrochoidal curves (need to specify number of revolutions k)'),
            ('EPITROCHOID', 'Epitrochoid', 'Epitrochoidal curve (need to specify number of revolutions k)')
        ],
        default='SQUARE'
    )

    revolution: IntProperty(
        name='Revolutions',
        description='Number of revolutions for the trochoidal family of curves',
        default = 8,
        min=3,
        max=12
    )

    ellipse_ratio: FloatProperty(
        name='Semi-Major/Minor Ratio',
        description="Ratio of semi-major axis vs. semi-minor axis for ellipse",
        default=2,
        min=1,
        max=10
    )

    height: FloatProperty(
        name='Height',
        description='Height of building in Z-direction',
        default=20,
        min=1,
        max=500,
        unit='LENGTH'
    )

    floors: IntProperty(
        name='Floors',
        description='Number of floors of the building',
        default=20,
        min=1,
        max=1000
    )

    model_floors: BoolProperty(
        name='Model Floors',
        description='Whether to model floors explicitly',
        default=True
    )

    max_rotation: FloatProperty(
        name='Max Rotation',
        description='Rotation (radians) of the cross section at the top',
        default=1.57,
        min=-31.4,
        max=31.4
    )

    rotate_func: EnumProperty(
        name='Rotation Function',
        description='Function (w.r.t. height) for calculating rotation angle',
        items=[
            ('LINEAR', 'Linear', 'Linear interpolation'),
            ('EXPONENTIAL', 'Exponential', 'Exponential interpolation'),
            ('LOGARITHMIC', 'Logarithmic', 'Logarithmic interpolation'),
            ('SIGMOID', 'Sigmoid', 'Sigmoid interpolation')
        ],
        default='LINEAR'
    )

    top_scale_x: FloatProperty(
        name='Scale X',
        description='Scale of the top floor relative to the bottom floor',
        default=1,
        min=0.01,
        max=3
    )

    scale_func_x: EnumProperty(
        name='Taper X',
        description='Function for calculating intermediate scaling factors',
        items=[
            ('LINEAR', 'Linear', 'Linear interpolation'),
            ('EXPONENTIAL', 'Exponential', 'Exponential interpolation'),
            ('LOGARITHMIC', 'Logarithmic', 'Logarithmic interpolation'),
            ('SIGMOID', 'Sigmoid', 'Sigmoid interpolation')
        ],
        default='LINEAR'
    )

    top_scale_y: FloatProperty(
        name='Scale Y',
        description='Scale of the top floor relative to the bottom floor',
        default=1,
        min=0.01,
        max=3
    )

    scale_func_y: EnumProperty(
        name='Taper Y',
        description='Function for calculating intermediate scaling factors',
        items=[
            ('LINEAR', 'Linear', 'Linear interpolation'),
            ('EXPONENTIAL', 'Exponential', 'Exponential interpolation'),
            ('LOGARITHMIC', 'Logarithmic', 'Logarithmic interpolation'),
            ('SIGMOID', 'Sigmoid', 'Sigmoid interpolation')
        ],
        default='LINEAR'
    )

    support: BoolProperty(
        name='Exterior Support',
        description='Whether to generate exterior support structures for the model',
        default=False
    )

    n_columns: IntProperty(
        name='Number of Columns',
        description='Number of columns present in the exterior support',
        default=4,
        min=1,
        max=20
    )

    column_size: FloatProperty(
        name='Column Thickness',
        description='How large the column cross sections are',
        default=0.5,
        min=0.01,
        max=5
    )

    # radius: FloatProperty(
    #     name='Support Radius',
    #     description='How far away the support columns are from the building itself',
    #     default=0.5,
    #     min=0,
    #     max=50,
    #     unit='LENGTH'
    # )

    height_ratio: FloatProperty(
        name='Support Height Ratio',
        description='Height of the support structures as a proportion of the entire building\'s height',
        default=0.8,
        min=0,
        max=1
    )

    column_shape: EnumProperty(
        name='Column Shape',
        description='Shape of the column cross sections',
        items=[
            ('SQUARE', 'Square', 'Square'),
            ('CIRCLE', 'Circle', 'Circle')
        ],
        default='CIRCLE'
    )

    bidirectional: BoolProperty(
        name='Bidirectional',
        description='Whether all columns rotate in the same direction',
        default=False
    )

    capped: BoolProperty(
        name='Capped Support',
        description='Whether or not the support columns are capped at the top',
        default=True
    )
    