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

import bpy_types
from bpy.types import Panel

class OBJECT_PT_FBSweepPanel(Panel):
    bl_idname = 'OBJECT_PT_fbsweep_panel'
    bl_label = 'Futuristic Building Sweeps'
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Futuristic Buildings'
    bl_context = 'objectmode'

    @classmethod
    def poll(self, context):
        return True

    def draw(self, context):
        layout = self.layout
        fbsweep_props = context.scene.fbsweep_properties

        row = layout.row()
        row.operator('wm.gen_wholecity', text='CREATE FULL CITY', icon='SEQ_HISTOGRAM')

        row = layout.row()
        col = layout.column(align=True)
        col.label(text='Base:')
        col.prop(fbsweep_props, 'x_width')
        col.prop(fbsweep_props, 'y_width')
        col.prop(fbsweep_props, 'curve')
        if fbsweep_props.curve in ['HYPOTROCHOID', 'EPITROCHOID']:
            col.prop(fbsweep_props, 'revolution')
        elif fbsweep_props.curve == 'ELLIPSE':
            col.prop(fbsweep_props, 'ellipse_ratio')

        row = layout.row()
        
        col.label(text='Height:')
        col.prop(fbsweep_props, 'height')
        col.prop(fbsweep_props, 'floors')
        col.prop(fbsweep_props, 'model_floors')

        row = layout.row()

        col.label(text='Rotations:')
        col.prop(fbsweep_props, 'max_rotation')
        col.prop(fbsweep_props, 'rotate_func')

        row = layout.row()
        col.label(text="Scaling:")
        col.prop(fbsweep_props, 'top_scale_x')
        col.prop(fbsweep_props, 'top_scale_y')
        col.prop(fbsweep_props, 'scale_func_x')
        col.prop(fbsweep_props, 'scale_func_y')

        row = layout.row()
        col.prop(fbsweep_props, 'support')
        if fbsweep_props.support:
            col.prop(fbsweep_props, 'n_columns')
            col.prop(fbsweep_props, 'column_size')
            # col.prop(fbsweep_props, 'radius')
            col.prop(fbsweep_props, 'height_ratio')
            col.prop(fbsweep_props, 'column_shape')
            col.prop(fbsweep_props, 'bidirectional')
            # col.prop(fbsweep_props, 'capped')

        row = layout.row()
        row.operator('wm.gen_fbsweep', text='Generate Single Building', icon='PHYSICS')
