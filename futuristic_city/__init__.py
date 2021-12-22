bl_info = {
    "name":"Futuristic Building Generator",
    "author":"Yifei Shen",
    "version":(1,0),
    "blender":(3,0,0),
    "location":"3D View > Tools",
    "description":"Creating futuristic buildings procedurally using sweeping",
    "warning":"",
    "wiki_url":"",
    "tracker_url":"",
    "category":"Add Mesh"
    }

import bpy

from bpy.props import PointerProperty

from . fbsweep_properties import FBSweepProperties
from . fbsweep_panel import OBJECT_PT_FBSweepPanel
from . fbsweep_gen import WM_OT_GenFBSweep
from . roads import WM_OT_GenWholeCity

classes = [FBSweepProperties, OBJECT_PT_FBSweepPanel, WM_OT_GenFBSweep, WM_OT_GenWholeCity]

# Register/unregister classes
def register():
    from bpy.utils import register_class
    for cls in classes:
        register_class(cls)
    bpy.types.Scene.fbsweep_properties = PointerProperty(type=FBSweepProperties)

def unregister():
    from bpy.utils import unregister_class
    for cls in reversed(classes):
        unregister_class(cls)
    del bpy.types.Scene.fbsweep_properties
