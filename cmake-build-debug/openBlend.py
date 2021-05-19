import bpy
import os
import sys

pathSt = sys.argv[4]
substring = "ply"

if substring in pathSt:
    bpy.ops.import_mesh.ply(filepath=pathSt)
else:
    bpy.ops.import_scene.obj(filepath=pathSt)