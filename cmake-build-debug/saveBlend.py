import bpy
import os
import sys

pathSt = sys.argv[5]

bpy.ops.import_mesh.ply(filepath=pathSt+"/rec3D/tvl1/tvl1_mesh.ply")
bpy.ops.wm.save_as_mainfile(filepath=pathSt+"/tv.blend")