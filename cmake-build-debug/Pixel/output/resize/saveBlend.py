import bpy
import os
import sys

pathSt = float(sys.argv[1])

bpy.ops.import_mesh.ply(filepath=pathSt+"/rec3D/tvl1/tvl1_mesh.ply")
bpy.ops.wm.save_as_mainfile(pathSt+"/tv.blend")