import bpy
import os
import sys

pathSt = float(sys.argv[1])

bpy.ops.import_mesh.ply(filepath="pathSt")