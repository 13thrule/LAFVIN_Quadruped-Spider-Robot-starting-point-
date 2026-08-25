"""One-off script: reports the bounding box (mm) of each reference STL,
downloaded from the xinlitech/quadruped-spider-for-esp8266 repo -- the same
family of OEM quadruped spider design this LAFVIN kit is a rebrand of."""
import glob
import os

from stl import mesh

for path in sorted(glob.glob(os.path.join(os.path.dirname(__file__), "*.stl"))):
    m = mesh.Mesh.from_file(path)
    pts = m.points.reshape(-1, 3)
    mins = pts.min(axis=0)
    maxs = pts.max(axis=0)
    size = maxs - mins
    name = os.path.basename(path)
    print(f"{name:16s} size(mm) X={size[0]:7.2f} Y={size[1]:7.2f} Z={size[2]:7.2f}")
