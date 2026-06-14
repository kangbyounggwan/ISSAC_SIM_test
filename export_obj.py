#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""export_obj.py — world_geometry 를 OBJ(+MTL) 로 내보내기.
Windows 11 '3D 뷰어', Blender, https://3dviewer.net 등에서 바로 열림. (USD/Isaac 불필요)
사용:  python export_obj.py   ->  jm_factory_world.obj / .mtl
"""
from world_geometry import build_scene, box_corners

BOX_FACES = [  # 박스 8코너 인덱스 면(반시계, 외향 노멀)
    (0, 1, 2, 3), (4, 7, 6, 5), (0, 4, 5, 1),
    (1, 5, 6, 2), (2, 6, 7, 3), (3, 7, 4, 0)]


def export(obj_path="jm_factory_world.obj"):
    parts = build_scene()
    mtl_path = obj_path[:-4] + ".mtl"
    mats = {}      # color -> name
    vlines, flines = [], []
    voff = 1       # OBJ 1-based

    def color_name(c):
        if c not in mats:
            mats[c] = "m%d" % len(mats)
        return mats[c]

    for p in parts:
        flines.append("usemtl " + color_name(p["color"]))
        flines.append("g " + p["floor"] + "__" + p["path"].strip("/").replace("/", "_"))
        if p["type"] == "box":
            for v in box_corners(p):
                vlines.append("v %.4f %.4f %.4f" % tuple(v))
            for f in BOX_FACES:
                flines.append("f " + " ".join(str(voff + i) for i in f))
            voff += 8
        else:
            for v in p["points"]:
                vlines.append("v %.4f %.4f %.4f" % tuple(v))
            for f in p["faces"]:
                flines.append("f " + " ".join(str(voff + i) for i in f))
            voff += len(p["points"])

    with open(obj_path, "w") as fo:
        fo.write("# JM Electronics Sihwa factory - generated from CAD\n")
        fo.write("mtllib %s\n" % mtl_path.split("\\")[-1].split("/")[-1])
        fo.write("\n".join(vlines))
        fo.write("\n")
        fo.write("\n".join(flines))
        fo.write("\n")
    with open(mtl_path, "w") as fm:
        for c, name in mats.items():
            r, g, b = c
            fm.write("newmtl %s\nKd %.3f %.3f %.3f\nKa %.3f %.3f %.3f\nKs 0 0 0\nillum 1\n\n"
                     % (name, r, g, b, r * 0.3, g * 0.3, b * 0.3))
    print("OBJ :", obj_path, "(verts=%d)" % (voff - 1))
    print("MTL :", mtl_path, "(materials=%d)" % len(mats))


if __name__ == "__main__":
    export(r"C:\issac sim test\jm_factory_world.obj")
