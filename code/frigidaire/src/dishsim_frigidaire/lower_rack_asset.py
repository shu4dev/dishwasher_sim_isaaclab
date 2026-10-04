# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
"""Photo-informed geometry for the *standalone* Bosch lower rack.

This does not modify rack_gen, benchmark slots, or planning cache keys. Coordinates
are metres: X across the rack, -Y toward its handle, Z up from its floor reference.
The four bottom_rack photographs determine topology; dimensions are estimates within
the existing appliance envelope. No photograph is embedded in the resulting USD.

The mesh construction and USDA writer require only numpy, allowing the geometry to
be built without launching Kit. Installation/conversion uses pxr in the setup script.
"""
from dataclasses import dataclass, field
import json
import math
from pathlib import Path

import numpy as np


MATERIALS = {
    "CoatedWire": ((0.135, 0.180, 0.205), 0.12, 0.29),
    "FoldableWire": ((0.026, 0.035, 0.041), 0.04, 0.31),
    "WheelPolymer": ((0.084, 0.106, 0.125), 0.0, 0.38),
    "HubPolymer": ((0.045, 0.056, 0.066), 0.0, 0.43),
    "ClipPolymer": ((0.103, 0.131, 0.151), 0.0, 0.37),
}


def unit(v):
    v = np.asarray(v, dtype=float)
    return v / np.linalg.norm(v)


def fillet_path(points, radius=0.009, steps=8):
    """Circular, tangent-continuous bends through a polyline (not elbow spheres)."""
    points = np.asarray(points, dtype=float)
    out = [points[0]]
    for a, b, c in zip(points[:-2], points[1:-1], points[2:]):
        u, v = unit(b - a), unit(c - b)
        angle = math.acos(float(np.clip(np.dot(u, v), -1, 1)))
        if angle < 1e-5:
            out.append(b)
            continue
        cut = min(radius * math.tan(angle / 2), np.linalg.norm(b-a)*0.40,
                  np.linalg.norm(c-b)*0.40)
        effective_radius = cut / math.tan(angle / 2)
        normal = unit(v - u * np.dot(u, v))
        start = b - u * cut
        center = start + normal * effective_radius
        for theta in np.linspace(0, angle, steps + 1):
            out.append(center + effective_radius * (-normal * math.cos(theta) + u * math.sin(theta)))
    out.append(points[-1])
    return np.asarray(out)


@dataclass
class Mesh:
    material: str
    points: list = field(default_factory=list)
    normals: list = field(default_factory=list)
    faces: list = field(default_factory=list)

    def surface(self, rings, normals, closed=False):
        offset = len(self.points)
        rings, normals = np.asarray(rings), np.asarray(normals)
        count, sides = rings.shape[:2]
        self.points.extend(rings.reshape(-1, 3).tolist())
        self.normals.extend(normals.reshape(-1, 3).tolist())
        for i in range(count if closed else count - 1):
            for j in range(sides):
                a, b = offset + i*sides+j, offset + i*sides+(j+1)%sides
                c, d = offset + ((i+1)%count)*sides+(j+1)%sides, offset + ((i+1)%count)*sides+j
                self.faces.extend(((a, b, c), (a, c, d)))

    def tube(self, path, radius, sides=16):
        """Swept circular wire with parallel-transport frames and hemispherical ends."""
        path = np.asarray(path, dtype=float)
        tangents = np.empty_like(path)
        tangents[0], tangents[-1] = unit(path[1]-path[0]), unit(path[-1]-path[-2])
        for i in range(1, len(path)-1):
            tangents[i] = unit(unit(path[i]-path[i-1])+unit(path[i+1]-path[i]))
        reference = np.eye(3)[np.argmin(abs(tangents[0]))]
        u = unit(np.cross(reference, tangents[0]))
        frames = []
        angles = np.arange(sides) * (2*math.pi/sides)
        for t in tangents:
            u = unit(u-t*np.dot(u, t))
            v = np.cross(t, u)
            frames.append(np.cos(angles)[:, None]*u + np.sin(angles)[:, None]*v)
        rings, normals = [], []
        # A small first ring avoids duplicate pole vertices and zero-area triangles.
        # The end discs are closed explicitly below.
        for theta in np.linspace(-math.pi/2 + 0.03, 0, 5)[:-1]:
            n = math.cos(theta)*frames[0] + math.sin(theta)*tangents[0]
            rings.append(path[0]+radius*n)
            normals.append(n)
        for p, radial in zip(path, frames):
            rings.append(p+radius*radial)
            normals.append(radial)
        for theta in np.linspace(0, math.pi/2-0.03, 5)[1:]:
            n = math.cos(theta)*frames[-1] + math.sin(theta)*tangents[-1]
            rings.append(path[-1]+radius*n)
            normals.append(n)
        offset = len(self.points)
        self.surface(rings, normals)
        for end, center, normal, reverse in (
            (offset, path[0]-radius*tangents[0], -tangents[0], True),
            (offset+(len(rings)-1)*sides, path[-1]+radius*tangents[-1], tangents[-1], False),
        ):
            pole = len(self.points)
            self.points.append(center.tolist())
            self.normals.append(normal.tolist())
            for j in range(sides):
                a, b = end+j, end+(j+1)%sides
                self.faces.append((pole, b, a) if reverse else (pole, a, b))

    def lathe_x(self, center, profile, sides=48):
        """Closed wheel section around X; profile is a clockwise (X, radius) loop."""
        profile = np.asarray(profile, dtype=float)
        rings, normals = [], []
        for i, (x, r) in enumerate(profile):
            tangent = unit(unit(profile[i]-profile[i-1]) + unit(profile[(i+1)%len(profile)]-profile[i]))
            nx, nr = -tangent[1], tangent[0]
            angle = np.arange(sides)*2*math.pi/sides
            rings.append(np.asarray(center)+np.column_stack((np.full(sides, x), r*np.cos(angle), r*np.sin(angle))))
            normals.append(np.column_stack((np.full(sides, nx), nr*np.cos(angle), nr*np.sin(angle))))
        self.surface(rings, normals, closed=True)


@dataclass
class Rack:
    meshes: dict = field(default_factory=dict)
    wires: list = field(default_factory=list)
    wheels: list = field(default_factory=list)

    def mesh(self, name, material):
        if name not in self.meshes:
            self.meshes[name] = Mesh(material)
        return self.meshes[name]

    def wire(self, name, points, radius=0.002, bend=0.009, material="CoatedWire", collision=True):
        path = fillet_path(points, bend)
        self.mesh(name, material).tube(path, radius)
        if collision:
            self.wires.append((name, path, radius))


def build_rack():
    rack = Rack()
    # Seven long U-shaped floor ribs rise into the rear and the lower front wall.
    # Shallow V saddles in the rear bay are visible particularly well in left.webp.
    for x in [-0.198, -0.146, -0.075, 0.0, 0.070, 0.120, 0.171]:
        rack.wire("LongitudinalFloor", [(x,-0.268,0.108), (x,-0.252,0.030),
            (x,-0.227,0.007), (x,0.055,0.007), (x,0.096,-0.004),
            (x,0.135,0.007), (x,0.228,0.007), (x,0.255,0.043),
            (x,0.264,0.161 if x == 0 else 0.166)])
    # The sparse cross ribs continue up each wall, including the wheel-adjacent pairs.
    for y in [-0.237, -0.218, -0.122, -0.025, 0.120, 0.207, 0.239]:
        top = 0.146 if y < -0.21 or y > 0.22 else 0.158
        rack.wire("CrossFloorAndSides", [(-0.256,y,top), (-0.247,y,0.051),
            (-0.229,y,0.026), (-0.212,y,0.011), (0.212,y,0.011),
            (0.229,y,0.026), (0.247,y,0.051), (0.256,y,top)], bend=0.007)
    for x in [-0.210, 0.210]:
        rack.wire("Underframe", [(x,-0.243,0.008),(x,0.239,0.008)], radius=0.0025)
    for y in [-0.220, -0.017, 0.219]:
        rack.wire("Underframe", [(-0.223,y,0.004),(0.223,y,0.004)], radius=0.0024)
    # Three front rails wrap around the corners and turn back along the sides.
    for z, setback in [(0.031,0.018),(0.076,0.010),(0.108,0.0)]:
        rack.wire("FrontRails", [(-0.256,-0.180,z),(-0.256,-0.252,z),
            (-0.243,-0.268+setback,z),(0.243,-0.268+setback,z),
            (0.256,-0.252,z),(0.256,-0.180,z)], radius=0.0025, bend=0.014)
    # Side top rails have small dropped ends, rather than a generic flat rectangle.
    for x in [-0.256,0.256]:
        rack.wire("SideRims", [(x,-0.244,0.143),(x,-0.204,0.143),
            (x,-0.190,0.158),(x,0.214,0.158),(x,0.230,0.146),(x,0.253,0.146)],
            radius=0.00255, bend=0.010)
    # Two rear rails: high rim and low rail both carry the central shallow dip.
    for z in [0.166,0.068]:
        rack.wire("RearRails", [(-0.256,0.239,z-0.023),(-0.251,0.264,z),
            (-0.043,0.264,z),(-0.031,0.264,z-0.005),(0.031,0.264,z-0.005),
            (0.044,0.264,z),(0.251,0.264,z),(0.256,0.239,z-0.023)],
            radius=0.0025, bend=0.010)
    # Rear plate banks run across X. The darker folding bank is taller and has
    # small hinge collars, not individual plastic sockets under every tine.
    for name,y,height,lean,material in [
        ("RearFoldingTines",0.144,0.114,-0.023,"FoldableWire"),
        ("RearFixedTines",0.045,0.078,-0.020,"CoatedWire"),
    ]:
        base_z = 0.019 if name == "RearFoldingTines" else 0.011
        if name == "RearFixedTines":
            rack.wire(name,[(-0.256,y,0.158),(-0.247,y,0.051),
                (-0.229,y,0.026),(-0.212,y,base_z),(0.212,y,base_z),
                (0.229,y,0.026),(0.247,y,0.051),(0.256,y,0.158)],bend=0.007)
        else:
            rack.wire(name,[(-0.217,y,base_z),(0.206,y,base_z)],radius=0.00225,material=material)
        for x in np.linspace(-0.193,0.185,17):
            rack.wire(name,[(x,y,base_z),(x-0.002,y,base_z+0.010),(x+lean,y+0.006,height)],
                      radius=0.00185, bend=0.006, material=material)
    for x in [-0.075,0.120]:
        # Axial split collars leave the black pivot wire visible between clips.
        for dx in [-0.003,0.003]:
            rack.wire("FoldingClips",[(x+dx,0.137,0.007),(x+dx,0.137,0.016),
                (x+dx,0.144,0.024),(x+dx,0.151,0.016),(x+dx,0.151,0.007)],
                radius=0.0024,bend=0.004,material="ClipPolymer")
    # Four front combs run front-to-back; seven teeth each lean toward the handle.
    # Their raised bases have turned-down feet welded into the sparse cross ribs.
    for index,x in enumerate([-0.165,-0.075,0.015,0.105]):
        name = f"FrontTineBank{index+1}"
        rack.wire(name,[(x,-0.234,0.011),(x,-0.223,0.034),
                       (x,-0.030,0.034),(x,-0.023,0.011)],radius=0.0022,bend=0.006)
        for y in np.linspace(-0.211,-0.045,7):
            rack.wire(name,[(x,y,0.034),(x,y-0.004,0.045),(x,y-0.024,0.086)],
                      radius=0.00185,bend=0.005)
    # Bare raised wire grip shown in front.webp, with a small forward offset.
    rack.wire("WireHandle",[(-0.084,-0.249,0.028),(-0.084,-0.268,0.060),
        (-0.084,-0.284,0.078),(-0.084,-0.284,0.153),
        (0.084,-0.284,0.153),(0.084,-0.284,0.078),
        (0.084,-0.268,0.060),(0.084,-0.249,0.028)],radius=0.0028,bend=0.008)
    # A single subtle capped rear stop and the folding-release paddle on the left.
    rack.wire("RearStop",[(0,0.261,0.093),(0.014,0.260,0.093)],radius=0.0018)
    rack.wire("RearStopCap",[(0.012,0.260,0.093),(0.018,0.260,0.093)],
              radius=0.0024,material="ClipPolymer")
    # Thin rounded moulded paddle, open underside and raised ribs. Stretch the
    # swept section across Y into a 52 mm wide, 4 mm thick curved panel.
    paddle = rack.mesh("ReleasePaddle", "ClipPolymer")
    paddle.tube(fillet_path([(-0.249,0.149,0.035),(-0.243,0.149,0.054),
                            (-0.229,0.149,0.060)],0.006),0.002,sides=32)
    pp, pn = np.asarray(paddle.points), np.asarray(paddle.normals)
    pp[:,1] = 0.149 + (pp[:,1]-0.149)*13
    pn[:,1] /= 13
    pn /= np.linalg.norm(pn,axis=1)[:,None]
    paddle.points, paddle.normals = pp.tolist(), pn.tolist()
    for yy in [0.124,0.145,0.174]:
        rack.wire("ReleasePaddleRibs",[(-0.251,yy,0.034),(-0.245,yy,0.057),
            (-0.227,yy,0.062)],radius=0.0016,material="HubPolymer")
    rack.wire("FoldingClips",[(-0.216,0.144,0.019),(-0.229,0.144,0.025),
        (-0.248,0.144,0.039)],radius=0.003,bend=0.006,material="ClipPolymer")
    # Eight 35 mm rollers: annular tread, inset open spokes, snap hub, clip/axle.
    wheel_profile = [(-0.0058,0.0124),(-0.0058,0.0158),(-0.0047,0.0171),
        (-0.0035,0.0175),(0.0035,0.0175),(0.0047,0.0171),(0.0058,0.0158),
        (0.0058,0.0124),(0.0044,0.0118),(-0.0044,0.0118)]
    for side in [-1,1]:
        for y in [-0.218,-0.122,0.120,0.207]:
            c = np.array([side*0.252,y,-0.008])
            rack.wheels.append(c.tolist())
            rack.mesh("WheelTreads","WheelPolymer").lathe_x(c,wheel_profile)
            hub_profile=[(-0.006,0.0025),(0.006,0.0025),(0.006,0.0055),
                         (0.004,0.0065),(-0.004,0.0065),(-0.006,0.0055)]
            rack.mesh("WheelHubs","HubPolymer").lathe_x(c,hub_profile[::-1])
            for a in np.arange(6)*math.pi/3:
                p=c+np.array([side*0.001,math.cos(a)*0.0055,math.sin(a)*0.0055])
                q=c+np.array([side*0.001,math.cos(a+0.12)*0.0125,math.sin(a+0.12)*0.0125])
                rack.wire("WheelSpokes",[p,q],radius=0.00145,material="HubPolymer",collision=False)
            rack.wire("WheelAxles",[(side*0.230,y,-0.008),(side*0.257,y,-0.008)],
                      radius=0.0025,material="HubPolymer",collision=False)
            rack.wire("WheelClips",[(side*0.231,y,0.028),(side*0.235,y,0.015),(side*0.238,y,0.001),
                       (side*0.244,y,-0.008)],radius=0.0035,bend=0.006,material="ClipPolymer")
    return rack


def _vec(v):
    return "("+", ".join(f"{x:.8g}" for x in v)+")"


def _array(rows):
    return "["+", ".join(_vec(v) for v in rows)+"]"


def collision_segments(path, tolerance=0.00020):
    """Simplify curved centre-lines to <=0.2 mm chord error for capsule colliders."""
    if len(path) <= 2:
        return path
    a,b=path[0],path[-1]
    d=b-a
    t=np.clip((path-a)@d/np.dot(d,d),0,1)
    dist=np.linalg.norm(path-(a+t[:,None]*d),axis=1)
    index=int(np.argmax(dist))
    if dist[index] <= tolerance:
        return np.array([a,b])
    return np.concatenate((collision_segments(path[:index+1],tolerance)[:-1],
                           collision_segments(path[index:],tolerance)))


def write_usda(rack, path):
    """Write a portable rigid rack with material-grouped visuals and open collisions."""
    path=Path(path)
    path.parent.mkdir(parents=True,exist_ok=True)
    with path.open("w") as f:
        def line(s):
            f.write(s+"\n")
        line('#usda 1.0\n(\n    defaultPrim = "LowerRack"\n    metersPerUnit = 1\n    kilogramsPerUnit = 1\n    upAxis = "Z"\n)')
        line('def Xform "LowerRack" (prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"])\n{')
        line('    float physics:mass = 3.2\n    bool physics:rigidBodyEnabled = true')
        line('    custom string referenceViews = "front.webp; left.webp; top.webp; top_right.webp"')
        line('    custom string geometryRevision = "bottom_rack_four_views_v1"')
        line('    def Scope "Looks"\n    {')
        for name,(color,metallic,roughness) in MATERIALS.items():
            line(f'        def Material "{name}"\n        {{')
            line(f'            token outputs:surface.connect = </LowerRack/Looks/{name}/Shader.outputs:surface>')
            line('            def Shader "Shader"\n            {\n                uniform token info:id = "UsdPreviewSurface"')
            line(f'                color3f inputs:diffuseColor = {_vec(color)}\n                float inputs:metallic = {metallic}\n                float inputs:roughness = {roughness}')
            line('                float inputs:ior = 1.48\n                token outputs:surface\n            }\n        }')
        line('        def Material "Contact" (prepend apiSchemas = ["PhysicsMaterialAPI"])\n        {\n            float physics:staticFriction = 0.55\n            float physics:dynamicFriction = 0.40\n            float physics:restitution = 0\n        }\n    }')
        line('    def Scope "Visuals"\n    {')
        for name,mesh in rack.meshes.items():
            line(f'        def Mesh "{name}" (prepend apiSchemas = ["MaterialBindingAPI"])\n        {{')
            line(f'            point3f[] points = {_array(mesh.points)}')
            line(f'            normal3f[] normals = {_array(mesh.normals)} (interpolation = "vertex")')
            line('            int[] faceVertexCounts = ['+', '.join('3' for _ in mesh.faces)+']')
            line('            int[] faceVertexIndices = ['+', '.join(str(i) for face in mesh.faces for i in face)+']')
            line('            uniform token subdivisionScheme = "none"\n            uniform token orientation = "rightHanded"')
            line(f'            float3[] extent = {_array([np.min(mesh.points,axis=0),np.max(mesh.points,axis=0)])}')
            line(f'            rel material:binding = </LowerRack/Looks/{mesh.material}>\n        }}')
        line('    }\n    def Scope "Collisions" (prepend apiSchemas = ["MaterialBindingAPI"])\n    {')
        line('        token visibility = "invisible"\n        rel material:binding:physics = </LowerRack/Looks/Contact>')
        count=0
        for name,wire,radius in rack.wires:
            segments=collision_segments(wire)
            for a,b in zip(segments[:-1],segments[1:]):
                direction=unit(b-a)
                w=1+direction[2]
                q=np.array([w,-direction[1],direction[0],0])
                q=unit(q) if np.linalg.norm(q)>1e-8 else np.array([0,1,0,0])
                line(f'        def Capsule "{name}_{count:04d}" (prepend apiSchemas = ["PhysicsCollisionAPI", "PhysxCollisionAPI"])\n        {{')
                line(f'            double radius = {radius}\n            double height = {np.linalg.norm(b-a):.8g}\n            uniform token axis = "Z"')
                line(f'            double3 xformOp:translate = {_vec((a+b)/2)}\n            quatf xformOp:orient = {_vec(q)}')
                line('            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]\n            float physxCollision:contactOffset = 0.001\n            float physxCollision:restOffset = 0\n        }')
                count+=1
        for i,c in enumerate(rack.wheels):
            line(f'        def Cylinder "Wheel_{i}" (prepend apiSchemas = ["PhysicsCollisionAPI", "PhysxCollisionAPI"])\n        {{\n            uniform token axis = "X"\n            double radius = 0.0175\n            double height = 0.0116\n            float physxCollision:contactOffset = 0.001\n            float physxCollision:restOffset = 0')
            line(f'            double3 xformOp:translate = {_vec(c)}\n            uniform token[] xformOpOrder = ["xformOp:translate"]\n        }}')
        line('    }\n    def Scope "Manipulation"\n    {')
        sites={"handle_center":(0,-0.284,0.153),"plate_slot_0":(-0.025,-0.169,0.142),
               "plate_slot_1":(-0.025,-0.1413,0.142),"rear_plate_slot":(-0.038,0.100,0.142)}
        for name,pos in sites.items():
            line(f'        def Xform "{name}"\n        {{\n            double3 xformOp:translate = {_vec(pos)}\n            uniform token[] xformOpOrder = ["xformOp:translate"]\n            custom string status = "Candidate site; requires renewed contact validation"\n        }}')
        line('    }\n}')
    return count+len(rack.wheels)


def validate_geometry(rack):
    """Mesh manifoldness, orientation, dimensions, and reference-layout checks."""
    report={"reference_views":["front.webp","left.webp","top.webp","top_right.webp"],
            "dimension_status":"Photo estimates within the appliance envelope, not measured CAD",
            "rear_banks":2,"rear_tines_per_bank":17,"front_banks":4,"front_tines_per_bank":7,
            "wheel_count":len(rack.wheels),"meshes":{},"isaac_sim_validated":False}
    all_points=[]
    for name,m in rack.meshes.items():
        p,n,t=np.asarray(m.points),np.asarray(m.normals),np.asarray(m.faces)
        assert np.isfinite(p).all() and np.isfinite(n).all(),name
        assert t.min()>=0 and t.max()<len(p),name
        face_n=np.cross(p[t[:,1]]-p[t[:,0]],p[t[:,2]]-p[t[:,0]])
        assert np.min(np.linalg.norm(face_n,axis=1))>1e-14,(name,"degenerate triangles")
        assert np.min(np.sum(face_n*n[t].mean(axis=1),axis=1))>0,(name,"inward winding")
        volume = np.sum(p[t[:,0]] * np.cross(p[t[:,1]], p[t[:,2]])) / 6
        assert volume > 0, (name, "inward volume")
        edges=np.sort(np.concatenate((t[:,[0,1]],t[:,[1,2]],t[:,[2,0]])),axis=1)
        _,counts=np.unique(edges,axis=0,return_counts=True)
        assert np.all(counts==2),(name,"non-manifold edges")
        report["meshes"][name]={"vertices":len(p),"triangles":len(t),"closed_manifold":True}
        all_points.extend(m.points)
    p=np.asarray(all_points)
    report["bounds_m"]=[p.min(axis=0).tolist(),p.max(axis=0).tolist()]
    report["size_m"]=(p.max(axis=0)-p.min(axis=0)).tolist()
    assert report["size_m"][0]<0.527 and report["size_m"][1]<0.56
    assert len(rack.wheels)==8
    assert all(np.isclose(abs(c[0]),0.252) for c in rack.wheels)
    for name in ["RearFoldingTines","RearFixedTines"]:
        assert sum(n==name for n,_,_ in rack.wires)==18
    for i in range(1,5):
        assert sum(n==f"FrontTineBank{i}" for n,_,_ in rack.wires)==8
    report["result"]="PASS (geometry only)"
    return report


def build(output_dir):
    output_dir=Path(output_dir)
    output_dir.mkdir(parents=True,exist_ok=True)
    rack=build_rack()
    report=validate_geometry(rack)
    report["colliders"]=write_usda(rack,output_dir/"lower_rack.usda")
    (output_dir/"geometry_validation.json").write_text(json.dumps(report,indent=2)+"\n")
    return rack,report
