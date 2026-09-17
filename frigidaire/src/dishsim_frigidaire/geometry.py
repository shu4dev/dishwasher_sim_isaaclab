# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
"""Reference-informed FDPC4221AS geometry, independent of the Bosch generators.

Coordinates are metres, X across the appliance, -Y toward its front, and Z up.
Meshes and collider paths are generated together so openings remain accessible.
The photographs establish topology; hidden dimensions and wire counts remain
explicit estimates, not manufacturer CAD or measured manufacturing tolerances.
This module requires only numpy and does not import Isaac Sim or USD.
"""
from dataclasses import dataclass, field
import math

import numpy as np


PARAMETERS = {
    "model": "Frigidaire FDPC4221AS",
    "coordinate_system": {"units": "metres", "up": "Z", "front": "-Y"},
    "appliance": {"width": .6096, "depth": .635, "height": .8509,
                  "door_open_depth": 1.25095,
                  "source": "assets/models/frigidaire_fdpc4221as/references/spec.pdf dimension drawing; 25 inch depth takes precedence over conflicting 24 inch table"},
    "published_rack_clearance": {
        "source": "assets/models/frigidaire_fdpc4221as/references/spec.pdf page 1",
        "upper": {"label": "Minimum Height Clearance", "value_m": .2032, "printed_value": "8 inches"},
        "lower": {"minimum_label": "Minimum Height Clearance", "minimum_m": .2794,
                  "maximum_label": "Maximum Height Clearance", "maximum_m": .3302,
                  "printed_values": "11 inches minimum; 13 inches maximum"},
        "measurement_datum": "unspecified in the supplied specification; no clearance-region diagram or dedicated footnote",
        "interpretation": "The upper 8-inch value is a published minimum, not an established global maximum.",
        "capacity_status": "Upper placements taller than 203.2 mm are unverified. The modeled floor-to-ceiling clearance is an estimate, not certified appliance capacity."},
    "origins": {"Cabinet": [0, 0, 0], "Door": [0, -.27425, .19125],
                "LowerRack": [0, .008, .215], "UpperRack": [0, .008, .590],
                "SilverwareBasket": [.2135, .128, .226]},
    "door_front": {"fascia_material": "BlackPlastic", "fascia_bottom_z": .53775,
                   "outer_skin_bottom_z": -.05525, "base_front_y": -.210,
                   "outer_skin_to_base_sweep_clearance_min": .011018,
                   "controls": ["Cycles", "Heat Dry", "Start/Cancel"],
                   "control_type": "flush membrane, visual only", "handle": "wide recessed pocket with central squeeze latch",
                   "source": ["https://frigidaire.bynder.com/transform/XL-1400/2f590bac-50b2-4e85-9db5-5bffc5692cbc/FDPC4221AS-CP-psd",
                              "https://frigidaire.bynder.com/transform/XL-1400/fa472cfb-2998-43c6-b702-c038cb226387/FDPC4221AS-34VL-psd"],
                   "dimension_status": "proportions estimated from official model photographs; no control or latch articulation"},
    "rack_dimension_datum": "Outer wire-rim envelope; wheels, hubs and front grip projections are reported separately. User inch estimates are reconstruction targets, not manufacturer measurements.",
    "interior_fit": {"hinge_yz": [-.27425, .19125],
                     "hinge_shift_from_first_reconstruction_yz": [-.02025, .02025],
                     "hinge_status": "inferred mechanical datum, not measured",
                     "reason": "The deeper lower rack conflicts with tall fixed door runways when closed. Recessing the liner and shifting the inferred hinge puts flush wheel strips at Z=180 mm when open while preserving the closed exterior and door-open depth.",
                     "lower_track_top_z": .180, "door_support_top_open_z": .180,
                     "tub_back_inner_y": .3015, "closed_liner_inner_y": -.2855,
                     "lower_rim_front_back_clearance": .00267,
                     "lower_rim_side_clearance": .00268},
    "lower_rack": {"wire_width": .54864, "wire_depth": .58166, "rim_height": .115,
                   "wire_diameter": .004, "rim_diameter": .0048,
                   "floor_cross_ribs": 21, "floor_longitudinal_ribs": 17,
                   "geometry_revision": "lower_tines_6x12_v1",
                   "tine_banks": 6, "tines_per_bank": 12,
                   "tine_repetition_axis": "X",
                   "tine_margins": {"left": .080, "right": .120,
                                    "front": .105, "rear": .110},
                   "tine_margin_datum": "Tine base centers to the outer wire-rim edge; front -Y, rear +Y.",
                   "tine_pitch_source": "Derived from rim dimensions, edge margins, and row/column counts.",
                   "tine_diameter": .0039, "tine_height": .105,
                   "tine_tip_offset_x": .008, "tine_base_z": .006,
                   "wheel_count": 8, "wheel_radius": .016,
                   "wheel_center_x_abs": .2693,
                   "wheel_center_z": -.019, "estimated_mass_kg": 3.7,
                   "basket_reserved_x": [.1675, .2595], "basket_reserved_y": [-.036, .276],
                   "basket_shift_x_from_previous": .0275,
                   "source": ["user estimate: 22.9 by 21.6 inches outer rim", "user lower-rack correction: 6 rows by 12 columns; preserve counts and 80/120/105/110 mm left/right/front/rear base margins, deriving 31.694545 mm X and 73.332 mm Y pitch; retain the basket's 27.5 mm rightward shift", "assets/models/frigidaire_fdpc4221as/references/lower_rack/front.jpg", "assets/models/frigidaire_fdpc4221as/references/lower_rack/right.jpg", "assets/models/frigidaire_fdpc4221as/references/lower_rack/front_left.jpg", "assets/models/frigidaire_fdpc4221as/references/overall/overall_1.webp", "assets/models/frigidaire_fdpc4221as/references/loaded/loaded_2.webp", "assets/models/frigidaire_fdpc4221as/references/loaded/loaded_3.webp", "assets/models/frigidaire_fdpc4221as/references/loaded/loaded_4.webp"]},
    "upper_rack": {"wire_width": .508, "wire_depth": .54864,
                   "rim_height": .112, "lowest_floor_center_z": -.018,
                   "floor_to_rim_center_height": .130, "floor_channels": 5,
                   "wire_diameter": .0038, "rim_diameter": .0044,
                   "floor_cross_ribs": 21, "floor_longitudinal_ribs": 9,
                   "front_uprights": 9, "front_horizontal_rails": 2,
                   "geometry_revision": "upper_tines_4x13_v1",
                   "tine_banks": 4, "tines_per_bank": 13, "tine_spacing": .033,
                   "tine_repetition_axis": "Y",
                   "tine_bank_x": [-.1325, -.0375, .0375, .1325],
                   "tine_rear_margin": .0515,
                   "tine_margin_datum": "Tine base centers to the outer wire-rim edge; front -Y, rear +Y.",
                   "tine_diameter": .0036, "tine_height": .091,
                   "tine_tip_offset_y": .008, "tine_base_floor_offset": .003,
                   "wheel_count": 4, "wheel_radius": .0105,
                   "estimated_mass_kg": 2.9,
                   "source": ["user estimate: 21.6 by 20.0 inches outer rim", "user upper-rack correction: 4 columns by 13 tines, 95/75/95 mm column gaps, 33 mm pitch, 51.5 mm rear base margin; retain footprint, giving 121.5 mm side and 101.14 mm front margins", "assets/models/frigidaire_fdpc4221as/references/upper_rack/front.jpg", "assets/models/frigidaire_fdpc4221as/references/upper_rack/right.jpg", "assets/models/frigidaire_fdpc4221as/references/upper_rack/front_left.jpg", "assets/models/frigidaire_fdpc4221as/references/overall/overall_1.webp", "assets/models/frigidaire_fdpc4221as/references/overall/overall_2.webp", "assets/models/frigidaire_fdpc4221as/references/loaded/loaded_1.webp", "assets/models/frigidaire_fdpc4221as/references/loaded/loaded_2.webp", "assets/models/frigidaire_fdpc4221as/references/loaded/loaded_3.webp", "assets/models/frigidaire_fdpc4221as/references/loaded/loaded_4.webp"]},
    "silverware_basket": {"width": .088, "depth": .312, "body_height": .152,
                          "handle_height": .203, "bottom_width": .0746,
                          "bottom_depth": .2969, "compartments": 4,
                          "lattice_pitch": .007, "lattice_vertical_pitch": .00918,
                          "lattice_wire_diameter": .0022,
                          "estimated_mass_kg": .30,
                          "utensil_candidate": {"orientation": "head down", "quat_wxyz": [0, 1, 0, 0],
                                                "site_xyz": [0, -.1115625, .096],
                                                "support": "27 mm head spans five longitudinal floor wires; initial bottom clearance 0.581 mm",
                                                "reason": "Narrow handle-down placement can thread through the real drainage lattice and snag."},
                          "proportion_fit": {"front_length_to_total_height": 1.537,
                                             "top_length_to_width": 3.545,
                                             "handle_span_fraction": .70,
                                             "status": "absolute scale inferred; aspect ratios fitted to front and top_down references"},
                          "source": ["assets/models/frigidaire_fdpc4221as/references/silverware_basket/front.jpg", "assets/models/frigidaire_fdpc4221as/references/silverware_basket/top.jpg", "assets/models/frigidaire_fdpc4221as/references/silverware_basket/top_down.jpg", "assets/models/frigidaire_fdpc4221as/references/silverware_basket/bottom.jpg"]},
    "reference_map": {
        "wire_detail.jpeg": "Exploded drawing confirms separate upper slides, two different rack assemblies, lower rollers and a removable side basket; no scale is supplied.",
        "lower/front": "Symmetric shallow floor, upright side U-ribs, continuous upper rim; front wires turn upward into the front wall.",
        "lower/right": "Four dark rollers per side, distinct tine combs separated by open loading bays, raised stepped rear rim.",
        "lower/front_left": "Rounded U-ribs continue floor-to-wall; two perimeter reinforcing rails; slight tine lean; basket-sized open side bay.",
        "upper/front": "Flared mouth, repeated curved cross-floor profiles forming side cup troughs and raised central ridges; narrow floor-to-wall bends.",
        "upper/right": "Three side rails, curved fore/aft floor transitions and small dark rollers on side supports.",
        "upper/front_left": "Sparse orthogonal supporting wires under denser transverse contoured ribs, short interior support tines and a continuous lip.",
        "overall/overall_1": "Front view establishes lower transverse plate slots and rear-right basket; upper five-channel floor, two front rails and sparse upright endpoints.",
        "overall/overall_2": "Empty oblique/front view confirms a shallow upper wall, raised intermediate floor shoulders and side carrier alignment.",
        "loaded/loaded_1": "Lower plates face sideways in two front/back banks. Inverted glasses lean outwards along the sloped upper side channels; central saucers face forward.",
        "loaded/loaded_2": "Underside view resolves separate inclined glass and intermediate mug floors; rear lower plates require rear tine combs.",
        "loaded/loaded_3": "Two tilted bowls occupy lower front-right ahead of the rearward basket; mugs occupy intermediate upper channels.",
        "loaded/loaded_4": "Both extended racks establish rim proportions and loaded clearance; rear central upper separators continue through usable depth.",
        "basket/front": "Dense square lattice with deep bottom rim and broad elevated arch handle; four compartments are inferred from dividers.",
        "basket/top_down": "Long narrow rounded rectangle, taper toward bottom, three cross partitions, no hinged lid.",
        "basket/bottom": "Perforated underside retained as open rib grid; reinforced longitudinal base edges.",
        "basket/top": "Open top and elongated handle aperture; body is dark molded plastic with taper and reinforced corners."},
    "assumptions": [
        "Photo resolutions do not identify every overlapping wire; counts, local bends and inaccessible structure are inferred.",
        "Rack outer wire-rim dimensions follow the user's inch estimates exactly; wire diameters, absolute interior depths and clearances are still estimates.",
        "Visible four-per-side lower wheels and four basket compartments guide topology; rack tines are fixed.",
        "Upper lowest side floor is -18 mm, central floor is -10 mm and rim is +112 mm; sloped outer channels and intermediate mug channels reproduce the overall and loaded views.",
        "Upper front and rear wall ribs terminate on the actual filleted cross-floor rib centerlines, rather than free-hanging above the troughs.",
        "Ribs and lattice are swept round sections with rounded ends; plastic molding draft and ribs are approximated.",
        "Basket length/height and length/width follow the near-frontal and overhead photo silhouettes; 312 by 88 by 203 mm is a proportion fit, not a measured part dimension.",
        "The lower 6-by-12 tine grid preserves the measured base margins; its right-side basket seat retains a 27.5 mm shift in +X. The outer row pairs provide front and rear plate supports. Installed rack rollers use simplified constrained slides.",
        "The estimated tub back and inner door liner are recessed to accommodate the deeper rack targets. The inferred hinge shifts 20.25 mm forward and 20.25 mm upward so flush door wheel strips align to the 180 mm tracks; closed exterior points and door-open depth are unchanged. Upper rack and carriers are raised 50 mm from the first reconstruction.",
        "Door front topology follows official model photographs; pocket dimensions, tub sill and roller runways remain functional approximations.",
        "Stainless skin extends 55.25 mm below the revised hinge, leaving the same 13 mm closed front seam; toe kick and plinth stay recessed for the full 0–90 degree sweep.",
        "Masses, friction, inertia and concealed mechanical resistance require calibration against a physical dishwasher."],
}


def _unit(v):
    v = np.asarray(v, dtype=float)
    norm = np.linalg.norm(v)
    if norm < 1e-12:
        raise ValueError("Cannot normalize zero-length vector")
    return v / norm


def fillet_path(points, radius=.009, steps=3):
    """Replace polyline elbows with tangent circular bends of limited radius."""
    p = np.asarray(points, dtype=float)
    p = p[np.r_[True, np.linalg.norm(np.diff(p, axis=0), axis=1) > 1e-10]]
    if len(p) < 2:
        raise ValueError("Wire needs two distinct points")
    out = [p[0]]
    for a, b, c in zip(p[:-2], p[1:-1], p[2:]):
        u, v = _unit(b-a), _unit(c-b)
        angle = math.acos(float(np.clip(np.dot(u, v), -1., 1.)))
        if angle < 1e-6:
            out.append(b)
            continue
        if math.pi-angle < 1e-6:
            raise ValueError("Wire contains a reversing elbow")
        cut = min(radius*math.tan(angle/2), np.linalg.norm(b-a)*.42,
                  np.linalg.norm(c-b)*.42)
        r = cut/math.tan(angle/2)
        normal = _unit(v-u*np.dot(u, v))
        start = b-u*cut
        center = start+normal*r
        count = max(2, int(math.ceil(steps*angle/(math.pi/2))))
        for t in np.linspace(0, angle, count+1):
            out.append(center+r*(-normal*math.cos(t)+u*math.sin(t)))
    out.append(p[-1])
    out = np.asarray(out)
    return out[np.r_[True, np.linalg.norm(np.diff(out, axis=0), axis=1) > 1e-10]]


@dataclass
class Mesh:
    material: str
    points: list = field(default_factory=list)
    normals: list = field(default_factory=list)
    faces: list = field(default_factory=list)

    def tube(self, path, radius, sides=12):
        """Sweep a smooth tube with parallel-transport normals and round caps."""
        path = np.asarray(path, dtype=float)
        tangents = np.empty_like(path)
        tangents[0], tangents[-1] = _unit(path[1]-path[0]), _unit(path[-1]-path[-2])
        for i in range(1, len(path)-1):
            tangents[i] = _unit(_unit(path[i]-path[i-1])+_unit(path[i+1]-path[i]))
        reference = np.eye(3)[np.argmin(np.abs(tangents[0]))]
        u = _unit(np.cross(reference, tangents[0]))
        angles = np.arange(sides)*2*math.pi/sides
        frames = []
        for t in tangents:
            u = _unit(u-t*np.dot(u, t))
            v = np.cross(t, u)
            frames.append(np.cos(angles)[:, None]*u+np.sin(angles)[:, None]*v)
        rings, normals = [], []
        for theta in [-math.pi/3, -math.pi/6]:
            n = math.cos(theta)*frames[0]+math.sin(theta)*tangents[0]
            rings.append(path[0]+radius*n)
            normals.append(n)
        for p, frame in zip(path, frames):
            rings.append(p+radius*frame)
            normals.append(frame)
        for theta in [math.pi/6, math.pi/3]:
            n = math.cos(theta)*frames[-1]+math.sin(theta)*tangents[-1]
            rings.append(path[-1]+radius*n)
            normals.append(n)
        offset = len(self.points)
        self.points.extend(np.asarray(rings).reshape(-1, 3).tolist())
        self.normals.extend(np.asarray(normals).reshape(-1, 3).tolist())
        for i in range(len(rings)-1):
            for j in range(sides):
                a, b = offset+i*sides+j, offset+i*sides+(j+1)%sides
                c, d = offset+(i+1)*sides+(j+1)%sides, offset+(i+1)*sides+j
                self.faces.extend([(a, b, c), (a, c, d)])
        for ring, p, normal, reverse in [
            (offset, path[0]-radius*tangents[0], -tangents[0], True),
            (offset+(len(rings)-1)*sides, path[-1]+radius*tangents[-1], tangents[-1], False),
        ]:
            pole = len(self.points)
            self.points.append(p.tolist())
            self.normals.append(normal.tolist())
            for j in range(sides):
                a, b = ring+j, ring+(j+1)%sides
                self.faces.append((pole, b, a) if reverse else (pole, a, b))


def _component(mass):
    return {"mass": mass, "meshes": {}, "wires": [], "solids": [], "sites": {}}


def _wire(c, name, points, radius=.002, bend=.009, material="CoatedWire", collision=True):
    path = fillet_path(points, bend)
    # Individual wire names retain useful semantic groups and stable indices.
    mesh = c["meshes"].setdefault(name, Mesh(material))
    mesh.tube(path, radius)
    if collision:
        c["wires"].append((name, path, radius))


def _box(c, name, center, size, material="TubPlastic", collision=True):
    c["solids"].append({"kind": "box", "name": name, "center": list(center),
                         "size": list(size), "material": material, "collision": collision})


def _cylinder(c, name, center, radius, height, axis="X", material="WheelPlastic", collision=True):
    c["solids"].append({"kind": "cylinder", "name": name, "center": list(center),
                         "radius": radius, "height": height, "axis": axis,
                         "material": material, "collision": collision})
    if name.startswith(("Wheel_", "Roller_")):
        c["solids"][-1]["physics_material"]="RollerContact"


def _perimeter(c, name, halfx, front, rear, z, radius=.0024, bend=.012, material="CoatedWire"):
    # Two welded half-loops avoid a duplicated seam with a spurious end tangent.
    _wire(c, name+"Front", [(halfx, 0, z), (halfx, front, z),
                           (-halfx, front, z), (-halfx, 0, z)], radius, bend, material)
    _wire(c, name+"Rear", [(-halfx, 0, z), (-halfx, rear, z),
                          (halfx, rear, z), (halfx, 0, z)], radius, bend, material)


def lower_tine_positions():
    """Return column X and front-to-back row Y base positions in local metres.

    Counts and base-center edge margins determine both pitches at the retained
    rack footprint. Placement and inspection consume the same generated grid.
    """
    p = PARAMETERS["lower_rack"]
    m = p["tine_margins"]
    xs = np.linspace(-p["wire_width"]/2+m["left"],
                     p["wire_width"]/2-m["right"], p["tines_per_bank"])
    ys = np.linspace(-p["wire_depth"]/2+m["front"],
                     p["wire_depth"]/2-m["rear"], p["tine_banks"])
    return xs, ys


def _lower_rack():
    c = _component(3.7)
    # User estimates describe the outer rim, including its 2.4 mm radius.
    # Six transverse combs carry twelve teeth each. Their measured margins
    # leave a right-side bay for the basket at its revised seat.
    hx, hy = .27192, .28843
    for i, y in enumerate(np.linspace(-.270, .270, 21)):
        _wire(c, f"FloorCrossU_{i:02d}", [(-hx, y, .115), (-.269, y, .038),
              (-.251, y, .004), (-.237, y, 0), (.237, y, 0),
              (.251, y, .004), (.269, y, .038), (hx, y, .115)], bend=.010)
    for i, x in enumerate(np.linspace(-.248, .248, 17)):
        _wire(c, f"FloorLongU_{i:02d}", [(x, -hy, .115), (x, -hy, .025),
              (x, -.266, .004), (x, .266, .004), (x, hy, .025),
              (x, hy, .115)], bend=.011)
    _perimeter(c, "UpperRim", hx, -hy, hy, .115)
    _perimeter(c, "MidRail", hx-.001, -hy+.001, hy-.001, .055, radius=.0022)
    _perimeter(c, "LowerReinforcement", hx-.003, -hy+.003, hy-.003,
               .021, radius=.0020)
    p = PARAMETERS["lower_rack"]
    xs, ys = lower_tine_positions()
    base_z = p["tine_base_z"]
    for bank, y in enumerate(ys):
        _wire(c, f"TineBank{bank}_Base", [(xs[0], y, base_z), (xs[-1], y, base_z)], .0021)
        for index, x in enumerate(xs):
            _wire(c, f"TineBank{bank}_Tooth{index:02d}", [(x, y, base_z),
                  (x, y, base_z+.014),
                  (x+p["tine_tip_offset_x"], y, base_z+p["tine_height"])],
                  p["tine_diameter"]/2, bend=.006)
    for x in [-.241, .241]:
        _wire(c, "UnderFloorLongitudinals", [(x, -.271, -.004),
              (x, .271, -.004)], .0025)
    for y in [-.244, -.095, .095, .244]:
        _wire(c, "WheelAxleCrossRails", [(-.2693, y, -.006),
              (.2693, y, -.006)], .0023)
    for side in [-1, 1]:
        for i, y in enumerate([-.244, -.095, .095, .244]):
            x = side*.2693
            _cylinder(c, f"Wheel_{side}_{i}", (x, y, -.019), .016, .010)
            _cylinder(c, f"WheelHub_{side}_{i}", (x+side*.0054, y, -.019),
                      .006, .0018, material="BlackPlastic", collision=False)
            _wire(c, "WheelBrackets", [(side*.248, y, .004), (side*.260, y, .001),
                  (side*.261, y, -.019)], .0031, .005, "WheelPlastic", collision=False)
    _wire(c, "FrontGrip", [(-.090, -hy, .115), (.090, -hy, .115)], .0032)
    c["sites"] = {"handle_center": [0, -hy, .115],
                  "plate": [-.16129, -.140, .1333],
                  "bowl": [.191, -.140, .006],
                  "front_plate_bank": [0, float((ys[0]+ys[1])/2), base_z],
                  "rear_plate_bank": [-.064516, float((ys[-2]+ys[-1])/2), base_z],
                  "basket_seat": (np.asarray(PARAMETERS["origins"]["SilverwareBasket"])
                                  - PARAMETERS["origins"]["LowerRack"]).tolist(),
                  "front_loading_region": [.191, -.140, .006]}
    # A pose seed only: normal approximately +X matches the sideways plates.
    c["site_quats"] = {"plate": [math.cos(math.radians(41)), 0.,
                                 math.sin(math.radians(41)), 0.]}
    return c


def upper_tine_positions():
    """Return column X and front-to-back base Y positions in rack-local metres.

    The rear margin takes priority over the front margin at the unchanged rim
    depth. These same positions define geometry, loading gaps and inspection.
    """
    p = PARAMETERS["upper_rack"]
    rear_y = p["wire_depth"]/2 - p["tine_rear_margin"]
    ys = rear_y + (np.arange(p["tines_per_bank"])-(p["tines_per_bank"]-1))*p["tine_spacing"]
    return np.array(p["tine_bank_x"], dtype=float), ys


def _upper_rack():
    c = _component(2.9)
    # Five channels visible in overall_1/2 and loaded_2: sloped outer glass
    # supports, intermediate mug cradles, and the central small-dish floor.
    # Lowest floor to upper rim centerline is 130 mm, not the old 173 mm.
    hx, hy = .2518, .27212
    section = [(-hx, .112), (-.249, .061), (-.245, .002),
               (-.234, -.018), (-.153, .018), (-.143, -.018),
               (-.075, -.005), (-.065, .005), (-.056, -.010),
               (.056, -.010), (.065, .005), (.075, -.005),
               (.143, -.018), (.153, .018), (.234, -.018),
               (.245, .002), (.249, .061), (hx, .112)]
    for i, y in enumerate(np.linspace(-.251, .251, 21)):
        _wire(c, f"ContouredCrossU_{i:02d}", [(x, y, z) for x, z in section], .0019, .009)
    cross_profile = fillet_path([(x, 0, z) for x, z in section], .009)
    floor_profile = cross_profile[np.abs(cross_profile[:, 0]) < .239]
    # Nine longitudinal U-ribs provide the front uprights. A second dense
    # wall array would invent endpoints absent from the overall photograph.
    longitudinal_x = [-.234, -.196, -.143, -.075, 0., .075, .143, .196, .234]
    for i, x in enumerate(longitudinal_x):
        z = float(np.interp(x, floor_profile[:, 0], floor_profile[:, 2]))
        _wire(c, f"LongitudinalCradle_{i:02d}", [(x, -hy, .112),
              (x, -hy, .039), (x, -.251, z), (x, .251, z),
              (x, hy, .039), (x, hy, .112)], .0019, .010)
    _perimeter(c, "TopRim", hx, -hy, hy, .112, .0022, .014)
    _perimeter(c, "MidRim", .248, -hy+.001, hy-.001, .055, .0020, .013)
    p = PARAMETERS["upper_rack"]
    tine_xs, tine_ys = upper_tine_positions()
    for bank, x in enumerate(tine_xs):
        # Each rail meets the rounded floor at its own channel height. Reusing
        # the central rail's Z for the outer columns would leave floating wires.
        floor_z = float(np.interp(x, floor_profile[:, 0], floor_profile[:, 2]))
        base_z = floor_z + p["tine_base_floor_offset"]
        _wire(c, f"BowlComb{bank}_Base", [(x, -.245, base_z),
              (x, .245, base_z)], .002)
        for i, y in enumerate(tine_ys):
            _wire(c, f"BowlComb{bank}_Tooth{i:02d}", [(x, y, base_z),
                  (x, y+.002, base_z+.019),
                  (x, y+p["tine_tip_offset_y"], base_z+p["tine_height"])],
                  p["tine_diameter"]/2, .006)
    for side in [-1, 1]:
        _wire(c, f"SideRollerCarrier_{side}", [(side*.2435, -.234, .046),
              (side*.2435, .234, .046)], .0030)
        for i, y in enumerate([-.178, .178]):
            _cylinder(c, f"Roller_{side}_{i}", (side*.2595, y, .037), .0105, .008)
            _cylinder(c, f"RollerHub_{side}_{i}", (side*.2638, y, .037),
                      .0042, .001, material="BlackPlastic", collision=False)
            _cylinder(c, f"RollerAxle_{side}_{i}", (side*.249, y, .037),
                      .003, .022, material="WheelPlastic", collision=False)
    _wire(c, "FrontGrip", [(-.080, -hy, .112), (.080, -hy, .112)], .0029)
    c["sites"] = {"handle_center": [0, -hy, .112],
                  "cup": [-.109, -.174, .042],
                  "left_cup_channel": [-.109, 0, -.009],
                  "right_cup_channel": [.109, 0, -.009],
                  "left_glass_channel": [-.195, 0, -.0007],
                  "right_glass_channel": [.195, 0, -.0007],
                  "central_saucer_bank": [0, 0, -.007]}
    # Inverted cup follows the intermediate channel's approximately 11 degree
    # inward slope. Its declared reference orientation is not vertical.
    c["site_quats"] = {"cup": [0., math.cos(math.radians(5.5)), 0.,
                               math.sin(math.radians(5.5))]}
    return c


def _basket():
    c = _component(.30)
    # Two close courses form the substantial molded bottom edge and top lip.
    for name, hx, front, rear, z, r in [
        ("BottomRim", .0435, -.1825, .1825, .005, .003),
        ("BottomReinforcement", .044, -.183, .183, .014, .0023),
        ("TopRim", .052, -.192, .192, .138, .003),
        ("TopLip", .0515, -.1915, .1915, .133, .0024),
    ]:
        _perimeter(c, name, hx, front, rear, z, r, .012, "BasketPlastic")
    # Perforated bottom, not a solid collision proxy. Grid pitch is small enough
    # to catch a utensil handle while preserving drainage-sized openings.
    for i, y in enumerate(np.linspace(-.179, .179, 43)):
        _wire(c, f"BottomCrossRib_{i:02d}", [(-.042, y, .003), (.042, y, .003)],
              .0011, material="BasketPlastic")
    for i, x in enumerate(np.linspace(-.041, .041, 11)):
        _wire(c, f"BottomLongRib_{i:02d}", [(x, -.180, .004), (x, .180, .004)],
              .0011, material="BasketPlastic")
    # Both long walls lean outward. Uprights and level horizontal courses make
    # the square lattice legible from every supplied reference direction.
    for side in [-1, 1]:
        for i, y in enumerate(np.linspace(-.180, .180, 43)):
            _wire(c, f"LongWall{side}_Upright{i:02d}",
                  [(side*.0435, y, .008), (side*.052, y*1.045, .137)],
                  .0011, material="BasketPlastic")
        for i, z in enumerate(np.linspace(.021, .123, 13)):
            fraction = (z-.008)/.129
            x = side*(.0435+.0085*fraction)
            hy = .1825+.0095*fraction
            _wire(c, f"LongWall{side}_Course{i:02d}", [(x, -hy, z), (x, hy, z)],
                  .0011, material="BasketPlastic")
    for end in [-1, 1]:
        for i, x in enumerate(np.linspace(-.039, .039, 11)):
            _wire(c, f"EndWall{end}_Upright{i:02d}",
                  [(x, end*.1825, .008), (x*1.18, end*.192, .137)],
                  .0011, material="BasketPlastic")
        for i, z in enumerate(np.linspace(.021, .123, 13)):
            fraction = (z-.008)/.129
            hx = .0435+.0085*fraction
            y = end*(.1825+.0095*fraction)
            _wire(c, f"EndWall{end}_Course{i:02d}", [(-hx, y, z), (hx, y, z)],
                  .0011, material="BasketPlastic")
    for side in [-1, 1]:
        for end in [-1, 1]:
            _wire(c, f"CornerPost_{side}_{end}", [(side*.039, end*.179, .004),
                  (side*.049, end*.188, .138)], .0025, material="BasketPlastic")
    # Three visible cross partitions form four long-axis compartments.
    for j, y in enumerate([-.094, 0, .094]):
        for i, x in enumerate(np.linspace(-.041, .041, 11)):
            _wire(c, f"Partition{j}_Upright{i:02d}", [(x, y, .005),
                  (x*1.17, y, .135)], .00125, material="BasketPlastic")
        for i, z in enumerate(np.linspace(.014, .133, 15)):
            hx = .0435+.0085*(z/.138)
            _wire(c, f"Partition{j}_Course{i:02d}", [(-hx, y, z), (hx, y, z)],
                  .0012, material="BasketPlastic")
        _wire(c, f"Partition{j}_TopEdge", [(-.051, y, .136), (.051, y, .136)],
              .0021, material="BasketPlastic")
    # Molded elongated handle lies above the rear long wall. Two rounded rails
    # and short end returns enclose an actual open aperture, rather than a decal.
    handle_x = .048
    _wire(c, "HandleUpper", [(handle_x, -.151, .136), (handle_x, -.136, .148),
          (handle_x, -.130, .181), (handle_x, -.110, .1885),
          (handle_x, .110, .1885), (handle_x, .130, .181),
          (handle_x, .136, .148), (handle_x, .151, .136)],
          .0065, .012, "BasketPlastic")
    _wire(c, "HandleLower", [(handle_x, -.136, .149), (handle_x, -.100, .144),
          (handle_x, .100, .144), (handle_x, .136, .149)], .0055, .014, "BasketPlastic")
    # Strengthened molded support straps, aligned with the partition locations.
    for y in [-.094, .094]:
        _wire(c, "HandleSupportStraps", [(handle_x-.005, y, .006),
              (handle_x, y, .136)], .0029, material="BasketPlastic")
    c["sites"] = {"handle_center": [handle_x, 0, .1885],
                  "handle_aperture": [handle_x, 0, .167],
                  "utensil": [0, -.140, .13],
                  "compartment_1": [0, -.140, .015], "compartment_2": [0, -.047, .015],
                  "compartment_3": [0, .047, .015], "compartment_4": [0, .140, .015]}
    # Fit the photo's independent front and overhead silhouette ratios while
    # retaining real circular rib sections. Transforming mesh vertices alone
    # would squash the wires and disagree with capsule contact geometry.
    # The 3 mm rim radius is included in the nominal 88 by 312 mm envelope.
    sx, sy = (.044-.003)/.052, (.156-.003)/.192
    body_scale_z = (.152-.003)/.138
    handle_scale_z = (.203-.0065-.149)/(.1885-.138)

    def fit_points(points):
        fitted = np.asarray(points, dtype=float).copy()
        fitted[..., 0] *= sx
        fitted[..., 1] *= sy
        z = fitted[..., 2].copy()
        fitted[..., 2] = np.where(z <= .138, z*body_scale_z,
                                  .149+(z-.138)*handle_scale_z)
        return fitted

    original_meshes, original_wires = c["meshes"], c["wires"]
    c["meshes"], c["wires"] = {}, []
    for name, path, radius in original_wires:
        fitted = fit_points(path)
        if name in {"HandleUpper", "HandleLower"}:
            # The front reference's arch spans about 70% of the body length.
            fitted[:, 1] *= .86
            fitted[:, 0] = .0375
        c["meshes"].setdefault(name, Mesh(original_meshes[name].material)).tube(fitted, radius)
        c["wires"].append((name, fitted, radius))
    c["sites"] = {name: fit_points(point).tolist() for name, point in c["sites"].items()}
    c["sites"]["handle_center"][0] = .0375
    c["sites"]["handle_aperture"][0] = .0375
    # This fixture rests its broad head across five floor wires. A handle-down
    # placement can thread the narrow shaft through the actual open lattice.
    c["sites"]["utensil"] = [0, -.1115625, .096]
    c["site_quats"] = {"utensil": [0., 1., 0., 0.]}
    return c


def _cabinet():
    c = _component(23.0)
    _box(c, "OuterLeft", (-.296, 0, .487), (.0176, .635, .7278), "Stainless", False)
    _box(c, "OuterRight", (.296, 0, .487), (.0176, .635, .7278), "Stainless", False)
    _box(c, "OuterBack", (0, .3075, .487), (.592, .020, .7278), "BlackPlastic", False)
    _box(c, "OuterTop", (0, 0, .8419), (.6096, .635, .018), "Stainless", False)
    _box(c, "TubLeft", (-.284, .018, .4845), (.014, .552, .665))
    _box(c, "TubRight", (.284, .018, .4845), (.014, .552, .665))
    _box(c, "TubBack", (0, .3095, .4845), (.568, .016, .665))
    _box(c, "TubFloor", (0, .0165, .144), (.568, .547, .016))
    _box(c, "TubCeiling", (0, .018, .824), (.568, .552, .014))
    _box(c, "Threshold", (0, -.249, .163), (.559, .012, .022))
    # Both bases are recessed behind the sweep of the below-hinge outer skin.
    # At 90 degrees its rearmost edge is Y=-.219, leaving 9 mm of clearance.
    _box(c, "ToeKick", (0, -.185, .067), (.594, .050, .112), "BlackPlastic")
    _box(c, "Plinth", (0, .04225, .112), (.581, .5045, .046), "BlackPlastic")
    for x in [-.2693, .2693]:
        _box(c, "LowerWheelTrack"+("Left" if x < 0 else "Right"),
             (x, .014, .174), (.018, .568, .012))
        # Carrier channel bottom is immediately below the upper rollers, with
        # an upper keeper that retains the constrained installed rack.
        _box(c, "UpperRollerTrack"+("Left" if x < 0 else "Right"),
             (np.sign(x)*.2615, .021, .6115), (.023, .546, .009), "WheelPlastic")
        _box(c, "UpperRollerKeeper"+("Left" if x < 0 else "Right"),
             (np.sign(x)*.268, .021, .643), (.010, .546, .006), "WheelPlastic")
    for x in [-.248, .248]:
        for y in [-.240, .248]:
            _cylinder(c, "LevelingFoot", (x, y, .025), .018, .05, "Z", "Rubber")
    # Rounded seal follows only the loading opening and does not close it.
    _wire(c, "OpeningGasket", [(-.279, -.256, .168), (-.279, -.256, .816),
          (.279, -.256, .816), (.279, -.256, .168)], .0035, .012, "Rubber", False)
    c["sites"] = {"opening_center": [0, -.254, .4945],
                  "door_hinge": [0, -.27425, .19125],
                  "lower_rail_left": [-.2693, -.20, .180],
                  "lower_rail_right": [.2693, -.20, .180]}
    return c


def _door():
    c = _component(5.2)
    # Official FDPC4221AS CP/34VL photographs show a black fascia, a broad
    # curved pocket ABOVE the controls, and three flush buttons on the right.
    # The generic rotary-dial outline in the dimension sheet is not used.
    _box(c, "StainlessFace", (0, -.0595, .2615), (.600, .008, .593), "Stainless")
    fascia = c["meshes"].setdefault("SculptedBlackFascia", Mesh("BlackPlastic"))

    def quad(vertices):
        points = np.asarray(vertices, dtype=float)
        normal = _unit(np.cross(points[1]-points[0], points[2]-points[0]))
        offset = len(fascia.points)
        fascia.points.extend(points.tolist())
        fascia.normals.extend([normal.tolist()]*4)
        fascia.faces.extend([(offset, offset+1, offset+2), (offset, offset+2, offset+3)])

    # Piecewise-smooth upper boundary makes the actual curved opening visible
    # in silhouette, instead of drawing a handle on an unbroken front panel.
    profile = [(-.300, .674), (-.285, .674), (-.273, .670), (-.245, .658),
               (-.200, .648), (-.145, .638), (-.085, .632), (0, .630),
               (.085, .632), (.145, .638), (.200, .648), (.245, .658),
               (.273, .670), (.285, .674), (.300, .674)]
    front, back, bottom = -.0630, -.0535, .558
    for (x0, z0), (x1, z1) in zip(profile[:-1], profile[1:]):
        quad([(x0, front, bottom), (x1, front, bottom), (x1, front, z1), (x0, front, z0)])
        quad([(x0, back, z0), (x1, back, z1), (x1, back, bottom), (x0, back, bottom)])
        quad([(x0, front, z0), (x1, front, z1), (x1, back, z1), (x0, back, z0)])
        quad([(x0, back, bottom), (x1, back, bottom), (x1, front, bottom), (x0, front, bottom)])
    quad([(-.300, back, bottom), (-.300, front, bottom), (-.300, front, .674), (-.300, back, .674)])
    quad([(.300, front, bottom), (.300, back, bottom), (.300, back, .674), (.300, front, .674)])
    _box(c, "FasciaContactBase", (0, -.0585, .592), (.600, .010, .068), "BlackPlastic")
    c["solids"][-1]["visual"] = False
    for sign in [-1, 1]:
        _box(c, "FasciaEndContact"+str(sign), (sign*.292, -.0585, .650), (.016, .010, .048), "BlackPlastic")
        c["solids"][-1]["visual"] = False
    _wire(c, "CurvedPocketLowerLip", [(x, -.06165, z) for x, z in profile],
          .0018, .014, "BlackPlastic")
    _box(c, "PocketTopRail", (0, -.0585, .67695), (.600, .010, .0059), "BlackPlastic")
    _box(c, "PocketRecessBack", (0, -.034, .652), (.552, .006, .044), "BlackPlastic")
    _box(c, "SqueezeLatch", (0, -.049, .661), (.095, .014, .018), "WheelPlastic")
    _wire(c, "SqueezeLatchLowerEdge", [(-.044, -.052, .653), (0, -.054, .650),
          (.044, -.052, .653)], .002, .010, "WheelPlastic")
    # Left pocket vent bars are appearance details, as washing-system airflow
    # is outside the robot's manipulation interface.
    for i, z in enumerate([.646, .653, .660, .667]):
        _box(c, f"PocketVentHorizontal{i}", (-.190, -.048, z), (.112, .003, .0018), "BlackPlastic", False)
    for i, x in enumerate([-.238, -.200, -.164, -.134]):
        _box(c, f"PocketVentVertical{i}", (x, -.047, .656), (.0018, .003, .026), "BlackPlastic", False)
    _box(c, "DoorCore", (0, -.047, .319), (.570, .017, .624), "BlackPlastic")
    # The broad inner liner is a continuous loading support surface once open.
    _box(c, "InnerLiner", (0, -.035, .3335), (.550, .007, .660), "TubPlastic")
    for sign in [-1, 1]:
        _box(c, "InnerSideRim"+str(sign), (sign*.285, -.021, .336), (.017, .032, .672), "TubPlastic")
        # Flush strips share the recessed liner top and cannot project into
        # the deeper rack while the door is closed. After the hinge transform
        # below, their open support surface is world Z=.180.
        _box(c, "OpenDoorWheelRunway"+str(sign), (sign*.2693, -.034, .346), (.018, .005, .645), "TubPlastic")
    _box(c, "InnerUpperRim", (0, -.010, .674), (.550, .011, .010), "TubPlastic")
    _box(c, "HingeBridge", (0, -.0385, .006), (.562, .014, .012), "BlackPlastic")
    # Flush membrane surrounds and faces remain within the specified front
    # plane; these are visual controls, not invented independent actuators.
    _box(c, "ControlInset", (.112, -.06325, .591), (.350, .0004, .048), "WheelPlastic", False)
    for name, x in [("Cycles", .150), ("HeatDry", .224), ("StartCancel", .269)]:
        _box(c, "ControlOutline"+name, (x, -.06342, .591), (.021, .00012, .017), "TubPlastic", False)
        _box(c, "ControlFace"+name, (x, -.06344, .591), (.0195, .00010, .0155), "BlackPlastic", False)
    for i, z in enumerate([.597, .584]):
        _cylinder(c, f"CycleIndicator{i}", (.179, -.06345, z), .0008, .00008, "Y", "Rubber", False)
    c["sites"] = {"handle_center": [0, -.052, .661],
                  "handle_recess": [0, -.049, .640],
                  "inner_loading_surface": [0, -.0315, .3335]}
    # New hinge is 20.25 mm forward and upward. An opposite local translation
    # preserves every closed-position exterior vertex and open door depth,
    # while raising open surfaces by 40.5 mm to match the lower wheel tracks.
    shift = np.asarray([0., .02025, -.02025])
    for mesh in c["meshes"].values():
        mesh.points = (np.asarray(mesh.points)+shift).tolist()
    c["wires"] = [(name, path+shift, radius) for name, path, radius in c["wires"]]
    for solid in c["solids"]:
        solid["center"] = (np.asarray(solid["center"])+shift).tolist()
    c["sites"] = {name: (np.asarray(point)+shift).tolist()
                  for name, point in c["sites"].items()}
    return c


def build_components():
    """Return independently authorable rigid-body components and contact paths."""
    return {"Cabinet": _cabinet(), "Door": _door(), "LowerRack": _lower_rack(),
            "UpperRack": _upper_rack(), "SilverwareBasket": _basket()}
