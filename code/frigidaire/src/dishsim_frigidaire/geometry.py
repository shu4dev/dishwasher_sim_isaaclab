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
                "SilverwareBasket": None},   # derived below: LowerRack origin + silverware_basket.seat
    "door_front": {"fascia_material": "BlackPlastic", "fascia_bottom_z": .53775,
                   "outer_skin_bottom_z": -.05525, "base_front_y": -.210,
                   "outer_skin_to_base_sweep_clearance_min": .011018,
                   "controls": ["Cycles", "Heat Dry", "Start/Cancel"],
                   "control_type": "flush membrane, visual only", "handle": "wide recessed pocket with central squeeze latch",
                   "source": ["https://frigidaire.bynder.com/transform/XL-1400/2f590bac-50b2-4e85-9db5-5bffc5692cbc/FDPC4221AS-CP-psd",
                              "https://frigidaire.bynder.com/transform/XL-1400/fa472cfb-2998-43c6-b702-c038cb226387/FDPC4221AS-34VL-psd"],
                   "dimension_status": "proportions estimated from official model photographs; no control or latch articulation"},
    "rack_dimension_datum": "Outer wire-rim envelope; wheels, hubs and front grip projections are reported separately. Rack outer sizes, tine counts, pitches and the front/left margins are the user's tape measurements (2026-09-21, re-measured 2026-09-28); the rear/right margins are the leftover of the rim.",
    "interior_fit": {"hinge_yz": [-.27425, .19125],
                     "hinge_shift_from_first_reconstruction_yz": [-.02025, .02025],
                     "hinge_status": "inferred mechanical datum, not measured",
                     "reason": "The deeper lower rack conflicts with tall fixed door runways when closed. Recessing the liner and shifting the inferred hinge puts flush wheel strips at Z=180 mm when open while preserving the closed exterior and door-open depth.",
                     "lower_track_top_z": .180, "door_support_top_open_z": .180,
                     "tub_back_inner_y": .3015, "closed_liner_inner_y": -.2855,
                     "lower_rim_front_back_clearance": .012,
                     "lower_rim_side_clearance": .0145},
    "lower_rack": {"wire_width": .525, "wire_depth": .561, "rim_height": .1036,   # rim wire centre; outer (floor wire underside to rim top) = the tape 108 mm
                   "outer_height_tape": .108,
                   "wire_diameter": .004, "rim_diameter": .0048,
                   "floor_cross_ribs": 21, "floor_longitudinal_ribs": 17,
                   "geometry_revision": "lower_tines_6x12_v4",
                   "tine_banks": 6, "tines_per_bank": 12,
                   "tine_repetition_axis": "X",
                   "tine_pitch": {"x": .0318, "y": [.081, .073, .067, .073, .081]},   # y: the five row gaps, front to back (tape 2026-09-28)
                   "tine_margins_tape_m": {"left": .073, "right": .102, "front": .101, "rear": .113},
                   "tine_margins": None,   # derived below: left and front are the tape values, right and rear the leftover of the rim
                   "tine_margin_datum": "Tine base centers to the outer wire-rim edge; front -Y, rear +Y.",
                   "tine_margin_derivation": "Outer size, tine counts, pitches and the LEFT and FRONT tape margins are trusted (the user's counting datums); the right and rear margins are the rim's leftover. Width closes exactly (right 102.2 vs tape 102 mm); the depth is over-determined by 28 mm (rear 85 vs tape 113 mm).",
                   "tine_diameter": .0039, "tine_height": .095,
                   "tine_short_height": .045, "tine_short_rows": [2, 3],
                   "tine_tip_offset_x": .008, "tine_base_z": .006,
                   "basket_bay": {"clearance": .006,
                                  "rule": "Tines whose base center (or leaning tip, in X) lies inside the basket footprint grown by the clearance are omitted, and the base rails of those rows end at the last remaining tine, so the basket rests on the floor wires."},
                   "basket_reserved_x": None, "basket_reserved_y": None,   # derived: footprint +- clearance
                   "wheel_count": 8, "wheel_radius": .016,
                   "wheel_center_x_abs": .2693,   # set by the cabinet's LowerWheelTrack, not by the rim
                   "wheel_center_z": -.019, "estimated_mass_kg": 3.7,
                   "source": ["user tape re-measurement 2026-09-28: 52.5 W x 56.1 D x 10.8 H cm outer rim; 12 columns at 3.18 cm; 6 rows at 8.1 / 7.3 / 6.7 / 7.3 / 8.1 cm front to back; tines 9.5 cm, rows 3-4 4.5 cm; margins 7.3 left, 10.2 right, 10.1 front, 11.3 rear cm (left and front applied, see tine_margin_derivation)", "user tape measurement 2026-09-21 (superseded): 52.5 x 56.3 x 11.5 cm; 3.6 x 8 cm pitch; margins 7.7 / 10.5 / 10.5 / 12 cm", "build/frigidaire_collection/references/lower_rack/front.jpg", "build/frigidaire_collection/references/lower_rack/right.jpg", "build/frigidaire_collection/references/lower_rack/front_left.jpg", "build/frigidaire_collection/references/overall/overall_1.webp", "build/frigidaire_collection/references/loaded/loaded_2.webp", "build/frigidaire_collection/references/loaded/loaded_3.webp", "build/frigidaire_collection/references/loaded/loaded_4.webp"]},
    "upper_rack": {"wire_width": .480, "wire_depth": .515,
                   "rim_height": .1041, "lowest_floor_center_z": -.018,   # rim wire centre; outer (lowest floor wire to rim top) = the tape 125 mm
                   "outer_height_tape": .125,
                   "floor_to_rim_center_height": .1221, "floor_channels": 5,
                   "wire_diameter": .0038, "rim_diameter": .0044,
                   "floor_cross_ribs": 21, "floor_longitudinal_ribs": 9,
                   "front_uprights": 9, "front_horizontal_rails": 2,
                   "geometry_revision": "upper_tines_4x13_v5",
                   "tine_banks": 4, "tines_per_bank": 13, "tine_spacing": .033,
                   "tine_column_gaps": [.092, .086, .092],   # column to column, left to right (tape 2026-09-28)
                   "tine_repetition_axis": "Y",
                   "tine_bank_x": None,           # derived: the tape column gaps, symmetric about the centre
                   "tine_side_margin": None, "tine_front_margin": None, "tine_rear_margin": None,   # derived
                   "tine_margins_tape_m": {"side": .116, "front": .082, "rear": .044},
                   "tine_margin_datum": "Tine base centers to the outer wire-rim edge; front -Y, rear +Y.",
                   "tine_margin_derivation": "Outer size, column gaps, tine count, pitch and the FRONT tape margin are trusted; the columns are symmetric (the tape is symmetric: 11.6 both sides), so the side margin is the leftover (105 vs tape 116 mm), and the rear margin is the leftover depth (37 vs tape 44 mm).",
                   "tine_absent": {"columns": [1, 2], "indices": [5, 6],
                                   "note": "the two middle columns have 11 tines: positions 6-7 of 13 from the front are absent (gap centered as closely as the odd count allows)"},
                   "tine_diameter": .0036, "tine_height": .091,
                   "tine_tip_offset_y": .008, "tine_base_floor_offset": .003,
                   "channel_profile": {"mug_valley_offset_from_column": .0105, "ridge_offset_from_column": .0205, "ridge_z": .004,
                                       "trough_inset_from_rim": .0178, "wall_foot_inset_from_rim": .0068, "central_ridge_half_width": .053,
                                       "note": "the photo-fitted five-channel section re-fitted inside the tape rim: mug valley and ridge anchored to the outer tine column, glass slope descending outward to the trough 17.8 mm inside the rim centreline and the wall foot 6.8 mm inside it (the v3 offsets); the ridge is lowered from +18 to +4 mm so an inverted 80 mm tumbler can still stand near-upright over it"},
                   "wheel_count": 4, "wheel_radius": .0105,
                   "roller_center_x_abs": .2595, "roller_hub_x_abs": .2638,   # cabinet UpperRollerTrack coupled
                   "estimated_mass_kg": 2.9,
                   "source": ["user tape re-measurement 2026-09-28: 48 W x 51.5 D x 12.5 H cm outer rim; 4 columns 9.2 / 8.6 / 9.2 cm apart; 13 positions at 3.3 cm in the outer columns, 11 in the middle columns (two missing at the center); margins 11.6 left/right, 8.2 front, 4.4 rear cm (front applied, see tine_margin_derivation)", "user tape measurement 2026-09-21 (superseded): 4 columns 9 cm apart, 3.7 cm pitch; margins 12 / 8 / 5.5 cm", "build/frigidaire_collection/references/upper_rack/front.jpg", "build/frigidaire_collection/references/upper_rack/right.jpg", "build/frigidaire_collection/references/upper_rack/front_left.jpg", "build/frigidaire_collection/references/overall/overall_1.webp", "build/frigidaire_collection/references/overall/overall_2.webp", "build/frigidaire_collection/references/loaded/loaded_1.webp", "build/frigidaire_collection/references/loaded/loaded_2.webp", "build/frigidaire_collection/references/loaded/loaded_3.webp", "build/frigidaire_collection/references/loaded/loaded_4.webp"]},
    "silverware_basket": {"geometry_revision": "basket_1x4_320x95_v4",
                          "length_y": .320, "width_x": .095, "body_height": .130, "handle_top_z": .220,
                          "envelope_datum": "outer wire surfaces of the TOP rim including the 3 mm rim radius (the walls taper inward toward the floor); basket origin at the bottom-face center (the lowest wire surface is z = 0, so the body and handle heights are outer heights), X across the width, Y along the length, Z up",
                          "design": "the photo-fitted v3 basket (tapered lattice body, double bottom edge and top lip, corner posts, three tapered cross partitions with a top edge, elongated loop handle with an open aperture over the +X long wall on two support straps) authored at the tape dimensions",
                          "taper": {"floor_width_ratio": .848, "floor_length_ratio": .952,
                                    "source": "v3 photo fit: 74.6 x 296.9 mm floor under an 88 x 312 mm rim"},
                          "rims": {"bottom_rim_z": .0035, "bottom_reinforcement_z": .0125, "bottom_reinforcement_outset": .0005,
                                   "top_lip_below_rim": .005, "top_lip_inset": .0005},
                          "compartments": 4,
                          "compartment_layout": "1x4",   # three cross partitions at y = -L/4, 0, +L/4 (equal quarters, as the top-down photo shows)
                          "compartment_centres_xy": None,   # derived from the layout
                          "floor_lattice": {"pitch": .007, "cross_ribs": None, "long_ribs": None, "floor_top_z": .0041,
                                            "note": "7 mm square lattice (the v3 pitch); rib counts derived from the floor envelope; apertures about 4.8 mm, smaller than any catalog cutlery section"},
                          "wall_lattice": {"pitch": .007, "course_pitch": .0092, "long_wall_uprights": None, "end_wall_uprights": None, "courses": None, "partition_courses": None},
                          "wire_radii": {"rim": .003, "reinforcement": .0023, "lip": .0024, "corner_post": .0025, "lattice": .0011,
                                         "partition": .00125, "partition_course": .0012, "partition_top": .0021,
                                         "handle_upper": .0065, "handle_lower": .0055, "strap": .0029},
                          "handle": {"style": "v3 elongated loop with an open aperture over the +X long wall (outer, toward the rack's right wall)",
                                     "plane_inset_from_rim_x": .004, "foot_y_abs": .109, "flat_top_half_span": .075, "loop_height": .050,
                                     "note": "the v3 loop profile translated so its top surface is handle_top_z; the legs continue straight down to the top rim and two straps at the partition lines brace them"},
                          "seat": {"rack": "LowerRack", "origin_in_rack_m": [.184, .096, .009],
                                   "rule": "rear-right inside the right rim: the tapered bottom rim clears the sloping right wall fillet and the rear floor bend by about 8 mm each (seat re-fitted for the 95 mm wide basket); the floor lattice sits about 3 mm above the rack floor wires; the tines and base rails under the 95 x 320 mm top-rim footprint are omitted"},
                          "estimated_mass_kg": .30,
                          "utensil_candidate": {"orientation": "head down", "quat_wxyz": [0, 1, 0, 0],
                                                "site_xyz": None,   # derived: first compartment center, z .096
                                                "support": "27 mm head spans four longitudinal floor ribs at 7 mm pitch",
                                                "reason": "Narrow handle-down placement can thread through the real drainage lattice and snag."},
                          "source": ["user tape measurement 2026-09-21, axes confirmed 2026-09-22 from the front view: 32 cm long x 9.5 cm wide top rim, 13 cm body height, 22 cm to the handle top, four large open compartments in a row (three cross partitions)",
                                     "user feedback 2026-09-22: the v3 (photo-fitted) basket design at these dimensions; taper ratios, lattice pitch and loop proportions from the v3 generator",
                                     "build/frigidaire_collection/references/silverware_basket/front.jpg", "build/frigidaire_collection/references/silverware_basket/top.jpg",
                                     "build/frigidaire_collection/references/silverware_basket/top_down.jpg", "build/frigidaire_collection/references/silverware_basket/bottom.jpg"]},
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
        "Rack outer wire-rim dimensions, tine counts, pitches (the upper column gaps and the lower row gaps are non-uniform) and the front/left margins follow the user's 2026-09-28 tape re-measurement; the rear/right margins are the rim's leftover (see tine_margin_derivation; the lower depth is over-determined by 28 mm); wire diameters, absolute interior depths and clearances are still estimates.",
        "Visible four-per-side lower wheels and four basket compartments guide topology; rack tines are fixed.",
        "Upper lowest side floor is -18 mm (the mug valley 10.5 mm outside the outer tine column and the glass trough 17.8 mm inside the rim centreline), central floor is -10 mm and the rim wire centre is +104.1 mm, so the rack measures the tape 125 mm outside (lowest floor wire to rim top); the photo-fitted sloped glass channels and intermediate mug channels are kept inside the tape-measured 480 mm rim, with the ridge between them lowered to +4 mm because the 75 mm between that ridge and the wall cannot pass an 80 mm tumbler mouth at floor level.",
        "Upper front and rear wall ribs terminate on the actual filleted cross-floor rib centerlines, rather than free-hanging above the troughs.",
        "Ribs and lattice are swept round sections with rounded ends; plastic molding draft and ribs are approximated.",
        "The basket is the v3 photo-fitted design (tapered lattice body, double rims, corner posts, loop handle with an open aperture over the outer long wall) authored at the user's tape size: 320 by 95 mm top rim, 130 mm body, handle top 220 mm, four equal open compartments in a row; the taper ratios, lattice pitch, wire radii, loop proportions and the seat inside the right rim are estimates.",
        "The lower 6-by-12 tine grid keeps all 72 positions except the 8 under the basket footprint (columns 11-12 of rows 3-6: column 11's base sits 2 mm ahead of the bay but its leaning tip enters it), whose base rails end at the last remaining tine; rows 3-4 are the short 45 mm tines. The outer row pairs provide front and rear plate supports. Installed rack rollers use simplified constrained slides.",
        "The estimated tub back and inner door liner are recessed to accommodate the deeper rack targets. The inferred hinge shifts 20.25 mm forward and 20.25 mm upward so flush door wheel strips align to the 180 mm tracks; closed exterior points and door-open depth are unchanged. Upper rack and carriers are raised 50 mm from the first reconstruction.",
        "Door front topology follows official model photographs; pocket dimensions, tub sill and roller runways remain functional approximations.",
        "Stainless skin extends 55.25 mm below the revised hinge, leaving the same 13 mm closed front seam; toe kick and plinth stay recessed for the full 0–90 degree sweep.",
        "Masses, friction, inertia and concealed mechanical resistance require calibration against a physical dishwasher."],
}


def _leftover(outer, near_margin, field, what):
    """Far margin left by the near tape margin and the tine field inside the outer size; raise if it does not fit."""
    far = outer - near_margin - field
    if far <= 0:
        raise ValueError("%s: tine field of %.4f m behind a %.4f m margin does not fit inside %.4f m"
                         % (what, field, near_margin, outer))
    return far


def _derive_parameters(p):
    """Fill every value marked None: tine columns/rows from the tape margins and pitches; basket seat and cells."""
    lower, tape = p["lower_rack"], p["lower_rack"]["tine_margins_tape_m"]
    field_x = (lower["tines_per_bank"] - 1) * lower["tine_pitch"]["x"]
    field_y = float(sum(lower["tine_pitch"]["y"]))
    if len(lower["tine_pitch"]["y"]) != lower["tine_banks"] - 1:
        raise ValueError("lower_rack tine_pitch['y'] must list one gap per pair of rows")
    lower["tine_margins"] = {"left": tape["left"], "front": tape["front"],
                             "right": _leftover(lower["wire_width"], tape["left"], field_x, "lower_rack width"),
                             "rear": _leftover(lower["wire_depth"], tape["front"], field_y, "lower_rack depth")}
    upper, tape = p["upper_rack"], p["upper_rack"]["tine_margins_tape_m"]
    gaps = list(upper["tine_column_gaps"])
    if len(gaps) != upper["tine_banks"] - 1:
        raise ValueError("upper_rack tine_column_gaps must list one gap per pair of columns")
    xs = np.concatenate([[0.], np.cumsum(gaps)])
    upper["tine_bank_x"] = (xs - xs[-1] / 2).tolist()          # symmetric about the centre, as the tape is
    upper["tine_side_margin"] = _leftover(upper["wire_width"], 0., float(xs[-1]), "upper_rack width") / 2
    upper["tine_front_margin"] = tape["front"]
    upper["tine_rear_margin"] = _leftover(upper["wire_depth"], tape["front"],
                                          (upper["tines_per_bank"] - 1) * upper["tine_spacing"], "upper_rack depth")
    basket = p["silverware_basket"]
    seat = np.asarray(basket["seat"]["origin_in_rack_m"], dtype=float)
    p["origins"]["SilverwareBasket"] = (np.asarray(p["origins"]["LowerRack"], dtype=float) + seat).tolist()
    hx, hy = basket["width_x"] / 2, basket["length_y"] / 2
    if basket["compartment_layout"] == "2x2":
        basket["compartment_centres_xy"] = [[-hx / 2, -hy / 2], [hx / 2, -hy / 2], [-hx / 2, hy / 2], [hx / 2, hy / 2]]
    elif basket["compartment_layout"] == "1x4":
        basket["compartment_centres_xy"] = [[0., (k - 1.5) * basket["length_y"] / 4] for k in range(4)]
    else:
        raise ValueError("Unsupported basket compartment layout: %r" % basket["compartment_layout"])
    basket["utensil_candidate"]["site_xyz"] = [*basket["compartment_centres_xy"][0], .096]
    r_rim = basket["wire_radii"]["rim"]
    hx_bot = basket["width_x"] / 2 * basket["taper"]["floor_width_ratio"] - r_rim
    hy_bot = basket["length_y"] / 2 * basket["taper"]["floor_length_ratio"] - r_rim
    pitch = basket["floor_lattice"]["pitch"]
    basket["floor_lattice"]["cross_ribs"] = int(round(2 * (hy_bot - .003) / pitch)) + 1
    basket["floor_lattice"]["long_ribs"] = int(round(2 * (hx_bot - .003) / pitch)) + 1
    wall_pitch = basket["wall_lattice"]["pitch"]
    basket["wall_lattice"]["long_wall_uprights"] = int(round(2 * (hy_bot - .003) / wall_pitch)) + 1
    basket["wall_lattice"]["end_wall_uprights"] = int(round(2 * (hx_bot - .0045) / wall_pitch)) + 1
    z_top, course_pitch = basket["body_height"] - r_rim, basket["wall_lattice"]["course_pitch"]
    basket["wall_lattice"]["courses"] = int(round((z_top - .012 - .021) / course_pitch)) + 1
    basket["wall_lattice"]["partition_courses"] = int(round((z_top - .009 - .014) / course_pitch)) + 1
    clearance = lower["basket_bay"]["clearance"]
    lower["basket_reserved_x"] = [float(seat[0] - hx - clearance), float(seat[0] + hx + clearance)]
    lower["basket_reserved_y"] = [float(seat[1] - hy - clearance), float(seat[1] + hy + clearance)]
    return p


_derive_parameters(PARAMETERS)


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

    The outer rim size, counts, pitches and the left/front margins are the tape
    measurements: columns run from the left margin at the column pitch, rows from
    the front margin at the five tape row gaps (see
    PARAMETERS["lower_rack"]["tine_margin_derivation"]). Placement and inspection
    consume the same generated grid, including positions that the basket bay omits
    (see lower_tine_mask).
    """
    p = PARAMETERS["lower_rack"]
    m = p["tine_margins"]
    xs = -p["wire_width"]/2 + m["left"] + np.arange(p["tines_per_bank"]) * p["tine_pitch"]["x"]
    ys = -p["wire_depth"]/2 + m["front"] + np.concatenate([[0.], np.cumsum(p["tine_pitch"]["y"])])
    return xs, ys


def lower_tine_heights():
    """Vertical rise per row, front to back; the short rows are the tape-measured 45 mm ones."""
    p = PARAMETERS["lower_rack"]
    heights = np.full(p["tine_banks"], float(p["tine_height"]))
    heights[list(p["tine_short_rows"])] = p["tine_short_height"]
    return heights


def lower_basket_footprint():
    """Basket outer footprint in the lower rack frame plus the tine-omission clearance."""
    basket = PARAMETERS["silverware_basket"]
    seat = basket["seat"]["origin_in_rack_m"]
    hx, hy = basket["width_x"]/2, basket["length_y"]/2
    return {"x": (seat[0]-hx, seat[0]+hx), "y": (seat[1]-hy, seat[1]+hy),
            "clearance": PARAMETERS["lower_rack"]["basket_bay"]["clearance"]}


def lower_tine_mask():
    """Boolean (rows, columns) array; False where the basket bay omits the tine.

    A tine is omitted when its base centre, or the tip it leans toward +X, lies
    inside the basket footprint grown by the bay clearance.
    """
    xs, ys = lower_tine_positions()
    footprint = lower_basket_footprint()
    c = footprint["clearance"]
    tips = xs + PARAMETERS["lower_rack"]["tine_tip_offset_x"]
    inside_x = (tips >= footprint["x"][0]-c) & (xs <= footprint["x"][1]+c)
    inside_y = (ys >= footprint["y"][0]-c) & (ys <= footprint["y"][1]+c)
    return ~(inside_y[:, None] & inside_x[None, :])


def lower_tine_present(column, row):
    return bool(lower_tine_mask()[row, column])


def lower_plate_gaps(bank):
    """Gap indices between consecutive present tines in both rows of a plate bank."""
    rows = (0, 1) if bank == "front" else (-2, -1)
    mask = lower_tine_mask()
    both = mask[rows[0]] & mask[rows[1]]
    return [i for i in range(len(both)-1) if both[i] and both[i+1]]


def _lower_rack():
    c = _component(3.7)
    p = PARAMETERS["lower_rack"]
    # The tape measurements give the outer rim; wall bends and floor spans are
    # rim-relative insets of the photo-fitted profile so the rack scales as one.
    rim_r, rim_z = p["rim_diameter"]/2, p["rim_height"]
    hx, hy = p["wire_width"]/2-rim_r, p["wire_depth"]/2-rim_r
    floor_x, bend_x, wall_x = hx-.03492, hx-.02092, hx-.00292
    cross_y, long_x, long_turn_y = hy-.01843, hx-.02392, hy-.02243
    for i, y in enumerate(np.linspace(-cross_y, cross_y, p["floor_cross_ribs"])):
        _wire(c, f"FloorCrossU_{i:02d}", [(-hx, y, rim_z), (-wall_x, y, .038),
              (-bend_x, y, .004), (-floor_x, y, 0), (floor_x, y, 0),
              (bend_x, y, .004), (wall_x, y, .038), (hx, y, rim_z)], bend=.010)
    for i, x in enumerate(np.linspace(-long_x, long_x, p["floor_longitudinal_ribs"])):
        _wire(c, f"FloorLongU_{i:02d}", [(x, -hy, rim_z), (x, -hy, .025),
              (x, -long_turn_y, .004), (x, long_turn_y, .004), (x, hy, .025),
              (x, hy, rim_z)], bend=.011)
    _perimeter(c, "UpperRim", hx, -hy, hy, rim_z)
    _perimeter(c, "MidRail", hx-.001, -hy+.001, hy-.001, .055, radius=.0022)
    _perimeter(c, "LowerReinforcement", hx-.003, -hy+.003, hy-.003,
               .021, radius=.0020)
    xs, ys = lower_tine_positions()
    mask, heights = lower_tine_mask(), lower_tine_heights()
    base_z = p["tine_base_z"]
    for row, y in enumerate(ys):
        present = np.flatnonzero(mask[row])
        # Rows beside the basket keep their rail only as far as the last tine.
        _wire(c, f"TineBank{row}_Base", [(xs[present[0]], y, base_z),
              (xs[present[-1]], y, base_z)], .0021)
        for column in present:
            x = xs[column]
            _wire(c, f"TineBank{row}_Tooth{column:02d}", [(x, y, base_z),
                  (x, y, base_z+.014),
                  (x+p["tine_tip_offset_x"], y, base_z+heights[row])],
                  p["tine_diameter"]/2, bend=.006)
    for x in [-(hx-.03092), hx-.03092]:
        _wire(c, "UnderFloorLongitudinals", [(x, -(hy-.01743), -.004),
              (x, hy-.01743, -.004)], .0025)
    wheel_x, wheel_z = p["wheel_center_x_abs"], p["wheel_center_z"]
    wheel_ys = [-(hy-.04443), -.095, .095, hy-.04443]
    for y in wheel_ys:
        _wire(c, "WheelAxleCrossRails", [(-wheel_x, y, -.006),
              (wheel_x, y, -.006)], .0023)
    for side in [-1, 1]:
        for i, y in enumerate(wheel_ys):
            x = side*wheel_x
            _cylinder(c, f"Wheel_{side}_{i}", (x, y, wheel_z), p["wheel_radius"], .010)
            _cylinder(c, f"WheelHub_{side}_{i}", (x+side*.0054, y, wheel_z),
                      .006, .0018, material="BlackPlastic", collision=False)
            _wire(c, "WheelBrackets", [(side*(hx-.0239), y, .004), (side*(wheel_x-.0093), y, .001),
                  (side*(wheel_x-.0083), y, wheel_z)], .0031, .005, "WheelPlastic", collision=False)
    grip = p["wire_width"]/6
    _wire(c, "FrontGrip", [(-grip, -hy, rim_z), (grip, -hy, rim_z)], .0032)
    mids = (xs[:-1]+xs[1:])/2
    front_y, rear_y = float((ys[0]+ys[1])/2), float((ys[-2]+ys[-1])/2)
    rear_columns = np.flatnonzero(mask[-1])
    seat = np.asarray(PARAMETERS["silverware_basket"]["seat"]["origin_in_rack_m"], dtype=float)
    c["sites"] = {"handle_center": [0, -hy, rim_z],
                  # Plate seed: the second front gap, 15 mm behind the front bank centre
                  # (forward of the short row-3 tines, inset behind the front rim), so the
                  # 260 mm fixture disc stands 3 mm above the floor wires and clear of tines and rim.
                  "plate": [float(mids[1]), round(front_y + .015, 6), .139],
                  "bowl": [.191, -.140, .006],
                  "front_plate_bank": [0, front_y, base_z],
                  "rear_plate_bank": [float((xs[rear_columns[0]]+xs[rear_columns[-1]])/2), rear_y, base_z],
                  "basket_seat": seat.tolist(),
                  "front_loading_region": [.191, -.140, .006]}
    # A pose seed only: normal approximately +X, leaning 4 degrees with the tines.
    c["site_quats"] = {"plate": [math.cos(math.radians(47)), 0.,
                                 math.sin(math.radians(47)), 0.]}
    return c


def upper_tine_positions():
    """Return column X and front-to-back base Y positions in rack-local metres.

    Columns follow the tape column gaps, symmetric about the centre; positions run
    at the tape pitch from the tape front margin. The same grid defines geometry,
    loading gaps and inspection; absent positions come from upper_tine_mask.
    """
    p = PARAMETERS["upper_rack"]
    front_y = -p["wire_depth"]/2 + p["tine_front_margin"]
    ys = front_y + np.arange(p["tines_per_bank"])*p["tine_spacing"]
    return np.array(p["tine_bank_x"], dtype=float), ys


def upper_tine_mask():
    """Boolean (positions, columns) array; False at the absent center positions."""
    p = PARAMETERS["upper_rack"]
    mask = np.ones((p["tines_per_bank"], p["tine_banks"]), dtype=bool)
    absent = p["tine_absent"]
    for column in absent["columns"]:
        mask[list(absent["indices"]), column] = False
    return mask


def upper_tine_present(column, index):
    return bool(upper_tine_mask()[index, column])


def upper_tine_gaps(column, regular_only=True):
    """(lower, upper) position indices of consecutive present tines in one column.

    The absent center positions of the middle columns leave one wide opening;
    regular_only drops it so saucer slots stay one pitch wide.
    """
    present = np.flatnonzero(upper_tine_mask()[:, column])
    gaps = [(int(a), int(b)) for a, b in zip(present[:-1], present[1:])]
    return [g for g in gaps if g[1]-g[0] == 1] if regular_only else gaps


def tine_counts():
    return {"UpperRack": int(upper_tine_mask().sum()), "LowerRack": int(lower_tine_mask().sum())}


def _upper_rack():
    c = _component(2.9)
    p = PARAMETERS["upper_rack"]
    rim_r, rim_z = p["rim_diameter"]/2, p["rim_height"]
    hx, hy = p["wire_width"]/2-rim_r, p["wire_depth"]/2-rim_r
    tine_xs, tine_ys = upper_tine_positions()
    profile = p["channel_profile"]
    outer = float(tine_xs[-1])
    ridge, shoulder = profile["central_ridge_half_width"], profile["central_ridge_half_width"]+.019
    valley = outer+profile["mug_valley_offset_from_column"]
    crest = outer+profile["ridge_offset_from_column"]
    trough = hx-profile["trough_inset_from_rim"]
    # Five channels as in the photo fit, inside the tape rim: the central ridge
    # carrying the inner columns, the intermediate mug cradles whose valleys sit
    # 10.5 mm outside the outer tine columns, and the sloped outer glass supports
    # that descend from a low ridge to a deep trough against the rim wall. The
    # ridge is a few millimetres high (the photo-fitted crest was +18 mm): the
    # 480 mm rim leaves 75 mm between it and the wall, so an inverted 80 mm
    # tumbler stands near-upright over the ridge with the wire inside its mouth.
    half = [(ridge, -.010), (ridge+.009, .005), (shoulder, -.005),
            (valley, -.018), (crest, profile["ridge_z"]), (trough, -.018),
            (hx-profile["wall_foot_inset_from_rim"], .002), (hx-.0028, .061), (hx, rim_z)]
    section = [(-x, z) for x, z in reversed(half)] + [(x, z) for x, z in half]
    cross_y = hy-.02112
    for i, y in enumerate(np.linspace(-cross_y, cross_y, p["floor_cross_ribs"])):
        _wire(c, f"ContouredCrossU_{i:02d}", [(x, y, z) for x, z in section], .0019, .009)
    cross_profile = fillet_path([(x, 0, z) for x, z in section], .009)
    floor_profile = cross_profile[np.abs(cross_profile[:, 0]) < hx-.005]
    # Nine longitudinal U-ribs provide the front uprights: trough bottoms, the
    # glass slopes, the mug valleys, the ridge shoulders and the center.
    longitudinal_x = [-trough, -(crest+trough)/2, -valley, -shoulder, 0.,
                      shoulder, valley, (crest+trough)/2, trough]
    for i, x in enumerate(longitudinal_x):
        z = float(np.interp(x, floor_profile[:, 0], floor_profile[:, 2]))
        _wire(c, f"LongitudinalCradle_{i:02d}", [(x, -hy, rim_z),
              (x, -hy, .039), (x, -cross_y, z), (x, cross_y, z),
              (x, hy, .039), (x, hy, rim_z)], .0019, .010)
    _perimeter(c, "TopRim", hx, -hy, hy, rim_z, .0022, .014)
    _perimeter(c, "MidRim", hx-.0038, -hy+.001, hy-.001, .055, .0020, .013)
    mask = upper_tine_mask()
    rail_ys = (float(tine_ys[0]-.008), float(min(tine_ys[-1]+.008, cross_y)))
    for bank, x in enumerate(tine_xs):
        # Each rail meets the rounded floor at its own channel height.
        floor_z = float(np.interp(x, floor_profile[:, 0], floor_profile[:, 2]))
        base_z = floor_z + p["tine_base_floor_offset"]
        _wire(c, f"BowlComb{bank}_Base", [(x, rail_ys[0], base_z),
              (x, rail_ys[1], base_z)], .002)
        for i, y in enumerate(tine_ys):
            if not mask[i, bank]:
                continue
            _wire(c, f"BowlComb{bank}_Tooth{i:02d}", [(x, y, base_z),
                  (x, y+.002, base_z+.019),
                  (x, y+p["tine_tip_offset_y"], base_z+p["tine_height"])],
                  p["tine_diameter"]/2, .006)
    roller_x, hub_x = p["roller_center_x_abs"], p["roller_hub_x_abs"]
    carrier_x = hx-.0083
    for side in [-1, 1]:
        _wire(c, f"SideRollerCarrier_{side}", [(side*carrier_x, -cross_y, .046),
              (side*carrier_x, cross_y, .046)], .0030)
        for i, y in enumerate([-.178, .178]):
            _cylinder(c, f"Roller_{side}_{i}", (side*roller_x, y, .037), p["wheel_radius"], .008)
            _cylinder(c, f"RollerHub_{side}_{i}", (side*hub_x, y, .037),
                      .0042, .001, material="BlackPlastic", collision=False)
            _cylinder(c, f"RollerAxle_{side}_{i}", (side*(carrier_x+roller_x)/2, y, .037),
                      .003, roller_x-carrier_x+.003, material="WheelPlastic", collision=False)
    grip = p["wire_width"]/6
    _wire(c, "FrontGrip", [(-grip, -hy, rim_z), (grip, -hy, rim_z)], .0029)
    cup_channel, glass_channel = (shoulder+valley)/2, (crest+trough)/2
    floor_top = lambda x: float(np.interp(x, floor_profile[:, 0], floor_profile[:, 2]))+.0019
    c["sites"] = {"handle_center": [0, -hy, rim_z],
                  "cup": [-cup_channel, -.174, .042],
                  "left_cup_channel": [-cup_channel, 0, floor_top(-cup_channel)],
                  "right_cup_channel": [cup_channel, 0, floor_top(cup_channel)],
                  "left_glass_channel": [-glass_channel, 0, floor_top(-glass_channel)],
                  "right_glass_channel": [glass_channel, 0, floor_top(glass_channel)],
                  "central_saucer_bank": [0, 0, -.007]}
    # Inverted cup follows the intermediate channel's approximately 10 degree
    # outward-descending slope. Its declared reference orientation is not vertical.
    c["site_quats"] = {"cup": [0., math.cos(math.radians(5.5)), 0.,
                               math.sin(math.radians(5.5))]}
    return c


def _basket():
    p = PARAMETERS["silverware_basket"]
    c = _component(p["estimated_mass_kg"])
    r, rims, taper = p["wire_radii"], p["rims"], p["taper"]
    # The v3 photo-fitted design at the tape size: the top rim's outer surface is
    # the 95 x 320 mm envelope (130 mm body), the body tapers to the floor by the
    # v3 ratios and the lowest wire surface (the corner-post feet) is z = 0.
    hx_top, hy_top = p["width_x"]/2-r["rim"], p["length_y"]/2-r["rim"]
    hx_bot = p["width_x"]/2*taper["floor_width_ratio"]-r["rim"]
    hy_bot = p["length_y"]/2*taper["floor_length_ratio"]-r["rim"]
    z_bot, z_top = rims["bottom_rim_z"], p["body_height"]-r["rim"]

    def half_widths(z):
        f = (z-z_bot)/(z_top-z_bot)
        return hx_bot+(hx_top-hx_bot)*f, hy_bot+(hy_top-hy_bot)*f
    # Two close courses form the substantial molded bottom edge and top lip.
    reinf_z, lip_z = rims["bottom_reinforcement_z"], z_top-rims["top_lip_below_rim"]
    rx, ry = half_widths(reinf_z)
    lx, ly = half_widths(lip_z)
    outset, inset = rims["bottom_reinforcement_outset"], rims["top_lip_inset"]
    for name, hx, hy, z, radius in [("BottomRim", hx_bot, hy_bot, z_bot, r["rim"]),
                                    ("BottomReinforcement", rx+outset, ry+outset, reinf_z, r["reinforcement"]),
                                    ("TopRim", hx_top, hy_top, z_top, r["rim"]),
                                    ("TopLip", lx-inset, ly-inset, lip_z, r["lip"])]:
        _perimeter(c, name, hx, -hy, hy, z, radius, .012, "BasketPlastic")
    # Perforated bottom, not a solid collision proxy: the 7 mm grid catches a
    # utensil handle while preserving drainage-sized openings.
    lattice = p["floor_lattice"]
    rib_x, rib_y = hx_bot-.003, hy_bot-.003
    for i, y in enumerate(np.linspace(-rib_y, rib_y, lattice["cross_ribs"])):
        _wire(c, f"BottomCrossRib_{i:02d}", [(-(hx_bot-.0011), y, .0015), (hx_bot-.0011, y, .0015)],
              r["lattice"], material="BasketPlastic")
    for i, x in enumerate(np.linspace(-rib_x, rib_x, lattice["long_ribs"])):
        _wire(c, f"BottomLongRib_{i:02d}", [(x, -rib_y, .003), (x, rib_y, .003)],
              r["lattice"], material="BasketPlastic")
    # All four walls lean outward. Uprights follow the taper and the level
    # courses widen with height, so the square lattice reads from every side.
    walls = p["wall_lattice"]
    courses = np.linspace(.021, z_top-.012, walls["courses"])
    long_ys = np.linspace(-(hy_bot-.003), hy_bot-.003, walls["long_wall_uprights"])
    end_xs = np.linspace(-(hx_bot-.0045), hx_bot-.0045, walls["end_wall_uprights"])
    x_scale, y_scale = hx_top/hx_bot, hy_top/hy_bot
    for side in [-1, 1]:
        for i, y in enumerate(long_ys):
            _wire(c, f"LongWall{side}_Upright{i:02d}", [(side*hx_bot, y, z_bot+.003),
                  (side*hx_top, y*y_scale, z_top)], r["lattice"], material="BasketPlastic")
        for i, z in enumerate(courses):
            hx, hy = half_widths(z)
            _wire(c, f"LongWall{side}_Course{i:02d}", [(side*hx, -hy, z), (side*hx, hy, z)],
                  r["lattice"], material="BasketPlastic")
    for end in [-1, 1]:
        for i, x in enumerate(end_xs):
            _wire(c, f"EndWall{end}_Upright{i:02d}", [(x, end*hy_bot, z_bot+.003),
                  (x*x_scale, end*hy_top, z_top)], r["lattice"], material="BasketPlastic")
        for i, z in enumerate(courses):
            hx, hy = half_widths(z)
            _wire(c, f"EndWall{end}_Course{i:02d}", [(-hx, end*hy, z), (hx, end*hy, z)],
                  r["lattice"], material="BasketPlastic")
    for side in [-1, 1]:
        for end in [-1, 1]:
            _wire(c, f"CornerPost_{side}_{end}", [(side*(hx_bot-.004), end*(hy_bot-.004), .0025),
                  (side*(hx_top-.003), end*(hy_top-.004), z_top)], r["corner_post"], material="BasketPlastic")
    # Three tapered cross partitions form four equal compartments along the length.
    if p["compartment_layout"] != "1x4":
        raise ValueError("The v3-design basket authors cross partitions only (compartment_layout 1x4)")
    partition_top = z_top-.002
    partition_zs = np.linspace(.014, z_top-.009, walls["partition_courses"])
    for j, y in enumerate([-p["length_y"]/4, 0., p["length_y"]/4]):
        for i, x in enumerate(end_xs):
            _wire(c, f"Partition{j}_Upright{i:02d}", [(x, y, .0035), (x*x_scale, y, partition_top)],
                  r["partition"], material="BasketPlastic")
        for i, z in enumerate(partition_zs):
            hx, _ = half_widths(z)
            _wire(c, f"Partition{j}_Course{i:02d}", [(-hx, y, z), (hx, y, z)],
                  r["partition_course"], material="BasketPlastic")
        hx, _ = half_widths(partition_top)
        _wire(c, f"Partition{j}_TopEdge", [(-hx, y, partition_top), (hx, y, partition_top)],
              r["partition_top"], material="BasketPlastic")
    # Molded elongated handle above the +X long wall: two rounded rails enclose an
    # open aperture (the v3 loop), lifted so its top surface is the tape height;
    # the legs run straight down to the top rim and two straps brace them.
    handle = p["handle"]
    hxh = hx_top-handle["plane_inset_from_rim_x"]
    foot, flat = handle["foot_y_abs"], handle["flat_top_half_span"]
    top_c = p["handle_top_z"]-r["handle_upper"]
    feet_z = top_c-handle["loop_height"]
    _wire(c, "HandleUpper", [(hxh, -foot, z_top), (hxh, -foot, feet_z),
          (hxh, -(foot-.010), feet_z+.011), (hxh, -(foot-.014), feet_z+.042),
          (hxh, -flat, top_c), (hxh, flat, top_c),
          (hxh, foot-.014, feet_z+.042), (hxh, foot-.010, feet_z+.011),
          (hxh, foot, feet_z), (hxh, foot, z_top)], r["handle_upper"], .012, "BasketPlastic")
    _wire(c, "HandleLower", [(hxh, -(foot-.010), feet_z+.0125), (hxh, -(foot-.035), feet_z+.008),
          (hxh, foot-.035, feet_z+.008), (hxh, foot-.010, feet_z+.0125)],
          r["handle_lower"], .014, "BasketPlastic")
    for y in [-p["length_y"]/4, p["length_y"]/4]:
        _wire(c, "HandleSupportStraps", [(hx_bot-.002, y, .0045), (hxh, y, z_top), (hxh, y, feet_z+.009)],
              r["strap"], material="BasketPlastic")
    centres = p["compartment_centres_xy"]
    c["sites"] = {"handle_center": [hxh, 0, top_c],
                  "handle_aperture": [hxh, 0, feet_z+.0285],
                  "utensil": list(p["utensil_candidate"]["site_xyz"])}
    for k, (x, y) in enumerate(centres):
        c["sites"][f"compartment_{k+1}"] = [x, y, .010]
    c["site_quats"] = {"utensil": list(p["utensil_candidate"]["quat_wxyz"])}
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
