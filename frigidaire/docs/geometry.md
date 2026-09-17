# FDPC4221AS current geometry

Coordinates are metres in USD: X left/right, −Y front, +Y rear, Z up. Rack layout
images report millimetres, with tine-base centres measured to the outer wire-rim
edge. Wheel and grip projections are outside the rim footprint.

| Measurement | Upper rack | Lower rack |
|---|---:|---:|
| Revision | `upper_tines_4x13_v1` | `lower_tines_6x12_v1` |
| Outer rim width × depth | 508.00 × 548.64 mm | 548.64 × 581.66 mm |
| Tines | 4 columns × 13 positions = 52 | 12 columns × 6 rows = 72 |
| X coordinates | −132.5, −37.5, +37.5, +132.5 mm | −194.32 + (348.64 / 11) × i mm, i=0…11 |
| Y coordinates | −173.18 + 33 × i mm, i=0…12 | −185.830 + 73.332 × i mm, i=0…5 |
| Left/right margins | 121.5 / 121.5 mm | 80 / 120 mm |
| Front/rear margins | 101.14 / 51.5 mm | 105 / 110 mm |
| Tine diameter | 3.6 mm | 3.9 mm |
| Vertical rise | 91 mm | 105 mm |
| Lean | 8 mm rearward | 8 mm rightward |

Rounded mesh wires and contact capsules share centreline paths. Upper tine base
rails follow the five contoured floor channels. The lower rack retains all 72
positions beside the removable basket.

The basket geometry is unchanged, with assembly origin `[0.2135, 0.128, 0.226]` m
and lower-rack-relative origin `[0.2135, 0.120, 0.011]` m. This is the previously
agreed 27.5 mm rightward shift. Source capsule audits measure a 5.23 mm conservative
X-envelope gap, 9.50 mm minimum tine/base-rail gap, 3.63 mm right-wall gap and
4.44 mm rear-wall gap. These are static geometry checks; basket motion and settled
support require fresh Isaac evidence.

The full appliance exterior is 609.6 × 635 × 850.9 mm (width × depth × height),
with open-door depth approximately 1250.95 mm. The dimension drawing is page 3 of
the supplied manufacturer PDF. Rack dimensions are user estimates; unmeasured
interior geometry, material parameters, mass and passive forces remain estimates.

The current collection includes annotated upper/lower layouts as PNG and SVG,
source mesh views, and `measurements.json` beside each rack's images. Those reports
include geometry-source and image hashes. Current USD authoring and assembly
simulation reports are separate.

The retained plate, bowl and cup fixture seeds intersect the polished tines.
They remain explicitly unvalidated candidates. The current example scene is
empty; earlier loaded scenes and 67-object results belong to historical geometry.
