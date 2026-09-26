# Frigidaire FDPC4221AS asset collection

The current model combines the complete dishwasher with the tape-measured 48-tine
upper rack, 64-tine lower rack and 1x4 removable basket (320 x 95 x 130 mm body,
220 mm handle) at its rear-right seat; every height is the tape height measured outside,
bottom to top (125 mm upper, 115 mm lower). Revisions: `upper_tines_4x13_v4`,
`lower_tines_6x12_v3`, `basket_1x4_320x95_v4` (2026-09-23).

Open `usd/fdpc4221as.usdc` for the appliance or `usd/example_scene.usda` for an empty,
lit scene. These entry points are created by the USD build; a staged collection
without them is incomplete. Copy the whole collection to preserve references.

| Folder | Contents |
|---|---|
| `usd/` | Assembly, five components, parameter and authoring reports; optional dish prototypes in `fixtures/` and `tableware/` |
| `images/upper_rack/` | Annotated 48-tine layout, front/oblique views, measured dimensions |
| `images/lower_rack/` | Annotated 64-tine layout, rack/basket view, basket front/top/oblique views, measured dimensions |
| `images/assembly/` | Fresh Isaac renders and assembly-only evidence, once run |
| `references/` | Supplied photographs and original manufacturer specification |
| `validation/` | Source clearance, dimension provenance, USD composition and collection records |
| `history/v1/` to `history/v4/` | Unchanged earlier models, loaded scenes, galleries and reports; v3 is the photo-fitted build archived 2026-09-22, v4 the first tape build with centreline rim heights archived 2026-09-23 with its `archive_manifest.json` and stale-results list |

## Layout and dimensions

- [Upper-rack layout and dimensions](images/upper_rack/upper_rack_overhead.png)
- [Lower-rack layout and dimensions](images/lower_rack/lower_rack_overhead.png)
- [Lower rack and basket](images/lower_rack/lower_rack_oblique.png)
- [Manufacturer dimension drawing](images/dimensions.png), rendered from page 3 of
  the [original specification](references/spec.pdf)

The annotated rack images project generated source geometry; their measurement
JSONs fingerprint the source and images. They are separate from the Isaac assembly
gallery. Rack outer-rim dimensions are 480 x 515 mm upper and 525 x 563 mm lower
(width x front-to-back depth), tape-measured on 2026-09-21 with the tine counts and
pitches; tine margins are derived from them. Unmeasured interior geometry, wire
diameters, mass, and forces remain estimates, not manufacturer CAD.

USD convention: metres, Z up, front −Y, width X; default prim
`/FrigidaireFDPC4221AS`. Door travel is 0–90 degrees, lower rack −0.49–0 m,
upper rack −0.44–0 m. The basket is an independent rigid body, with assembly origin
`[0.184, 0.104, 0.224]` m (seat `[0.184, 0.096, 0.009]` m in the lower rack).

## Validation and source

`usd/geometry_validation.json` covers authoring. `validation/composition.json`
checks composition and portability. `images/assembly/evidence.json` combines
matching assembly-only physics, render and contact reports. Missing reports mean
the corresponding work has not been completed; historical PASS reports do not
validate the current assembly.

Earlier loaded scenes, the prior 67-object capacity result and every v3 claim
manifest describe older geometry. On this assembly the HOTEC 24-piece loads settle in Isaac (runs v11, v12);
the capacity-claim layouts have an FCL dry run only. The empty
current scene does not use those historical placements.

Source and reproduction instructions are the separate `frigidaire/` folder in
the repository, with import package `dishsim_frigidaire`. The collection packager
checks current file/source hashes and requires fresh assembly evidence before
creating a release archive.
