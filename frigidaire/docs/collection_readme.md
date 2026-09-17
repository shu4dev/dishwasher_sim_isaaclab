# Frigidaire FDPC4221AS asset collection

The current model combines the complete dishwasher, polished 52-tine upper rack,
72-tine lower rack, and removable basket at the corrected mounting position.

Open `usd/fdpc4221as.usdc` for the appliance or `usd/example_scene.usda` for an empty,
lit scene. These entry points are created by the USD build; a staged collection
without them is incomplete. Copy the whole collection to preserve references.

| Folder | Contents |
|---|---|
| `usd/` | Assembly, five components, parameter and authoring reports; optional dish prototypes in `fixtures/` and `tableware/` |
| `images/upper_rack/` | Annotated 52-tine layout, front/oblique views, measured dimensions |
| `images/lower_rack/` | Annotated 72-tine layout, rack/basket view, measured dimensions |
| `images/assembly/` | Fresh Isaac renders and assembly-only evidence, once run |
| `references/` | Supplied photographs and original manufacturer specification |
| `validation/` | Source clearance, dimension provenance, USD composition and collection records |
| `history/v1/`, `history/v2/` | Unchanged earlier models, loaded scenes, galleries and reports |

## Layout and dimensions

- [Upper-rack layout and dimensions](images/upper_rack/upper_rack_overhead.png)
- [Lower-rack layout and dimensions](images/lower_rack/lower_rack_overhead.png)
- [Lower rack and basket](images/lower_rack/lower_rack_oblique.png)
- [Manufacturer dimension drawing](images/dimensions.png), rendered from page 3 of
  the [original specification](references/spec.pdf)

The annotated rack images project generated source geometry; their measurement
JSONs fingerprint the source and images. They are separate from the Isaac assembly
gallery. Rack outer-rim dimensions are 508.00 × 548.64 mm upper and
548.64 × 581.66 mm lower (width × front-to-back depth). Unmeasured interior geometry,
mass, and forces remain estimates, not manufacturer CAD.

USD convention: metres, Z up, front −Y, width X; default prim
`/FrigidaireFDPC4221AS`. Door travel is 0–90 degrees, lower rack −0.49–0 m,
upper rack −0.44–0 m. The basket is an independent rigid body, with assembly origin
`[0.2135, 0.128, 0.226]` m.

## Validation and source

`usd/geometry_validation.json` covers authoring. `validation/composition.json`
checks composition and portability. `images/assembly/evidence.json` combines
matching assembly-only physics, render and contact reports. Missing reports mean
the corresponding work has not been completed; historical PASS reports do not
validate the current assembly.

Earlier loaded scenes and the prior 67-object capacity result describe older
32/56-tine geometry. Current dish loading is unvalidated. The empty current scene
does not use those historical placements.

Source and reproduction instructions are the separate `frigidaire/` folder in
the repository, with import package `dishsim_frigidaire`. The collection packager
checks current file/source hashes and requires fresh assembly evidence before
creating a release archive.
