# Provenance: scoring section

Every rule and number in `sections/scoring.tex`, with the code or record it comes from
(repo-relative paths, line numbers as of 2026-09-26). Update this file with the text.

| Paper | Value / rule | Source |
|---|---|---|
| Occluders (eq. occluders) | both racks' wire triangles, basket, every dish's visual mesh incl. the scored one; no tub/door/cabinet | `frigidaire/src/dishsim_frigidaire/exposure.py:291` (`occluder_soup`) |
| Racks scored closed | rack origins at their pushed-in positions | `exposure.py:293-296` (`BODY_POSITIONS`) |
| Food contact (eq. food) | inward `n·rho < -0.1` or up `n_z > 0.5`, centroid above foot + 3 mm | `frigidaire/scripts/evaluation/frigidaire_hotec_exposure_search.py:104-107` |
| Rim ring (eq. rim), delta_r | food-contact vertices within 1 mm of the highest one | same file `:55` (`RIM_BAND_M`), `:109-111` |
| A_kappa | plate 0.050198, bowl 0.033996, cup 0.021536 m^2 (9120 / 12000 / 9696 food triangles) | `results/exposure/frigidaire/hotec_heightfix/search.json` `.hotec_kinds.*.food_contact_area_m2` |
| Sampling (eq. sampling), N | systematic area-CDF sampling, face centroid + face normal, weight A/N, N = 500 | `exposure.py:136-141` (`surface_samples`), `:45` (`DEFAULTS`) |
| Sunflower disc (eq. sunflower), K | `r = rho sqrt((i+.5)/K)`, angle `i pi (3 - sqrt 5)`, K = 64, u_k = 1/K | `exposure.py:169-174` (`disk_points`), `:177-187` (`rack_sources`) |
| Disc centres / radii | lower (0, .008, .185) m r .233; middle (0, .008, .540) m r .191; radius = rack half-width minus margin (0.2625 - 0.0295, 0.240 - 0.049; code comment rounds to 29 mm) | `exposure.py:52-59` (`ARM_SOURCES` + comment) |
| Rack -> arm | lower rack + basket -> lower arm; upper rack -> middle arm | `exposure.py:58-59` |
| Ceiling weight w_c | 0; disc weights become (1 - w_c)/K when > 0 | `exposure.py:46`, `:182-184` (upper rack only); point `:60` |
| epsilon | 0.1 mm start offset along the normal | `exposure.py:46` (`origin_offset_m`), `:352` |
| Segment test (eq. visibility) | ray query with max distance = distance to the source | `exposure.py:353-357`, `cast` `:316` |
| Impingement gamma (eq. cosine) | `max(n . d, 0)`, d from the lifted start | `exposure.py:356` |
| e_j (eq. sample-exposure) | weighted open share, 0 when no source faces the sample | `exposure.py:358-361` |
| E_o, S, W (eqs. object-exposure, score) | mean over samples; area-weighted mean; min | `exposure.py:416`, `:426-427` |
| Pooling (eq. pool), delta_p | interior min z < rim min z - 2 mm | `exposure.py:61`, `:190-199` |
| Feasibility | no object pools; goal search accepts a trial only if feasible and S rises > 1e-4; feasibility recorded beside S elsewhere | `exposure.py:424-428`; `frigidaire/scripts/experiment/frigidaire_bench.py:500` |
| CPU vs GPU agreement | about 1e-6 | `frigidaire/docs/exposure.md` (precision note) |
| Cheaper quantities | pre-rank = candidate alone vs rack/basket wires, no dishes, no self-occlusion, 200 x 32; trace = S at 120 x 32 | `frigidaire_bench.py:521`; `frigidaire_hotec_exposure_search.py:201-206`; `frigidaire_bench.py:988`, `:1009` |
| 1.5 bowls / 2.3 cups per plate | 502/340 = 1.48, 502/215 = 2.33 | from A_kappa above |
