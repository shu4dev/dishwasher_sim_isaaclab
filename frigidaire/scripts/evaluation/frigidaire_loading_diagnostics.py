"""Print first authored-collider conflicts for the finite loading candidates."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/"src"))
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "frigidaire/src"))
from dishsim_frigidaire.paths import ASSET_DIR
from dishsim_frigidaire.usd_bootstrap import ensure_usd
ensure_usd()
from dishsim_frigidaire.asset import BODY_POSITIONS, COMPONENT_FILES
from dishsim_frigidaire.loading import CollisionWorld, collision_parts, candidates
import numpy as np
import fcl

world = CollisionWorld(ASSET_DIR)
pool = candidates(world)
static = []
for body, filename in COMPONENT_FILES.items():
    parts = collision_parts(ASSET_DIR/filename)
    objects = world.transform(parts, np.eye(3), BODY_POSITIONS[body])
    static.extend(zip(parts,objects))
for kind in ("dinner_plate","salad_plate","saucer","mug","tumbler","bowl"):
    free = [c for c in pool[kind] if not world.collides(world.candidate_objects(c))]
    print(kind,"free",len(free),"of",len(pool[kind]),flush=True)
    for c in pool[kind][:3]:
        objects = world.candidate_objects(c)
        hits=[]
        for p, obj in static:
            for v in objects:
                result=fcl.CollisionResult()
                if fcl.collide(obj,v,fcl.CollisionRequest(),result):
                    hits.append(p.name)
                    break
            if len(hits)>=3:
                break
        print(c["slot"],c["variant"],c["position"],hits,flush=True)
