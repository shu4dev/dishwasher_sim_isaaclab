"""Rack-derived organized placement proposals and exact-inventory MILP search.

The candidate family is finite and never certifies global capacity. Candidate
support heights are vertical conservative advancement against authored FCL rack
geometry. A proposal must subsequently settle and pass full appliance physics.
"""
from __future__ import annotations
from collections import Counter, defaultdict
from copy import deepcopy
import hashlib
import json
import math
import time
import numpy as np
from .random_poses import compose_pose
from .organization import OrganizationGeometry, _bounds_distance, orientation_metrics, default_policy, ray_blocked
from .initial_state_candidates import InitialCollisionChecker, _frames, _pose, _graph


def _patterns(refinement_level=0):
    from .geometry import lower_tine_positions, upper_tine_positions
    from .loading import rotation, quaternion_xyzw
    lower_x, lower_y = lower_tine_positions()
    upper_x, upper_y = upper_tine_positions()
    patterns=[]
    def add(kind,rack,row,coordinate,x,y,orient,tier=0,variant=""):
        patterns.append({"kind":kind,"rack":rack,"slot_id":f"{rack}_{row}_{coordinate:.5f}",
            "position_xy_m":[float(x),float(y)],"quaternion_xyzw":quaternion_xyzw(orient),
            "generation_tier":tier,"variant":variant,
            "row_metadata":{"row_id":f"{kind}_{row}","coordinate_m":float(coordinate),
                "opening_normal_rack":orient[:,2].tolist(),"handle_direction_rack":orient[:,0].tolist()}})
    # Plates occupy the actual two outer pairs of lower tine banks.
    mids=(lower_x[:-1]+lower_x[1:])/2
    for bank,y in (("front",(lower_y[0]+lower_y[1])/2),("rear",(lower_y[-2]+lower_y[-1])/2)):
        for i,x in enumerate(mids):
            for lean in (0,-8,8):
                add("dinner_plate","LowerRack",f"plate_{bank}",x,x+.006,y,
                    rotation("Y",math.radians(90+lean)),variant=f"lean{lean}")
    # Mouth placement grids match channel widths and authored support-wire pitch.
    # The 0/15/30 degree mug families and four yaw families preserve handle grouping.
    for rack in ("UpperRack","LowerRack"):
        xs=([- .198,-.105,0.,.105,.198] if rack=="UpperRack" else [-.195,-.097,0.,.097,.195])
        ys=np.asarray([-.207,-.103,.001,.105,.209])
        for ix,x in enumerate(xs):
            for iy,y in enumerate(ys):
                for tilt,yaw in ((0,90),(0,270),(15,90),(15,270),(30,90),(30,270)):
                    orient=rotation("Y",math.radians(np.sign(x or 1)*tilt))@rotation("Z",math.radians(yaw))@rotation("X",math.pi)
                    add("mug",rack,f"channel{ix}",y,x,y,orient,variant=f"tilt{tilt}_yaw{yaw}")
        # Bowls align along actual tine gaps; steep tilts use slots spaced by 3 teeth.
        if rack=="UpperRack":
            bowlxs=[-.173,-.058,.058,.173]
            bowlys=(upper_y[:-1]+upper_y[1:])/2
        else:
            bowlxs=(lower_x[:-1]+lower_x[1:])/2
            bowlys=(lower_y[:-1]+lower_y[1:])/2
        for ix,x in enumerate(bowlxs):
            for iy,y in enumerate(bowlys):
                for tilt in (30,45,60,75):
                    for direction in (-1,1):
                        # Lower bowls align with the repeated X tine gaps; upper with Y.
                        axis="Y" if rack=="LowerRack" else "X"
                        orient=rotation(axis,math.radians(direction*(180-tilt)))
                        row=f"bowlbank{iy}" if rack=="LowerRack" else f"bowlchannel{ix}"
                        coordinate=x if rack=="LowerRack" else y
                        add("bowl",rack,row,coordinate,x,y,orient,variant=f"tilt{tilt}_dir{direction}")
    if refinement_level:
        base=list(patterns)
        # Offset along the repeating tine axis by 5 mm and broaden handle yaw.
        for p in base:
            for sign in (-1,1):
                q=deepcopy(p);q["generation_tier"]=1
                axis=0 if p["rack"]=="LowerRack" and p["kind"]!="mug" else 1
                q["position_xy_m"][axis]+=sign*.005
                q["variant"]+=f"_offset{sign*5}mm"
                patterns.append(q)
        if refinement_level>=2:
            for p in base:
                if p["kind"]!="mug":continue
                from .loading import matrix_xyzw
                for yaw in (90,-90):
                    q=deepcopy(p);q["generation_tier"]=2
                    orient=rotation("Z",math.radians(yaw))@matrix_xyzw(p["quaternion_xyzw"])
                    q["quaternion_xyzw"]=quaternion_xyzw(orient)
                    q["row_metadata"]["opening_normal_rack"]=orient[:,2].tolist()
                    q["row_metadata"]["handle_direction_rack"]=orient[:,0].tolist()
                    q["variant"]+=f"_additionalyaw{yaw}"
                    patterns.append(q)
    return patterns


def _support_pose(checker, geometry, pattern, frames, max_steps=80):
    """First rack contact under vertical translation using distance certificates.

    Distance is a lower bound on necessary translation before any contact.
    Advancing at most that distance cannot tunnel through a thin wire. The final
    positive 0.15 mm gap lets gravity establish contact in the physical run.
    """
    from .random_poses import quaternion_matrix_xyzw
    rack=pattern["rack"]; local_q=pattern["quaternion_xyzw"]
    orient=quaternion_matrix_xyzw(local_q)
    vertices=geometry.vertices[pattern["kind"]]@orient.T
    rack_rot=quaternion_matrix_xyzw(frames[rack]["quaternion_xyzw"])
    if not np.allclose(rack_rot,np.eye(3),atol=1e-5):
        raise ValueError("Support search currently requires the measured rack frame to be upright")
    z=.118-float(vertices[:,2].min())+.008
    lower_z=-.027-float(vertices[:,2].min())
    for iteration in range(max_steps):
        local=[*pattern["position_xy_m"],z]
        wp,wq=compose_pose(frames[rack]["position_m"],frames[rack]["quaternion_xyzw"],local,local_q)
        pose=_pose(wp,wq)
        body=checker._body(pattern["kind"],pose,"proposal")
        distance=geometry.distance(body,checker.components[rack])
        if distance<=.0002:
            test=checker.against_components(body)
            if not test["valid"]:return None,{"reason":test["reason"],"details":test}
            return (_pose(local,local_q),pose),{"reason":None,"support_gap_m":distance,"iterations":iteration+1,"initial_contacts":test}
        step=max(.000025,min(.025,distance-.00015))
        z-=step
        if z<lower_z:return None,{"reason":"no_rack_contact_before_floor"}
    return None,{"reason":"support_search_iteration_limit"}


def generate_catalog(asset_dir, baseline_components, seed=20260911, deadline=None,
                     progress=None, refinement_level=0, max_candidates=2400, previous_catalog=None):
    if refinement_level not in (0,1,2):raise ValueError("refinement_level must be 0, 1, or 2")
    from .tableware import CATALOG
    frames=_frames(baseline_components)
    checker=InitialCollisionChecker(asset_dir)
    checker.update_components(frames)
    geometry=OrganizationGeometry(checker=checker)
    patterns=_patterns(refinement_level)
    # Interleave kind/rack groups, so a deadline does not leave an all-plate pool.
    groups=defaultdict(list)
    for pattern in patterns:groups[(pattern["generation_tier"],pattern["kind"],pattern["rack"])].append(pattern)
    ordered=[]
    for tier in range(refinement_level+1):
        pools=[v for (t,k,r),v in sorted(groups.items()) if t==tier]
        for i in range(max(map(len,pools),default=0)):
            ordered.extend(p[i] for p in pools if i<len(p))
    started=time.monotonic();candidates=[];rejected=Counter();previous_processed=0
    if previous_catalog is not None:
        if (previous_catalog["baseline_components"] != frames or
                previous_catalog["asset_sha256"] != checker.asset_sha256 or
                previous_catalog["seed"] != int(seed)):
            raise ValueError("A resumed catalog must keep the same measured baseline, assets, and seed")
        if previous_catalog["refinement_level"] > refinement_level:
            raise ValueError("Cannot resume a catalog into a smaller refinement family")
        previous_processed = int(previous_catalog["processed_patterns"])
        if not 0 <= previous_processed <= len(ordered):
            raise ValueError("Invalid previous pattern cursor")
        candidates = deepcopy(previous_catalog["candidates"])
        if [c["candidate_index"] for c in candidates] != list(range(len(candidates))):
            raise ValueError("Resumed candidate indices must be contiguous")
        rejected = Counter(previous_catalog["rejected"])
    initial_candidate_count = len(candidates)
    result={"schema_version":1,"seed":int(seed),"baseline_components":frames,"asset_sha256":checker.asset_sha256,
            "refinement_level":refinement_level,"candidates":candidates,"complete":False,
            "generation":"rack tine/channel rows; fixed semantic orientations; exact FCL vertical support advancement",
            "pattern_count":len(patterns),"policy":default_policy(),"source_accepted_sha256":None}
    processed=previous_processed
    for ordinal,pattern in enumerate(ordered):
        if ordinal < previous_processed:continue
        if deadline is not None and time.monotonic()>=deadline:break
        if len(candidates)>=max_candidates:break
        processed+=1
        supported,detail=_support_pose(checker,geometry,pattern,frames)
        if supported is None:
            rejected[detail["reason"]]+=1
        else:
            local,world=supported
            if not orientation_metrics(pattern["kind"],pattern["rack"],world)["valid"]:
                rejected["orientation"]+=1;continue
            points=geometry.vertices[pattern["kind"]]
            from .random_poses import quaternion_matrix_xyzw
            localpoints=points@quaternion_matrix_xyzw(local["quaternion_xyzw"]).T+local["position_m"]
            # Full visual footprint must fit inside the rim; cabinet closure is still physical.
            hx,hy=(.2518,.27212) if pattern["rack"]=="UpperRack" else (.27192,.28843)
            if np.any(localpoints[:,:2].min(0)<[-hx-.001,-hy-.001]) or np.any(localpoints[:,:2].max(0)>[hx+.001,hy+.001]):
                rejected["rack_footprint"]+=1;continue
            index=len(candidates);cid=f"organized_{pattern['kind']}_{pattern['rack']}_{ordinal:05d}"
            candidates.append({**deepcopy(pattern),"candidate_index":index,"candidate_id":cid,"object_id":cid,
               "rack_local_pose":local,"pose_world":world,"mass_kg":CATALOG[pattern["kind"]]["mass_kg"],
               "size_m":CATALOG[pattern["kind"]]["size_m"],"support_search":detail,"source_trial_id":None})
        if progress and (ordinal+1)%25==0:
            progress(f"organized support {ordinal+1}/{len(ordered)}; {len(candidates)} eligible; elapsed {time.monotonic()-started:.1f}s")
    result.update(complete=processed==len(ordered),candidate_count=len(candidates),processed_patterns=processed,
                  rejected=dict(rejected),elapsed_s=time.monotonic()-started,
                  count_by_kind=dict(Counter(c["kind"] for c in candidates)))
    # A partial generated catalog is a valid explicitly smaller finite family.
    result["candidate_catalog_complete_within_processed_patterns"]=True
    result["resumed_processed_patterns"] = previous_processed
    result["new_candidate_count"] = len(candidates)-initial_candidate_count
    if previous_catalog is not None:
        result["previous_catalog_sha256"] = hashlib.sha256(json.dumps(previous_catalog,sort_keys=True,separators=(",",":")).encode()).hexdigest()
    return result


def pair_compatible(geometry,first,second,policy=None,check_opening=True):
    policy=default_policy() if policy is None else policy
    lower=_bounds_distance(first["bounds"],second["bounds"])
    if lower<policy["minimum_dish_clearance_m"]:
        if geometry.distance(first,second)<policy["minimum_dish_clearance_m"]-1e-9:
            return False,"separation"
    if lower<=0 and (geometry.nested(first,second) or geometry.nested(second,first)):
        return False,"nesting"
    if check_opening:
        for vessel,other in ((first,second),(second,first)):
            if vessel["kind"] not in ("mug","bowl"):continue
            origins,direction=geometry.opening_rays(vessel["kind"],vessel["pose"])
            ends=origins+direction*policy["opening_ray_length_m"]
            bounds=(np.minimum(origins.min(0),ends.min(0)),np.maximum(origins.max(0),ends.max(0)))
            if _bounds_distance(bounds,other["bounds"])>0:continue
            hits=ray_blocked(origins,direction,other["triangles"],policy["opening_ray_length_m"])
            if np.mean(~hits)+1e-12<policy["minimum_opening_exposure"]:
                return False,"opening_exposure"
    return True,None


def build_compatibility(catalog,asset_dir=None,deadline=None,progress=None):
    started=time.monotonic();geometry=OrganizationGeometry(asset_dir)
    candidates=catalog["candidates"]
    bodies=[geometry.body(c["kind"],c["pose_world"],c["candidate_id"]) for c in candidates]
    graph={"schema_version":1,"complete":False,"candidate_count":len(candidates),
       "allowed_indices":list(range(len(candidates))),"conflict_pairs":[],"conflict_reasons":{},
       "candidate_catalog_sha256":hashlib.sha256(json.dumps(catalog,sort_keys=True,separators=(",",":")).encode()).hexdigest(),
       "bound_scope":"finite organized candidate geometry only; individual rays can have additional aggregate obstruction",
       "tested_pairs":0,"unresolved_pairs":0}
    bounds=np.asarray([b["bounds"] for b in bodies])
    expanded=[]
    # Mouth rays can reach a dish with nonoverlapping body AABBs, so expand each
    # body's broadphase by the complete finite 100 mm ray length on all axes.
    for i,first in enumerate(bodies):
        if deadline is not None and time.monotonic()>=deadline:
            graph["allowed_indices"] = list(range(i))
            graph["conflict_pairs"] = [edge for edge in graph["conflict_pairs"] if edge[0] < i and edge[1] < i]
            graph.update(complete=True, full_catalog_complete=False, allowed_count=i,
                         elapsed_s=time.monotonic()-started,status="time_budget_exhausted_certified_prefix",
                         omitted_candidates=len(candidates)-i, bound_scope="fully checked candidate prefix only; unprocessed suffix omitted")
            return graph
        near=np.flatnonzero(np.all(bounds[i,0]-.100<=bounds[i+1:,1],axis=1)&np.all(bounds[i+1:,0]-.100<=bounds[i,1],axis=1))+i+1
        for j in near:
            if candidates[i]["slot_id"]==candidates[j]["slot_id"]:
                valid,reason=False,"same_slot"
            else:
                valid,reason=pair_compatible(geometry,first,bodies[j])
            graph["tested_pairs"]+=1
            if not valid:
                graph["conflict_pairs"].append([i,int(j)])
                graph["conflict_reasons"][reason]=graph["conflict_reasons"].get(reason,0)+1
        if progress and (i+1)%25==0:progress(f"organized graph {i+1}/{len(bodies)}; {len(graph['conflict_pairs'])} conflicts; {time.monotonic()-started:.1f}s")
    graph.update(complete=True,full_catalog_complete=True,status="complete",elapsed_s=time.monotonic()-started,allowed_count=len(bodies))
    return graph


def solve_inventory(catalog,graph,inventory,seed=0,time_limit_s=60.,excluded_sets=(),preferred_indices=()):
    """Solve exact per-kind counts; no inventory is silently removed.

    Ordered bounded objectives prefer upper mugs, fewer occupied type rows,
    common semantic orientation families, compact slot placement and seeded ties.
    The result remains a geometric proposal requiring aggregate exposure and physics.
    """
    from scipy.optimize import Bounds,LinearConstraint,milp
    from scipy.sparse import coo_matrix
    allowed,adjacency=_graph(graph)
    if not 0<time_limit_s<=60:raise ValueError("time_limit_s must be in (0,60]")
    inventory={k:int(v) for k,v in inventory.items() if v}
    if any(v<0 for v in inventory.values()) or set(inventory)-{"dinner_plate","bowl","mug"}:raise ValueError("Invalid inventory")
    allowed=[i for i in allowed if catalog["candidates"][i]["kind"] in inventory]
    n=len(allowed);lookup={i:j for j,i in enumerate(allowed)}
    groups=defaultdict(list);families=defaultdict(list)
    for i in allowed:
        c=catalog["candidates"][i]
        groups[(c["kind"],c["rack"],c["row_metadata"]["row_id"])].append(i)
        # All variants of one orientation family within a row share one active binary.
        families[(c["kind"],c["rack"],c["row_metadata"]["row_id"],_semantic_orientation_family(c))].append(i)
    auxiliary=[*groups.values(),*families.values()];nv=n+len(auxiliary)
    rows=[];cols=[];values=[];low=[];high=[]
    def constraint(indices,coeffs,lo,hi):
        row=len(low);rows.extend([row]*len(indices));cols.extend(indices);values.extend(coeffs);low.append(lo);high.append(hi)
    for a,b in graph["conflict_pairs"]:
        if a in lookup and b in lookup:constraint([lookup[a],lookup[b]],[1,1],-np.inf,1)
    for kind,count in inventory.items():
        match=[lookup[i] for i in allowed if catalog["candidates"][i]["kind"]==kind]
        constraint(match,[1]*len(match),count,count)
    cuts=[]
    for excluded in {frozenset(s) for s in excluded_sets}:
        if excluded.issubset(lookup):
            constraint([lookup[i] for i in excluded],[1]*len(excluded),-np.inf,len(excluded)-1);cuts.append(excluded)
    for offset,group in enumerate(auxiliary):
        for i in group:constraint([lookup[i],n+offset],[1,-1],-np.inf,0)
    rng=np.random.default_rng(seed);target=sum(inventory.values())
    objective=np.zeros(nv)
    # One family activation costs 1, above the entire lower-priority range
    # (< 0.71 for valid rack coordinates). One row activation dominates all
    # family activations; one preferred-rack mug dominates all active rows.
    # These dynamically scaled INTEGER units avoid the earlier division by
    # family count, which could let compactness override a family preference.
    family_weight = 1.
    row_weight = float(len(families)+1)
    upper_mug_weight = float((len(groups)+1)*row_weight+len(families)+1)
    for j,i in enumerate(allowed):
        c=catalog["candidates"][i]
        objective[j]=(-upper_mug_weight if c["kind"]=="mug" and c["rack"]=="UpperRack" else 0.)
        # Clip solely this soft score, not poses or any collision constraint.
        objective[j]+=min(.7,float(np.linalg.norm(c["position_xy_m"])))/max(1,target)
        objective[j]-=(.001/max(1,target))*(i in preferred_indices)
        objective[j]+=rng.random()*.00001/max(1,target)
    if groups:objective[n:n+len(groups)]=row_weight
    if families:objective[n+len(groups):]=family_weight
    matrix=coo_matrix((values,(rows,cols)),shape=(len(low),nv)).tocsc()
    started=time.monotonic()
    if nv==0:
        return {"selected_indices":None,"status":"empty_catalog","inventory":inventory,"count":0}
    result=milp(objective,integrality=np.ones(nv),bounds=Bounds(np.zeros(nv),np.ones(nv)),
       constraints=LinearConstraint(matrix,np.asarray(low),np.asarray(high)),
       options={"time_limit":float(time_limit_s),"mip_rel_gap":0.})
    selected=None
    if result.x is not None:
        proposed=[i for i,v in zip(allowed,result.x[:n]) if v>.5];s=frozenset(proposed)
        counts=Counter(catalog["candidates"][i]["kind"] for i in proposed)
        if counts==Counter(inventory) and s not in cuts and not any(adjacency[i]&s for i in proposed):selected=sorted(proposed)
    gap=getattr(result,"mip_gap",None);dual=getattr(result,"mip_dual_bound",None)
    return {"method":"exact_inventory_scipy_milp","selected_indices":selected,"count":len(selected or []),
       "inventory":inventory,"status":int(result.status),"message":str(result.message),"elapsed_s":time.monotonic()-started,
       "seed":int(seed),"time_limit_s":float(time_limit_s),"excluded_exact_sets":len(cuts),
       "mip_gap":float(gap) if gap is not None and np.isfinite(gap) else None,
       "mip_dual_bound":float(dual) if dual is not None and np.isfinite(dual) else None,
       "geometric_feasibility":"feasible_proposal" if selected is not None else ("infeasible_finite_catalog" if result.status==2 else "unresolved"),
       "objective":"upper mugs, occupied type rows, orientation families; then weighted compactness/preferred overlap/seeded ties",
       "objective_weights":{"upper_mug":upper_mug_weight,"occupied_type_row":row_weight,
                            "orientation_family":family_weight,"total_lower_priority_range_bound":.70101},
       "preference_scope":"lexicographic first three integer proposal objectives; geometric contiguity and clearance ranked separately on evaluated arrangements"}


def screening_order(catalog):
    """Diverse isolated-physics screening order, retaining source indices.

    Every distinct kind/rack/slot is visited before its second variant. Groups
    are interleaved so an interrupted screen still covers all supported types
    and racks. Low support heights are prioritized in 10 mm bands, spreading
    successive positions within a band; variants use exact height first and
    favor vertical plates, inverted upright mugs, and moderate bowl tilts.
    This is a proposal ordering heuristic, never a stability certificate.
    """
    candidates=catalog['candidates']
    slots=defaultdict(list)
    for index,candidate in enumerate(candidates):
        if candidate.get('candidate_index',index)!=index:
            raise ValueError('Screening requires contiguous catalog indices')
        key=(candidate['kind'],candidate['rack'],candidate['slot_id'])
        slots[key].append(index)
    def height(index):
        value=float(candidates[index]['rack_local_pose']['position_m'][2])
        if not np.isfinite(value):raise ValueError('Screening positions must be finite')
        return value
    def family(index):
        from .random_poses import quaternion_matrix_xyzw
        c=candidates[index]
        normal=quaternion_matrix_xyzw(c['rack_local_pose']['quaternion_xyzw'])[:,2]
        down=math.degrees(math.acos(float(np.clip(-normal[2],-1.,1.))))
        if c['kind']=='dinner_plate':return abs(float(normal[2]))
        if c['kind']=='mug':return down
        return abs(down-45.)
    for members in slots.values():
        members.sort(key=lambda i:(height(i),family(i),candidates[i].get('generation_tier',0),i))
    groups=defaultdict(list)
    for key,members in slots.items():groups[key[:2]].append((key,members))
    ordered_groups=[]
    for group,entries in sorted(groups.items()):
        remaining=list(entries);ordered=[];selected_positions=[]
        while remaining:
            def priority(entry):
                index=entry[1][0]
                point=np.asarray(candidates[index]['rack_local_pose']['position_m'][:2])
                separation=min((float(np.linalg.norm(point-other)) for other in selected_positions),default=0.)
                return (math.floor((height(index)+1e-10)/.01),-separation,height(index),family(index),index)
            best=min(remaining,key=priority);remaining.remove(best);ordered.append(best)
            selected_positions.append(np.asarray(candidates[best[1][0]]['rack_local_pose']['position_m'][:2]))
        ordered_groups.append(ordered)
    result=[]
    for variant in range(max((len(v) for v in slots.values()),default=0)):
        max_slots=max(map(len,ordered_groups),default=0)
        for rank in range(max_slots):
            for group in ordered_groups:
                if rank<len(group) and variant<len(group[rank][1]):result.append(group[rank][1][variant])
    if sorted(result)!=list(range(len(candidates))):raise AssertionError('Screening permutation lost candidate indices')
    return result


def _semantic_orientation_family(candidate):
    """Intended row directions survive small measured settling rotations.

    Bowls and plates have no meaningful axial spin. Plate normal sign is
    equivalent; a mug additionally retains its intended handle direction.
    """
    from .random_poses import quaternion_matrix_xyzw
    metadata=candidate.get('row_metadata',{})
    fallback=quaternion_matrix_xyzw(candidate.get('quaternion_xyzw',
        candidate.get('rack_local_pose',{}).get('quaternion_xyzw',[0.,0.,0.,1.])))
    normal=np.asarray(metadata.get('opening_normal_rack',fallback[:,2]),dtype=float)
    normal=normal/np.linalg.norm(normal)
    if candidate['kind']=='dinner_plate':
        first=next((v for v in normal if abs(v)>1e-9),1.)
        if first<0:normal=-normal
    key=list(np.round(normal,7))
    if candidate['kind']=='mug':
        handle=np.asarray(metadata.get('handle_direction_rack',fallback[:,0]),dtype=float)
        key.extend(np.round(handle/np.linalg.norm(handle),7))
    return tuple(key)
