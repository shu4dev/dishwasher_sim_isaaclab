#!/usr/bin/env python3
# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
"""Minimal Frigidaire loader with ordered opening/closing or passive handle pulling.

    scripts/run_kit.sh frigidaire/scripts/experiment/frigidaire_demo.py --headless --mode scripted
    scripts/run_kit.sh frigidaire/scripts/experiment/frigidaire_demo.py --headless --mode passive

The passive demonstration applies a small handle torque/force instead of position
targets. Geometry and force calibration remain photo-based approximations.
"""
import argparse
import math
from pathlib import Path
import sys

from isaaclab.app import AppLauncher

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument("--mode",choices=["scripted","passive"],default="scripted")
parser.add_argument("--usd", type=Path, default=None, help="Appliance USD; defaults to the current collection")
parser.add_argument("--x",type=float,default=0.)
parser.add_argument("--y",type=float,default=0.)
parser.add_argument("--yaw",type=float,default=0.,help="Spawn yaw in degrees")
AppLauncher.add_app_launcher_args(parser)
args=parser.parse_args()
app=AppLauncher(args).app
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/"src"))
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "frigidaire/src"))

import torch
import isaaclab.sim as sim_utils
from isaaclab.sim import SimulationContext
from dishsim_frigidaire.asset import spawn,step_passive
from dishsim.media import release_sim_for_close


def main():
    sim=SimulationContext(sim_utils.SimulationCfg(dt=1/120,device=args.device))
    sim_utils.GroundPlaneCfg().func("/World/Floor",sim_utils.GroundPlaneCfg())
    sim_utils.DomeLightCfg(intensity=1800).func("/World/Light",sim_utils.DomeLightCfg(intensity=1800))
    appliance,basket=spawn("/World/Dishwasher",position=(args.x,args.y,0),yaw=math.radians(args.yaw),mode=args.mode,usd_path=args.usd)
    sim.reset()
    indices={name:appliance.joint_names.index(name) for name in appliance.joint_names}
    target=appliance.data.default_joint_pos.clone()
    dt=sim.get_physics_dt()

    def check_state(label, expected=None):
        actual={name:float(appliance.data.joint_pos[0,j]) for name,j in indices.items()}
        if not all(math.isfinite(value) for value in actual.values()) or not bool(
                torch.isfinite(basket.data.root_state_w).all()):
            raise RuntimeError(f"{label}: nonfinite articulation or basket state")
        if expected is not None:
            errors={name:actual[name]-value for name,value in expected.items()}
            if any(abs(error) > (math.radians(.5) if name=="door_hinge" else .005)
                   for name,error in errors.items()):
                raise RuntimeError(f"{label}: endpoint errors exceed tolerance: {errors}")
        return actual

    def step():
        if args.mode=="passive":
            step_passive(appliance)
        else:
            appliance.set_joint_position_target(target)
        appliance.write_data_to_sim()
        sim.step()
        appliance.update(dt)
        basket.update(dt)

    if args.mode=="scripted":
        for label,state,seconds in [("door open",{"door_hinge":math.pi/2},4.),
                              ("racks extended",{"lower_slide":-.49,"upper_slide":-.44},5.),
                              ("extended hold",{},2.),
                              ("racks retracted",{"lower_slide":0.,"upper_slide":0.},5.),
                              ("door closed",{"door_hinge":0.},4.)]:
            start=target.clone()
            for i in range(round(seconds/dt)):
                a=min(1.,(i+1)/(seconds/dt))
                blend=a*a*(3-2*a)
                for name,q in state.items():
                    target[:,indices[name]]=start[:,indices[name]]*(1-blend)+q*blend
                step()
            if state:
                for _ in range(round(1./dt)):
                    step()
            check_state(label,{name:float(target[0,j]) for name,j in indices.items()})
        print("[RESULT] PASS: scripted example passed every endpoint check",flush=True)
    else:
        # Generalized handle efforts are supplied in addition to passive resistance.
        # The caller's robot may instead apply contact forces to the same bodies.
        for i in range(12*120):
            effort=step_passive(appliance).clone()
            if appliance.data.joint_pos[0,indices["door_hinge"]] < math.radians(88):
                effort[:,indices["door_hinge"]]+=5.
            elif i>5*120:
                effort[:,indices["lower_slide"]]-=12.
            appliance.set_joint_effort_target(effort)
            appliance.write_data_to_sim()
            sim.step()
            appliance.update(dt)
            basket.update(dt)
        actual=check_state("passive handle forces")
        if actual["door_hinge"] < math.radians(85) or actual["lower_slide"] > -.02:
            raise RuntimeError(f"Passive handle forces did not open the door and extend the lower rack: {actual}")
        print("[RESULT] PASS: passive handle forces opened the door and extended the lower rack",flush=True)
    print({n:float(appliance.data.joint_pos[0,j]) for n,j in indices.items()},flush=True)


try:
    main()
except Exception as error:
    print(f"[RESULT] FAIL: {error}",flush=True)
    raise
finally:
    release_sim_for_close()
    app.close()
