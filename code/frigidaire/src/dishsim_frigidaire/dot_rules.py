# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
"""Large-N dot-rule engine (Kit-free, Warp + torch on CUDA).

The seven per-dot rules of the "Dot Score Rules" page with the 64-point sunflower replaced by N points:
for a dot start s (surface point + 1e-4 m along the normal n) over a spray-arm disc (centre C, radius R),
rel = s - C, h = rel.z:

  fixed line k   x = R*U_kx - rel.x, y = R*U_ky - rel.y, d = (x, y, -h)/L          F = all N lines
  G_theta        the fixed lines with h/L >= cos(theta)
  J_theta line k x = a*U_kx, y = a*U_ky with a = h*tan(theta), kept where |rel_xy + (x, y)| <= R
  c = max(0, n.d), v = 0 when the segment to the disc hits the occluder mesh, else 1

  O = S_F cv / S_F c     D = S_F cv / N     DG = S_G cv / N     DJ = S_J cv / N
  New = S_F cv / W*_F    NG = S_G cv / W*_G  NJ = S_J cv / W*_J  with W*_X = max_m S_X max(0, m.d)

Rays are generated in-kernel from the fp64 sunflower table (exposure.disk_points); nothing N-sized per dot
is ever stored. W* is exact for sets whose lines all lie within 45 deg of straight down (pairwise d_i.d_j >= 0
gives W* = |S d|); otherwise a screened multi-start fixed-point ascent with the dot's own normal among the
seeds, and the safety net W* >= S_X c keeps every New score <= 1. W* is solved only where the numerator
S_X cv > 0 (a zero numerator scores 0 whatever W* is); wstar(all_sets=True) solves it for every dot of a non-empty
set, blocked ones included, for the object-level denominators S_dots W* (dot scores unchanged).
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass

import numpy as np
import torch
import warp as wp

from . import exposure as E
from .asset import BODY_POSITIONS

wp.set_module_options({"enable_backward": False})

THETAS_DEG = (5, 10, 20, 30, 35, 45, 60, 70, 80, 85)
GRID_TILTS = tuple(range(0, 181, 5))
GRID_H_CM = (3, 5, 8, 11, 14, 17, 20, 23, 26)
GRID_D_CM = tuple(range(0, 36, 5))
RULES = ("O", "D", "DG", "DJ", "N", "NG", "NJ")
OFFSET_M = 1e-4
GRID_ARM_R = E.ARM_SOURCES["lower_arm"]["radius"]
NQ = wp.constant(10)
vec10 = wp.types.vector(length=10, dtype=wp.float32)
MAX_THREADS = 2 ** 31 - 1
BIND_REL = 1e-5          # safety-net binds counted above fp32 chunk-sum noise


# --------------------------------------------------------------------------- kernels

@wp.kernel
def fixed_rays(mesh: wp.uint64, cast: int, n_pts: int, chunk: int, uv: wp.array(dtype=wp.vec2d),
               rel: wp.array(dtype=wp.vec3d), nrm: wp.array(dtype=wp.vec3), start_m: wp.array(dtype=wp.vec3),
               rot_wm: wp.mat33d, radius: wp.array(dtype=wp.float64), cos_t: vec10,
               f_cv: wp.array(dtype=wp.float64), f_c: wp.array(dtype=wp.float64),
               g_cv: wp.array2d(dtype=wp.float64), g_c: wp.array2d(dtype=wp.float64),
               g_n: wp.array2d(dtype=wp.int32), rays: wp.array(dtype=wp.float64)):
    ci, i = wp.tid()
    k0 = ci * chunk
    k1 = wp.min(k0 + chunk, n_pts)
    r64 = rel[i]
    R64 = radius[i]
    rx = wp.float32(r64[0])
    ry = wp.float32(r64[1])
    h = wp.float32(r64[2])
    R = wp.float32(R64)
    n = nrm[i]
    o = start_m[i]
    acv = float(0.0)
    ac = float(0.0)
    nr = int(0)
    gn = vec10()
    gc = vec10()
    gcv = vec10()
    for k in range(k0, k1):
        U = uv[k]
        x = R * wp.float32(U[0]) - rx
        y = R * wp.float32(U[1]) - ry
        L = wp.sqrt(x * x + y * y + h * h)
        cg = h / L
        c = (n[0] * x + n[1] * y - n[2] * h) / L
        for q in range(NQ):
            if cg >= cos_t[q]:
                gn[q] = gn[q] + 1.0
        if c > 0.0:
            v = float(1.0)
            if cast != 0:
                nr += 1
                x64 = R64 * U[0] - r64[0]          # fp64 direction: bit-parity with exposure.cast
                y64 = R64 * U[1] - r64[1]
                L64 = wp.sqrt(x64 * x64 + y64 * y64 + r64[2] * r64[2])
                dl = rot_wm * wp.vec3d(x64 / L64, y64 / L64, -r64[2] / L64)
                qr = wp.mesh_query_ray(mesh, o, wp.vec3(wp.float32(dl[0]), wp.float32(dl[1]), wp.float32(dl[2])),
                                       wp.float32(L64))
                if qr.result:
                    v = 0.0
            ac += c
            acv += c * v
            for q in range(NQ):
                if cg >= cos_t[q]:
                    gc[q] = gc[q] + c
                    gcv[q] = gcv[q] + c * v
    wp.atomic_add(f_cv, i, wp.float64(acv))
    wp.atomic_add(f_c, i, wp.float64(ac))
    if nr > 0:
        wp.atomic_add(rays, i, wp.float64(nr))
    for q in range(NQ):
        if gn[q] > 0.0:
            wp.atomic_add(g_n, i, q, wp.int32(gn[q]))
            wp.atomic_add(g_c, i, q, wp.float64(gc[q]))
            wp.atomic_add(g_cv, i, q, wp.float64(gcv[q]))


@wp.kernel
def cone_rays(mesh: wp.uint64, cast: int, chunk: int, uv: wp.array(dtype=wp.vec2d),
              rel: wp.array(dtype=wp.vec3d), nrm: wp.array(dtype=wp.vec3), start_m: wp.array(dtype=wp.vec3),
              rot_wm: wp.mat33, radius: wp.array(dtype=wp.float64), pair_dot: wp.array(dtype=wp.int32),
              pair_q: wp.array(dtype=wp.int32), pair_k0: wp.array(dtype=wp.int32), pair_k1: wp.array(dtype=wp.int32),
              tan_t: wp.array(dtype=wp.float32), j_cv: wp.array2d(dtype=wp.float64), j_c: wp.array2d(dtype=wp.float64),
              j_n: wp.array2d(dtype=wp.int32), rays: wp.array(dtype=wp.float64)):
    ci, pi = wp.tid()
    k0 = wp.max(ci * chunk, pair_k0[pi])
    k1 = wp.min(ci * chunk + chunk, pair_k1[pi])
    if k0 >= k1:
        return
    i = pair_dot[pi]
    q = pair_q[pi]
    r64 = rel[i]
    h = wp.float32(r64[2])
    if h <= 0.0:
        return
    rx = wp.float32(r64[0])
    ry = wp.float32(r64[1])
    R = wp.float32(radius[i])
    a = h * tan_t[q]
    n = nrm[i]
    o = start_m[i]
    acv = float(0.0)
    ac = float(0.0)
    nr = int(0)
    cnt = int(0)
    for k in range(k0, k1):
        U = uv[k]
        x = a * wp.float32(U[0])
        y = a * wp.float32(U[1])
        ex = rx + x
        ey = ry + y
        if ex * ex + ey * ey <= R * R:
            cnt += 1
            L = wp.sqrt(x * x + y * y + h * h)
            d = wp.vec3(x / L, y / L, -h / L)
            c = wp.dot(n, d)
            if c > 0.0:
                v = float(1.0)
                if cast != 0:
                    nr += 1
                    qr = wp.mesh_query_ray(mesh, o, rot_wm * d, L)
                    if qr.result:
                        v = 0.0
                ac += c
                acv += c * v
    if cnt > 0:
        wp.atomic_add(j_n, i, q, cnt)
        wp.atomic_add(j_c, i, q, wp.float64(ac))
        wp.atomic_add(j_cv, i, q, wp.float64(acv))
        if nr > 0:
            wp.atomic_add(rays, i, wp.float64(nr))


@wp.kernel
def vsum(chunk: int, uv: wp.array(dtype=wp.vec2), kind: wp.array(dtype=wp.int32), rel: wp.array(dtype=wp.vec3),
         radius: wp.array(dtype=wp.float32), cos_t: wp.array(dtype=wp.float32), tan_t: wp.array(dtype=wp.float32),
         k0a: wp.array(dtype=wp.int32), k1a: wp.array(dtype=wp.int32), seeds: wp.array2d(dtype=wp.vec3),
         shared: int, out: wp.array3d(dtype=wp.float64)):
    ci, si, p = wp.tid()
    k0 = wp.max(ci * chunk, k0a[p])
    k1 = wp.min(ci * chunk + chunk, k1a[p])
    if k0 >= k1:
        return
    sp = p
    if shared != 0:
        sp = 0
    m = seeds[sp, si]
    r = rel[p]
    h = r[2]
    R = radius[p]
    ct = cos_t[p]
    a = h * tan_t[p]
    kd = kind[p]
    ax = float(0.0)
    ay = float(0.0)
    az = float(0.0)
    for k in range(k0, k1):
        u = uv[k]
        x = float(0.0)
        y = float(0.0)
        inside = int(1)
        if kd == 2:
            x = a * u[0]
            y = a * u[1]
            ex = r[0] + x
            ey = r[1] + y
            if ex * ex + ey * ey > R * R:
                inside = 0
        else:
            x = R * u[0] - r[0]
            y = R * u[1] - r[1]
        if inside != 0:
            L = wp.sqrt(x * x + y * y + h * h)
            if kd == 1:
                if h / L < ct:
                    inside = 0
            if inside != 0:
                dx = x / L
                dy = y / L
                dz = -h / L
                if m[0] * dx + m[1] * dy + m[2] * dz > 0.0:
                    ax += dx
                    ay += dy
                    az += dz
    wp.atomic_add(out, p, si, 0, wp.float64(ax))
    wp.atomic_add(out, p, si, 1, wp.float64(ay))
    wp.atomic_add(out, p, si, 2, wp.float64(az))


# --------------------------------------------------------------------------- host data

@dataclass
class Dots:
    rel: np.ndarray        # (P, 3) fp64, start minus disc centre (disc frame)
    nrm: np.ndarray        # (P, 3) unit normals (disc frame)
    start_m: np.ndarray    # (P, 3) ray origin in the occluder-mesh frame
    radius: np.ndarray     # (P,) disc radius
    rot_wm: np.ndarray     # (3, 3) disc-frame direction -> mesh frame


@dataclass
class Sums:
    f_cv: torch.Tensor
    f_c: torch.Tensor
    g_cv: torch.Tensor
    g_c: torch.Tensor
    g_n: torch.Tensor
    j_cv: torch.Tensor
    j_c: torch.Tensor
    j_n: torch.Tensor
    rays: torch.Tensor


def _cut(n, per_item):
    """Items per launch so that per_item * items stays under 2^31 - 1 threads."""
    return max(1, MAX_THREADS // max(1, per_item))


def k_range(kind, rel_xy_norm, h, R, a, n):
    """Conservative sunflower index range [k0, k1) that can contain set members (torch fp64 in, int32 out).
    kind 0: every index; 1 (G): disc points within distance a of the foot; 2 (J): cone points inside the disc."""
    rho = rel_xy_norm
    if kind == 0:
        z = torch.zeros_like(rho)
        return z.int(), torch.full_like(rho, n).int()
    if kind == 1:
        lo, hi = (rho - a).clamp_min(0.) / R, (rho + a) / R
    else:
        aa = a.clamp_min(1e-30)
        lo, hi = (rho - R).clamp_min(0.) / aa, (rho + R) / aa
    hi = torch.minimum(hi, torch.full_like(hi, 2.))
    k0 = torch.floor(n * lo * lo * (1 - 1e-5) - 4).clamp(0, n)
    k1 = torch.ceil(n * hi * hi * (1 + 1e-5) + 4).clamp(0, n)
    return k0.int(), k1.int()


class DotRuleEngine:
    def __init__(self, n_pts, device="cuda:0", ray_chunk=1024, w_chunk=2048, n_seeds=8, iters=12,
                 screen_dirs=256, screen_pts=512, tol=1e-7, polish=True, every_line_max=1024):
        self.N = int(n_pts)
        self.device = device
        self.tdev = torch.device(device)
        self.ray_chunk = int(min(ray_chunk, self.N))
        self.w_chunk = int(w_chunk)
        self.n_seeds, self.iters, self.tol, self.polish = n_seeds, iters, tol, polish
        self.every_line_max = every_line_max
        U = E.disk_points((0., 0., 0.), 1.0, self.N)[:, :2]
        self.uv64 = wp.array(np.ascontiguousarray(U), dtype=wp.vec2d, device=device)
        self.uv32 = wp.array(np.ascontiguousarray(U.astype(np.float32)), dtype=wp.vec2, device=device)
        self.uv32_t = torch.as_tensor(np.ascontiguousarray(U.astype(np.float32)), device=self.tdev)
        self.n_coarse = int(screen_pts)
        Uc = E.disk_points((0., 0., 0.), 1.0, self.n_coarse)[:, :2]
        self.uvc = wp.array(np.ascontiguousarray(Uc.astype(np.float32)), dtype=wp.vec2, device=device)
        i = np.arange(screen_dirs)
        z = -(i + .5) / screen_dirs
        r = np.sqrt(1. - z * z)
        ph = i * math.pi * (3. - math.sqrt(5.))
        self.sphere = torch.tensor(np.column_stack((r * np.cos(ph), r * np.sin(ph), z)), dtype=torch.float32,
                                   device=self.tdev).contiguous()
        self.down = torch.tensor([0., 0., -1.], dtype=torch.float32, device=self.tdev)
        self.stats = {"vsum_terms": 0.0, "vsum_s": 0.0, "ray_s": 0.0, "rays": 0.0}

    # ---------------------------------------------------------------- meshes and rays
    def build_mesh(self, tris, constructor="sah", leaf_size=2):
        """BVH over a triangle soup; SAH with 2-triangle leaves traverses ~1.5x faster than Warp's default
        (lbvh) on both the dish and the load soups, with bit-identical sums (measured 2026-10-09)."""
        pts = wp.array(np.ascontiguousarray(np.asarray(tris, dtype=np.float32).reshape(-1, 3)), dtype=wp.vec3,
                       device=self.device)
        idx = wp.array(np.arange(pts.shape[0], dtype=np.int32), dtype=wp.int32, device=self.device)
        kw = {}
        if constructor:
            kw["bvh_constructor"] = constructor
        if leaf_size:
            kw["bvh_leaf_size"] = leaf_size
        try:
            return wp.Mesh(points=pts, indices=idx, **kw)
        except TypeError:
            return wp.Mesh(points=pts, indices=idx)

    def ray_sums(self, mesh, dots, thetas_deg, cast=True, fixed=True):
        th = [float(t) for t in thetas_deg]
        Q = len(th)
        assert 1 <= Q <= 10
        P = len(dots.radius)
        dev = self.tdev
        cos_np = np.full(10, 2., np.float32)
        tan_np = np.zeros(10, np.float32)
        cos_np[:Q] = np.cos(np.radians(th)).astype(np.float32)
        tan_np[:Q] = np.tan(np.radians(th)).astype(np.float32)
        rel64 = torch.as_tensor(np.ascontiguousarray(dots.rel, dtype=np.float64), device=dev)
        nrm32 = torch.as_tensor(np.ascontiguousarray(dots.nrm, dtype=np.float32), device=dev)
        st32 = torch.as_tensor(np.ascontiguousarray(dots.start_m, dtype=np.float32), device=dev)
        R64 = torch.as_tensor(np.ascontiguousarray(dots.radius, dtype=np.float64), device=dev)
        f64 = dict(dtype=torch.float64, device=dev)
        s = Sums(torch.zeros(P, **f64), torch.zeros(P, **f64), torch.zeros((P, 10), **f64),
                 torch.zeros((P, 10), **f64), torch.zeros((P, 10), dtype=torch.int32, device=dev),
                 torch.zeros((P, 10), **f64), torch.zeros((P, 10), **f64),
                 torch.zeros((P, 10), dtype=torch.int32, device=dev), torch.zeros(P, **f64))
        rot = np.asarray(dots.rot_wm, dtype=np.float64)
        rot_d = wp.mat33d(*rot.reshape(-1).tolist())
        rot_f = wp.mat33(*rot.astype(np.float32).reshape(-1).tolist())
        mesh_id = wp.uint64(mesh.id) if (cast and mesh is not None) else wp.uint64(0)
        cflag = 1 if (cast and mesh is not None) else 0
        cos_v = vec10(*cos_np.tolist())
        tan_w = wp.array(tan_np, dtype=wp.float32, device=self.device)
        ch = self.ray_chunk
        nc = (self.N + ch - 1) // ch
        t0 = time.perf_counter()
        torch.cuda.synchronize()
        step = _cut(P, nc)
        for b in range(0, P, step) if fixed else ():     # fixed=False: J sets only (New+J, D+J)
            e = min(P, b + step)
            wp.launch(fixed_rays, dim=(nc, e - b), device=self.device, inputs=[
                mesh_id, cflag, self.N, ch, self.uv64, wp.from_torch(rel64[b:e], dtype=wp.vec3d),
                wp.from_torch(nrm32[b:e], dtype=wp.vec3), wp.from_torch(st32[b:e], dtype=wp.vec3), rot_d,
                wp.from_torch(R64[b:e]), cos_v, wp.from_torch(s.f_cv[b:e]), wp.from_torch(s.f_c[b:e]),
                wp.from_torch(s.g_cv[b:e]), wp.from_torch(s.g_c[b:e]), wp.from_torch(s.g_n[b:e]),
                wp.from_torch(s.rays[b:e])])
        # J pairs, theta-major, dots ascending; conservative index ranges
        h = rel64[:, 2]
        rho = rel64[:, :2].norm(dim=1)
        pd, pq, p0, p1 = [], [], [], []
        for q in range(Q):
            idx = torch.nonzero(h > 0).flatten()
            if idx.numel() == 0:
                continue
            a = h[idx] * float(tan_np[q])
            k0, k1 = k_range(2, rho[idx], h[idx], R64[idx], a, self.N)
            ok = k1 > k0
            pd.append(idx[ok].int()); pq.append(torch.full((int(ok.sum()),), q, dtype=torch.int32, device=dev))
            p0.append(k0[ok]); p1.append(k1[ok])
        if pd:
            pd, pq, p0, p1 = (torch.cat(v).contiguous() for v in (pd, pq, p0, p1))
            npairs = pd.numel()
            step = _cut(npairs, nc)
            torch.cuda.synchronize()
            for b in range(0, npairs, step):
                e = min(npairs, b + step)
                wp.launch(cone_rays, dim=(nc, e - b), device=self.device, inputs=[
                    mesh_id, cflag, ch, self.uv64, wp.from_torch(rel64, dtype=wp.vec3d),
                    wp.from_torch(nrm32, dtype=wp.vec3), wp.from_torch(st32, dtype=wp.vec3), rot_f,
                    wp.from_torch(R64), wp.from_torch(pd[b:e]), wp.from_torch(pq[b:e]), wp.from_torch(p0[b:e]),
                    wp.from_torch(p1[b:e]), tan_w, wp.from_torch(s.j_cv), wp.from_torch(s.j_c),
                    wp.from_torch(s.j_n), wp.from_torch(s.rays)])
        wp.synchronize_device(self.device)
        self.stats["ray_s"] += time.perf_counter() - t0
        self.stats["rays"] += float(s.rays.sum())
        for name in ("g_cv", "g_c", "g_n", "j_cv", "j_c", "j_n"):
            setattr(s, name, getattr(s, name)[:, :Q].contiguous())
        return s

    # ---------------------------------------------------------------- W* search
    def _vsum(self, kind, rel32, R32, cos32, tan32, k0, k1, seeds, shared, uv, n_uv):
        P = rel32.shape[0]
        S = seeds.shape[1]
        out = torch.zeros((P, S, 3), dtype=torch.float64, device=self.tdev)
        if P == 0:
            return out
        ch = int(min(self.w_chunk, n_uv))
        nc = (n_uv + ch - 1) // ch
        step = _cut(P, nc * S)
        t0 = time.perf_counter()
        torch.cuda.synchronize()
        for b in range(0, P, step):
            e = min(P, b + step)
            sd = seeds if shared else seeds[b:e]
            wp.launch(vsum, dim=(nc, S, e - b), device=self.device, inputs=[
                ch, uv, wp.from_torch(kind[b:e]), wp.from_torch(rel32[b:e], dtype=wp.vec3), wp.from_torch(R32[b:e]),
                wp.from_torch(cos32[b:e]), wp.from_torch(tan32[b:e]), wp.from_torch(k0[b:e]), wp.from_torch(k1[b:e]),
                wp.from_torch(sd.contiguous(), dtype=wp.vec3), int(shared), wp.from_torch(out[b:e])])
        wp.synchronize_device(self.device)
        self.stats["vsum_s"] += time.perf_counter() - t0
        self.stats["vsum_terms"] += float((k1 - k0).clamp_min(0).double().sum()) * S
        return out

    def best_facing(self, pr, nrm32, certified):
        """W* and its direction for a problem table ``pr`` (torch tensors on device)."""
        P = pr["kind"].numel()
        dev = self.tdev
        if P == 0:
            return torch.zeros(0, dtype=torch.float64, device=dev), torch.zeros((0, 3), dtype=torch.float32, device=dev)
        args = (pr["kind"], pr["rel32"], pr["R32"], pr["cos32"], pr["tan32"])
        if certified:
            V = self._vsum(*args, pr["k0"], pr["k1"], self.down.view(1, 1, 3), 1, self.uv32, self.N)[:, 0]
            W = V.norm(dim=1)
            return W, (V / W.clamp_min(1e-300)[:, None]).float()
        # 1. screen on a coarse sample of the same region
        best_dirs = []
        for b in range(0, P, 8192):
            e = min(P, b + 8192)
            sk = pr["screen_kind"][b:e]
            V = self._vsum(sk, pr["rel32"][b:e], pr["R32"][b:e], pr["cos32"][b:e], pr["tan32"][b:e],
                           pr["sk0"][b:e], pr["sk1"][b:e], self.sphere.view(1, -1, 3), 1, self.uvc, self.n_coarse)
            Wsc = (V * self.sphere.double()[None]).sum(-1)
            best_dirs.append(Wsc.topk(4, dim=1).indices)
        d4 = self.sphere[torch.cat(best_dirs)]                         # (P, 4, 3)
        top = d4[:, 0]
        rot = []
        for ang in (2 * math.pi / 3, -2 * math.pi / 3):
            c, s_ = math.cos(ang), math.sin(ang)
            rot.append(torch.stack((c * top[:, 0] - s_ * top[:, 1], s_ * top[:, 0] + c * top[:, 1], top[:, 2]), 1))
        m = torch.cat([nrm32[:, None], self.down.expand(P, 1, 3), d4, rot[0][:, None], rot[1][:, None]], 1)
        best = torch.zeros(P, dtype=torch.float64, device=dev)
        bestm = m[:, 1].clone()
        self._ascend(pr, args, m.contiguous(), best, bestm)
        if self.N <= self.every_line_max:     # small sets: the page's own start set (every line of the set)
            for b in range(0, P, 4096):
                e = min(P, b + 4096)
                self._ascend(pr, args, self._set_lines(pr, b, e), best, bestm, torch.arange(b, e, device=dev))
        if self.polish:                       # hop between sunflower-ripple optima around the best direction
            b = bestm
            seeds = []
            for deg in (1., -1., 2., -2., 4., -4., 8., -8.) + tuple(range(30, 360, 30)):
                c, s_ = math.cos(math.radians(deg)), math.sin(math.radians(deg))
                seeds.append(torch.stack((c * b[:, 0] - s_ * b[:, 1], s_ * b[:, 0] + c * b[:, 1], b[:, 2]), 1))
            az = b[:, :2] / b[:, :2].norm(dim=1, keepdim=True).clamp_min(1e-12)
            k = torch.stack((-az[:, 1], az[:, 0], torch.zeros_like(az[:, 0])), 1)
            kxm = torch.cross(k, b, dim=1)
            for deg in (2., -2., 5., -5.):
                seeds.append(b * math.cos(math.radians(deg)) + kxm * math.sin(math.radians(deg)))
            m2 = torch.stack(seeds, 1)
            m2 = (m2 / m2.norm(dim=2, keepdim=True).clamp_min(1e-12)).contiguous()
            self._ascend(pr, args, m2, best, bestm)
        return best, bestm

    def _set_lines(self, pr, b, e):
        """(e-b, N, 3) unit directions of every line of problems b:e (non-members -> straight down)."""
        U = self.uv32_t
        rel, R = pr["rel32"][b:e], pr["R32"][b:e, None]
        kind, ct, tt = pr["kind"][b:e, None], pr["cos32"][b:e, None], pr["tan32"][b:e, None]
        h = rel[:, 2:3]
        a = h * tt
        xJ, yJ = a * U[None, :, 0], a * U[None, :, 1]
        xF, yF = R * U[None, :, 0] - rel[:, 0:1], R * U[None, :, 1] - rel[:, 1:2]
        isJ = kind == 2
        x, y = torch.where(isJ, xJ, xF), torch.where(isJ, yJ, yF)
        L = torch.sqrt(x * x + y * y + h * h)
        member = torch.where(isJ, (rel[:, 0:1] + xJ) ** 2 + (rel[:, 1:2] + yJ) ** 2 <= R * R,
                             torch.where(kind == 1, h / L >= ct, torch.ones_like(L, dtype=torch.bool)))
        D = torch.stack((x / L, y / L, -h / L), 2)
        D = torch.where(member[..., None], D, self.down.view(1, 1, 3))
        return D.contiguous()

    def _ascend(self, pr, args, m, best, bestm, rows=None):
        idx = torch.arange(m.shape[0], device=self.tdev) if rows is None else rows
        for _ in range(self.iters):
            sub = [t[idx] for t in args]
            V = self._vsum(*sub, pr["k0"][idx], pr["k1"][idx], m, 0, self.uv32, self.N)
            nV = V.norm(dim=2)
            w, si = nV.max(1)
            up = w > best[idx]
            if bool(up.any()):
                iu = idx[up]
                best[iu] = w[up]
                bestm[iu] = (V[up, si[up]] / w[up, None]).float()
            new = torch.where(nV[..., None] > 0, V / nV[..., None].clamp_min(1e-300), m.double()).float()
            keep = (new - m).abs().amax((1, 2)) > self.tol
            m, idx = new[keep].contiguous(), idx[keep]
            if idx.numel() == 0:
                break

    def wstar(self, dots, sums, thetas_deg, all_f=False, all_sets=False):
        """(WF (P,), WG (P,Q), WJ (P,Q), mF (P,3) fp32, diag). W* where S cv > 0; all_f: every F problem;
        all_sets: every dot of a non-empty set (F, G and J), blocked dots included (object-level denominators)."""
        all_f = all_f or all_sets
        t0 = time.perf_counter()
        dev = self.tdev
        th = [float(t) for t in thetas_deg]
        Q = len(th)
        rel64 = torch.as_tensor(np.ascontiguousarray(dots.rel, dtype=np.float64), device=dev)
        rel32 = rel64.float().contiguous()
        R64 = torch.as_tensor(np.ascontiguousarray(dots.radius, dtype=np.float64), device=dev)
        R32 = R64.float().contiguous()
        nrm32 = torch.as_tensor(np.ascontiguousarray(dots.nrm, dtype=np.float32), device=dev)
        P = rel64.shape[0]
        h = rel64[:, 2]
        rho = rel64[:, :2].norm(dim=1)
        WF = torch.zeros(P, dtype=torch.float64, device=dev)
        WG = torch.zeros((P, Q), dtype=torch.float64, device=dev)
        WJ = torch.zeros((P, Q), dtype=torch.float64, device=dev)
        mF = torch.zeros((P, 3), dtype=torch.float32, device=dev)
        mG = torch.zeros((P, Q, 3), dtype=torch.float32, device=dev)
        mJ = torch.zeros((P, Q, 3), dtype=torch.float32, device=dev)
        diag = {"certified": 0, "uncertified": 0, "skipped": 0, "reused": 0, "binds": 0, "max_bind_rel": 0.0}
        # problem rows: (dot, kind, q, cos32, tan32)
        rows = {True: [], False: []}

        def add(idx, kind, q, cert):
            for flag in (True, False):
                sel = idx[cert == flag]
                if sel.numel():
                    rows[flag].append((sel, kind, q))

        fsel = torch.arange(P, device=dev) if all_f else torch.nonzero(sums.f_cv > 0).flatten()
        diag["skipped"] += P - fsel.numel()
        add(fsel, 0, -1, (R64[fsel] + rho[fsel]) <= h[fsel])
        gfull = []
        for q in range(Q):
            a = h * math.tan(math.radians(th[q]))
            for kind, n_, cv in ((1, sums.g_n[:, q], sums.g_cv[:, q]), (2, sums.j_n[:, q], sums.j_cv[:, q])):
                live = (n_ > 0) & (h > 0) if all_sets else (n_ > 0) & (cv > 0) & (h > 0)
                diag["skipped"] += int(((n_ > 0) & ~live).sum())
                if kind == 1:
                    full = live & (n_ == self.N)
                    gfull.append(full)
                    diag["reused"] += int(full.sum())
                    live = live & ~full
                idx = torch.nonzero(live).flatten()
                reach = torch.minimum(a[idx], R64[idx] + rho[idx])
                cert = (reach <= h[idx]) | (th[q] <= 45.0 + 1e-9)
                add(idx, kind, q, cert)
        cos32_all = np.cos(np.radians(th)).astype(np.float32)
        tan32_all = np.tan(np.radians(th)).astype(np.float32)

        def table(parts):
            dot = torch.cat([p[0] for p in parts])
            kind = torch.cat([torch.full((p[0].numel(),), p[1], dtype=torch.int32, device=dev) for p in parts])
            qq = torch.cat([torch.full((p[0].numel(),), p[2], dtype=torch.int64, device=dev) for p in parts])
            c32 = torch.tensor(np.append(cos32_all, np.float32(2.)), device=dev)[qq]
            t32 = torch.tensor(np.append(tan32_all, np.float32(0.)), device=dev)[qq]
            hh, rr, RR = h[dot], rho[dot], R64[dot]
            a = hh * t32.double()
            k0 = torch.zeros(dot.numel(), dtype=torch.int32, device=dev)
            k1 = torch.full((dot.numel(),), self.N, dtype=torch.int32, device=dev)
            sk = kind.clone()
            sk0 = torch.zeros_like(k0)
            sk1 = torch.full_like(k1, self.n_coarse)
            for kd in (1, 2):
                msk = kind == kd
                if bool(msk.any()):
                    a0, a1 = k_range(kd, rr[msk], hh[msk], RR[msk], a[msk], self.N)
                    k0[msk], k1[msk] = a0, a1
            g = kind > 0                                          # G and J screen the same region disc n cone
            sk[g] = torch.where(a[g] < RR[g], 2, 1).int()
            for kd in (1, 2):
                msk = g & (sk == kd)
                if bool(msk.any()):
                    a0, a1 = k_range(kd, rr[msk], hh[msk], RR[msk], a[msk], self.n_coarse)
                    sk0[msk], sk1[msk] = a0, a1
            return {"dot": dot, "kind": kind.contiguous(), "q": qq, "rel32": rel32[dot].contiguous(),
                    "R32": R32[dot].contiguous(), "cos32": c32.contiguous(), "tan32": t32.contiguous(),
                    "k0": k0.contiguous(), "k1": k1.contiguous(), "screen_kind": sk.contiguous(),
                    "sk0": sk0.contiguous(), "sk1": sk1.contiguous()}

        for cert in (True, False):
            if not rows[cert]:
                continue
            pr = table(rows[cert])
            W, m = self.best_facing(pr, nrm32[pr["dot"]].contiguous(), cert)
            diag["certified" if cert else "uncertified"] += int(W.numel())
            for kd, target, mt in ((0, None, None), (1, WG, mG), (2, WJ, mJ)):
                msk = pr["kind"] == kd
                if not bool(msk.any()):
                    continue
                if kd == 0:
                    WF[pr["dot"][msk]] = W[msk]
                    mF[pr["dot"][msk]] = m[msk]
                else:
                    target[pr["dot"][msk], pr["q"][msk]] = W[msk]
                    mt[pr["dot"][msk], pr["q"][msk]] = m[msk]
        for q in range(Q):
            full = gfull[q]
            WG[full, q] = WF[full]
            mG[full, q] = mF[full]
        self.last_m = (mF, mG, mJ)
        # safety net: W* >= S_X c (the dot's own normal is a feasible m)
        for W, c in ((WF, sums.f_c), (WG, sums.g_c), (WJ, sums.j_c)):
            live = W > 0
            gap = torch.where(live, c / W.clamp_min(1e-300) - 1., torch.zeros_like(W))
            diag["binds"] += int((gap > BIND_REL).sum())
            if gap.numel():
                diag["max_bind_rel"] = max(diag["max_bind_rel"], float(gap.max()))
            W.copy_(torch.maximum(W, c))
        diag["seconds"] = time.perf_counter() - t0
        return WF, WG, WJ, mF, diag

    def scores(self, sums, WF, WG, WJ):
        N = float(self.N)

        def ratio(a, b):
            ok = b > 0
            return torch.where(ok, a / torch.where(ok, b, torch.ones_like(b)), torch.zeros_like(a))

        out = {"O": ratio(sums.f_cv, sums.f_c), "D": sums.f_cv / N, "DG": sums.g_cv / N, "DJ": sums.j_cv / N,
               "N": ratio(sums.f_cv, WF), "NG": ratio(sums.g_cv, WG), "NJ": ratio(sums.j_cv, WJ)}
        above = {k: int((v > 1. + 1e-9).sum()) for k, v in out.items()}
        return out, sums.g_n == 0, sums.j_n == 0, above


# --------------------------------------------------------------------------- posing helpers

def scorer_kind(kind):
    return kind if kind.startswith("hotec_") else f"hotec_{kind}"


def dish_geometry(kind):
    name = scorer_kind(kind)
    fc = E.food_contact(name)
    s = E.surface_samples(fc, E.DEFAULTS["samples_per_object"])
    tris = np.concatenate(E.dish_visuals(name))
    a = fc.areas
    centroid = (a[:, None] * fc.triangles.mean(axis=1)).sum(axis=0) / a.sum()
    return {"kind": kind, "name": name, "p_loc": s.points, "n_loc": s.normals, "tris": tris,
            "verts": tris.reshape(-1, 3), "centroid": centroid, "area": fc.area_m2}


def grid_rotation(tilt_deg):
    """R_y(180 deg - tilt): local +Z (food side) -> (sin t, 0, -cos t)."""
    a = math.radians(180. - tilt_deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0., s], [0., 1., 0.], [-s, 0., c]])


def grid_cells(geom, tilt_deg):
    """[(hi, di, p)] for the non-skipped (h, d) cells of a tilt (skip: any visual vertex below the disc)."""
    R = grid_rotation(tilt_deg)
    vz = (geom["verts"] @ R.T)[:, 2]
    cz = R @ geom["centroid"]
    out = []
    for hi, hcm in enumerate(GRID_H_CM):
        if (vz - cz[2]).min() + hcm / 100. < 0.:
            continue
        for di, dcm in enumerate(GRID_D_CM):
            out.append((hi, di, np.array([dcm / 100., 0., hcm / 100.]) - cz))
    return out


def grid_dots(geom, tilt_deg, cells):
    R = grid_rotation(tilt_deg)
    st_loc = geom["p_loc"] + OFFSET_M * geom["n_loc"]
    st = st_loc @ R.T
    nrm = geom["n_loc"] @ R.T
    rel = np.concatenate([st + p for _, _, p in cells])
    n = len(cells)
    return Dots(rel, np.tile(nrm, (n, 1)), np.tile(st_loc, (n, 1)), np.full(len(rel), GRID_ARM_R), R.T.copy())


def dot_entity_dots(tilts=GRID_TILTS, h_cm=GRID_H_CM, d_cm=GRID_D_CM, sign=1.):
    """The unblocked single dot at every (tilt, h, d) node, d along +x (sign=-1 mirrors to -d)."""
    rel, nrm = [], []
    for t in tilts:
        tr = math.radians(t)
        n = np.array([math.sin(tr), 0., -math.cos(tr)])
        for hcm in h_cm:
            for dcm in d_cm:
                rel.append(np.array([sign * dcm / 100., 0., hcm / 100.]) + OFFSET_M * n)
                nrm.append(n)
    rel, nrm = np.array(rel), np.array(nrm)
    return Dots(rel, nrm, rel.copy(), np.full(len(rel), GRID_ARM_R), np.eye(3))


def world_objects(entries):
    """frigidaire_hotec_exposure_search.world_objects: rack-local (racks in) -> world, scorer kinds."""
    objs = []
    for e in entries:
        p, q = E.compose_pose(BODY_POSITIONS[e["rack"]], E.IDENTITY, e["position"], e["quaternion_xyzw"])
        objs.append({"id": e["id"], "kind": scorer_kind(e["kind"]), "rack": e["rack"], "position_m": p,
                     "quaternion_xyzw": q})
    return objs


def load_dots(entries):
    """Dots of a settled load (per-rack arms), the dot owner index and per-dish records."""
    objs = world_objects(entries)
    rel, nrm, st, rad, owner, recs = [], [], [], [], [], []
    for i, o in enumerate(objs):
        fc = E.food_contact(o["kind"])
        s = E.surface_samples(fc, E.DEFAULTS["samples_per_object"])
        rot = E.quaternion_matrix_xyzw(o["quaternion_xyzw"])
        points = s.points @ rot.T + np.asarray(o["position_m"])
        normals = s.normals @ rot.T
        starts = points + E.DEFAULTS["origin_offset_m"] * normals
        arm = E.source_for(o["rack"])
        spec = E.ARM_SOURCES[arm]
        rel.append(starts - np.asarray(spec["center"], dtype=float))
        nrm.append(normals)
        st.append(starts)
        rad.append(np.full(len(starts), float(spec["radius"])))
        owner.append(np.full(len(starts), i))
        recs.append({"id": o["id"], "kind": o["kind"], "rack": o["rack"], "arm": arm, "area_m2": fc.area_m2,
                     "pools": bool(E.pools(o["kind"], o["position_m"], o["quaternion_xyzw"])),
                     "position_m": [float(v) for v in o["position_m"]],
                     "quaternion_xyzw": [float(v) for v in o["quaternion_xyzw"]]})
    dots = Dots(np.concatenate(rel), np.concatenate(nrm), np.concatenate(st), np.concatenate(rad), np.eye(3))
    return dots, np.concatenate(owner), recs, objs


def load_soup(entries, objs=None):
    """The revision-5 occluder soup of a load: every dish, both racks, the basket (racks in)."""
    objs = objs if objs is not None else world_objects(entries)
    basket = (np.asarray(BODY_POSITIONS["SilverwareBasket"], dtype=float), np.asarray(E.IDENTITY, dtype=float))
    return E.occluder_soup(E.Arrangement("load", "", "", objs, basket), appliance=True)


def score_load(engine, entries, theta_deg=30., load_id="load"):
    """All seven rules at one theta for a settled load: per dish, S (area-weighted), W (worst), pooling."""
    t = {"start": time.perf_counter()}
    dots, owner, recs, objs = load_dots(entries)
    soup = load_soup(entries, objs)
    t["soup"] = time.perf_counter()
    mesh = engine.build_mesh(soup)
    wp.synchronize_device(engine.device)
    t["bvh"] = time.perf_counter()
    sums = engine.ray_sums(mesh, dots, [theta_deg])
    t["rays"] = time.perf_counter()
    WF, WG, WJ, _, diag = engine.wstar(dots, sums, [theta_deg])
    t["wstar"] = time.perf_counter()
    sc, zG, zJ, above = engine.scores(sums, WF, WG, WJ)
    per = {k: (v[:, 0] if v.dim() == 2 else v).cpu().numpy() for k, v in sc.items()}
    zG, zJ = zG[:, 0].cpu().numpy(), zJ[:, 0].cpu().numpy()
    rays = sums.rays.cpu().numpy()
    areas = np.array([r["area_m2"] for r in recs])
    per_dish = []
    for i, r in enumerate(recs):
        sel = owner == i
        per_dish.append({**r, "E": {k: float(per[k][sel].mean()) for k in RULES},
                         "zG": int(zG[sel].sum()), "zJ": int(zJ[sel].sum()), "rays": float(rays[sel].sum())})
    Emat = np.array([[d["E"][k] for k in RULES] for d in per_dish])
    S = {k: float((areas * Emat[:, j]).sum() / areas.sum()) for j, k in enumerate(RULES)}
    W = {k: float(Emat[:, j].min()) for j, k in enumerate(RULES)}
    t["end"] = time.perf_counter()
    pooling = [d["id"] for d in per_dish if d["pools"]]
    del mesh
    return {"load_id": load_id, "N": engine.N, "theta_deg": float(theta_deg), "n_dishes": len(recs),
            "arms": {k: {"center": list(v["center"]), "radius": v["radius"], "racks": list(v["racks"])}
                     for k, v in E.ARM_SOURCES.items()},
            "S": S, "W": W, "feasible": not pooling, "pooling_count": len(pooling), "pooling": pooling,
            "n_above_1": above, "wstar": diag, "rays": float(rays.sum()), "triangles": int(len(soup)),
            "seconds": {"soup": t["soup"] - t["start"], "bvh": t["bvh"] - t["soup"], "rays": t["rays"] - t["bvh"],
                        "wstar": t["wstar"] - t["rays"], "total": t["end"] - t["start"]},
            "warp": wp.__version__, "device": engine.device, "per_dish": per_dish}
