# -*- coding: utf-8 -*-
"""
cm_phase1_reference.py
======================
HF_CAMPO_MARZIO_GRAMMAR Phase 1 の「参照実装」(Houdini 外)。

目的
----
* VEX (../vex/*.vfl) と同じ判定式・採点式を Python で再現し、
  Phase 1 の合格条件 (はみ出しなし・重複なし・残余の更新・シード再現性・
  未充填領域の記録) をアルゴリズムとして検証する。
* Houdini 上で結果を比べるときの目安となる SVG プレビューを出力する。

これは HDA の一部ではない (HDA は Python SOP を使わない)。
Residual = Domain - Union(Buildings) の Boolean は Houdini の Boolean SOP の
代わりに shapely で計算する。位置サンプルの乱数列は VEX の rand() とは異なるため、
同じシードでも Houdini と配置が一致するわけではない (再現性は各実装内で成立する)。

使い方
------
    pip install shapely numpy
    python cm_phase1_reference.py --seed 1 --svg out.svg
"""

import argparse
import math
from dataclasses import dataclass, field

import numpy as np
import shapely
from shapely.geometry import MultiLineString, Point, Polygon
from shapely.ops import unary_union

REASONS = ["valid", "outside_domain", "overlap_building",
           "below_min_area", "access_sliver", "invalid_shape"]


# ---------------------------------------------------------------------------
# パラメータ (HDA の既定値と同じ)
# ---------------------------------------------------------------------------
@dataclass
class Params:
    domain_width: float = 200.0
    domain_height: float = 140.0
    target_fill_ratio: float = 0.60
    minimum_building_area: float = 80.0
    base_size: float = 20.0
    min_scale: float = 0.6
    max_scale: float = 1.8
    max_aspect: float = 2.5
    aspect_sample_count: int = 3
    angle_step: float = 5.0
    position_sample_count: int = 32
    min_samples_per_region: int = 3
    scale_sample_count: int = 3
    snap_to_boundary: bool = True
    max_candidates: int = 60000
    max_iterations: int = 40
    random_seed: int = 1
    clearance: float = 0.0
    contact_tolerance: float = 0.5
    min_passage_width: float = 4.0
    max_sliver_ratio: float = 0.5
    perimeter_samples: int = 6
    weight_fill: float = 0.40
    weight_fit: float = 0.25
    weight_access: float = 0.20
    weight_variety: float = 0.10
    weight_axis: float = 0.05
    variety_window: int = 5
    axis_radius: float = 40.0
    minimum_residual_area: float = 100.0
    minimum_improvement: float = 0.0005
    stop_when_no_candidate: bool = True


@dataclass
class Building:
    building_id: int
    poly: Polygon
    orientation: float
    scale: float
    generation: int
    width: float
    depth: float
    score: float = -1.0
    shape_type: int = 0


@dataclass
class State:
    domain: Polygon
    buildings: list
    iter: int = 0
    stop: bool = False
    stop_reason: str = ""
    fill_ratio: float = 0.0
    next_building_id: int = 0
    recent_orient: list = field(default_factory=list)
    recent_scale: list = field(default_factory=list)
    history: list = field(default_factory=list)


# ---------------------------------------------------------------------------
# 形状 (D01 の規約と同じ)
# ---------------------------------------------------------------------------
def rect_corners(c, theta_deg, w, d):
    t = math.radians(theta_deg)
    ex = np.array([math.cos(t), math.sin(t)])
    ey = np.array([-math.sin(t), math.cos(t)])
    c = np.asarray(c, dtype=float)
    return np.array([c - ex * w / 2 - ey * d / 2,
                     c + ex * w / 2 - ey * d / 2,
                     c + ex * w / 2 + ey * d / 2,
                     c - ex * w / 2 + ey * d / 2])


def rect_poly(c, theta_deg, w, d):
    return Polygon(rect_corners(c, theta_deg, w, d))


def angdiff90(a, b):
    x = np.abs(np.asarray(a) - np.asarray(b)) % 90.0
    return np.minimum(x, 90.0 - x)


# ---------------------------------------------------------------------------
# C: RESIDUAL_EXTRACT
# ---------------------------------------------------------------------------
def extract_residual(state, prm):
    """Residual = Domain - Union(Buildings) を連結領域に分け、特徴量と境界線分を返す。"""
    union = unary_union([b.poly for b in state.buildings]) if state.buildings else None
    res = state.domain.difference(union) if union is not None else state.domain
    parts = [g for g in getattr(res, "geoms", [res]) if not g.is_empty and g.area > 1e-7 * state.domain.area]

    tol = 1e-4 * prm.base_size
    regions = []
    for rid, g in enumerate(parts):
        rings = [g.exterior] + list(g.interiors)
        perim = sum(r.length for r in rings)
        contact = 0.0
        segs = []
        for ring in rings:
            xy = np.asarray(ring.coords)
            for a, b in zip(xy[:-1], xy[1:]):
                if np.allclose(a, b):
                    continue
                mid = Point((a + b) / 2)
                src = -1
                for bl in state.buildings:
                    if bl.poly.exterior.distance(mid) <= 10 * tol:
                        src = bl.building_id
                        break
                if src >= 0:
                    contact += float(np.linalg.norm(b - a))
                segs.append((a, b, src))
        minx, miny, maxx, maxy = g.bounds
        sz = (maxx - minx, maxy - miny)
        regions.append(dict(
            residual_id=rid, geom=g, area=g.area, perimeter=perim,
            centroid=np.array(g.centroid.coords[0]),
            bbmin=np.array([minx, miny]), bbmax=np.array([maxx, maxy]),
            elongation=perim ** 2 / (4 * math.pi * max(g.area, 1e-12)),
            bbox_aspect=max(sz) / max(min(sz), 1e-12),
            approx_width=2 * g.area / max(perim, 1e-12),
            building_contact_ratio=contact / max(perim, 1e-12),
            ignored=g.area < prm.minimum_building_area,
            segments=segs,
        ))
    return regions


def boundary_arrays(regions):
    A, B, S, R = [], [], [], []
    for r in regions:
        for a, b, src in r["segments"]:
            A.append(a); B.append(b); S.append(src); R.append(r["residual_id"])
    return (np.array(A).reshape(-1, 2), np.array(B).reshape(-1, 2),
            np.array(S, dtype=int), np.array(R, dtype=int))


# ---------------------------------------------------------------------------
# E01: CANDIDATE_GENERATE
# ---------------------------------------------------------------------------
def generate_candidates(regions, seg_a, seg_b, state, prm):
    base = prm.base_size
    tol = 1e-4 * base
    ns, na = max(1, prm.scale_sample_count), max(1, prm.aspect_sample_count)
    scales = [0.5 * (prm.min_scale + prm.max_scale)] if ns == 1 else list(np.linspace(prm.min_scale, prm.max_scale, ns))
    aspects = [1.0] if na == 1 else list(np.linspace(1.0, prm.max_aspect, na))
    nang = max(1, int(math.floor((180.0 - 1e-6) / prm.angle_step)) + 1)
    angles = [k * prm.angle_step for k in range(nang)]

    total = sum(r["area"] for r in regions if not r["ignored"])
    lines = MultiLineString([[tuple(a), tuple(b)] for a, b in zip(seg_a, seg_b)]) if len(seg_a) else None

    cands = []
    sid = 0
    for r in regions:
        if r["ignored"] or total <= 0:
            continue
        rid = r["residual_id"]
        nr = max(prm.min_samples_per_region, int(round(prm.position_sample_count * r["area"] / total)))
        g = r["geom"]
        samples = []
        if g.distance(Point(r["centroid"])) <= tol:
            samples.append(r["centroid"])
        rng = np.random.default_rng([prm.random_seed, state.iter, rid])
        tries = 0
        while len(samples) < nr and tries < nr * 80:
            u = rng.random(2)
            p = r["bbmin"] + u * (r["bbmax"] - r["bbmin"])
            if g.distance(Point(p)) <= tol:
                samples.append(p)
            tries += 1

        for P in samples:
            for th in angles:
                for s in scales:
                    for asp in aspects:
                        cands.append((P[0], P[1], th, base * s * math.sqrt(asp), base * s / math.sqrt(asp),
                                      s, asp, rid, r["area"], sid, 0))
            if prm.snap_to_boundary and lines is not None:
                # 最寄りの境界線分
                d2 = _point_seg_dist(np.asarray(P)[None, :], seg_a, seg_b)[0]
                k = int(np.argmin(d2))
                a, b = seg_a[k], seg_b[k]
                Q = _closest_on_seg(P, a, b)
                if np.linalg.norm(P - Q) > tol:
                    t = (b - a) / np.linalg.norm(b - a)
                    n = (P - Q) / np.linalg.norm(P - Q)
                    th = math.degrees(math.atan2(t[1], t[0])) % 180.0
                    for s in scales:
                        for asp in aspects:
                            for sw in (0, 1):
                                w, d = base * s * math.sqrt(asp), base * s / math.sqrt(asp)
                                if sw:
                                    w, d = d, w
                                c = Q + n * (0.5 * d + prm.clearance)
                                cands.append((c[0], c[1], th, w, d, s, asp, rid, r["area"], sid, 1))
            sid += 1
            if len(cands) >= prm.max_candidates:
                break
    cands = cands[:prm.max_candidates]
    dt = [("x", float), ("y", float), ("theta", float), ("width", float), ("depth", float),
          ("scale", float), ("aspect", float), ("residual_id", int), ("region_area", float),
          ("sample_id", int), ("snapped", int)]
    return np.array(cands, dtype=dt)


def _point_seg_dist(P, A, B):
    """P (N,2) と線分 A-B (M,2) の距離 (N,M)。"""
    AB = B - A
    L2 = np.maximum((AB ** 2).sum(1), 1e-30)
    AP = P[:, None, :] - A[None, :, :]
    t = np.clip((AP * AB[None]).sum(2) / L2[None], 0.0, 1.0)
    proj = A[None] + t[..., None] * AB[None]
    return np.linalg.norm(P[:, None, :] - proj, axis=2)


def _closest_on_seg(P, a, b):
    ab = b - a
    t = np.clip(np.dot(P - a, ab) / max(np.dot(ab, ab), 1e-30), 0.0, 1.0)
    return a + t * ab


# ---------------------------------------------------------------------------
# E02: CANDIDATE_VALIDATE (VEX と同じ判定式)
# ---------------------------------------------------------------------------
def _cr(ax, ay, bx, by):
    return ax * by - ay * bx


def rect_in_residual_mask(cx, cy, th, w, d, seg_a, seg_b, clearance, eps):
    """(2)(3) の判定: 図形の辺が境界と交差せず、境界の端点・中点が内部にない。

    返り値: hit (N,) bool, hit_seg (N,) int (-1 = なし)
    """
    N = len(cx)
    hit = np.zeros(N, bool)
    hit_seg = -np.ones(N, int)
    if len(seg_a) == 0 or N == 0:
        return hit, hit_seg
    t = np.radians(th)
    exx, exy = np.cos(t), np.sin(t)
    eyx, eyy = -np.sin(t), np.cos(t)
    thw = 0.5 * w + clearance - eps
    thd = 0.5 * d + clearance - eps
    sx = np.array([-1, 1, 1, -1]); sy = np.array([-1, -1, 1, 1])
    Cx = cx[:, None] + exx[:, None] * thw[:, None] * sx + eyx[:, None] * thd[:, None] * sy   # (N,4)
    Cy = cy[:, None] + exy[:, None] * thw[:, None] * sx + eyy[:, None] * thd[:, None] * sy

    for i0 in range(0, N, 512):
        sl = slice(i0, min(N, i0 + 512))
        cxs, cys = cx[sl, None], cy[sl, None]
        # (3) 端点・中点が内部にあるか
        inside = np.zeros((cxs.shape[0], len(seg_a)), bool)
        for T in (seg_a, seg_b, 0.5 * (seg_a + seg_b)):
            lx = T[None, :, 0] - cxs
            ly = T[None, :, 1] - cys
            u = lx * exx[sl, None] + ly * exy[sl, None]
            v = lx * eyx[sl, None] + ly * eyy[sl, None]
            inside |= (np.abs(u) < thw[sl, None]) & (np.abs(v) < thd[sl, None])
        # (2) 辺の交差 (厳密な交差のみ)
        cross = np.zeros_like(inside)
        for e in range(4):
            p1x, p1y = Cx[sl, e][:, None], Cy[sl, e][:, None]
            p2x, p2y = Cx[sl, (e + 1) % 4][:, None], Cy[sl, (e + 1) % 4][:, None]
            q1x, q1y = seg_a[None, :, 0], seg_a[None, :, 1]
            q2x, q2y = seg_b[None, :, 0], seg_b[None, :, 1]
            d1 = _cr(p2x - p1x, p2y - p1y, q1x - p1x, q1y - p1y)
            d2 = _cr(p2x - p1x, p2y - p1y, q2x - p1x, q2y - p1y)
            d3 = _cr(q2x - q1x, q2y - q1y, p1x - q1x, p1y - q1y)
            d4 = _cr(q2x - q1x, q2y - q1y, p2x - q1x, p2y - q1y)
            cross |= (d1 * d2 < 0) & (d3 * d4 < 0)
        h = inside | cross
        any_h = h.any(1)
        hit[sl] = any_h
        first = np.where(any_h, h.argmax(1), -1)
        hit_seg[sl] = first
    return hit, hit_seg


def validate(cands, regions, seg_a, seg_b, seg_src, state, prm):
    eps = 1e-4 * prm.base_size
    N = len(cands)
    code = np.zeros(N, int)
    w, d = cands["width"], cands["depth"]
    bad = ~np.isfinite(w) | ~np.isfinite(d) | ~np.isfinite(cands["theta"]) | (w <= eps) | (d <= eps)
    code[bad] = 5
    small = (~bad) & (w * d < prm.minimum_building_area)
    code[small] = 3

    todo = np.where(code == 0)[0]
    if len(todo):
        res_union = unary_union([r["geom"] for r in regions]) if regions else Polygon()
        dc = shapely.distance(res_union, shapely.points(cands["x"][todo], cands["y"][todo]))
        out_c = dc > 10 * eps
        dd = shapely.distance(state.domain, shapely.points(cands["x"][todo], cands["y"][todo]))
        code[todo[out_c & (dd <= 10 * eps)]] = 2
        code[todo[out_c & (dd > 10 * eps)]] = 1
        todo = todo[~out_c]

    if len(todo):
        c = cands[todo]
        hit, hs = rect_in_residual_mask(c["x"], c["y"], c["theta"], c["width"], c["depth"],
                                        seg_a, seg_b, prm.clearance, eps)
        src = np.where(hs >= 0, seg_src[np.maximum(hs, 0)], 0)
        code[todo[hit & (src < 0)]] = 1
        code[todo[hit & (src >= 0)]] = 2
    return code


# ---------------------------------------------------------------------------
# F01: CANDIDATE_SCORE (VEX と同じ採点式)
# ---------------------------------------------------------------------------
def score(cands, code, seg_a, seg_b, state, prm):
    N = len(cands)
    sc = -np.ones(N)
    terms = np.zeros((N, 5))
    idx = np.where(code == 0)[0]
    if len(idx) == 0:
        return sc, terms, code
    c = cands[idx]
    m = max(1, prm.perimeter_samples)
    lines = MultiLineString([[tuple(a), tuple(b)] for a, b in zip(seg_a, seg_b)])

    # 周長サンプル
    t = np.radians(c["theta"])
    ex = np.stack([np.cos(t), np.sin(t)], 1)
    ey = np.stack([-np.sin(t), np.cos(t)], 1)
    P = np.stack([c["x"], c["y"]], 1)
    hw, hd = (0.5 * c["width"])[:, None], (0.5 * c["depth"])[:, None]
    C = [P - ex * hw - ey * hd, P + ex * hw - ey * hd, P + ex * hw + ey * hd, P - ex * hw + ey * hd]
    samples = []
    for e in range(4):
        for k in range(m):
            f = (k + 0.5) / m
            samples.append(C[e] * (1 - f) + C[(e + 1) % 4] * f)
    S = np.stack(samples, 1)                        # (n, 4m, 2)
    dist = shapely.distance(lines, shapely.points(S[..., 0].ravel(), S[..., 1].ravel())).reshape(S.shape[:2])
    maxd = prm.clearance + prm.min_passage_width
    near = dist < maxd
    contact = near & (dist <= prm.clearance + prm.contact_tolerance)
    sliver = near & ~contact
    fit = contact.mean(1)
    sl = sliver.mean(1)
    access = 1.0 - sl

    rej = sl > prm.max_sliver_ratio
    code = code.copy()
    code[idx[rej]] = 4

    amax = (prm.base_size * prm.max_scale) ** 2
    fill = np.clip(c["width"] * c["depth"] / np.maximum(np.minimum(c["region_area"], amax), 1e-9), 0, 1)

    if state.recent_orient:
        lr = max(math.log(prm.max_scale / prm.min_scale), 1e-6)
        ro = np.array(state.recent_orient)[None, :]
        rs = np.array(state.recent_scale)[None, :]
        da = angdiff90(c["theta"][:, None], ro) / 45.0
        ds = np.clip(np.abs(np.log(c["scale"][:, None] / np.maximum(rs, 1e-9))) / lr, 0, 1)
        variety = (0.5 * (da + ds)).mean(1)
    else:
        variety = np.ones(len(c))

    axis = np.full(len(c), 0.5)
    if state.buildings:
        bl = state.buildings
        bd = np.stack([shapely.distance(b.poly, shapely.points(c["x"], c["y"])) for b in bl], 1)
        k = bd.argmin(1)
        ok = bd[np.arange(len(c)), k] <= prm.axis_radius
        o = np.array([b.orientation for b in bl])[k]
        axis = np.where(ok, 1.0 - angdiff90(c["theta"], o) / 45.0, 0.5)

    W = np.array([prm.weight_fill, prm.weight_fit, prm.weight_access, prm.weight_variety, prm.weight_axis])
    T = np.stack([fill, fit, access, variety, axis], 1)
    s = (T * W).sum(1) / max(W.sum(), 1e-9)
    s[rej] = -1.0
    sc[idx] = s
    terms[idx] = T
    return sc, terms, code


# ---------------------------------------------------------------------------
# G: ITERATION_CONTROLLER
# ---------------------------------------------------------------------------
def init_state(prm, footprints=()):
    hw, hh = prm.domain_width / 2, prm.domain_height / 2
    domain = Polygon([(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)])
    st = State(domain=domain, buildings=[])
    for (c, th, w, d) in footprints:
        p = rect_poly(c, th, w, d)
        if not domain.buffer(1e-6).contains(p):
            continue
        if any(p.intersection(b.poly).area > 1e-6 for b in st.buildings):
            continue
        st.buildings.append(Building(len(st.buildings), p, th % 180, math.sqrt(w * d) / prm.base_size, 0, w, d))
    st.next_building_id = len(st.buildings)
    st.fill_ratio = sum(b.poly.area for b in st.buildings) / domain.area
    K = max(1, prm.variety_window)
    st.recent_orient = [b.orientation for b in st.buildings][-K:]
    st.recent_scale = [b.scale for b in st.buildings][-K:]
    return st


def step(state, prm):
    """1 反復 = 残余抽出 -> 候補生成 -> 検査 -> 採点 -> 1 棟配置 -> 終了判定。"""
    regions = extract_residual(state, prm)
    seg_a, seg_b, seg_src, _ = boundary_arrays(regions)
    cands = generate_candidates(regions, seg_a, seg_b, state, prm)
    code = validate(cands, regions, seg_a, seg_b, seg_src, state, prm)
    sc, terms, code = score(cands, code, seg_a, seg_b, state, prm)

    valid = np.where(code == 0)[0]
    placed = None
    if len(valid):
        best = valid[np.argmax(sc[valid])]    # 同点なら index の小さい方
        c = cands[best]
        poly = rect_poly((c["x"], c["y"]), c["theta"], c["width"], c["depth"])
        placed = Building(state.next_building_id, poly, float(c["theta"]), float(c["scale"]),
                          state.iter + 1, float(c["width"]), float(c["depth"]), float(sc[best]))
        state.buildings.append(placed)
        state.next_building_id += 1
        K = max(1, prm.variety_window)
        state.recent_orient = (state.recent_orient + [placed.orientation])[-K:]
        state.recent_scale = (state.recent_scale + [placed.scale])[-K:]

    state.iter += 1
    prev = state.fill_ratio
    fill = sum(b.poly.area for b in state.buildings) / state.domain.area
    state.fill_ratio = fill
    maxres = max((r["area"] for r in regions), default=0.0)
    counts = np.bincount(code, minlength=6)
    state.history.append(dict(iter=state.iter, building_id=placed.building_id if placed else -1,
                              score=placed.score if placed else -1.0, n_candidates=len(cands),
                              n_valid=int(counts[0]), reject_counts=counts.tolist(),
                              fill=fill, max_residual=maxres))

    reason = ""
    if placed is None and prm.stop_when_no_candidate:
        reason = "no_candidate"
    elif fill >= prm.target_fill_ratio:
        reason = "target_fill"
    elif maxres < prm.minimum_residual_area:
        reason = "min_residual_area"
    elif placed is not None and (fill - prev) < prm.minimum_improvement:
        reason = "min_improvement"
    elif state.iter >= prm.max_iterations:
        reason = "max_iterations"
    state.stop = reason != ""
    state.stop_reason = reason
    return dict(regions=regions, cands=cands, code=code, score=sc, terms=terms, placed=placed)


def run(prm, footprints=(), verbose=False):
    st = init_state(prm, footprints)
    last = None
    for _ in range(prm.max_iterations):
        if st.stop:
            break
        last = step(st, prm)
        if verbose:
            h = st.history[-1]
            print("iter %3d  placed=%3d  score=%.3f  valid=%5d/%5d  fill=%.3f  rejects=%s"
                  % (h["iter"], h["building_id"], h["score"], h["n_valid"], h["n_candidates"], h["fill"],
                     h["reject_counts"]))
    final_regions = extract_residual(st, prm)
    return st, final_regions, last


# ---------------------------------------------------------------------------
# H01: 独立検証
# ---------------------------------------------------------------------------
def check(state, final_regions, tol=1e-6):
    bl = state.buildings
    overlap_pairs = 0
    for i in range(len(bl)):
        for j in range(i + 1, len(bl)):
            if bl[i].poly.intersection(bl[j].poly).area > tol:
                overlap_pairs += 1
    outside = sum(1 for b in bl if b.poly.difference(state.domain).area > tol)
    built = sum(b.poly.area for b in bl)
    res = sum(r["area"] for r in final_regions)
    balance = abs(state.domain.area - built - res) / state.domain.area
    return dict(overlap_pairs=overlap_pairs, outside=outside, area_balance=balance,
                passed=overlap_pairs == 0 and outside == 0 and balance < 1e-4,
                fill=built / state.domain.area, unfilled_area=res,
                residual_count=len(final_regions), iterations=state.iter,
                stop_reason=state.stop_reason or "loop_count", buildings=len(bl))


# ---------------------------------------------------------------------------
# SVG プレビュー
# ---------------------------------------------------------------------------
def to_svg(state, final_regions, prm, path, last=None, scale=4.0):
    hw, hh = prm.domain_width / 2, prm.domain_height / 2
    W, H = prm.domain_width * scale + 20, prm.domain_height * scale + 20

    def tx(x, y):
        return 10 + (x + hw) * scale, 10 + (hh - y) * scale

    def path_d(poly):
        out = []
        for ring in [poly.exterior] + list(poly.interiors):
            pts = [tx(x, y) for x, y in ring.coords]
            out.append("M" + " L".join("%.2f,%.2f" % p for p in pts) + " Z")
        return " ".join(out)

    s = ['<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d" viewBox="0 0 %d %d">' % (W, H, W, H),
         '<rect width="100%" height="100%" fill="#f4f1ea"/>']
    for r in final_regions:
        fill = "#e8b4a8" if r["ignored"] else "#fbfaf6"
        s.append('<path d="%s" fill="%s" fill-rule="evenodd" stroke="#c9c2b2" stroke-width="0.6"/>'
                 % (path_d(r["geom"]), fill))
    gmax = max(1, max((b.generation for b in state.buildings), default=1))
    for b in state.buildings:
        if b.generation == 0:
            col = "#333333"
        else:
            g = b.generation / gmax
            col = "#%02x%02x%02x" % (int(40 + 150 * g), int(70 + 90 * g), int(140 - 60 * g))
        s.append('<path d="%s" fill="%s" stroke="#111" stroke-width="0.8"/>' % (path_d(b.poly), col))
    s.append('<path d="%s" fill="none" stroke="#000" stroke-width="1.5"/>' % path_d(state.domain))
    s.append("</svg>")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(s))


TEST_FOOTPRINTS = [  # T01_TEST_FOOTPRINTS と同じ
    ((-40.0, 10.0), 0.0, 36.0, 22.0),
    ((55.0, -25.0), 30.0, 30.0, 18.0),
    ((0.0, -61.0), 0.0, 50.0, 18.0),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--iterations", type=int, default=40)
    ap.add_argument("--svg", default="")
    ap.add_argument("--no-initial", action="store_true", help="初期建築なし (initial_layout_mode=None)")
    a = ap.parse_args()
    prm = Params(random_seed=a.seed, max_iterations=a.iterations)
    st, regions, last = run(prm, () if a.no_initial else TEST_FOOTPRINTS, verbose=True)
    res = check(st, regions)
    print(res)
    if a.svg:
        to_svg(st, regions, prm, a.svg, last)
        print("wrote", a.svg)


if __name__ == "__main__":
    main()
