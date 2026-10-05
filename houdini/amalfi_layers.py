# -*- coding: utf-8 -*-
"""
amalfi_layers.py
================
アマルフィ型の「積層集落」を手続き的に生成するモデル。

考え方
------
1. 段丘状の地形(谷筋が低く、海から山へ段々に上がる)をグリッドで作る。
2. 高さを「絶対レベル k = 階高 × k」で切り、レベルごとに建てられるセルを求める。
     ・地面の段がちょうどレベル k のセル(その段の1層目になる)
     ・レベル k-1 に建物があるセル(屋根の上に積む)
     ・両側を建物に挟まれた通路の上(路地を跨ぐ部屋 = ソットポルティコ)
3. 候補セルを離散ボロノイ(シード点からの地理的BFS)で部屋に分割する。
   下の層のシード点を引き継ぐので、壁が上下でおおむね揃う。
4. 部屋を確率的に建て、建てた結果「どの部屋も港から歩いて(階段込みで)
   たどり着けるか」を検査し、ダメなら取り消す。空き地 = 通路とみなす。
5. 所有者(owner)を割り当て、購入(結合)・相続/売却(分割)の履歴を走らせる。
6. 港から各住戸の入口までの最短経路を求め、段差を越える箇所に階段を生成する。

使い方
------
A) Houdini の Python Shell から(推奨)
       import sys; sys.path.append(r"<このファイルのあるフォルダ>")
       import amalfi_layers; amalfi_layers.install()
   → /obj/amalfi_layers の中に Python SOP が作られ、パラメータ付きで生成される。
     SOP の中身はこのファイルを読み込む数行のローダーだけ。

B) 手動
   Geometry ノードの中に Python SOP を作り、Python Code 欄の既存コードを
   全部消して、次の数行だけを貼る(パラメータが無い場合は DEFAULTS を使う)。
       import sys, importlib
       folder = r"<このファイルのあるフォルダ>"
       if folder not in sys.path:
           sys.path.append(folder)
       import amalfi_layers
       importlib.reload(amalfi_layers)
       amalfi_layers.cook_python_sop()

C) Houdini なしで確認
       python amalfi_layers.py --seed 7 --obj out.obj
   レベルごとの平面をテキストで表示し、OBJ を書き出す。

出力される primitive 属性
-------------------------
  kind     : "room" / "terrain" / "stair" / "path" / "sea"
  face     : room の面種別 "facade" / "party_wall" / "roof" / "slab" / "soffit"
             (soffit = 路地を跨ぐ部屋の天井側=トンネルの天井)
  level    : レベル番号
  piece    : 部屋(住戸ユニット)の ID
  owner    : 所有世帯の ID
  on_ground: 1 = 地面の上に直接建つ部屋(ヴォールトの倉庫層の候補)
  bridge   : 1 = 路地を跨いでいるセルを含む部屋
  usage    : 階段・通路を通る住戸の数(多いほど主要動線)
  Cd       : owner ごとの色
detail 属性 "history" に所有の変化の履歴が入る。
"""

import colorsys
import heapq
import math
import os
import random
from collections import Counter, defaultdict, deque

DIRS = ((1, 0), (-1, 0), (0, 1), (0, -1))
DIR_SIDE = {(1, 0): "+x", (-1, 0): "-x", (0, 1): "+z", (0, -1): "-z"}

# パラメータ名: (既定値, ラベル)
PARM_SPEC = [
    ("seed", 7, "Seed"),
    ("nx", 36, "グリッド X(谷の幅方向)"),
    ("ny", 30, "グリッド Z(海→山)"),
    ("cell", 3.0, "セル寸法 (m)"),
    ("floor_h", 3.0, "階高 = 段の高さ (m)"),
    ("levels", 9, "最大レベル数"),
    ("max_stack", 4, "1か所に積める最大階数"),
    ("harbor_rows", 1, "港(建築不可)の行数"),
    ("terrace_rows", 3.0, "段丘1段の奥行き(セル)"),
    ("valley_width", 14.0, "谷底の幅(セル)"),
    ("side_rise", 1.2, "谷の両斜面の立ち上がり"),
    ("noise", 0.8, "地形ノイズ"),
    ("piece_cells", 5, "1部屋の目安セル数"),
    ("min_cells", 2, "最小部屋セル数"),
    ("seed_jitter", 0.3, "下層シードのずれ確率"),
    ("build_prob", 0.8, "地面の上に建つ確率"),
    ("roof_prob", 0.65, "屋根の上に積む確率"),
    ("bridge_prob", 0.5, "路地を跨ぐ候補の確率"),
    ("owner_inherit", 0.6, "真下と同じ世帯が上に増築する確率"),
    ("merge_events", 14, "購入(結合)イベント数"),
    ("split_events", 8, "相続・売却(分割)イベント数"),
    ("stair_cost", 2.5, "階段の経路コスト"),
    ("stair_steps", 8, "階段の段数"),
    ("wide_stair", 4, "幅広階段にする利用住戸数"),
]
DEFAULTS = {name: val for name, val, _ in PARM_SPEC}


# ---------------------------------------------------------------------------
# 地形
# ---------------------------------------------------------------------------
def value_noise(nx, ny, rng, step=5):
    """-0.5..0.5 のなめらかな2Dノイズ(格子値ノイズ)。"""
    gx, gy = nx // step + 2, ny // step + 2
    lat = [[rng.random() - 0.5 for _ in range(gy)] for _ in range(gx)]

    def smooth(t):
        return t * t * (3 - 2 * t)

    out = [[0.0] * ny for _ in range(nx)]
    for i in range(nx):
        fi = i / step
        i0 = int(fi)
        ti = smooth(fi - i0)
        for j in range(ny):
            fj = j / step
            j0 = int(fj)
            tj = smooth(fj - j0)
            a = lat[i0][j0] * (1 - ti) + lat[i0 + 1][j0] * ti
            b = lat[i0][j0 + 1] * (1 - ti) + lat[i0 + 1][j0 + 1] * ti
            out[i][j] = a * (1 - tj) + b * tj
    return out


# ---------------------------------------------------------------------------
# 集落モデル本体(Houdini 非依存)
# ---------------------------------------------------------------------------
class Town(object):
    def __init__(self, params):
        P = dict(DEFAULTS)
        P.update(params or {})
        self.P = P
        self.rng = random.Random(int(P["seed"]))
        self.nx, self.ny, self.L = int(P["nx"]), int(P["ny"]), int(P["levels"])
        self.ground = self._make_terrain()
        self.built = {}  # (i, j, k) -> piece id
        self.pieces = {}  # pid -> dict(level, cells, seed, owner)
        self.next_pid = 0
        self.next_owner = 0
        self.prev_seeds = []
        self.history = []
        self.stairs = []  # (lower(i,j,k), upper(i,j,k), usage)
        self.path_usage = Counter()
        self.doors = {}

    # --- 地形 ---------------------------------------------------------------
    def _make_terrain(self):
        P, nx, ny = self.P, self.nx, self.ny
        cx = (nx - 1) / 2.0
        nz = value_noise(nx, ny, self.rng)
        hr = int(P["harbor_rows"])
        g = [[0] * ny for _ in range(nx)]
        for i in range(nx):
            for j in range(ny):
                if j < hr:
                    v = 0
                else:
                    t = (j - hr + 1) / float(P["terrace_rows"])
                    side = P["side_rise"] * (abs(i - cx) / (P["valley_width"] / 2.0)) ** 2
                    v = int(math.floor(t + side + P["noise"] * nz[i][j]))
                g[i][j] = max(0, min(self.L - 1, v))
        return g

    # --- 基本判定 -----------------------------------------------------------
    def inb(self, i, j):
        return 0 <= i < self.nx and 0 <= j < self.ny

    def solid(self, i, j, k):
        """セル(i,j)のレベル k が土か建物で埋まっているか。"""
        return k < self.ground[i][j] or (i, j, k) in self.built

    def surface(self, i, j, k):
        """(i,j) のレベル k の床面が歩ける(上が空いている地面か屋根)か。"""
        if k < 0 or k > self.L:
            return False
        if self.solid(i, j, k):
            return False
        return self.ground[i][j] == k or (i, j, k - 1) in self.built

    def walk_edges(self, node):
        """歩行グラフの隣接: 同じ高さは平坦、1段差は階段(下側セルに頭上空間が必要)。"""
        i, j, k = node
        out = []
        for di, dj in DIRS:
            a, c = i + di, j + dj
            if not self.inb(a, c):
                continue
            if self.surface(a, c, k):
                out.append(((a, c, k), 1.0, False))
            if self.surface(a, c, k + 1) and not self.solid(i, j, k + 1):
                out.append(((a, c, k + 1), self.P["stair_cost"], True))
            if self.surface(a, c, k - 1) and not self.solid(a, c, k):
                out.append(((a, c, k - 1), self.P["stair_cost"], True))
        return out

    def roots(self):
        hr = max(1, int(self.P["harbor_rows"]))
        return [(i, j, 0) for i in range(self.nx) for j in range(hr) if self.surface(i, j, 0)]

    def compute_reach(self):
        seen = set(self.roots())
        dq = deque(seen)
        while dq:
            node = dq.popleft()
            for nb, _, _ in self.walk_edges(node):
                if nb not in seen:
                    seen.add(nb)
                    dq.append(nb)
        return seen

    def piece_accessible(self, pid, reach):
        p = self.pieces[pid]
        k = p["level"]
        for i, j in p["cells"]:
            for di, dj in DIRS:
                if (i + di, j + dj, k) in reach:
                    return True
        return False

    # --- 候補セルと分割 -----------------------------------------------------
    def candidates(self, k):
        P, g, b = self.P, self.ground, self.built
        out = set()
        for i in range(self.nx):
            for j in range(int(P["harbor_rows"]), self.ny):
                gk = g[i][j]
                if gk > k or (i, j, k) in b or k - gk >= P["max_stack"]:
                    continue
                if gk == k or (i, j, k - 1) in b:
                    out.add((i, j))
                    continue
                # 路地を跨ぐ: 下が歩ける通路で、両側が埋まっている
                if k >= 1 and self.surface(i, j, k - 1):
                    def sol(a, c):
                        return self.inb(a, c) and self.solid(a, c, k - 1)
                    span = (sol(i - 1, j) and sol(i + 1, j)) or (sol(i, j - 1) and sol(i, j + 1))
                    if span and self.rng.random() < P["bridge_prob"]:
                        out.add((i, j))
        return out

    def partition(self, cand):
        """候補セルを離散ボロノイ(地理的BFS)で部屋に分ける。"""
        P, rng = self.P, self.rng
        seeds = []
        seedset = set()
        for s in self.prev_seeds:  # 下の層のシードを引き継ぐ → 壁が揃う
            i, j = s
            if rng.random() < P["seed_jitter"]:
                di, dj = rng.choice(DIRS)
                i, j = i + di, j + dj
            if (i, j) in cand and (i, j) not in seedset:
                seeds.append((i, j))
                seedset.add((i, j))

        # 連結成分ごとに必要なシード数を補う
        comp_seen = set()
        for start in sorted(cand):
            if start in comp_seen:
                continue
            comp = []
            dq = deque([start])
            comp_seen.add(start)
            while dq:
                c = dq.popleft()
                comp.append(c)
                for di, dj in DIRS:
                    n = (c[0] + di, c[1] + dj)
                    if n in cand and n not in comp_seen:
                        comp_seen.add(n)
                        dq.append(n)
            need = max(1, int(round(len(comp) / float(P["piece_cells"]))))
            have = sum(1 for c in comp if c in seedset)
            pool = sorted(comp)
            rng.shuffle(pool)
            for c in pool:
                if have >= need:
                    break
                if c not in seedset:
                    seeds.append(c)
                    seedset.add(c)
                    have += 1

        label = {}
        dq = deque()
        for idx, s in enumerate(seeds):
            label[s] = idx
            dq.append(s)
        while dq:
            c = dq.popleft()
            for di, dj in DIRS:
                n = (c[0] + di, c[1] + dj)
                if n in cand and n not in label:
                    label[n] = label[c]
                    dq.append(n)

        groups = defaultdict(list)
        for c, idx in label.items():
            groups[idx].append(c)

        # 小さすぎる部屋は隣に吸収
        changed = True
        while changed:
            changed = False
            for idx in sorted(groups):
                cells = groups.get(idx)
                if not cells or len(cells) >= P["min_cells"]:
                    continue
                nbr = Counter()
                for c in cells:
                    for di, dj in DIRS:
                        n = (c[0] + di, c[1] + dj)
                        if n in label and label[n] != idx:
                            nbr[label[n]] += 1
                if nbr:
                    tgt = nbr.most_common(1)[0][0]
                    for c in cells:
                        label[c] = tgt
                    groups[tgt].extend(cells)
                del groups[idx]
                changed = True
        return [(seeds[idx], sorted(cells)) for idx, cells in sorted(groups.items())]

    # --- 部屋の追加・削除 ---------------------------------------------------
    def add_piece(self, k, cells, seed):
        pid = self.next_pid
        self.next_pid += 1
        below = Counter(self.built[(i, j, k - 1)] for i, j in cells if (i, j, k - 1) in self.built)
        owner = None
        if below:
            bp, cnt = below.most_common(1)[0]
            if cnt * 2 >= len(cells) and self.rng.random() < self.P["owner_inherit"]:
                owner = self.pieces[bp]["owner"]
        if owner is None:
            owner = self.next_owner
            self.next_owner += 1
        self.pieces[pid] = {"level": k, "cells": list(cells), "seed": seed, "owner": owner}
        for i, j in cells:
            self.built[(i, j, k)] = pid
        return pid

    def remove_piece(self, pid):
        p = self.pieces.pop(pid)
        for i, j in p["cells"]:
            self.built.pop((i, j, p["level"]), None)

    # --- レベルごとの生成 ---------------------------------------------------
    def build_level(self, k):
        P, rng = self.P, self.rng
        cand = self.candidates(k)
        if not cand:
            self.prev_seeds = []
            return
        groups = self.partition(cand)
        rng.shuffle(groups)
        reach0 = self.compute_reach()
        new, claimed = [], set()
        height_decay = 1.0 - 0.35 * k / max(1, self.L - 1)
        for seed, cells in groups:
            on_roof = sum(1 for i, j in cells if (i, j, k - 1) in self.built)
            p = P["roof_prob"] if on_roof * 2 >= len(cells) else P["build_prob"]
            if rng.random() > p * height_decay:
                continue
            cellset = set(cells)
            door = False
            for i, j in cells:
                for di, dj in DIRS:
                    n = (i + di, j + dj)
                    if n not in cellset and n not in claimed and (n[0], n[1], k) in reach0:
                        door = True
                        break
                if door:
                    break
            if not door:
                continue
            new.append(self.add_piece(k, cells, seed))
            claimed |= cellset

        # 全住戸が到達可能か検査し、壊した新しい部屋を取り消す
        while new:
            reach = self.compute_reach()
            bad = [pid for pid in self.pieces if not self.piece_accessible(pid, reach)]
            if not bad:
                break
            newset = set(new)
            victims = [pid for pid in bad if pid in newset] or [new[-1]]
            for pid in victims:
                self.remove_piece(pid)
                new.remove(pid)
        self.prev_seeds = [self.pieces[pid]["seed"] for pid in new]

    # --- 所有の履歴 ---------------------------------------------------------
    def adjacency(self):
        adj = defaultdict(set)
        for (i, j, k), pid in self.built.items():
            for n in [(i + di, j + dj, k) for di, dj in DIRS] + [(i, j, k + 1), (i, j, k - 1)]:
                q = self.built.get(n)
                if q is not None and q != pid:
                    adj[pid].add(q)
        return adj

    def history_pass(self):
        P, rng = self.P, self.rng
        adj = self.adjacency()
        events = ["merge"] * int(P["merge_events"]) + ["split"] * int(P["split_events"])
        rng.shuffle(events)
        pids = sorted(self.pieces)
        for step, ev in enumerate(events, 1):
            if not pids:
                break
            if ev == "merge":
                a = rng.choice(pids)
                oa = self.pieces[a]["owner"]
                nb = sorted(q for q in adj[a] if self.pieces[q]["owner"] != oa)
                if not nb:
                    continue
                b = rng.choice(nb)
                pb = self.pieces[b]
                self.history.append(
                    "%02d 購入: 世帯%d が隣接住戸 #%d (L%d, 旧世帯%d) を取得"
                    % (step, oa, b, pb["level"], pb["owner"]))
                pb["owner"] = oa
            else:
                by_owner = defaultdict(list)
                for pid in pids:
                    by_owner[self.pieces[pid]["owner"]].append(pid)
                multi = sorted(o for o, ps in by_owner.items() if len(ps) >= 2)
                if not multi:
                    continue
                o = rng.choice(multi)
                pid = rng.choice(by_owner[o])
                new_owner = self.next_owner
                self.next_owner += 1
                self.history.append(
                    "%02d 分割: 住戸 #%d (L%d) が世帯%d から新世帯%d へ(相続・売却)"
                    % (step, pid, self.pieces[pid]["level"], o, new_owner))
                self.pieces[pid]["owner"] = new_owner

    # --- 動線と階段 ---------------------------------------------------------
    def route(self):
        dist, parent, heap = {}, {}, []
        for r in self.roots():
            dist[r] = 0.0
            heapq.heappush(heap, (0.0, r))
        while heap:
            d, u = heapq.heappop(heap)
            if d > dist.get(u, 1e18):
                continue
            for v, cost, _ in self.walk_edges(u):
                nd = d + cost
                if nd < dist.get(v, 1e18):
                    dist[v] = nd
                    parent[v] = u
                    heapq.heappush(heap, (nd, v))

        edge_use = Counter()
        for pid in sorted(self.pieces):
            p = self.pieces[pid]
            k = p["level"]
            best = None
            for i, j in p["cells"]:
                for di, dj in DIRS:
                    n = (i + di, j + dj, k)
                    if n in dist and (best is None or dist[n] < best[0]):
                        best = (dist[n], (i, j), n)
            if best is None:
                continue
            self.doors[pid] = (best[1], best[2])
            v = best[2]
            self.path_usage[v] += 1
            while v in parent:
                u = parent[v]
                edge_use[(u, v) if u < v else (v, u)] += 1
                self.path_usage[u] += 1
                v = u

        for (a, b), use in sorted(edge_use.items()):
            if a[2] != b[2]:
                lower, upper = (a, b) if a[2] < b[2] else (b, a)
                self.stairs.append((lower, upper, use))

    # --- 全体 ---------------------------------------------------------------
    def generate(self):
        for k in range(self.L):
            self.build_level(k)
        self.history_pass()
        self.route()
        return self

    # --- 形状(Houdini 非依存のポリゴンリスト) -------------------------------
    def to_mesh(self):
        P = self.P
        s, h = float(P["cell"]), float(P["floor_h"])
        mesh = []
        base = {"kind": "", "face": "", "level": -1, "piece": -1, "owner": -1,
                "on_ground": 0, "bridge": 0, "usage": 0, "Cd": (1.0, 1.0, 1.0)}

        def add(pts, **attrs):
            a = dict(base)
            a.update(attrs)
            mesh.append((pts, a))

        # 地形
        terrain_cd = (0.55, 0.5, 0.45)
        for i in range(self.nx):
            for j in range(self.ny):
                gy = self.ground[i][j] * h
                x0, x1, z0, z1 = i * s, (i + 1) * s, j * s, (j + 1) * s
                add(box_face(x0, x1, gy, gy, z0, z1, "+y"), kind="terrain", level=self.ground[i][j], Cd=terrain_cd)
                for d in DIRS:
                    a, c = i + d[0], j + d[1]
                    ny_ = self.ground[a][c] * h if self.inb(a, c) else -0.5 * h
                    if ny_ < gy:
                        add(box_face(x0, x1, ny_, gy, z0, z1, DIR_SIDE[d]), kind="terrain",
                            level=self.ground[i][j], Cd=terrain_cd)

        # 海
        add(box_face(-2 * s, (self.nx + 2) * s, -0.3 * h, -0.3 * h, -8 * s, 0, "+y"),
            kind="sea", Cd=(0.25, 0.45, 0.6))

        # 部屋
        bridge_pids = set()
        for (i, j, k), pid in self.built.items():
            if k > self.ground[i][j] and (i, j, k - 1) not in self.built:
                bridge_pids.add(pid)
        for (i, j, k), pid in sorted(self.built.items()):
            p = self.pieces[pid]
            cd = owner_color(p["owner"])
            common = dict(kind="room", level=k, piece=pid, owner=p["owner"],
                          on_ground=int(self.ground[i][j] == k), bridge=int(pid in bridge_pids), Cd=cd)
            x0, x1, z0, z1 = i * s, (i + 1) * s, j * s, (j + 1) * s
            y0, y1 = k * h, (k + 1) * h
            above = self.built.get((i, j, k + 1))
            add(box_face(x0, x1, y0, y1, z0, z1, "+y"), face="slab" if above is not None else "roof", **common)
            if k > self.ground[i][j] and (i, j, k - 1) not in self.built:
                add(box_face(x0, x1, y0, y1, z0, z1, "-y"), face="soffit", **common)
            for d in DIRS:
                a, c = i + d[0], j + d[1]
                q = self.built.get((a, c, k)) if self.inb(a, c) else None
                if q == pid:
                    continue
                if q is not None:
                    if pid < q:
                        add(box_face(x0, x1, y0, y1, z0, z1, DIR_SIDE[d]), face="party_wall", **common)
                    continue
                if self.inb(a, c) and self.ground[a][c] > k:
                    continue  # 土に接する面(擁壁)は省略
                add(box_face(x0, x1, y0, y1, z0, z1, DIR_SIDE[d]), face="facade", **common)

        # 階段: 下側セルの中で、上側セルに向かって上がる
        n = max(1, int(P["stair_steps"]))
        for lower, upper, use in self.stairs:
            i, j, k = lower
            d = (upper[0] - i, upper[1] - j)
            wide = use >= P["wide_stair"]
            w0, w1 = (0.0, 1.0) if wide else (0.25, 0.75)
            for st in range(n):
                t0, t1 = st / float(n), (st + 1) / float(n)
                top = k * h + (st + 1) * h / n
                if d[0] != 0:
                    xa, xb = (t0, t1) if d[0] > 0 else (1 - t1, 1 - t0)
                    za, zb = w0, w1
                else:
                    za, zb = (t0, t1) if d[1] > 0 else (1 - t1, 1 - t0)
                    xa, xb = w0, w1
                bx = ((i + xa) * s, (i + xb) * s, k * h, top, (j + za) * s, (j + zb) * s)
                for side in ("+y", "+x", "-x", "+z", "-z"):
                    add(box_face(*(bx + (side,))), kind="stair", level=k, usage=use, Cd=(0.92, 0.9, 0.85))

        # 通路(経路が通る歩行面)
        ins = 0.15 * s
        for (i, j, k), use in sorted(self.path_usage.items()):
            y = k * h + 0.03
            add(box_face(i * s + ins, (i + 1) * s - ins, y, y, j * s + ins, (j + 1) * s - ins, "+y"),
                kind="path", level=k, usage=use, Cd=(0.85, 0.3, 0.25))
        return mesh

    # --- テキスト表示 -------------------------------------------------------
    def ascii_levels(self):
        lines = []
        letters = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz"
        for k in range(self.L):
            rows = []
            for j in reversed(range(self.ny)):
                row = []
                for i in range(self.nx):
                    pid = self.built.get((i, j, k))
                    if pid is not None:
                        row.append(letters[self.pieces[pid]["owner"] % len(letters)])
                    elif k < self.ground[i][j]:
                        row.append("#")
                    elif self.surface(i, j, k):
                        row.append("+" if (i, j, k) in self.path_usage else "_")
                    else:
                        row.append(" ")
                rows.append("".join(row))
            cnt = sum(1 for p in self.pieces.values() if p["level"] == k)
            lines.append("== Level %d (部屋 %d) ==  #土 _歩行面 +経路 文字=世帯" % (k, cnt))
            lines.extend(rows)
        return "\n".join(lines)

    def stats(self):
        owners = set(p["owner"] for p in self.pieces.values())
        return ("部屋 %d / 世帯 %d / 建物セル %d / 階段 %d / 路地跨ぎ部屋 %d"
                % (len(self.pieces), len(owners), len(self.built), len(self.stairs),
                   len(set(pid for (i, j, k), pid in self.built.items()
                           if k > self.ground[i][j] and (i, j, k - 1) not in self.built))))


def box_face(x0, x1, y0, y1, z0, z1, side):
    """軸平行ボックスの1面を、外向き法線・反時計回り(右手系)で返す。"""
    if side == "+y":
        return [(x0, y1, z0), (x0, y1, z1), (x1, y1, z1), (x1, y1, z0)]
    if side == "-y":
        return [(x1, y0, z0), (x1, y0, z1), (x0, y0, z1), (x0, y0, z0)]
    if side == "+x":
        return [(x1, y0, z0), (x1, y1, z0), (x1, y1, z1), (x1, y0, z1)]
    if side == "-x":
        return [(x0, y0, z1), (x0, y1, z1), (x0, y1, z0), (x0, y0, z0)]
    if side == "+z":
        return [(x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    if side == "-z":
        return [(x0, y1, z0), (x1, y1, z0), (x1, y0, z0), (x0, y0, z0)]
    raise ValueError(side)


def owner_color(owner):
    hue = (owner * 0.61803398875) % 1.0
    return colorsys.hsv_to_rgb(hue, 0.45, 0.92)


def generate(params=None):
    return Town(params).generate()


# ---------------------------------------------------------------------------
# 書き出し
# ---------------------------------------------------------------------------
def write_obj(path, mesh):
    with open(path, "w") as f:
        f.write("# amalfi_layers\n")
        vi = 1
        cur = None
        for pts, a in mesh:
            grp = a["kind"] + ("_" + a["face"] if a["face"] else "")
            if grp != cur:
                f.write("g %s\n" % grp)
                cur = grp
            for p in pts:
                f.write("v %.4f %.4f %.4f\n" % p)
            f.write("f %s\n" % " ".join(str(vi + n) for n in range(len(pts))))
            vi += len(pts)


def write_houdini(geo, mesh, town):
    import hou

    int_attrs = ("level", "piece", "owner", "on_ground", "bridge", "usage")
    str_attrs = ("kind", "face")
    for name in int_attrs:
        geo.addAttrib(hou.attribType.Prim, name, 0)
    for name in str_attrs:
        geo.addAttrib(hou.attribType.Prim, name, "")
    geo.addAttrib(hou.attribType.Prim, "Cd", (1.0, 1.0, 1.0))
    geo.addAttrib(hou.attribType.Global, "history", "")
    geo.addAttrib(hou.attribType.Global, "stats", "")

    positions = [hou.Vector3(p) for pts, _ in mesh for p in pts]
    try:
        points = geo.createPoints(positions)
    except AttributeError:  # 古いバージョン向け
        points = []
        for pos in positions:
            pt = geo.createPoint()
            pt.setPosition(pos)
            points.append(pt)

    prims = []
    idx = 0
    for pts, _ in mesh:
        poly = geo.createPolygon()
        # Houdini は時計回りが表面なので頂点順を反転する
        for n in reversed(range(len(pts))):
            poly.addVertex(points[idx + n])
        idx += len(pts)
        prims.append(poly)

    attrs = [a for _, a in mesh]
    try:
        for name in int_attrs:
            geo.setPrimIntAttribValues(name, [a[name] for a in attrs])
        for name in str_attrs:
            geo.setPrimStringAttribValues(name, [a[name] for a in attrs])
        geo.setPrimFloatAttribValues("Cd", [c for a in attrs for c in a["Cd"]])
    except AttributeError:
        for prim, a in zip(prims, attrs):
            for name in int_attrs + str_attrs + ("Cd",):
                prim.setAttribValue(name, a[name])

    geo.setGlobalAttribValue("history", "\n".join(town.history))
    geo.setGlobalAttribValue("stats", town.stats())


def cook_python_sop():
    """Python SOP の中で実行されたときの処理。"""
    import hou

    node = hou.pwd()
    params = {}
    for name, default, _ in PARM_SPEC:
        parm = node.parm(name)
        if parm is not None:
            params[name] = type(default)(parm.eval())
    town = generate(params)
    write_houdini(node.geometry(), town.to_mesh(), town)


# Python SOP に入れる短いローダー(長いコードを貼ると字下げが崩れやすいため)
LOADER_CODE = """import sys, importlib
folder = r"%s"
if folder not in sys.path:
    sys.path.append(folder)
import amalfi_layers
importlib.reload(amalfi_layers)
amalfi_layers.cook_python_sop()
"""


def install(parent_path="/obj", name="amalfi_layers"):
    """Python Shell から呼ぶと、パラメータ付きの Python SOP を作る。"""
    import hou

    folder_path = os.path.dirname(os.path.abspath(__file__)).replace("\\", "/")
    code = LOADER_CODE % folder_path

    parent = hou.node(parent_path)
    geo = parent.createNode("geo", name)
    for child in geo.children():
        child.destroy()
    py = geo.createNode("python", "generate")

    ptg = py.parmTemplateGroup()
    folder = hou.FolderParmTemplate("amalfi", "Amalfi Layers")
    for pname, default, label in PARM_SPEC:
        if isinstance(default, int):
            folder.addParmTemplate(hou.IntParmTemplate(pname, label, 1, default_value=(default,)))
        else:
            folder.addParmTemplate(hou.FloatParmTemplate(pname, label, 1, default_value=(default,)))
    ptg.append(folder)
    py.setParmTemplateGroup(ptg)
    py.parm("python").set(code)
    py.setDisplayFlag(True)
    py.setRenderFlag(True)
    geo.layoutChildren()
    return py


def _in_python_sop():
    try:
        import hou
        node = hou.pwd()
        return isinstance(node, hou.SopNode) and node.type().name() == "python"
    except Exception:
        return False


if __name__ != "amalfi_layers" and _in_python_sop():
    # コードを Python SOP に直接貼り付けた場合(import された場合は呼び出し側が cook する)
    cook_python_sop()
elif __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser(description="Amalfi layered settlement generator")
    ap.add_argument("--seed", type=int, default=DEFAULTS["seed"])
    ap.add_argument("--obj", help="OBJ の書き出し先")
    ap.add_argument("--quiet", action="store_true", help="レベル平面を表示しない")
    args = ap.parse_args()
    t = generate({"seed": args.seed})
    if not args.quiet:
        print(t.ascii_levels())
    print(t.stats())
    print("\n".join(t.history))
    if args.obj:
        m = t.to_mesh()
        write_obj(args.obj, m)
        print("OBJ: %s (%d polygons)" % (args.obj, len(m)))
