# -*- coding: utf-8 -*-
"""
ruined_city.py
==============
「廃墟の都市を探索する」授業用サンプル。
訪問履歴(visited)を入力として、都市の「接続関係」を再計算する。

考え方
------
Houdini のネットワークには時間が無いので、時間をシミュレーションする代わりに
「これまでに訪れた場所」を CTRL の文字列として持ち、都市の状態を毎回作り直す。

    visited    : 過去の行為   例) "archive tower"   (訪れた順に空白区切り)
    current    : 現在位置     例) "tower"           (空なら visited の最後)
    gate_state : 現在の接続状態(visited から計算される出力)

都市の形は変わらない。変わるのは接続関係(どの道が通れるか)だけ。

ゲートの条件(rule)
-------------------
各 Path(道)は rule を持つ。rule は visited に含まれる名前で書く。
    ""                 : 常に開いている
    "archive"          : archive を訪れたら開く               (Lock and key)
    "archive|tower"    : どちらかを訪れたら開く
    "archive&tower"    : 両方を訪れたら開く                   (ショートカット → Loop)
    "!bridge"          : bridge を訪れたら閉じる               (古い橋が崩れる)
    "device" / "!device" の組 : 一方が開くと他方が閉じる       (排他的な扉)
| は & より弱い("a&b|c" = (a かつ b) または c)。
oneway=1 の道は u → v の向きにしか通れない(高所へ上がると戻れない)。

授業での対応
------------
    C03–C04   鍵を見つけて扉を開く                 Tree・分岐
    C04       Archive を訪れると Tower への道が開く  条件付き接続
    C04       両方を訪れると Exit へのショートカット Loop・Semilattice
    C04       古い橋を渡ると崩れる                 接続の消失
    C06       高所へ上がると戻れない               Directed Path
    Sample 5  Tower が見えるか                     Landmark・視認性

使い方
------
A) Houdini の Python Shell から
       import sys; sys.path.append(r"<このファイルのあるフォルダ>")
       import ruined_city; ruined_city.install()
   → /obj/ruined_city の中に Python SOP が作られ、visited / current を
     パラメータとして書き換えられる。1語ずつ visited に足していくと、
     その時点で行ける範囲(reach)と通れる道が変わる。

B) Houdini なしで確認
       python ruined_city.py --visited "archive tower" --svg out.svg
   1歩ずつの経過・ゲートの状態・構造(Tree か Loop か)を表示する。

出力される属性
--------------
point (場所)  : name, kind="place", reach(1=今行ける), visited(訪問順, 0=未訪問),
                current(1=現在位置), tower_visible(1=Tower が見える)
point (道の上): kind="path_sample", tower_visible
prim  (道)    : name, rule, state("open"/"closed"), oneway, reach, on_route
                (現在位置から Exit への最短経路), Cd
detail        : visited, current, gate_state, structure, log
"""

import math
import os
from collections import deque

# ---------------------------------------------------------------------------
# 都市のデータ(学生はここを書き換えて自分の都市を設計する)
# ---------------------------------------------------------------------------
# 場所: 名前 -> (x, z, 高さ y, 種類)
PLACES = {
    "entry":   (0.0, 0.0, 0.0, "node"),
    "a":       (0.0, 12.0, 0.0, "node"),
    "b":       (0.0, 24.0, 0.0, "node"),
    "c":       (0.0, 36.0, 0.0, "node"),
    "archive": (16.0, 14.0, 0.0, "goal"),
    "tower":   (-16.0, 26.0, 0.0, "goal"),
    "exit":    (0.0, 50.0, 0.0, "goal"),
    "bridge":  (14.0, 36.0, 0.0, "event"),
    "device":  (22.0, 44.0, 0.0, "event"),
    "terrace": (-12.0, 40.0, 6.0, "event"),
}

# 道: (名前, u, v, rule, oneway)
PATHS = [
    ("street_1",      "entry",   "a",       "",              0),
    ("street_2",      "a",       "b",       "",              0),
    ("street_3",      "b",       "c",       "",              0),
    ("lane_archive",  "a",       "archive", "",              0),
    ("lane_tower",    "b",       "tower",   "",              0),
    # Lock and key: Archive で出口の情報を得ると、C から Exit への門が開く
    ("gate_exit",     "c",       "exit",    "archive",       0),
    # 条件付き接続: どちらかを先に訪れると、Archive と Tower を結ぶ別経路が見つかる
    ("archive_tower", "archive", "tower",   "archive|tower", 0),
    # ショートカット: 両方を訪れると Tower から Exit へ直接抜けられる(Tree → Loop)
    ("shortcut",      "tower",   "exit",    "archive&tower", 0),
    # 接続の消失: 古い橋は渡ると崩れる
    ("old_bridge",    "c",       "bridge",  "!bridge",       0),
    ("bridge_device", "bridge",  "device",  "",              0),
    # 排他的な扉: 装置を操作すると、こちらが開き……
    ("device_gate",   "device",  "exit",    "device",        0),
    # ……別の場所の扉が閉じる
    # (Archive で見つかる裏道。装置を操作すると閉じる)
    ("archive_back",  "archive", "c",       "archive&!device", 0),
    # Directed Path: 高台へ上がる階段は一方通行
    ("stairs_up",     "c",       "terrace", "",              1),
    ("high_road",     "terrace", "exit",    "archive",       0),
]

# 視線を遮る廃墟の塊(x0, z0, x1, z1)。Tower の視認性の計算に使う。
OBSTACLES = [
    (-10.0, 6.0, -3.0, 20.0),
    (3.0, 16.0, 8.0, 24.0),
    (-9.0, 30.0, -3.0, 36.0),
    (5.0, 2.0, 12.0, 9.0),
]

LANDMARK = "tower"
GOAL = "exit"
START = "entry"
SAMPLE_STEP = 1.0  # 道の上の視認性サンプル間隔

DEFAULT_VISITED = "archive tower"


# ---------------------------------------------------------------------------
# ゲート条件
# ---------------------------------------------------------------------------
def eval_rule(rule, visited):
    """rule を visited(集合)に対して評価する。"""
    rule = rule.strip()
    if not rule:
        return True
    for alt in rule.split("|"):
        ok = True
        for term in alt.split("&"):
            term = term.strip()
            if not term:
                continue
            if term.startswith("!"):
                ok = ok and term[1:].strip() not in visited
            else:
                ok = ok and term in visited
        if ok:
            return True
    return False


def parse_visited(text):
    return [w for w in text.replace(",", " ").split() if w]


# ---------------------------------------------------------------------------
# 都市の状態
# ---------------------------------------------------------------------------
class CityState(object):
    def __init__(self, visited_list, places=None, paths=None):
        self.places = places or PLACES
        self.paths = paths or PATHS
        self.visited_list = list(visited_list)
        self.visited = set(visited_list)
        self.open = {name: eval_rule(rule, self.visited)
                     for name, _, _, rule, _ in self.paths}

    def neighbors(self, node):
        """今の状態で node から1歩で行ける場所。"""
        for name, u, v, _, oneway in self.paths:
            if not self.open[name]:
                continue
            if u == node:
                yield v, name
            elif v == node and not oneway:
                yield u, name

    def reach(self, start):
        seen = {start}
        q = deque([start])
        while q:
            n = q.popleft()
            for m, _ in self.neighbors(n):
                if m not in seen:
                    seen.add(m)
                    q.append(m)
        return seen

    def route(self, start, goal):
        """start から goal への最短経路(道の名前のリスト)。無ければ None。"""
        def dist(a, b):
            pa, pb = self.places[a], self.places[b]
            return math.sqrt((pa[0] - pb[0]) ** 2 + (pa[1] - pb[1]) ** 2 + (pa[2] - pb[2]) ** 2)

        best = {start: 0.0}
        prev = {}
        todo = [(0.0, start)]
        while todo:
            todo.sort()
            d, n = todo.pop(0)
            if n == goal:
                break
            if d > best.get(n, 1e18):
                continue
            for m, pname in self.neighbors(n):
                nd = d + dist(n, m)
                if nd < best.get(m, 1e18):
                    best[m] = nd
                    prev[m] = (n, pname)
                    todo.append((nd, m))
        if goal not in best:
            return None
        out = []
        n = goal
        while n != start:
            n, pname = prev[n]
            out.append(pname)
        return list(reversed(out))

    def structure(self, start):
        """start と(向きを無視して)つながっている部分の構造。
        閉路の数 = E - V + 1。0 なら Tree。"""
        adj = {n: set() for n in self.places}
        for name, u, v, _, _ in self.paths:
            if self.open[name]:
                adj[u].add(v)
                adj[v].add(u)
        comp = {start}
        stack = [start]
        while stack:
            k = stack.pop()
            for m in adj[k]:
                if m not in comp:
                    comp.add(m)
                    stack.append(m)
        edges = sum(1 for name, u, v, _, _ in self.paths if self.open[name] and u in comp)
        loops = edges - len(comp) + 1
        kind = "tree" if loops == 0 else "loops=%d" % loops
        cut = len(self.places) - len(comp)
        if cut:
            kind += " (切り離された場所 %d)" % cut
        return kind

    def gate_state(self):
        return " ".join("%s:%s" % (n, "open" if self.open[n] else "closed")
                        for n, _, _, rule, _ in self.paths if rule)


def simulate(visited_text, current_text=""):
    """visited を1語ずつ再生し、各時点で到達可能だったかを検査する。"""
    seq = parse_visited(visited_text)
    log = []
    here = START
    done = []
    for step, place in enumerate(seq, 1):
        if place not in PLACES:
            log.append("%d. %s : 都市にない場所(無視)" % (step, place))
            continue
        before = CityState(done)
        ok = place in before.reach(here)
        done.append(place)
        after = CityState(done)
        changed = [n for n in after.open if after.open[n] != before.open[n]]
        msg = "%d. %s -> %s" % (step, here, place)
        if not ok:
            msg += "  [!] この時点では到達できない"
        if changed:
            msg += "  | " + ", ".join(
                "%s %s" % (n, "開" if after.open[n] else "閉") for n in changed)
        log.append(msg)
        here = place
    state = CityState(done)
    current = current_text.strip() or here
    if current not in PLACES:
        log.append("current=%s は都市にない場所。%s を使う" % (current, here))
        current = here
    elif current != here and current not in state.reach(here):
        log.append("[!] current=%s には %s から行けない" % (current, here))
    return state, current, log


# ---------------------------------------------------------------------------
# 視認性(Landmark)
# ---------------------------------------------------------------------------
def _seg_hits_box(p, q, box):
    """線分 pq が矩形 box と交わるか(Liang–Barsky)。"""
    x0, z0, x1, z1 = box
    t0, t1 = 0.0, 1.0
    dx, dz = q[0] - p[0], q[1] - p[1]
    for pk, qk in ((-dx, p[0] - x0), (dx, x1 - p[0]), (-dz, p[1] - z0), (dz, z1 - p[1])):
        if pk == 0:
            if qk < 0:
                return False
        else:
            r = qk / pk
            if pk < 0:
                t0 = max(t0, r)
            else:
                t1 = min(t1, r)
            if t0 > t1:
                return False
    return True


def tower_visible(x, z, y=0.0):
    tx, tz, _, _ = PLACES[LANDMARK]
    if y > 0.0:  # 高所からは廃墟越しに見える
        return 1
    for box in OBSTACLES:
        if _seg_hits_box((x, z), (tx, tz), box):
            return 0
    return 1


def path_samples(u, v):
    pu, pv = PLACES[u], PLACES[v]
    length = math.sqrt((pv[0] - pu[0]) ** 2 + (pv[1] - pu[1]) ** 2)
    n = max(1, int(length / SAMPLE_STEP))
    return [tuple(pu[k] + (pv[k] - pu[k]) * i / n for k in range(3)) for i in range(n + 1)]


# ---------------------------------------------------------------------------
# 書き出し
# ---------------------------------------------------------------------------
COL_CLOSED = (0.75, 0.15, 0.12)
COL_OPEN = (0.55, 0.55, 0.55)
COL_REACH = (0.20, 0.45, 0.95)
COL_ROUTE = (1.00, 0.80, 0.10)


def path_color(state, reach, on_route):
    if not state:
        return COL_CLOSED
    if on_route:
        return COL_ROUTE
    return COL_REACH if reach else COL_OPEN


def build(visited_text=DEFAULT_VISITED, current_text=""):
    state, current, log = simulate(visited_text, current_text)
    reach = state.reach(current)
    route = state.route(current, GOAL)
    order = {}
    for i, p in enumerate(state.visited_list, 1):
        order.setdefault(p, i)

    places = []
    for name, (x, z, y, kind) in PLACES.items():
        places.append({
            "name": name, "P": (x, y, z), "kind": kind,
            "reach": int(name in reach), "visited": order.get(name, 0),
            "current": int(name == current), "tower_visible": tower_visible(x, z, y),
        })
    paths = []
    for name, u, v, rule, oneway in PATHS:
        is_open = state.open[name]
        r = int(is_open and (u in reach or (not oneway and v in reach)))
        on = int(name in (route or []))
        pts = path_samples(u, v)
        lift = max(PLACES[u][2], PLACES[v][2]) > 0
        paths.append({
            "name": name, "u": u, "v": v, "rule": rule, "oneway": oneway,
            "state": "open" if is_open else "closed", "reach": r, "on_route": on,
            "Cd": path_color(is_open, r, on),
            "points": [((x, y, z), tower_visible(x, z, y if lift else 0.0)) for x, z, y in pts],
        })
    detail = {
        "visited": " ".join(state.visited_list),
        "current": current,
        "gate_state": state.gate_state(),
        "structure": state.structure(current),
        "route": ("(Exit に到着)" if current == GOAL else
                  " ".join(route) if route else "(Exit へは行けない)"),
        "log": "\n".join(log),
    }
    return places, paths, detail


def write_houdini(geo, places, paths, detail):
    import hou

    geo.addAttrib(hou.attribType.Point, "name", "")
    geo.addAttrib(hou.attribType.Point, "kind", "")
    for a in ("reach", "visited", "current", "tower_visible"):
        geo.addAttrib(hou.attribType.Point, a, 0)
    geo.addAttrib(hou.attribType.Prim, "name", "")
    geo.addAttrib(hou.attribType.Prim, "rule", "")
    geo.addAttrib(hou.attribType.Prim, "state", "")
    for a in ("oneway", "reach", "on_route"):
        geo.addAttrib(hou.attribType.Prim, a, 0)
    geo.addAttrib(hou.attribType.Prim, "Cd", (1.0, 1.0, 1.0))
    for k in detail:
        geo.addAttrib(hou.attribType.Global, k, "")

    for p in places:
        pt = geo.createPoint()
        pt.setPosition(hou.Vector3(p["P"]))
        for a in ("name", "reach", "visited", "current", "tower_visible"):
            pt.setAttribValue(a, p[a])
        pt.setAttribValue("kind", "place")

    for e in paths:
        poly = geo.createPolygon()
        poly.setIsClosed(False)
        for pos, vis in e["points"]:
            pt = geo.createPoint()
            pt.setPosition(hou.Vector3(pos))
            pt.setAttribValue("kind", "path_sample")
            pt.setAttribValue("tower_visible", vis)
            poly.addVertex(pt)
        for a in ("name", "rule", "state", "oneway", "reach", "on_route"):
            poly.setAttribValue(a, e[a])
        poly.setAttribValue("Cd", e["Cd"])

    for k, v in detail.items():
        geo.setGlobalAttribValue(k, v)


def write_svg(path, places, paths, detail, scale=12.0, margin=40.0):
    xs = [p["P"][0] for p in places] + [b[0] for b in OBSTACLES] + [b[2] for b in OBSTACLES]
    zs = [p["P"][2] for p in places] + [b[1] for b in OBSTACLES] + [b[3] for b in OBSTACLES]
    minx, maxx, minz, maxz = min(xs), max(xs), min(zs), max(zs)
    w = (maxx - minx) * scale + margin * 2 + 60  # 右側はラベルの分を空ける
    h = (maxz - minz) * scale + margin * 2 + 60

    def tx(x, z):  # z が上(海→山のように下から上へ進む)
        return (x - minx) * scale + margin, (maxz - z) * scale + margin

    def rgb(c):
        return "rgb(%d,%d,%d)" % tuple(int(255 * k) for k in c)

    out = ['<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d" '
           'font-family="sans-serif" font-size="12">' % (w, h),
           '<rect width="100%" height="100%" fill="#f6f3ec"/>',
           '<defs><marker id="arr" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" '
           'markerHeight="7" orient="auto"><path d="M0,0L10,5L0,10z" fill="#333"/></marker></defs>']
    for b in OBSTACLES:
        x0, y1 = tx(b[0], b[1])
        x1, y0 = tx(b[2], b[3])
        out.append('<rect x="%.1f" y="%.1f" width="%.1f" height="%.1f" fill="#c9bfae"/>'
                   % (x0, y0, x1 - x0, y1 - y0))
    for e in paths:
        pts = e["points"]
        # 視認性: Tower が見える区間を道の下に薄い緑で敷く
        for (a, va), (b, vb) in zip(pts, pts[1:]):
            if va and vb:
                (x0, y0), (x1, y1) = tx(a[0], a[2]), tx(b[0], b[2])
                out.append('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="#9fd89a" '
                           'stroke-width="12" stroke-linecap="round"/>' % (x0, y0, x1, y1))
    for e in paths:
        (x0, y0), (x1, y1) = tx(e["points"][0][0][0], e["points"][0][0][2]), \
            tx(e["points"][-1][0][0], e["points"][-1][0][2])
        dash = ' stroke-dasharray="6,5"' if e["state"] == "closed" else ""
        mark = ' marker-end="url(#arr)"' if e["oneway"] else ""
        out.append('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="%s" stroke-width="%d"%s%s/>'
                   % (x0, y0, x1, y1, rgb(e["Cd"]), 5 if e["on_route"] else 3, dash, mark))
        if e["rule"]:
            # 線の中点から法線方向に少しずらしてラベルを置く
            ln = math.hypot(x1 - x0, y1 - y0) or 1.0
            nx, ny = -(y1 - y0) / ln, (x1 - x0) / ln
            if ny > 0:
                nx, ny = -nx, -ny
            out.append('<text x="%.1f" y="%.1f" fill="#555" font-size="10" text-anchor="middle">%s</text>'
                       % ((x0 + x1) / 2 + nx * 9, (y0 + y1) / 2 + ny * 9 + 3,
                          e["rule"].replace("&", "&amp;")))
    for p in places:
        x, y = tx(p["P"][0], p["P"][2])
        fill = rgb(COL_REACH) if p["reach"] else "#bbb"
        stroke = "#000" if p["current"] else "#fff"
        r = 11 if p["kind"] == "goal" else 8
        out.append('<circle cx="%.1f" cy="%.1f" r="%d" fill="%s" stroke="%s" stroke-width="3"/>'
                   % (x, y, r, fill, stroke))
        label = p["name"] + (" (%d)" % p["visited"] if p["visited"] else "")
        out.append('<text x="%.1f" y="%.1f" font-weight="%s">%s</text>'
                   % (x + 13, y + 4, "bold" if p["current"] else "normal", label))
    y = h - 46
    for line in ("visited: %s   current: %s" % (detail["visited"] or "-", detail["current"]),
                 "structure: %s   route to exit: %s" % (detail["structure"], detail["route"])):
        out.append('<text x="%d" y="%d">%s</text>' % (margin, y, line))
        y += 18
    out.append("</svg>")
    with open(path, "w") as f:
        f.write("\n".join(out))


# ---------------------------------------------------------------------------
# Houdini
# ---------------------------------------------------------------------------
def cook_python_sop():
    import hou

    node = hou.pwd()
    vparm, cparm = node.parm("visited"), node.parm("current")
    visited = vparm.eval() if vparm is not None else DEFAULT_VISITED
    current = cparm.eval() if cparm is not None else ""
    places, paths, detail = build(visited, current)
    write_houdini(node.geometry(), places, paths, detail)


LOADER_CODE = """import sys, importlib
folder = r"%s"
if folder not in sys.path:
    sys.path.append(folder)
import ruined_city
importlib.reload(ruined_city)
ruined_city.cook_python_sop()
"""


def install(parent_path="/obj", name="ruined_city"):
    """Python Shell から呼ぶと、visited / current パラメータ付きの Python SOP を作る。"""
    import hou

    folder_path = os.path.dirname(os.path.abspath(__file__)).replace("\\", "/")
    parent = hou.node(parent_path)
    geo = parent.createNode("geo", name)
    for child in geo.children():
        child.destroy()
    py = geo.createNode("python", "city_state")

    ptg = py.parmTemplateGroup()
    folder = hou.FolderParmTemplate("ctrl", "CTRL")
    folder.addParmTemplate(hou.StringParmTemplate(
        "visited", "visited(訪れた順)", 1, default_value=(DEFAULT_VISITED,)))
    folder.addParmTemplate(hou.StringParmTemplate(
        "current", "current(空 = 最後の訪問先)", 1, default_value=("",)))
    ptg.append(folder)
    py.setParmTemplateGroup(ptg)
    py.parm("python").set(LOADER_CODE % folder_path)
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


if __name__ != "ruined_city" and _in_python_sop():
    cook_python_sop()
elif __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser(description="Ruined city: visited history -> connectivity")
    ap.add_argument("--visited", default=DEFAULT_VISITED, help='例: "archive tower"')
    ap.add_argument("--current", default="", help="現在位置(空なら最後の訪問先)")
    ap.add_argument("--svg", help="SVG の書き出し先")
    args = ap.parse_args()
    pl, pa, de = build(args.visited, args.current)
    print(de["log"] or "(まだどこも訪れていない)")
    print("current   :", de["current"])
    print("reach     :", " ".join(p["name"] for p in pl if p["reach"]))
    print("gate_state:", de["gate_state"])
    print("structure :", de["structure"])
    print("route     :", de["route"])
    print("Tower が見える場所:", " ".join(p["name"] for p in pl if p["tower_visible"]))
    if args.svg:
        write_svg(args.svg, pl, pa, de)
        print("SVG:", args.svg)
