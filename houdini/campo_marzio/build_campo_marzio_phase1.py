# -*- coding: utf-8 -*-
"""
build_campo_marzio_phase1.py
============================
HF_CAMPO_MARZIO_GRAMMAR (Phase 1) の SOP ネットワークを Houdini 上に組み立てる
ビルドスクリプト。Python SOP は使わない。生成されるのは標準 SOP と
Attribute Wrangle だけで、このスクリプトはノードを「作るだけ」である
(生成後の HDA はこのスクリプトに依存しない)。

使い方 (Houdini の Python Shell / Windows > Python Shell):

    import sys
    sys.path.append(r"<このファイルのあるフォルダ>")
    import build_campo_marzio_phase1 as b
    b.build()                       # /obj/campo_marzio_phase1 を作る
    # b.build(make_hda=True)        # さらに .hda として保存を試みる (未検証)

注意 (未検証事項):
  * このスクリプトは Houdini が無い環境で書かれており、実機では未実行。
  * Boolean / Block Begin / Block End / Connectivity / PolyExtrude の
    パラメータ名とメニュー項目はバージョンで異なりうる。見つからない場合は
    例外にせず、最後に「手動で設定すべき項目」として一覧表示する。
    README の「手動設定チェックリスト」に従って確認すること。
"""

import os

try:
    import hou
except ImportError:      # Houdini 外で import された場合 (構文チェック用)
    hou = None

HERE = os.path.dirname(os.path.abspath(__file__))
VEX_DIR = os.path.join(HERE, "vex")

HDA_NODE_NAME = "HF_CAMPO_MARZIO_GRAMMAR"
HDA_TYPE_NAME = "hf_campo_marzio_grammar"

RUN_OVER = {"detail": 0, "prim": 1, "point": 2}

_WARN = []


# ---------------------------------------------------------------------------
# ヘルパー
# ---------------------------------------------------------------------------
def _vex(name):
    with open(os.path.join(VEX_DIR, name + ".vfl"), encoding="utf-8") as f:
        return f.read()


def _warn(msg):
    _WARN.append(msg)
    print("[campo_marzio] WARNING:", msg)


def _connect(node, inputs):
    for i, src in enumerate(inputs):
        if src is None:
            continue
        if isinstance(src, tuple):
            node.setInput(i, src[0], src[1])
        else:
            node.setInput(i, src)


def _wrangle(parent, name, snippet, run_over, inputs=()):
    n = parent.createNode("attribwrangle", name)
    n.parm("class").set(RUN_OVER[run_over])
    n.parm("snippet").set(_vex(snippet))
    _connect(n, inputs)
    return n


def _node(parent, type_name, name, inputs=()):
    n = parent.createNode(type_name, name)
    _connect(n, inputs)
    return n


def _set(node, names, value):
    """候補名のどれかのパラメータに値を設定する。見つからなければ警告。"""
    for nm in names:
        p = node.parm(nm)
        if p is not None:
            try:
                p.set(value)
                return True
            except Exception as e:  # noqa: BLE001
                _warn("%s: %s を %r に設定できません (%s)" % (node.path(), nm, value, e))
                return False
    _warn("%s: パラメータ %s が見つかりません (値 %r を手動で設定)" % (node.path(), "/".join(names), value))
    return False


def _set_menu(node, names, wanted):
    """メニューパラメータを、トークンまたはラベル (大文字小文字無視) で設定する。"""
    wanted_l = [w.lower() for w in wanted]
    for nm in names:
        p = node.parm(nm)
        if p is None:
            continue
        tmpl = p.parmTemplate()
        try:
            items = list(tmpl.menuItems())
            labels = list(tmpl.menuLabels())
        except Exception:  # noqa: BLE001
            items, labels = [], []
        for idx, (it, lb) in enumerate(zip(items, labels)):
            if it.lower() in wanted_l or lb.lower() in wanted_l:
                try:
                    if tmpl.type() == hou.parmTemplateType.Int:
                        p.set(idx)
                    else:
                        p.set(it)
                    return True
                except Exception as e:  # noqa: BLE001
                    _warn("%s: %s を %s に設定できません (%s)" % (node.path(), nm, it, e))
                    return False
        _warn("%s: %s のメニューに %s がありません (候補: %s)"
              % (node.path(), nm, wanted, ", ".join(labels)))
        return False
    _warn("%s: メニューパラメータ %s が見つかりません (%s を手動で設定)"
          % (node.path(), "/".join(names), wanted[0]))
    return False


def _expr(node, name, expr):
    p = node.parm(name)
    if p is None:
        _warn("%s: パラメータ %s が見つかりません (式 %s を手動で設定)" % (node.path(), name, expr))
        return
    p.setExpression(expr, hou.exprLanguage.Hscript)


def _subnet(parent, name, inputs=()):
    s = parent.createNode("subnet", name)
    for c in s.children():          # 既定で作られる output 等を消す
        c.destroy()
    _connect(s, inputs)
    return s


def _output(parent, name, src, index):
    o = parent.createNode("output", name)
    o.setInput(0, src[0] if isinstance(src, tuple) else src,
               src[1] if isinstance(src, tuple) else 0)
    _set(o, ["outputidx"], index)
    return o


# ---------------------------------------------------------------------------
# HDA パラメータ
# ---------------------------------------------------------------------------
def _parm_templates():
    T = hou
    I = lambda n, l, v, h="", **k: T.IntParmTemplate(n, l, 1, default_value=(v,), help=h, **k)   # noqa: E731
    F = lambda n, l, v, h="", **k: T.FloatParmTemplate(n, l, 1, default_value=(v,), help=h, **k)  # noqa: E731
    B = lambda n, l, v, h="": T.ToggleParmTemplate(n, l, default_value=v, help=h)                  # noqa: E731
    S = lambda n, l, v, h="": T.StringParmTemplate(n, l, 1, default_value=(v,), help=h)            # noqa: E731

    domain = [
        I("domain_mode", "domain_mode  都市領域の指定方法", 1,
          "0: input 0 のポリゴン / 1: 矩形を生成",
          menu_items=("0", "1"), menu_labels=("Input Polygon  入力ポリゴン", "Rectangle  矩形")),
        F("domain_width", "domain_width  矩形の幅", 200.0, "domain_mode=Rectangle のときの X 方向の幅"),
        F("domain_height", "domain_height  矩形の高さ", 140.0, "domain_mode=Rectangle のときの Y 方向の幅"),
        F("target_fill_ratio", "target_fill_ratio  目標充填率", 0.60, "建築面積 / 都市領域面積 がこの値に達したら終了",
          min=0.0, max=1.0),
        F("minimum_building_area", "minimum_building_area  最小建築面積", 80.0,
          "これより小さい候補は棄却。これより小さい残余領域は候補生成から除外"),
        I("initial_layout_mode", "initial_layout_mode  初期配置", 1,
          "Phase 1 で使えるのは None / Manual Footprints のみ",
          menu_items=("0", "1", "2", "3"),
          menu_labels=("None  なし", "Manual Footprints  input 1 の建築",
                       "Voronoi Seeds  (Phase 1 未実装)", "Boundary Anchors  (Phase 1 未実装)")),
    ]
    shape = [
        S("shape_weights", "shape_weights  形状の重み", "1 0 0 0 0",
          "Rectangle HalfCircle Circle CompoundRectangle RectangleWithSemicircle。Phase 1 は Rectangle のみ使用"),
        F("base_size", "base_size  基準寸法", 20.0, "scale=1 の正方形の一辺"),
        F("min_scale", "min_scale  最小スケール", 0.6),
        F("max_scale", "max_scale  最大スケール", 1.8),
        F("max_aspect", "max_aspect  最大縦横比", 2.5, "width/depth の最大値 (1 以上)"),
        I("aspect_sample_count", "aspect_sample_count  縦横比の段階数", 3, "1 から max_aspect までを等分"),
        F("shape_variety", "shape_variety  形状の多様性 (Phase 2 予約)", 0.5),
        I("symmetry_mode", "symmetry_mode  対称性 (Phase 2 予約)", 0,
          menu_items=("0", "1", "2"), menu_labels=("None", "Bilateral", "Radial")),
    ]
    search = [
        F("angle_step", "angle_step  回転角の刻み (度)", 5.0, "0〜180 度未満をこの刻みで探索 (5 度なら 0..175)"),
        I("position_sample_count", "position_sample_count  位置サンプル数", 32, "1 反復あたりの位置サンプル総数 (面積比で各残余に配分)"),
        I("min_samples_per_region", "min_samples_per_region  領域ごとの最小サンプル数", 3),
        I("scale_sample_count", "scale_sample_count  スケールの段階数", 3),
        B("snap_to_boundary", "snap_to_boundary  境界への吸着候補を追加", True,
          "最寄りの境界線分に平行・接触する候補を追加する"),
        I("max_candidates", "max_candidates  候補数の上限", 60000),
        I("max_iterations", "max_iterations  最大反復回数", 40, "1 反復で建築を 1 棟配置"),
        I("random_seed", "random_seed  乱数シード", 1, "同じ値なら同じ結果になる"),
    ]
    placement = [
        F("clearance", "clearance  建築間の最小間隔", 0.0, "0 なら建築同士・都市境界への接触を許可"),
        F("contact_tolerance", "contact_tolerance  接触とみなす距離", 0.5),
        F("min_passage_width", "min_passage_width  最小通路幅", 4.0, "これ未満の隙間は通れない細片 (sliver) とみなす"),
        F("max_sliver_ratio", "max_sliver_ratio  細片率の上限", 0.5, "周長のうち sliver になる割合がこれを超える候補は棄却",
          min=0.0, max=1.0),
        I("perimeter_samples", "perimeter_samples  辺あたりの周長サンプル数", 6),
    ]
    evaluation = [
        F("weight_fill", "weight_fill  充填", 0.40, min=0.0, max=1.0),
        F("weight_fit", "weight_fit  適合", 0.25, min=0.0, max=1.0),
        F("weight_access", "weight_access  アクセス (暫定)", 0.20, min=0.0, max=1.0),
        F("weight_variety", "weight_variety  多様性", 0.10, min=0.0, max=1.0),
        F("weight_axis", "weight_axis  軸線", 0.05, min=0.0, max=1.0),
        I("variety_window", "variety_window  多様性の参照棟数", 5, "直近何棟と比較するか"),
        F("axis_radius", "axis_radius  軸線を参照する半径", 40.0),
    ]
    termination = [
        F("minimum_residual_area", "minimum_residual_area  最小残余面積", 100.0, "最大残余面積がこれを下回ったら終了"),
        F("minimum_improvement", "minimum_improvement  最小改善量", 0.0005, "1 反復の充填率の改善がこれ未満なら終了"),
        B("stop_when_no_candidate", "stop_when_no_candidate  候補なしで終了", True,
          "OFF のときは次の反復で別のサンプルを試す (反復回数は消費する)"),
    ]
    debug = [
        I("debug_mode", "debug_mode  表示モード", 0, "",
          menu_items=("0", "1", "2", "3", "4"),
          menu_labels=("Buildings  建築", "Residual  残余空間", "Candidates  候補",
                       "Score Visualization  スコア", "Generation Index  生成順")),
        B("show_candidates", "show_candidates  候補を表示", True),
        B("show_rejected_candidates", "show_rejected_candidates  棄却候補を表示", False),
        B("show_residual_ids", "show_residual_ids  残余 ID 点を出力", False),
        I("debug_top_k", "debug_top_k  上位候補の輪郭数", 15),
        B("keep_debug_in_loop", "keep_debug_in_loop  ループ内の残余・候補を保持", True,
          "ON: 最後の反復の残余・候補を出力 2/3 で確認できる (メモリを使う)"),
    ]

    def folder(name, label, items):
        return T.FolderParmTemplate(name, label, parm_templates=items, folder_type=T.folderType.Tabs)

    return [
        folder("f_domain", "Domain  都市領域", domain),
        folder("f_shape", "Shape  形状", shape),
        folder("f_search", "Search  探索", search),
        folder("f_place", "Placement  配置制約", placement),
        folder("f_eval", "Evaluation  評価", evaluation),
        folder("f_term", "Termination  終了条件", termination),
        folder("f_debug", "Debug  デバッグ", debug),
    ]


# ---------------------------------------------------------------------------
# サブネット
# ---------------------------------------------------------------------------
def _build_input_domain(hda):
    A = _subnet(hda, "INPUT_DOMAIN", [hda.indirectInputs()[0]])
    ii = A.indirectInputs()
    a01 = _wrangle(A, "A01_PARAMS", "a01_params", "detail", [ii[0]])
    a02 = _wrangle(A, "A02_DOMAIN_BUILD", "a02_domain_build", "detail", [a01])
    a03 = _wrangle(A, "A03_DOMAIN_CHECK", "a03_domain_check", "detail", [a02])
    _output(A, "OUT_DOMAIN", a03, 0)
    A.layoutChildren()
    return A


def _build_shape_library(hda, A):
    D = _subnet(hda, "SHAPE_LIBRARY", [A])
    d01 = _wrangle(D, "D01_SHAPE_LIBRARY", "d01_shape_library", "detail", [D.indirectInputs()[0]])
    _output(D, "OUT_SHAPES", d01, 0)
    D.layoutChildren()
    return D


def _build_initial_layout(hda, A):
    B = _subnet(hda, "INITIAL_LAYOUT", [A, hda.indirectInputs()[1]])
    ii = B.indirectInputs()
    b01 = _wrangle(B, "B01_INITIAL_BUILDINGS", "b01_initial_buildings", "detail", [ii[0], ii[1]])
    b02 = _wrangle(B, "B02_INIT_STATE", "b02_init_state", "detail", [b01])
    _output(B, "OUT_STATE", b02, 0)
    B.layoutChildren()
    return B


def _build_residual_extract(parent, state_src, name="RESIDUAL_EXTRACT"):
    """C: Residual = Domain - Union(Buildings)。output 0 = 残余ポリゴン, 1 = 境界線分。"""
    C = _subnet(parent, name, [state_src])
    st = C.indirectInputs()[0]
    c01 = _wrangle(C, "C01_KEEP_DOMAIN", "c01_keep_domain", "prim", [st])
    c02 = _wrangle(C, "C02_KEEP_BUILDINGS", "c02_keep_buildings", "prim", [st])

    ext = _node(C, "polyextrude", "C_EXTRUDE", [c02])
    _set(ext, ["dist"], 2.0)
    _set(ext, ["outputback"], 1)

    c03 = _wrangle(C, "C03_CENTER_SOLID_Z", "c03_center_solid_z", "point", [ext])

    boo = _node(C, "boolean", "C_BOOLEAN", [c01, c03])
    _set_menu(boo, ["asurface"], ["surface"])
    _set_menu(boo, ["bsurface"], ["solid"])
    _set_menu(boo, ["booleanop", "operation"], ["subtract"])
    _set_menu(boo, ["subtractchoices"], ["aminusb", "A - B", "A Minus B"])

    # 建築が 0 棟のときは Boolean を通さず都市領域をそのまま使う
    sw = _node(C, "switch", "C_SWITCH_NO_BUILDINGS", [boo, c01])
    _expr(sw, "input", 'if(npoints("../C02_KEEP_BUILDINGS") == 0, 1, 0)')

    fuse = _node(C, "fuse", "C_FUSE", [sw])

    c04 = _wrangle(C, "C04_RESIDUAL_CLEAN", "c04_residual_clean", "prim", [fuse, st])

    conn = _node(C, "connectivity", "C_CONNECTIVITY", [c04])
    _set_menu(conn, ["connecttype"], ["prim", "primitive", "Primitive"])
    _set(conn, ["attribname"], "residual_id")

    c05 = _wrangle(C, "C05_RESIDUAL_STATS", "c05_residual_stats", "detail", [conn, st])
    c06 = _wrangle(C, "C06_RESIDUAL_BOUNDARY", "c06_residual_boundary", "detail", [c05, st])

    _output(C, "OUT_RESIDUAL", c05, 0)
    _output(C, "OUT_BOUNDARY", c06, 1)
    C.layoutChildren()
    return C


def _build_iteration_controller(hda, B):
    G = _subnet(hda, "ITERATION_CONTROLLER", [B])
    st_in = G.indirectInputs()[0]

    begin = _node(G, "block_begin", "foreach_begin1", [st_in])
    end = G.createNode("block_end", "foreach_end1")

    _set_menu(begin, ["method"], ["feedback", "Fetch Feedback"])
    _set(begin, ["blockpath"], "../foreach_end1")
    _set_menu(end, ["itermethod"], ["count", "By Count"])
    _set_menu(end, ["method"], ["feedback", "Feedback Each Iteration"])
    _set(end, ["blockpath"], "../foreach_begin1")
    _set(end, ["templatepath"], "../foreach_begin1")
    _expr(end, "iterations", 'ch("../../max_iterations")')

    g01 = _wrangle(G, "G01_CLEAR_TRANSIENT", "g01_clear_transient", "detail", [begin])

    C = _build_residual_extract(G, g01)
    res, bnd = (C, 0), (C, 1)

    E = _subnet(G, "CANDIDATE_PLACEMENT", [res, bnd, g01])
    ei = E.indirectInputs()
    e01 = _wrangle(E, "E01_CANDIDATE_GENERATE", "e01_candidate_generate", "detail", [ei[0], ei[1], ei[2]])
    e02 = _wrangle(E, "E02_CANDIDATE_VALIDATE", "e02_candidate_validate", "point", [e01, ei[1], ei[0], ei[2]])
    _output(E, "OUT_CANDIDATES", e02, 0)
    E.layoutChildren()

    F = _subnet(G, "CANDIDATE_SCORING", [E, bnd, res, g01])
    fi = F.indirectInputs()
    f01 = _wrangle(F, "F01_CANDIDATE_SCORE", "f01_candidate_score", "point", [fi[0], fi[1], fi[2], fi[3]])
    _output(F, "OUT_SCORED", f01, 0)
    F.layoutChildren()

    g02 = _wrangle(G, "G02_SELECT_PLACE", "g02_select_place", "detail", [g01, F])
    g03 = _wrangle(G, "G03_RESIDUAL_FLAG", "g03_residual_flag", "prim", [res, g02])
    g04 = _wrangle(G, "G04_TERMINATION", "g04_termination", "detail", [g02, res])

    dbg_merge = _node(G, "merge", "G_DEBUG_MERGE", [g04, g03, F])
    dbg_sw = _node(G, "switch", "G_DEBUG_SWITCH", [g04, dbg_merge])
    _expr(dbg_sw, "input", 'detail("../G04_TERMINATION", "cfg_keep_debug_in_loop", 0)')

    # 停止後の反復は入力を素通しする (Block End の Stop Condition が無くても安全)
    stop_sw = _node(G, "switch", "G_STOP_SWITCH", [dbg_sw, begin])
    _expr(stop_sw, "input", 'detail("../foreach_begin1", "stop", 0)')

    end.setInput(0, stop_sw)
    _output(G, "OUT_STATE", end, 0)
    G.layoutChildren()
    return G


def _build_output_and_debug(hda, G):
    H = _subnet(hda, "OUTPUT_AND_DEBUG", [G])
    st = H.indirectInputs()[0]

    clean = _wrangle(H, "H00_CLEAN_STATE", "g01_clear_transient", "detail", [st])
    C = _build_residual_extract(H, clean, "FINAL_RESIDUAL_EXTRACT")
    h01 = _wrangle(H, "H01_FINAL_STATS", "h01_final_stats", "detail", [clean, (C, 0)])
    h02 = _wrangle(H, "H02_FINAL_RESIDUAL_FLAG", "h02_final_residual_flag", "prim", [(C, 0), h01])
    h03 = _wrangle(H, "H03_OUT_BUILDINGS", "h03_out_buildings", "detail", [h01])
    h04 = _wrangle(H, "H04_OUT_CANDIDATES", "h04_out_candidates", "detail", [st])
    h05 = _wrangle(H, "H05_TOPK_SHAPES", "h05_topk_shapes", "detail", [h04])
    h08 = _wrangle(H, "H08_RESIDUAL_LABELS", "h08_residual_labels", "detail", [h02, h01])

    vm = _node(H, "merge", "VIEW_MERGE", [h01, h02, h04, h05, h08])
    h06 = _wrangle(H, "H06_VIEW_PRIMS", "h06_view_prims", "prim", [vm, h01])
    h07 = _wrangle(H, "H07_VIEW_POINTS", "h07_view_points", "point", [h06, h01])

    _output(H, "OUT_BUILDINGS", h03, 0)
    _output(H, "OUT_RESIDUAL", h02, 1)
    _output(H, "OUT_VIEW", h07, 2)
    _output(H, "OUT_CANDIDATES", h04, 3)
    H.layoutChildren()
    return H


# ---------------------------------------------------------------------------
# エントリポイント
# ---------------------------------------------------------------------------
def build(obj_name="campo_marzio_phase1", make_hda=False, hda_path=None):
    """テスト用の geo ノードと、その中に HDA 化前のサブネットを作る。"""
    if hou is None:
        raise RuntimeError("Houdini の中で実行してください。")
    del _WARN[:]

    obj = hou.node("/obj")
    old = obj.node(obj_name)
    if old is not None:
        raise RuntimeError("/obj/%s は既に存在します。既存ノードを壊さないため中止しました。"
                           "名前を変えるか、既存ノードを手動で削除してください。" % obj_name)
    geo = obj.createNode("geo", obj_name)
    for c in geo.children():
        c.destroy()

    # テスト入力
    test_domain = geo.createNode("null", "TEST_DOMAIN_EMPTY")   # domain_mode=Rectangle なので空でよい
    test_fp = _wrangle(geo, "T01_TEST_FOOTPRINTS", "t01_test_footprints", "detail")

    hda = _subnet(geo, HDA_NODE_NAME, [test_domain, test_fp])
    ptg = hda.parmTemplateGroup()
    for t in _parm_templates():
        ptg.append(t)
    hda.setParmTemplateGroup(ptg)

    A = _build_input_domain(hda)
    D = _build_shape_library(hda, A)
    B = _build_initial_layout(hda, A)
    G = _build_iteration_controller(hda, B)
    H = _build_output_and_debug(hda, G)

    _output(hda, "output0", (H, 0), 0)   # Buildings
    _output(hda, "output1", (H, 1), 1)   # Residual
    _output(hda, "output2", (H, 2), 2)   # Debug view
    _output(hda, "output3", (H, 3), 3)   # Candidates (last iteration)
    hda.layoutChildren()

    # 表示用
    view = geo.createNode("null", "VIEW")
    view.setInput(0, hda, 2)
    view.setDisplayFlag(True)
    view.setRenderFlag(True)
    for i, nm in enumerate(["OUT_BUILDINGS", "OUT_RESIDUAL", "OUT_DEBUG_VIEW", "OUT_CANDIDATES"]):
        n = geo.createNode("null", nm)
        n.setInput(0, hda, i)
    geo.layoutChildren()

    if make_hda:
        _make_hda(hda, hda_path)

    print("[campo_marzio] built %s" % hda.path())
    if _WARN:
        print("[campo_marzio] 手動で確認・設定すべき項目 (%d 件):" % len(_WARN))
        for w in _WARN:
            print("   -", w)
    return hda


def _make_hda(subnet, hda_path=None):
    """サブネットを HDA に変換する (未検証。失敗した場合は手動で Create Digital Asset)。"""
    if hda_path is None:
        hda_path = os.path.join(HERE, HDA_TYPE_NAME + ".hda")
    try:
        node = subnet.createDigitalAsset(
            name=HDA_TYPE_NAME,
            hda_file_name=hda_path,
            description="HF Campo Marzio Grammar",
            min_num_inputs=0,
            max_num_inputs=3,
        )
        definition = node.type().definition()
        definition.setParmTemplateGroup(node.parmTemplateGroup())
        try:
            definition.setMaxNumOutputs(4)
        except Exception:  # noqa: BLE001
            _warn("HDA の出力数を 4 に設定できませんでした (Type Properties で設定)")
        print("[campo_marzio] HDA を保存しました:", hda_path)
        return node
    except Exception as e:  # noqa: BLE001
        _warn("HDA の自動作成に失敗しました (%s)。サブネットを右クリック > Create Digital Asset で作成してください。" % e)
        return None
