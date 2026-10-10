# HF_CAMPO_MARZIO_GRAMMAR — Phase 1(最小実装)

ピラネージ《カンポ・マルツィオ》型の空間充填文法を Houdini SOP で実装する HDA の **Phase 1** です。
矩形の都市領域・矩形の初期建築・長方形候補だけで、「残余抽出 → 候補生成 → 検査 → 採点 → 1 棟配置 → 残余再抽出」を反復します。

> **状態(正直な申告)**
> - VEX スニペットとネットワーク構築スクリプトは、Houdini の無い環境で書きました。**Houdini 上では一度も実行していません。** 実機で確認すべき点は [8. 未検証の API・機能](#8-未検証の-api機能) にまとめています。
> - アルゴリズム自体は、同じ判定式・採点式を使う Python の参照実装([`reference/`](reference/))で検証済みです。Phase 1 の合格条件を確認するテスト 8 件がすべて通っています([7.3](#73-参照実装による検証houdini-外))。
> - Phase 2 以降(半円・円・複合矩形、Voronoi 初期配置、Campo Marzio モードなど)は**実装していません**。

```
houdini/campo_marzio/
├── README.md                       ← この文書
├── build_campo_marzio_phase1.py    ← Houdini の Python Shell でネットワークを自動構築
├── vex/                            ← 各 Attribute Wrangle の VEX (そのまま貼り付け可)
│   ├── a01..a03  INPUT_DOMAIN
│   ├── b01..b02  INITIAL_LAYOUT
│   ├── c01..c06  RESIDUAL_EXTRACT
│   ├── d01       SHAPE_LIBRARY
│   ├── e01..e02  CANDIDATE_PLACEMENT
│   ├── f01       CANDIDATE_SCORING
│   ├── g01..g04  ITERATION_CONTROLLER
│   ├── h01..h08  OUTPUT_AND_DEBUG
│   └── t01       テスト用初期建築
└── reference/                      ← Houdini 外の参照実装とテスト (HDA の一部ではない)
    ├── cm_phase1_reference.py
    ├── test_phase1_reference.py
    └── preview_reference_seed1.png
```

既存の `houdini/amalfi_layers.py` には変更を加えていません。

---

## 1. HDA 内部のノード構成図

```
HF_CAMPO_MARZIO_GRAMMAR  (SOP HDA, inputs: 0 domain / 1 footprints / 2 予約)
│
├─ A INPUT_DOMAIN ─────────────────────────────── in: HDA input 0
│    A01_PARAMS (W:detail) → A02_DOMAIN_BUILD (W:detail) → A03_DOMAIN_CHECK (W:detail) → out0
│
├─ D SHAPE_LIBRARY ────────────────────────────── in: A
│    D01_SHAPE_LIBRARY (W:detail) → out0          (プレビュー/形状規約。フローには不使用)
│
├─ B INITIAL_LAYOUT ───────────────────────────── in: A, HDA input 1
│    B01_INITIAL_BUILDINGS (W:detail, in1=footprints) → B02_INIT_STATE (W:detail) → out0 = 状態
│
├─ G ITERATION_CONTROLLER ─────────────────────── in: B
│    foreach_begin1 (Block Begin: Fetch Feedback)
│      ├──────────────────────────────────────────────────────────────┐
│      ▼                                                              │
│    G01_CLEAR_TRANSIENT (W:detail)                                   │
│      ▼                                                              │
│    C RESIDUAL_EXTRACT (subnet) ──out0 残余ポリゴン──┐                │
│      │                        ──out1 境界線分──────┤                │
│      ▼                                             ▼                │
│    E CANDIDATE_PLACEMENT (subnet)                                   │
│        E01_CANDIDATE_GENERATE (W:detail)  in: 残余, 境界, 状態      │
│        E02_CANDIDATE_VALIDATE (W:point)   in: 候補, 境界, 残余, 状態│
│      ▼                                                              │
│    F CANDIDATE_SCORING (subnet)                                     │
│        F01_CANDIDATE_SCORE (W:point)      in: 候補, 境界, 残余, 状態│
│      ▼                                                              │
│    G02_SELECT_PLACE (W:detail)            in: 状態, 採点済み候補     │
│      ├→ G03_RESIDUAL_FLAG (W:prim)        in: 残余, G02  (デバッグ)  │
│      ▼                                                              │
│    G04_TERMINATION (W:detail)             in: G02, 残余             │
│      ▼                                                              │
│    G_DEBUG_MERGE (Merge: G04, G03, F) ─┐                             │
│    G_DEBUG_SWITCH (Switch: G04 | MERGE) ◄ cfg_keep_debug_in_loop     │
│      ▼                                                              │
│    G_STOP_SWITCH (Switch: DEBUG_SWITCH | foreach_begin1) ◄──────────┘ stop==1 なら素通し
│      ▼
│    foreach_end1 (Block End: Feedback, By Count, Iterations = max_iterations) → out0
│
└─ H OUTPUT_AND_DEBUG ─────────────────────────── in: G
     H00_CLEAN_STATE (W:detail, g01 と同じコード)
     FINAL_RESIDUAL_EXTRACT (C と同じ構成のサブネット)
     H01_FINAL_STATS (W:detail)      in: H00, 最終残余        ← 合格条件の独立検証
     H02_FINAL_RESIDUAL_FLAG (W:prim) in: 最終残余, H01
     H03_OUT_BUILDINGS (W:detail)    in: H01                  → out0 Buildings
     H04_OUT_CANDIDATES (W:detail)   in: G 出力                → out3 Candidates
     H05_TOPK_SHAPES (W:detail)      in: H04
     H08_RESIDUAL_LABELS (W:detail)  in: H02, H01
     VIEW_MERGE (Merge: H01, H02, H04, H05, H08)
     H06_VIEW_PRIMS (W:prim)  → H07_VIEW_POINTS (W:point)      → out2 Debug View
     H02                                                       → out1 Residual

C RESIDUAL_EXTRACT (subnet, in: 状態) — Residual = Domain − Union(Buildings)
     C01_KEEP_DOMAIN (W:prim) ─────────────────────────────┐ A: Surface
     C02_KEEP_BUILDINGS (W:prim) → C_EXTRUDE (PolyExtrude)   │
        → C03_CENTER_SOLID_Z (W:point) ────────────────────┤ B: Solid
                                     C_BOOLEAN (Boolean: Subtract A−B)
     C_SWITCH_NO_BUILDINGS (Switch: BOOLEAN | KEEP_DOMAIN) ◄ 建築 0 棟なら Boolean を迂回
     → C_FUSE (Fuse) → C04_RESIDUAL_CLEAN (W:prim, in1=状態)
     → C_CONNECTIVITY (Connectivity: Primitive, residual_id)
     → C05_RESIDUAL_STATS (W:detail, in1=状態)   → out0 残余ポリゴン
     → C06_RESIDUAL_BOUNDARY (W:detail, in1=状態) → out1 境界線分
```

`W:detail` = Attribute Wrangle(Run Over: Detail (only once))、`W:prim` = Primitives、`W:point` = Points。

### 設計の要点

- **状態ジオメトリ**: ループが受け渡すのは「都市領域ポリゴン 1 枚 + 建築ポリゴン群 + detail 属性(`cfg_*` と反復状態)」です。残余空間は毎反復 Boolean で作り直します(仕様 G の 1→7)。
- **パラメータの受け渡し**: `ch()` を呼ぶのは `A01_PARAMS` だけです。HDA のパラメータを `cfg_*` detail 属性にして状態と一緒に運ぶので、ループやサブネットの深さに関係なく同じ値を読めます。
- **厳密な内外判定**: 候補の判定に境界ボックスは使いません(広域判定の `primfind` にだけ使います)。「残余境界 = 都市領域の外周 + 既存建築の外周」なので、候補が残余内に完全に収まることを ①中心が残余上、②辺が境界線分と交差しない、③境界線分の端点・中点が図形内部にない、の 3 条件で確定します。これで「はみ出さない」と「重ならない」を同時に保証します。非凸・穴あきの残余にもそのまま使えます。
- **1 反復 1 棟**: 全残余領域の全候補から最良の 1 つだけを採用します。同点のときは `cand_id` の小さい方を採るので、結果は決定的です。
- **停止の二重化**: `G04` が `i@stop` を立てると、以降の反復は `G_STOP_SWITCH` が入力をそのまま通すので状態は変わりません。Block End の Stop Condition が無くても、意味を取り違えても、結果は正しくなります。
- **棄却理由を残す**: 全候補に `i@reject_code` / `s@reject_reason` を付けます。反復ごとの集計は `i[]@reject_counts` です。

---

## 2. ノードごとの役割と接続

| ノード | 種別 | 入力 | 役割 |
|---|---|---|---|
| A01_PARAMS | Wrangle detail | HDA in0 | HDA パラメータ → `cfg_*` detail 属性。重みの合計・スケール範囲などを検証 |
| A02_DOMAIN_BUILD | Wrangle detail | A01 | `domain_mode` に従い、矩形を生成するか入力ポリゴンを採用。Z=0 に投影し、`class="domain"` を付ける |
| A03_DOMAIN_CHECK | Wrangle detail | A02 | 頂点数・重複点・ゼロ面積・自己交差を検査。`f@domain_area` を書く |
| D01_SHAPE_LIBRARY | Wrangle detail | A | Rectangle の原点基準プロトタイプを出力。形状規約(角の順序・width/depth の定義)の基準 |
| B01_INITIAL_BUILDINGS | Wrangle detail | A, HDA in1 | Manual Footprints を検証して建築化する。はみ出し・重複・ゼロ面積のものは除外して警告 |
| B02_INIT_STATE | Wrangle detail | B01 | `iter`・`stop`・`fill_ratio`・`next_building_id`・Variety 用履歴・ヒストリ配列を初期化 |
| foreach_begin1 / foreach_end1 | Block Begin / End | B | Fetch Feedback で状態を受け渡す。`max_iterations` 回 |
| G01_CLEAR_TRANSIENT | Wrangle detail | begin | 前回のデバッグ用残余・候補を状態から削除 |
| C (RESIDUAL_EXTRACT) | subnet | G01 | Boolean で残余を作り、連結領域ごとの特徴量と境界線分を出力 |
| E01_CANDIDATE_GENERATE | Wrangle detail | 残余, 境界, 状態 | 位置サンプル × 角度 × スケール × 縦横比 + 境界吸着候補 を点として生成 |
| E02_CANDIDATE_VALIDATE | Wrangle point | 候補, 境界, 残余, 状態 | 不正形状・最小面積・はみ出し・重複を判定 (`reject_code` 1,2,3,5) |
| F01_CANDIDATE_SCORE | Wrangle point | 同上 | Fill/Fit/Access/Variety/Axis を計算してスコア化。細片を作る候補を棄却 (code 4) |
| G02_SELECT_PLACE | Wrangle detail | G01, F | 最良候補を矩形ポリゴンとして追加。候補がなければ配置しない |
| G03_RESIDUAL_FLAG | Wrangle prim | 残余, G02 | 領域ごとの有効候補数と `unfilled` を記録(デバッグ表示用) |
| G04_TERMINATION | Wrangle detail | G02, 残余 | 充填率・改善量・ヒストリを更新し、終了条件を判定 |
| G_DEBUG_MERGE / SWITCH | Merge / Switch | | `keep_debug_in_loop=1` のとき、残余・候補を状態に合流させて出力 |
| G_STOP_SWITCH | Switch | | `stop==1` なら begin の入力を素通し |
| H00_CLEAN_STATE | Wrangle detail | G | 状態から一時データを除く(g01 と同じコード) |
| FINAL_RESIDUAL_EXTRACT | subnet | H00 | 最後の配置の後の残余を再抽出 |
| H01_FINAL_STATS | Wrangle detail | H00, 最終残余 | 最終充填率と未充填面積を確定し、重複・はみ出し・面積収支を独立に検証 |
| H02_FINAL_RESIDUAL_FLAG | Wrangle prim | 最終残余, H01 | 未充填の理由を分類 (`too_small` / `no_candidate` / `remaining`) |
| H03 / H04 | Wrangle detail | | 建築だけ / 最後の反復の候補だけを取り出す |
| H05_TOPK_SHAPES | Wrangle detail | H04 | 上位 `debug_top_k` 候補の輪郭線 |
| H08_RESIDUAL_LABELS | Wrangle detail | H02, H01 | 残余重心に ID 点を置く(`show_residual_ids`) |
| H06 / H07 | Wrangle prim / point | VIEW_MERGE | `debug_mode` に応じた色付けと表示切り替え |

HDA の出力: **0 = Buildings**、**1 = Residual**、**2 = Debug View**(`debug_mode` で切り替え)、**3 = Candidates**(最後の反復の全候補)。

---

## 3. パラメータと初期値

UI ラベルは「パラメータ名  日本語説明」の形式です(`build_campo_marzio_phase1.py` の `_parm_templates()`)。

### Domain(都市領域)
| 名前 | 型 | 初期値 | 説明 |
|---|---|---|---|
| domain_mode | int menu | 1 | 0 = Input Polygon(input 0)、1 = Rectangle |
| domain_width / domain_height | float | 200 / 140 | Rectangle モードのサイズ |
| target_fill_ratio | float | 0.60 | 充填率がこれに達したら終了 |
| minimum_building_area | float | 80 | 最小建築面積。これより小さい残余領域は候補生成しない |
| initial_layout_mode | int menu | 1 | 0 None / 1 Manual Footprints / 2 Voronoi Seeds(未実装)/ 3 Boundary Anchors(未実装) |

### Shape(形状)
| 名前 | 初期値 | 説明 |
|---|---|---|
| shape_weights | `"1 0 0 0 0"` | Phase 1 は Rectangle のみ。他が 0 より大きいと D01 が警告 |
| base_size | 20 | scale=1 の正方形の一辺 |
| min_scale / max_scale | 0.6 / 1.8 | スケール範囲 |
| max_aspect | 2.5 | width/depth の最大値 |
| aspect_sample_count | 3 | 縦横比 1..max_aspect の段階数 |
| shape_variety / symmetry_mode | 0.5 / 0 | **Phase 2 用に予約(未使用)** |

### Search(探索)
| 名前 | 初期値 | 説明 |
|---|---|---|
| angle_step | 5 | 0〜175 度を 5 度刻み(36 段階) |
| position_sample_count | 32 | 1 反復の位置サンプル総数(面積比で配分) |
| min_samples_per_region | 3 | 領域ごとの最小サンプル数 |
| scale_sample_count | 3 | スケールの段階数 |
| snap_to_boundary | ON | 最寄り境界線分に平行・接触する候補を追加 |
| max_candidates | 60000 | 1 反復の候補数の上限(超えたら打ち切り、警告) |
| max_iterations | 40 | 最大反復回数(= 最大配置棟数) |
| random_seed | 1 | 乱数シード |

1 反復の候補数の目安: 32 サンプル ×(36 角度 × 3 スケール × 3 縦横比 + 吸着 18)≈ 11,000。

### Placement(配置制約)
| 名前 | 初期値 | 説明 |
|---|---|---|
| clearance | 0 | 建築間・都市境界との最小間隔(0 なら接触を許可) |
| contact_tolerance | 0.5 | これ以下の距離を「接触」とみなす |
| min_passage_width | 4 | これ未満の隙間は通れない細片(sliver) |
| max_sliver_ratio | 0.5 | 周長のうち sliver になる割合の上限 |
| perimeter_samples | 6 | 辺あたりの周長サンプル数 |

### Evaluation(評価)
| 名前 | 初期値 |
|---|---|
| weight_fill | 0.40 |
| weight_fit | 0.25 |
| weight_access | 0.20 |
| weight_variety | 0.10 |
| weight_axis | 0.05 |
| variety_window | 5(直近何棟と比較するか) |
| axis_radius | 40(軸線を参照する半径) |

重みの合計が 1 でない場合、A01 が警告を出し、F01 が合計で割って正規化します。

### Termination(終了条件)
| 名前 | 初期値 | 説明 |
|---|---|---|
| minimum_residual_area | 100 | 最大残余面積がこれを下回ったら終了 |
| minimum_improvement | 0.0005 | 1 反復の充填率改善がこれ未満なら終了 |
| stop_when_no_candidate | ON | OFF のときは次の反復で別のサンプルを試す |

### Debug
| 名前 | 初期値 | 説明 |
|---|---|---|
| debug_mode | 0 | 0 Buildings / 1 Residual / 2 Candidates / 3 Score Visualization / 4 Generation Index |
| show_candidates | ON | 候補点を表示 |
| show_rejected_candidates | OFF | 棄却候補も表示(Candidates モード) |
| show_residual_ids | OFF | 残余重心に ID 点を出力 |
| debug_top_k | 15 | 上位候補の輪郭数 |
| keep_debug_in_loop | ON | 最後の反復の残余・候補を状態に残す(出力 2/3 用) |

---

## 4. 使用する標準 SOP と設定

| ノード | SOP | 設定 |
|---|---|---|
| C_EXTRUDE | PolyExtrude | Distance = 2、Output Back = ON(閉じたソリッドにする) |
| C_BOOLEAN | Boolean | Operation = Subtract、A = Surface、B = Solid、Subtract = A − B |
| C_SWITCH_NO_BUILDINGS | Switch | Select Input = `if(npoints("../C02_KEEP_BUILDINGS") == 0, 1, 0)` |
| C_FUSE | Fuse | 既定値(Boolean 出力の一致点を統合して、領域内の分割辺を共有辺にする) |
| C_CONNECTIVITY | Connectivity | Connectivity Type = Primitive、Attribute = `residual_id` |
| foreach_begin1 | Block Begin | Method = Fetch Feedback、Block Path = `../foreach_end1` |
| foreach_end1 | Block End | Iteration = By Count、Gather Method = Feedback Each Iteration、Iterations = `ch("../../max_iterations")`、Block Path / Template Path = `../foreach_begin1` |
| G_DEBUG_SWITCH | Switch | Select Input = `detail("../G04_TERMINATION", "cfg_keep_debug_in_loop", 0)` |
| G_STOP_SWITCH | Switch | Select Input = `detail("../foreach_begin1", "stop", 0)` |
| G_DEBUG_MERGE / VIEW_MERGE | Merge | 既定値 |

**Boolean の使い方**: 2D ポリゴン同士を直接 Boolean にかけると、同一平面での処理が不安定になりがちです。そこで建築だけを z ∈ [−1, 1] のソリッドにし(C03 で押し出し方向に関係なく Z 中心を 0 に揃えます)、Z=0 の都市領域「面」をそれで切り抜きます。この方法には、ポリゴンの巻き方向に依存しないという利点もあります。穴を持つ残余(建築が残余の内側にある場合)や非凸の残余は、Boolean が三角形分割した複数のポリゴンとして出てきます。このため C05/C06 は「1 領域 = 複数プリミティブ」を前提に、half-edge の共有の有無(`hedge_equivcount == 1`)で外周を取り出します。

---

## 5. VEX コード

`vex/*.vfl` がそのまま各 Wrangle の **VEXpression** に貼り付けられるコードです。ファイル先頭のコメントに、Run Over と各入力に何を接続するかを書いてあります。関数定義はスニペットの先頭に置いています(Wrangle で使える形式)。外部の `#include` は使っていません。

形状規約(D01 / E01 / E02 / F01 / G02 / H05 で共通):

```
ex = (cos θ, sin θ),  ey = (−sin θ, cos θ)
corner0 = c − ex·w/2 − ey·d/2      corner1 = c + ex·w/2 − ey·d/2
corner2 = c + ex·w/2 + ey·d/2      corner3 = c − ex·w/2 + ey·d/2
width = base_size·scale·√aspect,   depth = base_size·scale/√aspect
```

### 評価関数(F01)

`Score = (w_fill·Fill + w_fit·Fit + w_access·Access + w_variety·Variety + w_axis·Axis) / Σw`

| 項 | 定義(0..1) |
|---|---|
| Fill | 面積 / min(領域面積, 最大候補面積)。広い領域では大きい図形ほど高く、狭い領域では領域を埋める割合 |
| Fit | 周長サンプルのうち、残余**ポリゴン境界**までの距離が `clearance + contact_tolerance` 以下の割合(bbox ではない) |
| Access | 1 − (距離が接触より大きく `clearance + min_passage_width` 未満の、通れない隙間になる割合)。暫定評価 |
| Variety | 直近 `variety_window` 棟との角度差(90° 周期で 0..45° → 0..1)と尺度差(log 比)の平均。乱数ではない |
| Axis | `axis_radius` 内の最寄り建築の軸線を継承する度合い(1 − 角度差/45°)。近くに建築がなければ 0.5 |

### reject_code

| code | reason | 判定箇所 |
|---|---|---|
| 0 | valid | |
| 1 | outside_domain | E02(交差した境界線分が都市外周、または中心が都市外) |
| 2 | overlap_building | E02(交差した境界線分が建築外周、または中心が建築内) |
| 3 | below_min_area | E02 |
| 4 | access_sliver | F01(sliver 割合 > `max_sliver_ratio`) |
| 5 | invalid_shape | E02(NaN・非正の寸法など) |

---

## 6. 入出力ジオメトリと属性

### 状態ジオメトリ(B の出力 → ループ → H の入力)

| 対象 | 属性 |
|---|---|
| 都市領域 prim(1 枚) | `s@class="domain"`、group `domain` |
| 建築 prim | `s@class="building"`、group `building`、`i@building_id`、`i@shape_type`(0 Rectangle / −1 非矩形の手入力)、`f@orientation`(度、[0,180))、`f@scale`、`i@generation`(0 = 初期、n = n 回目の反復で配置)、`f@width`、`f@depth`、`f@area`、`f@score`(初期建築は −1)、`f@score_fill/fit/access/variety/axis`、`i@placed_residual_id`、`i@snapped`、`i@cand_id` |
| detail(設定) | `cfg_*`(全パラメータ)、`f@domain_area`、`v@domain_bbmin/bbmax`、`i@domain_winding_sign` |
| detail(状態) | `i@iter`、`i@stop`、`s@stop_reason`、`i@next_building_id`、`f@built_area`、`f@fill_ratio`、`f@improvement`、`f@unfilled_area`、`f@max_residual_area`、`i@building_count`、`i@placed`、`i@placed_building_id`、`i@placed_residual_id`、`i@n_candidates`、`i@n_valid`、`i[]@reject_counts`、`f@last_score`、`f[]@recent_orient`、`f[]@recent_scale`、`i[]@region_valid_count` |
| detail(履歴) | `i[]@hist_building_id`(候補なしの反復は −1)、`f[]@hist_score`、`i[]@hist_n_candidates`、`i[]@hist_n_valid`、`f[]@hist_fill_ratio`、`f[]@hist_max_residual` |

### 残余ポリゴン(C out0 / HDA out1)
prim: `s@class="residual"`、group `residual`、`i@residual_id`、`f@region_area`、`f@region_perimeter`、`v@region_centroid`、`v@region_bbmin/bbmax`、`f@elongation`(P²/4πA)、`f@bbox_aspect`、`f@approx_width`(2A/P)、`f@building_contact_ratio`、`i@boundary_edge_count`、`i@ignored`、`i@unfilled`。
ループ内(G03)では、加えて `i@valid_candidates`、`i@placed_here`。最終出力(H02)では、加えて `i@unfilled_class`(0 too_small / 1 no_candidate / 2 remaining)と `s@unfilled_reason`。
detail: `i@residual_count`、`i@residual_ignored_count`、`f@total_residual_area`、`f@max_residual_area`、`f[]@region_areas`。

### 境界線分(C out1)
2 点ポリライン。prim: `i@residual_id`、`i@src_building_id`(−1 = 都市外周)、`s@class="boundary"`。

### 候補点(E/F の出力 / HDA out3)
point: `P`(図形中心)、`f@theta`、`f@width`、`f@depth`、`f@scale`、`f@aspect`、`i@shape_type`、`i@residual_id`、`f@region_area`、`i@sample_id`、`i@snapped`、`i@cand_id`、`i@reject_code`、`s@reject_reason`、`i@hit_segment`、`f@area`、`f@score`、`f@score_fill/fit/access/variety/axis`、`i@axis_ref_building`。group `candidate`。

### 建築出力(HDA out0)の detail(H01)
`i@check_overlap_pairs`、`i@check_outside_count`、`f@check_area_balance`、`i@check_passed`、`f@final_fill_ratio`、`f@final_unfilled_area`、`i@final_residual_count`、`f@final_max_residual`、`i@final_iterations`、`s@final_stop_reason`、`i@building_count`、および状態の detail 一式。

---

## 7. 作成手順と検証手順

### 7.1 自動構築(推奨)

1. Houdini 21 を起動し、**Windows › Python Shell** を開きます。
2. 次を実行します。
   ```python
   import sys
   sys.path.append(r"<リポジトリ>/houdini/campo_marzio")
   import build_campo_marzio_phase1 as b
   b.build()
   ```
3. `/obj/campo_marzio_phase1` の中に、テスト入力(`TEST_DOMAIN_EMPTY`、`T01_TEST_FOOTPRINTS`)、HDA 化前のサブネット `HF_CAMPO_MARZIO_GRAMMAR`、出力確認用の Null(`VIEW`、`OUT_*`)が作られます。既に同名ノードがある場合は、既存ノードを壊さないよう中止します。
4. 最後に **「手動で確認・設定すべき項目」** が表示された場合は、該当するパラメータを [4 章](#4-使用する標準-sop-と設定) の表に従って手で設定してください(パラメータ名やメニュー項目がバージョンで異なる可能性があるため)。
5. 動作を確認したら、サブネットを右クリック › **Create Digital Asset…** で HDA にします(Operator Name `hf_campo_marzio_grammar`、Inputs 0〜3、Outputs 4)。`b.build(make_hda=True)` でも試せますが、この経路は未検証です。

### 7.2 手動構築

1. Geometry ノードの中に Subnet `HF_CAMPO_MARZIO_GRAMMAR` を作り、**Edit Parameter Interface** で [3 章](#3-パラメータと初期値) のパラメータを作ります(名前は表のとおり)。
2. [1 章](#1-hda-内部のノード構成図) の図のとおり、サブネット A〜H とその中のノードを作ります。Wrangle には `vex/` の同名ファイルを貼り、ファイル先頭のコメントにある Run Over を設定します。
3. RESIDUAL_EXTRACT は G の中と H の中(`FINAL_RESIDUAL_EXTRACT`)に同じ構成で 2 つ必要です。G の中で完成させてからコピーしてください。H00 には `g01_clear_transient.vfl` をもう一度使います。
4. ループは Tab › **For-Each Loop › For Loop with Feedback** で作り、[4 章](#4-使用する標準-sop-と設定) の表のとおりに設定します。begin/end のノード名を `foreach_begin1` / `foreach_end1` にしておくと、Switch の式がそのまま使えます。

### 7.3 参照実装による検証(Houdini 外)

```bash
cd houdini/campo_marzio/reference
pip install shapely numpy
python -m unittest -v test_phase1_reference         # 8 tests, OK
python cm_phase1_reference.py --seed 1 --svg out.svg
```

この環境での実行結果(seed 1、既定パラメータ、テスト用初期建築 3 棟): 13 反復で 13 棟を配置し、充填率 0.604 で `target_fill` により終了しました。`overlap_pairs=0`、`outside=0`、`area_balance=0`。プレビューは `reference/preview_reference_seed1.png` です(黒 = 初期建築、青→黄 = 生成順)。

| テスト | 確認内容 |
|---|---|
| test_no_overlap_no_outside | 4 つのシードで、はみ出しなし・重複なし・面積収支の一致 |
| test_no_initial_buildings | 初期建築なしでも動作する |
| test_reproducible_with_same_seed | 同じシードなら同一結果、違うシードなら異なる結果 |
| test_residual_updates_every_iteration | 各反復で、残余面積が配置した建築の面積だけ減る |
| test_unfilled_recorded_when_no_candidate | 候補がなくなったら配置を強行せず `no_candidate` で止まり、残余を未充填として残す |
| test_reject_reasons_are_counted | 棄却理由の集計が候補総数と一致する |
| test_matches_shapely_containment | E02 と同じ 3 条件の判定が、穴あき・非凸の残余に対する shapely の厳密な包含判定と 4000 ケースで一致する |
| test_touching_is_allowed | 境界にぴったり接する配置は有効で、1 だけ食い込む配置は無効 |

参照実装の乱数列は VEX の `rand()` と異なります。そのため、同じシードでも Houdini と配置は一致しません。比べるのは傾向(充填率・反復数・棄却理由の分布)です。

### 7.4 Houdini 上での Phase 1 検証チェックリスト

`OUT_BUILDINGS` の Geometry Spreadsheet(Detail)で確認します。

| 合格条件 | 確認方法 |
|---|---|
| 建築が都市領域からはみ出さない | `check_outside_count == 0` |
| 建築同士が重複しない | `check_overlap_pairs == 0` |
| (上の 2 つの独立確認) | `check_area_balance` が 1e-4 未満(Boolean の残余面積 + 建築面積 = 都市面積) |
| 各反復で残余が更新される | `hist_max_residual` と `hist_fill_ratio` が反復ごとに変化する。`max_iterations` を 1, 2, 3… と変えて `debug_mode=Residual` の表示が変わる |
| 同じシードで再現できる | `random_seed` を変えて戻し、`OUT_BUILDINGS` の点座標が一致する(Spreadsheet を比較、または Python Shell で `hou.node(...).geometry().points()` の座標を比較) |
| 未充填領域が正しく表示される | `debug_mode=Residual`。赤 = 面積不足(too_small)、それ以外は residual_id ごとの色。`OUT_RESIDUAL` の `unfilled_reason` を確認 |
| 棄却理由を確認できる | `debug_mode=Candidates` + `show_rejected_candidates=ON`(緑 valid / 青 outside / 赤 overlap / 灰 min_area / 橙 sliver / 紫 invalid)。`OUT_CANDIDATES` の `reject_reason` |

さらに次も確認してください。

- `domain_mode=Input Polygon` にして、L 字型などの非凸ポリゴンを input 0 に接続しても動くこと。
- `initial_layout_mode=None` で動くこと(建築 0 棟のとき `C_SWITCH_NO_BUILDINGS` が Boolean を迂回すること)。
- 初期建築を都市境界に接して置いたときに、残余にゼロ面積の細片が残らないこと(C04 が除去します)。

---

## 8. 未検証の API・機能

Houdini の無い環境で書いたので、以下は実機での確認が必要です。

1. **Boolean SOP**: パラメータ名 `asurface` / `bsurface` / `booleanop` / `subtractchoices` とメニュー項目。A = Surface・B = Solid の Subtract で、Z=0 の面が期待どおり切り抜かれるか。切り抜かれた面の出力形式(三角形分割されるか、n-gon か)。
2. **Block Begin / Block End**: パラメータ名 `method` / `blockpath` / `itermethod` / `iterations` / `templatepath` とメニュー項目。Stop Condition は使っていません(`G_STOP_SWITCH` で代替)。使う場合は、式が「0 で停止」か「非 0 で停止」かをドキュメントで確認してください。
3. **ループ内からの `detail("../foreach_begin1", "stop", 0)`**: Block Begin の出力をループ内の Switch の式から参照して、反復ごとに正しく評価されるか。
4. **PolyExtrude** の `dist` / `outputback`。**Connectivity** の `connecttype` / `attribname`。**Subnet output** の `outputidx`。
5. **VEX 関数**: `primfind(geo, min, max)`、`xyzdist(geo, group, P, prim, uv, maxdist)` の該当なし時の戻り値(コードでは `prim < 0` と距離の両方で判定)、`hedge_equivcount` / `hedge_next` / `primhedge`、配列スライス、`detail()` の配列読み出し、`setprimattrib` 等による属性の自動作成(念のため `add*attrib` を先に呼んでいます)。
6. **Detail Wrangle で自身のジオメトリを全削除してから追加する書き方**(E01、C06、H04、H05、H08、D01)。
7. **Merge で detail 属性が衝突したとき**、最初の入力(G04 / H01)の値が優先されるか。
8. **`createDigitalAsset` による HDA 化**(`make_hda=True`)と、spare parameter が HDA のパラメータに引き継がれるか。
9. **Visualizer** で `residual_id` を文字表示する UI の場所(`show_residual_ids` は点と属性を出力するだけです)。
10. 性能: 既定値で 1 反復あたり約 11,000 候補、E02/F01 は候補ごとに `primfind` と周長 24 点の `xyzdist` を行います。参照実装では 1 反復 0.3 秒程度でしたが、VEX での実測はしていません。

---

## 既知の制限(Phase 1 の範囲として)

- 形状は Rectangle のみです。`shape_weights`、`shape_variety`、`symmetry_mode` は予約で、未使用です。
- 初期配置モードの Voronoi Seeds / Boundary Anchors は未実装です(警告を出して None として扱います)。
- 都市領域は穴のない 1 枚のポリゴンに限ります。
- 頂点 1 点だけで接する 2 つの残余は、Connectivity では 1 領域になります(点連結)。通行可能性は Phase 3 / Path Explorer で扱います。
- `max_residual_area` による終了判定は、その反復の配置前の残余で行います(1 反復の遅れ)。
- 既定の重みでは Fill(0.40)が支配的なので、空間が十分にある間は最大寸法(base × max_scale)の候補が選ばれやすくなります。初期建築が無いと、Axis(軸線の継承)と境界への吸着によって全棟が都市の軸に揃いがちです。これは Phase 1 の採点式どおりの挙動です。尺度・軸線の不均質性は Phase 4(Campo Marzio モード)で制御します。
- 途中の反復の候補を見るには `max_iterations` を小さくしてください(出力 3 には最後の反復の候補だけが残ります)。
