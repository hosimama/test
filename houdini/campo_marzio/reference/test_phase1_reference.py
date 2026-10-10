# -*- coding: utf-8 -*-
"""
Phase 1 合格条件を参照実装で検証するテスト。

    cd houdini/campo_marzio/reference
    python -m unittest -v test_phase1_reference

Houdini 上の HDA そのものを検証するものではない。VEX と同じ判定式
(E02 の矩形内外判定) が shapely の厳密な包含判定と一致することと、
逐次配置アルゴリズムが合格条件を満たすことを確認する。
"""

import math
import unittest

import numpy as np
import shapely
from shapely.geometry import Polygon

import cm_phase1_reference as ref


class TestAcceptance(unittest.TestCase):

    def _run(self, seed, footprints=ref.TEST_FOOTPRINTS, **kw):
        prm = ref.Params(random_seed=seed, **kw)
        st, regions, last = ref.run(prm, footprints)
        return prm, st, regions, ref.check(st, regions)

    def test_no_overlap_no_outside(self):
        """建築が都市領域からはみ出さず、建築同士が重複しない。"""
        for seed in (1, 2, 3, 7):
            with self.subTest(seed=seed):
                _, st, _, res = self._run(seed)
                self.assertEqual(res["overlap_pairs"], 0)
                self.assertEqual(res["outside"], 0)
                self.assertLess(res["area_balance"], 1e-6)
                self.assertGreater(len(st.buildings), len(ref.TEST_FOOTPRINTS))

    def test_no_initial_buildings(self):
        """initial_layout_mode=None (初期建築なし) でも動作する。"""
        _, st, _, res = self._run(4, footprints=())
        self.assertTrue(res["passed"])
        self.assertGreater(len(st.buildings), 0)

    def test_reproducible_with_same_seed(self):
        """同じ乱数シードで結果を再現できる / 違うシードでは変わる。"""
        def coords(st):
            return [np.round(np.asarray(b.poly.exterior.coords), 9).tolist() for b in st.buildings]
        _, a, _, _ = self._run(5)
        _, b, _, _ = self._run(5)
        _, c, _, _ = self._run(6)
        self.assertEqual(coords(a), coords(b))
        self.assertNotEqual(coords(a), coords(c))

    def test_residual_updates_every_iteration(self):
        """各反復で残余領域が更新される (配置した建築の面積だけ減る)。"""
        prm = ref.Params(random_seed=2, max_iterations=8, target_fill_ratio=0.99)
        st = ref.init_state(prm, ref.TEST_FOOTPRINTS)
        prev = sum(r["area"] for r in ref.extract_residual(st, prm))
        for _ in range(prm.max_iterations):
            if st.stop:
                break
            out = ref.step(st, prm)
            now = sum(r["area"] for r in ref.extract_residual(st, prm))
            if out["placed"] is not None:
                self.assertAlmostEqual(prev - now, out["placed"].poly.area, places=6)
            else:
                self.assertAlmostEqual(prev, now, places=9)
            prev = now

    def test_unfilled_recorded_when_no_candidate(self):
        """候補がなくなったら配置を強行せず停止し、残余を未充填として残す。"""
        prm, st, regions, res = self._run(1, footprints=(), domain_width=60.0, domain_height=40.0,
                                          target_fill_ratio=0.99, minimum_residual_area=1.0,
                                          minimum_improvement=0.0)
        self.assertEqual(st.stop_reason, "no_candidate")
        self.assertTrue(res["passed"])
        self.assertGreater(res["unfilled_area"], 0.0)
        self.assertEqual(st.history[-1]["building_id"], -1)

    def test_reject_reasons_are_counted(self):
        """候補の棄却理由が記録される。"""
        prm = ref.Params(random_seed=3, max_iterations=3)
        st = ref.init_state(prm, ref.TEST_FOOTPRINTS)
        ref.step(st, prm)
        counts = st.history[-1]["reject_counts"]
        self.assertEqual(sum(counts), st.history[-1]["n_candidates"])
        self.assertGreater(counts[1] + counts[2], 0)   # outside / overlap が発生している


class TestPredicate(unittest.TestCase):
    """E02 の判定 (中心が残余内 AND 辺の交差なし AND 境界点が内部にない) が
    shapely の厳密な包含判定と一致すること。穴あき・非凸の残余で確認する。"""

    def test_matches_shapely_containment(self):
        prm = ref.Params()
        domain = Polygon([(-100, -70), (100, -70), (100, 70), (-100, 70)])
        st = ref.State(domain=domain, buildings=[])
        rng = np.random.default_rng(0)
        # 回転した建築を置いて、穴あき・非凸の残余を作る
        for k, (c, th, w, d) in enumerate([((-40, 10), 0, 36, 22), ((55, -25), 30, 30, 18),
                                            ((0, -61), 0, 50, 18), ((20, 40), 65, 40, 12),
                                            ((-75, -40), 12, 20, 30)]):
            st.buildings.append(ref.Building(k, ref.rect_poly(c, th, w, d), th, 1, 0, w, d))
        regions = ref.extract_residual(st, prm)
        seg_a, seg_b, _, _ = ref.boundary_arrays(regions)
        res_union = shapely.union_all([r["geom"] for r in regions])

        N = 4000
        cx = rng.uniform(-110, 110, N)
        cy = rng.uniform(-80, 80, N)
        th = rng.uniform(0, 180, N)
        w = rng.uniform(4, 60, N)
        d = rng.uniform(4, 60, N)
        eps = 1e-4 * prm.base_size

        hit, _ = ref.rect_in_residual_mask(cx, cy, th, w, d, seg_a, seg_b, 0.0, eps)
        center_in = shapely.distance(res_union, shapely.points(cx, cy)) <= 10 * eps
        ours = center_in & ~hit

        polys = [ref.rect_poly((x, y), t, ww, dd) for x, y, t, ww, dd in zip(cx, cy, th, w, d)]
        truth = np.array([res_union.buffer(1e-6).contains(p) for p in polys])
        # 境界すれすれ (誤差内) のケースは除外して比較
        margin = np.array([abs(res_union.boundary.distance(p.boundary)) for p in polys])
        clear = margin > 1e-3
        self.assertGreater(ours[clear].sum(), 50)        # 十分な数の「収まる」ケースがある
        np.testing.assert_array_equal(ours[clear], truth[clear])

    def test_touching_is_allowed(self):
        """境界にぴったり接する配置は有効 (eps だけ縮めて判定するため)。"""
        prm = ref.Params()
        domain = Polygon([(-100, -70), (100, -70), (100, 70), (-100, 70)])
        st = ref.State(domain=domain, buildings=[ref.Building(0, ref.rect_poly((0, 0), 0, 20, 20), 0, 1, 0, 20, 20)])
        regions = ref.extract_residual(st, prm)
        seg_a, seg_b, _, _ = ref.boundary_arrays(regions)
        eps = 1e-4 * prm.base_size
        # 右隣に接する矩形 / 都市領域の角に接する矩形 / 1 だけ食い込む矩形
        cx = np.array([20.0, -90.0, 19.0])
        cy = np.array([0.0, -60.0, 0.0])
        th = np.zeros(3); w = np.full(3, 20.0); d = np.full(3, 20.0)
        hit, _ = ref.rect_in_residual_mask(cx, cy, th, w, d, seg_a, seg_b, 0.0, eps)
        self.assertEqual(hit.tolist(), [False, False, True])


if __name__ == "__main__":
    unittest.main()
