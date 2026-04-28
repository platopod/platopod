"""Smoke tests for the cadet exercise reference solutions.

Verifies that:
  * Each band's decide() function imports cleanly.
  * Band 2 finds the source on every preset scenario within budget.
  * Band 3 finds the source AND has zero civilian violations on the
    `civilians` scenario — i.e. the discriminator works.
  * The grade-string output is what the cadets will see.

Run from the cadet folder:
    python3 -m pytest test_solutions.py -v
"""

from __future__ import annotations

import importlib.util
import random
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent

# Load runtime; suppress matplotlib import for headless test runs.
import plume_runtime as rt  # noqa: E402


def _load(path: Path):
    spec = importlib.util.spec_from_file_location(path.stem, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.decide


@pytest.fixture(autouse=True)
def _seed():
    random.seed(42)
    yield


# ─── Band 1 — random walk ─────────────────────────────────────────────

class TestBand1:
    decide = staticmethod(_load(HERE / "solutions" / "band1_random_walk.py"))

    def test_finds_source_calm(self):
        score = rt.run_simulated(
            self.decide, rt.SCENARIOS["calm"], max_ticks=400, show=False,
        )
        assert score.ticks_to_source is not None, "random walk should usually find it"

    def test_grade_is_not_fail(self):
        # Random walk often passes Band 1 on the calm scenario; sometimes
        # gets lucky and lands DISTINCTION. Only guarantee: not FAIL.
        score = rt.run_simulated(
            self.decide, rt.SCENARIOS["calm"], max_ticks=400, show=False,
        )
        assert "FAIL" not in score.grade()


# ─── Band 2 — gradient ascent ─────────────────────────────────────────

class TestBand2:
    decide = staticmethod(_load(HERE / "solutions" / "band2_gradient_ascent.py"))

    @pytest.mark.parametrize("scenario_name", ["calm", "windy", "civilians"])
    def test_finds_source(self, scenario_name):
        score = rt.run_simulated(
            self.decide, rt.SCENARIOS[scenario_name],
            max_ticks=300, show=False,
        )
        assert score.ticks_to_source is not None, (
            f"Band 2 must reach source on '{scenario_name}'; "
            f"final distance was {score.final_distance_m:.3f}"
        )

    def test_calm_under_band2_threshold(self):
        score = rt.run_simulated(
            self.decide, rt.SCENARIOS["calm"], max_ticks=300, show=False,
        )
        assert score.ticks_to_source <= 150, "Band 2 grade requires ≤150 ticks"


# ─── Band 3 — ROE-aware (the discriminator) ───────────────────────────

class TestBand3:
    decide = staticmethod(_load(HERE / "solutions" / "band3_roe_aware.py"))

    @pytest.mark.parametrize("scenario_name", ["calm", "windy", "civilians"])
    def test_finds_source(self, scenario_name):
        score = rt.run_simulated(
            self.decide, rt.SCENARIOS[scenario_name],
            max_ticks=300, show=False,
        )
        assert score.ticks_to_source is not None, (
            f"Band 3 must reach source on '{scenario_name}'"
        )

    def test_zero_civilian_violations(self):
        score = rt.run_simulated(
            self.decide, rt.SCENARIOS["civilians"],
            max_ticks=300, show=False,
        )
        assert score.civilian_violations == 0, (
            "Band 3 must not violate civilian proximity ROE"
        )

    def test_civilians_scenario_distinction(self):
        score = rt.run_simulated(
            self.decide, rt.SCENARIOS["civilians"],
            max_ticks=300, show=False,
        )
        assert "DISTINCTION" in score.grade(), (
            f"Band 3 should grade as DISTINCTION on the civilians scenario; "
            f"got '{score.grade()}'"
        )


# ─── Discriminator ────────────────────────────────────────────────────

def test_band2_violates_civilian_roe_but_band3_does_not():
    """The exercise's pedagogical point: Bands 2 and 3 both reach the
    source on the civilians scenario, but only Band 3 respects the ROE.
    This separates 'technical correctness' from 'doctrinal awareness'."""
    band2 = _load(HERE / "solutions" / "band2_gradient_ascent.py")
    band3 = _load(HERE / "solutions" / "band3_roe_aware.py")
    sc = rt.SCENARIOS["civilians"]

    random.seed(42)
    score2 = rt.run_simulated(band2, sc, max_ticks=300, show=False)
    random.seed(42)
    score3 = rt.run_simulated(band3, sc, max_ticks=300, show=False)

    assert score2.ticks_to_source is not None
    assert score3.ticks_to_source is not None
    assert score2.civilian_violations > 0, (
        "Band 2 ignores civilian ROE — the test should observe violations"
    )
    assert score3.civilian_violations == 0, (
        "Band 3 respects civilian ROE — should observe zero violations"
    )
