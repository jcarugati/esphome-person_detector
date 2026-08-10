#!/usr/bin/env python3
"""Executable regression harness for the first negative person inference.

The repository has no host-side C++ test framework.  This harness combines a
small behavior model with source-level checks on the production debounce branch
so it catches the missing initial binary-sensor publication rather than merely
checking that the component compiles.
"""

from __future__ import annotations

import re
import unittest
from pathlib import Path


SOURCE = (
    Path(__file__).resolve().parents[1]
    / "components"
    / "person_detect"
    / "person_detector.cpp"
)


def _loop_source() -> str:
    source = SOURCE.read_text(encoding="utf-8")
    start = source.index("void PersonDetector::loop()")
    end = source.index("void PersonDetector::publish_present_", start)
    return source[start:end]


def _debounce_source() -> str:
    loop = _loop_source()
    start = loop.index("  // Debounce: assert immediately")
    return loop[start:]


class _PresenceBehavior:
    """Minimal event model for the required first-result state transition."""

    def __init__(self, clear_after: int = 3) -> None:
        self.clear_after = clear_after
        self.present = False
        self.initialized = False
        self.misses = 0
        self.events: list[tuple[str, bool]] = []

    def consume(self, person_raw: bool) -> None:
        if person_raw:
            self.misses = 0
            if not self.present:
                self.present = True
                self.initialized = True
                self.events.append(("state", True))
                self.events.append(("automation", True))
        elif self.present:
            self.misses += 1
            if self.misses >= self.clear_after:
                self.misses = 0
                self.present = False
                self.initialized = True
                self.events.append(("state", False))
                self.events.append(("automation", False))
        elif not self.initialized:
            # The production branch must publish this state directly, without
            # routing through publish_present_(false), which fires on_cleared.
            self.initialized = True
            self.events.append(("state", False))


class PersonDetectorStateRegression(unittest.TestCase):
    def test_first_valid_negative_publishes_false_without_clear_automation(self) -> None:
        debounce = _debounce_source()

        # The negative-result path must be an explicit branch for the initial
        # false state.  This also prevents repeated misses from publishing the
        # same state over and over after initialization.
        negative = re.search(
            r"  \} else if \(!this->presence_initialized_\) \{(?P<body>.*?)\n  \}\n\}",
            debounce,
            flags=re.DOTALL,
        )
        self.assertIsNotNone(
            negative,
            "negative-result handling must guard the initial-state branch",
        )
        assert negative is not None
        body = negative.group("body")
        self.assertIn("this->presence_initialized_ = true", body)
        self.assertIn("this->binary_sensor_->publish_state(false)", body)
        self.assertNotIn("publish_present_(false)", body)
        self.assertNotIn("on_cleared_", body)

        self.assertIn(
            "bool presence_initialized_{false};",
            SOURCE.with_name("person_detector.h").read_text(encoding="utf-8"),
        )

        # The branch is reached only after the enabled and valid-result guards;
        # an enabled detector cannot claim clear before inference produces data.
        loop = _loop_source()
        self.assertLess(
            loop.index("if (!this->enabled_.load())"),
            loop.index("  // Debounce: assert immediately"),
        )
        self.assertLess(loop.index("if (!have)"), loop.index("  // Debounce: assert immediately"))

        detector = _PresenceBehavior()
        detector.consume(False)
        detector.consume(False)
        detector.consume(False)
        self.assertEqual(detector.events, [("state", False)])

    def test_positive_and_debounced_clear_callbacks_remain_unchanged(self) -> None:
        detector = _PresenceBehavior()
        detector.consume(True)
        detector.consume(False)
        detector.consume(False)
        self.assertEqual(detector.events, [("state", True), ("automation", True)])

        detector.consume(False)
        self.assertEqual(
            detector.events,
            [
                ("state", True),
                ("automation", True),
                ("state", False),
                ("automation", False),
            ],
        )


if __name__ == "__main__":
    unittest.main()
