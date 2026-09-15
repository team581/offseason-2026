import unittest

from analyze_loop_timing import analyze, compare, percentile, robot_mode, summarize


class LoopTimingTest(unittest.TestCase):
    def test_percentile_interpolates(self):
        self.assertEqual(percentile([0.0, 1.0, 2.0, 3.0], 0.5), 1.5)

    def test_summarize_counts_overruns(self):
        result = summarize([0.001, 0.010, 0.021, float("nan"), -1.0])
        self.assertEqual(result["count"], 3)
        self.assertEqual(result["overruns_20ms"], 1)

    def test_robot_mode(self):
        self.assertEqual(robot_mode(False, True, False), "disabled")
        self.assertEqual(robot_mode(True, True, False), "auto")
        self.assertEqual(robot_mode(True, False, False), "teleop")
        self.assertEqual(robot_mode(True, False, True), "test")

    def test_compare_reports_improvement(self):
        key = "/Robot/Scheduler/RobotPeriodicExecution"
        baseline = {"metrics": {key: {"p99": 0.010}}}
        candidate = {"metrics": {key: {"p99": 0.008}}}
        result = compare(candidate, baseline)
        self.assertAlmostEqual(result[key]["p99_improvement_percent"], 20.0)

    def test_analyze_empty_input(self):
        report = analyze([], "auto", 2.0)
        self.assertEqual(report["metrics"], {})
        self.assertNotIn("instrumentation_overhead", report)


if __name__ == "__main__":
    unittest.main()
