# SPDX-License-Identifier: GPL-3.0-or-later
"""Tests for the ROS-free core of glim_supervisor. Run: python3 -m pytest (or unittest)."""
import datetime as dt
import json
import os
import sys
import tempfile
import textwrap
import time
import unittest

sys.path.insert(0, os.path.dirname(__file__))
import glim_supervisor as gs  # noqa: E402

# Fake GLIM programs: a python script standing in for glim_rosnode /
# offline_viewer, parsing the same arguments the supervisor passes.
FAKE = textwrap.dedent("""
    import os, signal, sys, time
    args = sys.argv[1:]
    if "--ros-args" in args:                      # glim_rosnode
        dump = [a.split(":=", 1)[1] for a in args if a.startswith("dump_path:=")][0]
        done = False
        def stop(*_):
            global done
            done = True
        signal.signal(signal.SIGINT, stop)
        print("mapping", flush=True)
        while not done and not os.environ.get("FAKE_CRASH"):
            time.sleep(0.02)
        if os.environ.get("FAKE_CRASH"):
            sys.exit(3)
        os.makedirs(dump, exist_ok=True)
        open(os.path.join(dump, "graph.bin"), "w").write("g")
        sys.exit(0)
    if "--export_path" in args:                    # offline_viewer auto export
        out = args[args.index("--export_path") + 1]
        if os.environ.get("FAKE_EXPORT_FAIL"):
            sys.exit(1)
        open(out, "w").write("ply " + args[0])
        sys.exit(0)
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))  # interactive viewer
    while True:
        time.sleep(0.02)
""")


class SupervisorTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        root = self.tmp.name
        fake = os.path.join(root, "fake_glim.py")
        with open(fake, "w") as f:
            f.write(FAKE)
        self.cfg = gs.Config(
            sessions_dir=os.path.join(root, "sessions"),
            active_map_path=os.path.join(root, "active", "garden_map.ply"),
            config_path="/glim/config",
            keep_backups=2,
            mapping_cmd=[sys.executable, fake],
            viewer_cmd=[sys.executable, fake],
        )
        self.changes = 0
        self.sup = gs.Supervisor(self.cfg, on_change=self._changed)
        for k in ("FAKE_CRASH", "FAKE_EXPORT_FAIL"):
            os.environ.pop(k, None)

    def tearDown(self):
        self.sup.shutdown(timeout_s=5)
        self.tmp.cleanup()

    def _changed(self):
        self.changes += 1

    def wait_state(self, state, timeout=10.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.sup.poll()
            if self.sup.status(rescan_s=0)["state"] == state:
                return self.sup.status(rescan_s=0)
            time.sleep(0.02)
        self.fail(f"state {state} not reached: {self.sup.status(rescan_s=0)}")

    def make_session(self, name, complete=True):
        path = os.path.join(self.cfg.sessions_dir, name)
        os.makedirs(path)
        if complete:
            open(os.path.join(path, "graph.bin"), "w").write("g")
        return path

    def test_mapping_creates_new_session_each_time(self):
        ok, name1 = self.sup.start_mapping()
        self.assertTrue(ok)
        self.assertEqual(self.sup.status()["state"], "mapping")
        ok2, msg = self.sup.start_mapping()
        self.assertFalse(ok2)  # only one GLIM program at a time
        self.assertIn("Busy", msg)
        time.sleep(0.2)
        self.assertTrue(self.sup.stop_mapping()[0])
        st = self.wait_state("idle")
        self.assertIn(name1, st["message"])
        meta = json.load(open(os.path.join(self.cfg.sessions_dir, name1, "session.json")))
        self.assertEqual(meta["kind"], "mapping")
        self.assertEqual(meta["exit_code"], 0)
        # Second run within the same second must not reuse the directory.
        os.makedirs(os.path.join(self.cfg.sessions_dir, gs.new_session_name([])), exist_ok=True)
        ok, name2 = self.sup.start_mapping()
        self.assertTrue(ok)
        self.assertNotEqual(name1, name2)
        time.sleep(0.2)
        self.sup.stop_mapping()
        self.wait_state("idle")
        names = [s["name"] for s in self.sup.status(rescan_s=0)["sessions"]]
        self.assertIn(name1, names)
        self.assertIn(name2, names)

    def test_mapping_crash_is_reported(self):
        os.environ["FAKE_CRASH"] = "1"
        ok, name = self.sup.start_mapping()
        self.assertTrue(ok)
        st = self.wait_state("error")
        self.assertIn("without a complete dump", st["message"])

    def test_viewer_requires_valid_complete_session(self):
        self.assertFalse(self.sup.open_viewer("")[0])
        self.assertFalse(self.sup.open_viewer("../etc")[0])
        self.assertFalse(self.sup.open_viewer("missing")[0])
        self.make_session("partial", complete=False)
        ok, msg = self.sup.open_viewer("partial")
        self.assertFalse(ok)
        self.assertIn("graph.bin", msg)

    def test_viewer_open_blocks_mapping_and_closes(self):
        self.make_session("merged")
        ok, name = self.sup.open_viewer("merged")
        self.assertTrue(ok)
        self.assertEqual(name, "merged")
        self.assertFalse(self.sup.start_mapping()[0])
        self.assertTrue(self.sup.close_viewer()[0])
        self.wait_state("idle")

    def test_export_installs_map_and_keeps_backups(self):
        self.make_session("a")
        self.make_session("b")
        for i, name in enumerate(["a", "b", "a", "b"]):
            ok, _ = self.sup.export_map(name)
            self.assertTrue(ok)
            st = self.wait_state("idle")
            self.assertEqual(st["active_map"]["source_session"], name)
            time.sleep(1.05)  # backup names have 1 s resolution
        active = self.cfg.active_map_path
        self.assertIn("sessions/b", open(active).read())
        backups = gs.list_backups(active)
        self.assertEqual(len(backups), 2)  # keep_backups
        self.assertFalse(os.path.exists(os.path.splitext(active)[0] + ".exporting.ply"))

    def test_failed_export_leaves_active_map(self):
        self.make_session("a")
        os.makedirs(os.path.dirname(self.cfg.active_map_path), exist_ok=True)
        open(self.cfg.active_map_path, "w").write("old")
        os.environ["FAKE_EXPORT_FAIL"] = "1"
        self.assertTrue(self.sup.export_map("a")[0])
        st = self.wait_state("error")
        self.assertIn("unchanged", st["message"])
        self.assertEqual(open(self.cfg.active_map_path).read(), "old")
        self.assertEqual(gs.list_backups(self.cfg.active_map_path), [])

    def test_shutdown_saves_running_mapping(self):
        ok, name = self.sup.start_mapping()
        time.sleep(0.2)
        self.sup.shutdown(timeout_s=5)
        self.assertTrue(os.path.isfile(os.path.join(self.cfg.sessions_dir, name, "session.json")))

    def test_helpers(self):
        when = dt.datetime(2026, 9, 27, 16, 40, 5)
        self.assertEqual(gs.new_session_name([], when), "mapping_2026-09-27_164005")
        self.assertEqual(gs.new_session_name(["mapping_2026-09-27_164005"], when), "mapping_2026-09-27_164005_2")
        for bad in ("", ".hidden", "a/b", "..", "a..b", "x" * 200):
            self.assertFalse(gs.valid_session_name(bad), bad)
        self.assertTrue(gs.valid_session_name("merged_garden-v2.1"))
        self.make_session("saved_by_viewer")
        s = gs.list_sessions(self.cfg.sessions_dir)
        self.assertEqual(s[0]["kind"], "saved")
        self.assertTrue(s[0]["complete"])


if __name__ == "__main__":
    unittest.main()
