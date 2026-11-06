#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""
glim_supervisor — keeps the GLIM container running and starts/stops GLIM on
demand (MowgliNext GUI → foxglove_bridge → ROS services).

Why a supervisor and not "stop the node": glim_rosnode has no start/stop
services. It maps from the moment the process starts and writes its dump only
when the process ends (SIGINT → spin returns → wait() → save(dump_path), see
glim_ros2/src/glim_rosnode.cpp). So "stop mapping" means "end the process and
wait until the dump is written". The supervisor owns that, and runs at most
ONE GLIM program at a time (mapping, offline viewer, or an export), so they
never fight over the same dump.

Storage (container paths; mounted from the host in docker-compose.yaml):
  /glim/sessions/<name>/        one directory per dump. Mapping runs are named
                                mapping_YYYY-MM-DD_HHMMSS; maps you merge and
                                "Save Map" in the offline viewer can use any
                                name and show up here too. Nothing is ever
                                overwritten: every mapping gets a new name.
  /glim/active_map/garden_map.ply
                                the ONE map lidar_localization reads (the same
                                host dir is mounted there at /maps/garden_map).
                                Written only by an explicit export; the
                                previous file is kept as a backup.

ROS interface (node name: glim_supervisor):
  services (std_srvs/Trigger):
    ~/start_mapping   start glim_rosnode, dump into a new session directory
    ~/stop_mapping    SIGINT GLIM and wait for the dump (state "saving")
    ~/open_viewer     offline_viewer on session <target_session>
    ~/close_viewer    end the offline viewer (unsaved changes are lost)
    ~/export_map      export session <target_session> to the active map
  parameter:
    target_session    session name for open_viewer / export_map. Cleared after
                      each use; the service response names the session it used
                      so a caller can verify it was not stale.
  topic:
    ~/status          std_msgs/String (transient local), JSON — see status().
"""

from __future__ import annotations

import collections
import datetime as _dt
import json
import os
import re
import shutil
import signal
import subprocess
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Deque, Dict, List, Optional, Sequence

SESSION_NAME_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")
SESSION_META = "session.json"
# GLIM's GlobalMapping::save() writes these; either proves a complete dump.
DUMP_MARKERS = ("graph.bin", "graph.txt")
ACTIVE_SOURCE_SUFFIX = ".source.json"

STATE_IDLE = "idle"
STATE_MAPPING = "mapping"
STATE_SAVING = "saving"
STATE_VIEWER = "viewer"
STATE_EXPORTING = "exporting"
STATE_ERROR = "error"  # last run failed; behaves like idle

# GLIM's offline viewer takes the start folder of its file dialogs from the
# "recent files" list of iridescence (guik::RecentFiles), stored in this ini
# file inside the container: one line per tag, "tag=path1;path2;". Paths that
# do not exist are ignored. Tags used by offline_viewer.cpp:
RECENT_FILES_INI = "/tmp/tmp_recent_files.ini"
RECENT_TAG_OPEN = "offline_viewer_open"     # File → Open (Additional) Map
RECENT_TAG_SAVE = "offline_viewer_save"     # File → Save → Save Map


def now_utc() -> _dt.datetime:
    return _dt.datetime.now(_dt.timezone.utc)


def iso(ts: float) -> str:
    return _dt.datetime.fromtimestamp(ts, _dt.timezone.utc).isoformat(timespec="seconds")


def valid_session_name(name: str) -> bool:
    """A plain directory name — no separators, no '..', no hidden entries."""
    return bool(SESSION_NAME_RE.match(name)) and ".." not in name


def new_session_name(existing: Sequence[str], when: Optional[_dt.datetime] = None) -> str:
    base = "mapping_" + (when or _dt.datetime.now()).strftime("%Y-%m-%d_%H%M%S")
    name, n = base, 2
    taken = set(existing)
    while name in taken:
        name = f"{base}_{n}"
        n += 1
    return name


def is_complete_dump(path: str) -> bool:
    return any(os.path.isfile(os.path.join(path, m)) for m in DUMP_MARKERS)


def dir_size_bytes(path: str) -> int:
    total = 0
    for root, _dirs, files in os.walk(path):
        for f in files:
            try:
                total += os.path.getsize(os.path.join(root, f))
            except OSError:
                pass
    return total


def read_json(path: str) -> Optional[dict]:
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        return data if isinstance(data, dict) else None
    except (OSError, ValueError):
        return None


def write_json(path: str, data: dict) -> None:
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
    os.replace(tmp, path)


def list_sessions(sessions_dir: str) -> List[dict]:
    """Every directory in sessions_dir, newest first."""
    out = []
    try:
        entries = os.listdir(sessions_dir)
    except OSError:
        return out
    for name in entries:
        path = os.path.join(sessions_dir, name)
        if not os.path.isdir(path) or not valid_session_name(name):
            continue
        meta = read_json(os.path.join(path, SESSION_META)) or {}
        complete = is_complete_dump(path)
        try:
            mtime = os.path.getmtime(path)
        except OSError:
            mtime = 0.0
        out.append({
            "name": name,
            # "mapping" = recorded by this supervisor; "saved" = a dump written
            # by some other tool (e.g. "Save Map" in the offline viewer).
            "kind": meta.get("kind", "saved"),
            "complete": complete,
            "started_at": meta.get("started_at"),
            "finished_at": meta.get("finished_at") or iso(mtime),
            "duration_s": meta.get("duration_s"),
            "size_bytes": dir_size_bytes(path),
            "mtime": mtime,
        })
    out.sort(key=lambda s: (s.get("started_at") or s["finished_at"] or "", s["mtime"]), reverse=True)
    for s in out:
        s.pop("mtime", None)
    return out


def new_merged_name(existing: Sequence[str], when: Optional[_dt.datetime] = None) -> str:
    base = "merged_" + (when or _dt.datetime.now()).strftime("%Y-%m-%d_%H%M%S")
    name, n = base, 2
    taken = set(existing)
    while name in taken:
        name = f"{base}_{n}"
        n += 1
    return name


def seed_recent_files(ini_path: str, entries: Dict[str, str]) -> None:
    """Put `entries` (tag → path) first in iridescence's recent-files ini so
    the viewer's dialogs open there. Other tags and later history are kept."""
    lines: Dict[str, str] = {}
    try:
        with open(ini_path, "r", encoding="utf-8") as f:
            for line in f:
                line = line.rstrip("\n")
                if not line:
                    break  # the reader stops at the first empty line too
                if "=" in line:
                    k, v = line.split("=", 1)
                    lines[k] = v
    except OSError:
        pass
    for tag, path in entries.items():
        rest = [p for p in lines.get(tag, "").split(";") if p and p != path]
        lines[tag] = ";".join([path] + rest[:9]) + ";"
    tmp = ini_path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        for k, v in lines.items():
            f.write(f"{k}={v}\n")
    os.replace(tmp, ini_path)


def backup_name(active_path: str, when: Optional[_dt.datetime] = None) -> str:
    stem, ext = os.path.splitext(active_path)
    return f"{stem}.backup-{(when or _dt.datetime.now()).strftime('%Y-%m-%d_%H%M%S')}{ext}"


def list_backups(active_path: str) -> List[str]:
    directory = os.path.dirname(active_path) or "."
    stem, ext = os.path.splitext(os.path.basename(active_path))
    prefix = f"{stem}.backup-"
    try:
        names = [n for n in os.listdir(directory) if n.startswith(prefix) and n.endswith(ext)]
    except OSError:
        return []
    return sorted((os.path.join(directory, n) for n in names), reverse=True)  # newest first


def install_active_map(tmp_path: str, active_path: str, source: dict, keep_backups: int,
                       when: Optional[_dt.datetime] = None) -> Optional[str]:
    """Move a freshly exported map into place, keeping the previous one.

    Returns the backup path (None when there was no previous map). Old
    backups beyond keep_backups are deleted (maps are large).
    """
    backup = None
    if os.path.exists(active_path):
        backup = backup_name(active_path, when)
        os.replace(active_path, backup)
        old_source = active_path + ACTIVE_SOURCE_SUFFIX
        if os.path.exists(old_source):
            os.replace(old_source, backup + ACTIVE_SOURCE_SUFFIX)
    os.replace(tmp_path, active_path)
    write_json(active_path + ACTIVE_SOURCE_SUFFIX, source)
    for stale in list_backups(active_path)[max(0, keep_backups):]:
        for p in (stale, stale + ACTIVE_SOURCE_SUFFIX):
            try:
                os.remove(p)
            except OSError:
                pass
    return backup


def active_map_info(active_path: str) -> dict:
    info: Dict[str, object] = {"path": active_path, "exists": os.path.isfile(active_path)}
    if info["exists"]:
        st = os.stat(active_path)
        info["size_bytes"] = st.st_size
        info["modified_at"] = iso(st.st_mtime)
        src = read_json(active_path + ACTIVE_SOURCE_SUFFIX)
        if src:
            info["source_session"] = src.get("session")
            info["exported_at"] = src.get("exported_at")
    info["backups"] = [os.path.basename(p) for p in list_backups(active_path)]
    return info


@dataclass
class Config:
    sessions_dir: str = "/glim/sessions"
    active_map_path: str = "/glim/active_map/garden_map.ply"
    config_path: str = "/glim/config"
    keep_backups: int = 3
    mapping_cmd: List[str] = field(default_factory=lambda: ["ros2", "run", "glim_ros", "glim_rosnode"])
    viewer_cmd: List[str] = field(default_factory=lambda: ["ros2", "run", "glim_ros", "offline_viewer"])
    log_lines: int = 40
    recent_files_ini: str = RECENT_FILES_INI


class Supervisor:
    """ROS-free core: state machine + process handling. Thread-safe."""

    def __init__(self, cfg: Config, on_change: Callable[[], None] = lambda: None,
                 popen: Callable[..., subprocess.Popen] = subprocess.Popen):
        self.cfg = cfg
        self._on_change = on_change
        self._popen = popen
        self._lock = threading.RLock()
        self._proc: Optional[subprocess.Popen] = None
        self._state = STATE_IDLE
        self._message = "Ready."
        self._session: Optional[str] = None
        self._started_wall: Optional[float] = None
        self._started_mono: Optional[float] = None
        self._export_tmp: Optional[str] = None
        # Empty folder prepared for "Save Map" while the viewer is open.
        self._save_target: Optional[str] = None
        self._log: Deque[str] = collections.deque(maxlen=cfg.log_lines)
        self._sessions_cache: List[dict] = []
        self._sessions_scanned = 0.0
        os.makedirs(cfg.sessions_dir, exist_ok=True)
        os.makedirs(os.path.dirname(cfg.active_map_path) or ".", exist_ok=True)

    # ── helpers ──────────────────────────────────────────────────────
    def _busy(self) -> bool:
        return self._state in (STATE_MAPPING, STATE_SAVING, STATE_VIEWER, STATE_EXPORTING)

    def _set(self, state: str, message: str) -> None:
        self._state, self._message = state, message
        self._on_change()

    def _spawn(self, argv: List[str], cwd: Optional[str] = None) -> subprocess.Popen:
        self._log.clear()
        # Own process group, so SIGINT reaches `ros2 run` AND the program it
        # started; stdout is drained continuously so the pipe never blocks.
        proc = self._popen(argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                           stdin=subprocess.DEVNULL, start_new_session=True, text=True, bufsize=1,
                           cwd=cwd)
        if proc.stdout is not None:
            threading.Thread(target=self._drain, args=(proc,), daemon=True).start()
        return proc

    def _drain(self, proc: subprocess.Popen) -> None:
        for line in proc.stdout:  # type: ignore[union-attr]
            line = line.rstrip("\n")
            print(line, flush=True)  # keep `docker logs` useful
            with self._lock:
                self._log.append(line)

    def _signal(self, sig: int) -> None:
        if self._proc is None:
            return
        try:
            os.killpg(os.getpgid(self._proc.pid), sig)
        except (ProcessLookupError, PermissionError, OSError):
            pass

    def _resolve_session(self, name: str) -> str:
        if not name:
            raise ValueError("No session selected (target_session is empty).")
        if not valid_session_name(name):
            raise ValueError(f"Invalid session name '{name}'.")
        path = os.path.join(self.cfg.sessions_dir, name)
        if not os.path.isdir(path):
            raise ValueError(f"Session '{name}' does not exist.")
        if not is_complete_dump(path):
            raise ValueError(f"Session '{name}' has no complete GLIM dump (graph.bin missing).")
        return path

    # ── actions (return (success, message)) ──────────────────────────
    def start_mapping(self) -> tuple:
        with self._lock:
            if self._busy():
                return False, f"Busy: {self._state}."
            name = new_session_name(os.listdir(self.cfg.sessions_dir))
            dump = os.path.join(self.cfg.sessions_dir, name)
            argv = list(self.cfg.mapping_cmd) + [
                "--ros-args", "-p", f"config_path:={self.cfg.config_path}", "-p", f"dump_path:={dump}"]
            try:
                self._proc = self._spawn(argv)
            except OSError as e:
                self._set(STATE_ERROR, f"Could not start GLIM: {e}")
                return False, self._message
            self._session = name
            self._started_wall, self._started_mono = time.time(), time.monotonic()
            self._set(STATE_MAPPING, f"Mapping into {name}.")
            return True, name

    def stop_mapping(self) -> tuple:
        with self._lock:
            if self._state == STATE_SAVING:
                return True, f"Already saving {self._session}."
            if self._state != STATE_MAPPING:
                return False, "Not mapping."
            self._signal(signal.SIGINT)
            self._set(STATE_SAVING, f"Saving {self._session} — GLIM finishes optimisation and writes the dump.")
            return True, self._session

    def open_viewer(self, session: str) -> tuple:
        with self._lock:
            if self._busy():
                return False, f"Busy: {self._state}."
            try:
                path = self._resolve_session(session)
            except ValueError as e:
                return False, str(e)
            # A fresh, empty folder for "Save Map", and the dialogs pointed at
            # the sessions directory: the file dialog does not list /tmp or
            # /glim on its own, and it can only pick an existing folder.
            target = os.path.join(self.cfg.sessions_dir, new_merged_name(os.listdir(self.cfg.sessions_dir)))
            try:
                os.makedirs(target)
                seed_recent_files(self.cfg.recent_files_ini, {
                    RECENT_TAG_SAVE: target,
                    RECENT_TAG_OPEN: self.cfg.sessions_dir.rstrip("/") + "/",
                })
            except OSError as e:
                print(f"glim_supervisor: could not prepare the save folder: {e}", flush=True)
            argv = list(self.cfg.viewer_cmd) + [path, "--config_path", self.cfg.config_path]
            try:
                self._proc = self._spawn(argv, cwd=self.cfg.sessions_dir)
            except OSError as e:
                self._remove_if_empty(target)
                self._set(STATE_ERROR, f"Could not start the offline viewer: {e}")
                return False, self._message
            self._session = session
            self._save_target = target if os.path.isdir(target) else None
            self._started_wall, self._started_mono = time.time(), time.monotonic()
            self._set(STATE_VIEWER, f"Offline viewer open on {session}."
                      + (f" Save merged maps to {target}." if self._save_target else ""))
            return True, session

    @staticmethod
    def _remove_if_empty(path: Optional[str]) -> bool:
        if not path:
            return False
        try:
            os.rmdir(path)  # only succeeds for an empty directory
            return True
        except OSError:
            return False

    def close_viewer(self) -> tuple:
        with self._lock:
            if self._state != STATE_VIEWER:
                return False, "The offline viewer is not open."
            self._signal(signal.SIGTERM)
            return True, self._session

    def export_map(self, session: str) -> tuple:
        with self._lock:
            if self._busy():
                return False, f"Busy: {self._state}."
            try:
                path = self._resolve_session(session)
            except ValueError as e:
                return False, str(e)
            stem, ext = os.path.splitext(self.cfg.active_map_path)
            tmp = f"{stem}.exporting{ext}"
            try:
                os.remove(tmp)
            except OSError:
                pass
            # --export_path: the viewer loads the dump (no re-optimisation),
            # writes the PLY and exits on its own (glim offline_viewer.cpp).
            argv = list(self.cfg.viewer_cmd) + [path, "--config_path", self.cfg.config_path, "--export_path", tmp]
            try:
                self._proc = self._spawn(argv)
            except OSError as e:
                self._set(STATE_ERROR, f"Could not start the export: {e}")
                return False, self._message
            self._session, self._export_tmp = session, tmp
            self._started_wall, self._started_mono = time.time(), time.monotonic()
            self._set(STATE_EXPORTING, f"Exporting {session} to {os.path.basename(self.cfg.active_map_path)}.")
            return True, session

    def shutdown(self, timeout_s: float = 170.0) -> None:
        """Container stop: never lose a mapping run — let GLIM save first."""
        with self._lock:
            proc, state = self._proc, self._state
            if proc is None:
                return
            self._signal(signal.SIGINT if state in (STATE_MAPPING, STATE_SAVING) else signal.SIGTERM)
        try:
            proc.wait(timeout=timeout_s)
        except subprocess.TimeoutExpired:
            self._signal(signal.SIGKILL)
        self.poll()

    # ── periodic ─────────────────────────────────────────────────────
    def poll(self) -> None:
        """Detect the end of the running program and finish its bookkeeping."""
        with self._lock:
            if self._proc is None or self._proc.poll() is None:
                return
            code = self._proc.returncode
            state, session = self._state, self._session
            self._proc = None
            duration = time.monotonic() - (self._started_mono or time.monotonic())

            if state in (STATE_MAPPING, STATE_SAVING):
                path = os.path.join(self.cfg.sessions_dir, session or "")
                if session and os.path.isdir(path) and is_complete_dump(path):
                    write_json(os.path.join(path, SESSION_META), {
                        "kind": "mapping",
                        "started_at": iso(self._started_wall or time.time()),
                        "finished_at": iso(time.time()),
                        "duration_s": round(duration, 1),
                        "exit_code": code,
                    })
                    unexpected = state == STATE_MAPPING
                    msg = (f"GLIM exited on its own (code {code}); the dump {session} was saved."
                           if unexpected else f"Saved {session}.")
                    self._set(STATE_ERROR if unexpected else STATE_IDLE, msg)
                else:
                    self._set(STATE_ERROR, f"GLIM exited (code {code}) without a complete dump for {session}.")
            elif state == STATE_VIEWER:
                target, self._save_target = self._save_target, None
                if target and not self._remove_if_empty(target) and is_complete_dump(target):
                    write_json(os.path.join(target, SESSION_META), {
                        "kind": "merged",
                        "opened_from": session,
                        "finished_at": iso(time.time()),
                    })
                    self._set(STATE_IDLE, f"Offline viewer closed; saved map {os.path.basename(target)}.")
                else:
                    self._set(STATE_IDLE, f"Offline viewer closed ({session}).")
            elif state == STATE_EXPORTING:
                tmp = self._export_tmp
                self._export_tmp = None
                if code == 0 and tmp and os.path.isfile(tmp) and os.path.getsize(tmp) > 0:
                    backup = install_active_map(tmp, self.cfg.active_map_path, {
                        "session": session, "exported_at": now_utc().isoformat(timespec="seconds")},
                        self.cfg.keep_backups)
                    note = f" Previous map kept as {os.path.basename(backup)}." if backup else ""
                    self._set(STATE_IDLE, f"Active map exported from {session}.{note}")
                else:
                    if tmp:
                        try:
                            os.remove(tmp)
                        except OSError:
                            pass
                    self._set(STATE_ERROR, f"Export of {session} failed (code {code}); the active map is unchanged.")
            self._session = None
            self._started_wall = self._started_mono = None
            self._sessions_scanned = 0.0  # force a rescan

    def status(self, rescan_s: float = 10.0) -> dict:
        with self._lock:
            if time.monotonic() - self._sessions_scanned > rescan_s or not self._sessions_scanned:
                self._sessions_cache = list_sessions(self.cfg.sessions_dir)
                self._sessions_scanned = time.monotonic()
            elapsed = (time.monotonic() - self._started_mono) if self._started_mono else None
            return {
                "state": self._state,
                "message": self._message,
                "session": self._session,
                "started_at": iso(self._started_wall) if self._started_wall else None,
                "elapsed_s": round(elapsed, 1) if elapsed is not None else None,
                "sessions_dir": self.cfg.sessions_dir,
                # The empty "Save Map" folder is not a session until something is saved.
                "sessions": [x for x in self._sessions_cache
                             if not (self._save_target and x["name"] == os.path.basename(self._save_target)
                                     and not x["complete"])],
                "active_map": active_map_info(self.cfg.active_map_path),
                "log_tail": list(self._log)[-15:],
                "save_target": self._save_target,
            }


# ── ROS node ─────────────────────────────────────────────────────────────────
def main() -> None:  # pragma: no cover - needs a ROS installation
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.parameter import Parameter
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    class GlimSupervisorNode(Node):
        def __init__(self) -> None:
            super().__init__("glim_supervisor")
            cfg = Config(
                sessions_dir=self.declare_parameter("sessions_dir", Config.sessions_dir).value,
                active_map_path=self.declare_parameter("active_map_path", Config.active_map_path).value,
                config_path=self.declare_parameter("config_path", Config.config_path).value,
                keep_backups=int(self.declare_parameter("keep_backups", Config.keep_backups).value),
            )
            self.declare_parameter("target_session", "")
            self._dirty = threading.Event()
            self.sup = Supervisor(cfg, on_change=self._dirty.set)
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.pub = self.create_publisher(String, "~/status", qos)
            self.create_service(Trigger, "~/start_mapping", self._wrap(lambda: self.sup.start_mapping()))
            self.create_service(Trigger, "~/stop_mapping", self._wrap(lambda: self.sup.stop_mapping()))
            self.create_service(Trigger, "~/open_viewer", self._wrap(lambda: self.sup.open_viewer(self._take_target())))
            self.create_service(Trigger, "~/close_viewer", self._wrap(lambda: self.sup.close_viewer()))
            self.create_service(Trigger, "~/export_map", self._wrap(lambda: self.sup.export_map(self._take_target())))
            self._last_pub = 0.0
            self.create_timer(0.5, self._tick)
            self._publish()
            self.get_logger().info(f"glim_supervisor ready (sessions: {cfg.sessions_dir}, "
                                   f"active map: {cfg.active_map_path})")

        def _take_target(self) -> str:
            name = str(self.get_parameter("target_session").value or "")
            # Clear it so a later call can never reuse a stale selection.
            self.set_parameters([Parameter("target_session", value="")])
            return name

        def _wrap(self, fn):
            def handler(_req, res):
                ok, msg = fn()
                res.success, res.message = bool(ok), str(msg)
                self._publish()
                return res
            return handler

        def _tick(self) -> None:
            self.sup.poll()
            busy = self.sup._busy()
            period = 1.0 if busy else 10.0
            if self._dirty.is_set() or time.monotonic() - self._last_pub >= period:
                self._publish()

        def _publish(self) -> None:
            self._dirty.clear()
            self._last_pub = time.monotonic()
            msg = String()
            msg.data = json.dumps(self.sup.status())
            self.pub.publish(msg)

    rclpy.init()
    node = GlimSupervisorNode()

    def on_term(_signum, _frame):
        # docker stop: let a running mapping save its dump before exiting
        # (compose sets stop_grace_period long enough for that).
        node.get_logger().info("Stopping — waiting for GLIM to finish.")
        node.sup.shutdown()
        rclpy.try_shutdown()

    signal.signal(signal.SIGTERM, on_term)
    signal.signal(signal.SIGINT, on_term)
    try:
        rclpy.spin(node)
    except Exception:  # noqa: BLE001 - shutting down
        pass
    finally:
        node.sup.shutdown()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
