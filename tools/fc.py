#!/usr/bin/env python3
"""USB debug CLI for Pixhawk 6C: MAVFTP, scripting, STATUSTEXT. QGC optional via UDP 14550."""

from __future__ import annotations

import argparse
import logging
import os
import socket
import subprocess
import sys
import time
from pathlib import Path

from pymavlink import mavftp, mavutil
from serial.tools import list_ports

ROOT = Path(__file__).resolve().parents[1]
I2C_SCAN_LUA = ROOT / "i2c_scan.lua"
BME280_TEST_LUA = ROOT / "bme280_test.lua"
REMOTE_SCRIPTS = "APM/scripts"
REMOTE_SCAN = f"{REMOTE_SCRIPTS}/i2c_scan.lua"
REMOTE_TEST = f"{REMOTE_SCRIPTS}/bme280_test.lua"
TCP_HUB = "tcp:127.0.0.1:14551"
TCP_HUB_PORT = 14551
QGC_UDP = "udp:127.0.0.1:14550"
SOURCE_SYSTEM = 253
HOLYBRO_VID = 0x3162
HOLYBRO_PID = 0x0053

log = logging.getLogger("fc")


def _tcp_hub_up() -> bool:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(0.3)
        try:
            s.connect(("127.0.0.1", TCP_HUB_PORT))
            return True
        except OSError:
            return False


def list_pixhawk_ports() -> list[tuple[str, str]]:
    found = []
    for p in list_ports.comports():
        hwid = p.hwid or ""
        if p.vid == HOLYBRO_VID and p.pid == HOLYBRO_PID:
            iface = "MAVLink" if "MI_00" in hwid else "SLCAN/aux"
            found.append((p.device, f"{p.description} [{iface}] {hwid}"))
    return found


def detect_mavlink_com() -> str:
    ports = list_pixhawk_ports()
    if not ports:
        raise SystemExit("Pixhawk 6C not found (USB VID 3162 PID 0053). Plug USB and close nothing that holds COM.")
    for device, desc in ports:
        if "MI_00" in desc:
            return device
    for device, _desc in ports:
        if device.upper() == "COM18":
            return device
    return ports[0][0]


def resolve_device(explicit: str | None) -> str:
    if explicit:
        return explicit
    if _tcp_hub_up():
        time.sleep(0.2)
        return TCP_HUB
    return detect_mavlink_com()


def connect(device: str | None = None, timeout: float = 10.0):
    dev = resolve_device(device)
    log.info("Connecting %s", dev)
    master = mavutil.mavlink_connection(
        dev,
        baud=115200,
        source_system=SOURCE_SYSTEM,
        source_component=190,
    )
    hb = master.wait_heartbeat(timeout=timeout)
    if hb is None:
        master.close()
        raise SystemExit(f"No heartbeat on {dev}. If QGC holds USB, run: py -3.12 tools/fc.py serve")
    if master.target_component in (0, None):
        master.target_component = 1
    master.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0,
        0,
        0,
    )
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        4,
        1,
    )
    log.info(
        "Heartbeat sys=%s comp=%s type=%s autopilot=%s",
        master.target_system,
        master.target_component,
        hb.type,
        hb.autopilot,
    )
    return master


def ftp_client(master) -> mavftp.MAVFTP:
    return mavftp.MAVFTP(
        master,
        target_system=master.target_system,
        target_component=master.target_component,
    )


def ftp_put(master, local: Path, remote: str) -> None:
    ftp = ftp_client(master)
    ret = ftp.cmd_put([str(local), remote])
    ret = ftp.process_ftp_reply("put", timeout=60)
    if ret.error_code:
        ret.display_message()
        raise SystemExit(f"MAVFTP put failed: {remote}")
    log.info("Uploaded %s -> %s", local.name, remote)


def ftp_list(master, path: str) -> list:
    ftp = ftp_client(master)
    ret = ftp.cmd_list([path])
    if ret.error_code:
        ret.display_message()
        return []
    return list(ftp.list_result or [])


def ftp_rm(master, path: str) -> None:
    ftp = ftp_client(master)
    ret = ftp.cmd_rm([path])
    if ret.error_code and ret.error_code != mavftp.FtpError.FileNotFound:
        ret.display_message()


def ftp_mkdir(master, path: str) -> None:
    ftp = ftp_client(master)
    ret = ftp.cmd_mkdir([path])
    if ret.error_code and ret.error_code not in (
        mavftp.FtpError.FileExists,
        mavftp.FtpError.Fail,
        mavftp.FtpError.FailErrno,
    ):
        ret.display_message()


def param_get(master, name: str, timeout: float = 5.0) -> float | None:
    name_b = name.encode("ascii")
    master.mav.param_request_read_send(
        master.target_system, master.target_component, name_b, -1
    )
    t0 = time.time()
    while time.time() - t0 < timeout:
        m = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if m is None:
            continue
        pname = m.param_id
        if isinstance(pname, bytes):
            pname = pname.decode("ascii", "ignore")
        pname = pname.split("\x00", 1)[0]
        if pname == name:
            return float(m.param_value)
    return None


def param_set(master, name: str, value: float, timeout: float = 5.0) -> float | None:
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        name.encode("ascii"),
        float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
    )
    t0 = time.time()
    while time.time() - t0 < timeout:
        m = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if m is None:
            continue
        pname = m.param_id
        if isinstance(pname, bytes):
            pname = pname.decode("ascii", "ignore")
        pname = pname.split("\x00", 1)[0]
        if pname == name:
            return float(m.param_value)
    return None


def scripting_restart(master) -> None:
    master.mav.command_int_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        mavutil.mavlink.MAV_CMD_SCRIPTING,
        0,
        0,
        mavutil.mavlink.SCRIPTING_CMD_STOP_AND_RESTART,
        0,
        0,
        0,
        0,
        0,
        0,
    )
    ack = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=3)
    if ack:
        log.info("SCRIPTING restart ack result=%s", ack.result)
    else:
        log.warning("No COMMAND_ACK for scripting restart")


def reboot(master) -> None:
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
        0,
        1,
        0,
        0,
        0,
        0,
        0,
        0,
    )
    log.info("Reboot requested")


def statustext_str(msg) -> str:
    text = msg.text
    if isinstance(text, bytes):
        text = text.decode("utf-8", "replace")
    return text.split("\x00", 1)[0]


def monitor_messages(master, seconds: float, grep: str | None = None) -> list[str]:
    collected: list[str] = []
    t_end = time.time() + seconds
    while time.time() < t_end:
        msg = master.recv_match(type="STATUSTEXT", blocking=True, timeout=0.5)
        if msg is None:
            continue
        text = statustext_str(msg)
        line = f"[sev {msg.severity}] {text}"
        if grep and grep.lower() not in text.lower():
            continue
        print(line, flush=True)
        collected.append(text)
    return collected


def ensure_scripts_dir(master) -> None:
    ftp_mkdir(master, "APM")
    ftp_mkdir(master, REMOTE_SCRIPTS)


def cmd_ports(_args) -> None:
    if _tcp_hub_up():
        print(f"USB hub up: {TCP_HUB}  QGC: {QGC_UDP}")
    ports = list_pixhawk_ports()
    if not ports:
        print("No Pixhawk 6C USB ports")
        return
    for device, desc in ports:
        print(f"{device}\t{desc}")


def cmd_ping(args) -> None:
    master = connect(args.device)
    print(
        f"OK sys={master.target_system} comp={master.target_component} "
        f"device={resolve_device(args.device)}"
    )
    master.close()


def _mavproxy_cmd(com: str, tablet: str | None = None) -> list[str]:
    mavproxy = Path(sys.executable).parent / "Scripts" / "mavproxy.py"
    if not mavproxy.exists():
        raise SystemExit(f"mavproxy.py not found at {mavproxy}")
    state = ROOT / ".fc_state"
    state.mkdir(exist_ok=True)
    cmd = [
        sys.executable,
        str(mavproxy),
        f"--master={com}",
        "--baudrate=115200",
        f"--out={QGC_UDP}",
        f"--out=tcpin:0.0.0.0:{TCP_HUB_PORT}",
        f"--source-system={SOURCE_SYSTEM}",
        "--nodtr",
        "--nowait",
        f"--state-basedir={state}",
    ]
    if tablet:
        cmd.append(f"--out=udp:{tablet}")
    return cmd


def _forward_msg(src, destinations) -> None:
    try:
        msg = src.recv_msg()
    except OSError:
        return
    if msg is None:
        return
    buf = msg.get_msgbuf()
    if not buf:
        return
    for dst in destinations:
        try:
            dst.write(buf)
        except OSError:
            pass


def cmd_serve(args) -> None:
    if _tcp_hub_up():
        raise SystemExit(f"Hub already listening on {TCP_HUB_PORT}")
    com = args.device or detect_mavlink_com()
    print("MAVProxy:")
    print(f"  master {com}")
    print(f"  QGC    {QGC_UDP}  (CommLink: UDP listen 14550, not USB)")
    print(f"  CLI    {TCP_HUB}")
    if args.tablet:
        print(f"  Tablet udp:{args.tablet}  (Router UDP in, usually :14540)")
    if args.builtin:
        serial_m = mavutil.mavlink_connection(
            com, baud=115200, autoreconnect=True, source_system=SOURCE_SYSTEM
        )
        udp_m = mavutil.mavlink_connection("udpout:127.0.0.1:14550", source_system=SOURCE_SYSTEM)
        tcp_m = mavutil.mavlink_connection(
            f"tcpin:0.0.0.0:{TCP_HUB_PORT}", source_system=SOURCE_SYSTEM
        )
        destinations = [udp_m, tcp_m]
        if args.tablet:
            tablet_m = mavutil.mavlink_connection(
                f"udpout:{args.tablet}", source_system=SOURCE_SYSTEM
            )
            destinations.append(tablet_m)
        print("Built-in mux running. Ctrl+C to stop.", flush=True)
        try:
            while True:
                _forward_msg(serial_m, destinations)
                _forward_msg(udp_m, [serial_m])
                _forward_msg(tcp_m, [serial_m])
                time.sleep(0.001)
        except KeyboardInterrupt:
            print("Hub stopped")
        return

    cmd = _mavproxy_cmd(com, args.tablet)
    print(" ".join(cmd), flush=True)
    if sys.platform == "win32":
        subprocess.Popen(cmd, cwd=str(ROOT), creationflags=subprocess.CREATE_NEW_CONSOLE)
        print("MAVProxy started in a new console. Leave that window open.")
        return
    os.execv(sys.executable, cmd)


def cmd_ls(args) -> None:
    master = connect(args.device)
    path = args.path or REMOTE_SCRIPTS
    entries = ftp_list(master, path)
    print(f"{path}:")
    if not entries:
        print("  (empty or missing)")
    for e in entries:
        kind = "DIR " if e.is_dir else "FILE"
        print(f"  {kind} {e.name}\t{e.size_b}")
    master.close()


def cmd_put(args) -> None:
    local = Path(args.local).resolve()
    if not local.is_file():
        raise SystemExit(f"Missing file: {local}")
    remote = args.remote or f"{REMOTE_SCRIPTS}/{local.name}"
    master = connect(args.device)
    ensure_scripts_dir(master)
    ftp_put(master, local, remote)
    master.close()


def cmd_param_get(args) -> None:
    master = connect(args.device)
    value = param_get(master, args.name)
    if value is None:
        raise SystemExit(f"No reply for {args.name}")
    print(f"{args.name} = {value}")
    master.close()


def cmd_param_set(args) -> None:
    master = connect(args.device)
    value = param_set(master, args.name, args.value)
    if value is None:
        raise SystemExit(f"No ACK for {args.name}")
    print(f"{args.name} = {value}")
    master.close()


def cmd_enable(args) -> None:
    master = connect(args.device)
    current = param_get(master, "SCR_ENABLE")
    print(f"SCR_ENABLE was {current}")
    if current == 1.0:
        print("Already enabled")
        master.close()
        return
    written = param_set(master, "SCR_ENABLE", 1.0)
    print(f"SCR_ENABLE = {written}")
    print("Reboot required (was off at boot). Rebooting...")
    reboot(master)
    master.close()
    print("Wait ~12s for USB to come back, then ping/scan.")


def cmd_restart(args) -> None:
    master = connect(args.device)
    scripting_restart(master)
    master.close()


def cmd_rm(args) -> None:
    master = connect(args.device)
    ftp_rm(master, args.remote)
    print(f"removed {args.remote}")
    master.close()


def cmd_test_bme(args) -> None:
    if not BME280_TEST_LUA.is_file():
        raise SystemExit(f"Missing {BME280_TEST_LUA}")
    master = connect(args.device)
    ensure_scripts_dir(master)
    ftp_rm(master, REMOTE_SCAN)
    ftp_rm(master, f"{REMOTE_SCRIPTS}/bme280_leak_monitor.lua")
    ftp_put(master, BME280_TEST_LUA, REMOTE_TEST)
    scripting_restart(master)
    print("--- BME280 STATUSTEXT / NAMED_VALUE_FLOAT ---", flush=True)
    t_end = time.time() + args.seconds
    last_hb = 0.0
    while time.time() < t_end:
        now = time.time()
        if now - last_hb > 1:
            master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0,
            )
            last_hb = now
        msg = master.recv_match(
            type=["STATUSTEXT", "STATUSTEXT_LONG", "NAMED_VALUE_FLOAT"],
            blocking=True,
            timeout=0.4,
        )
        if msg is None:
            continue
        kind = msg.get_type()
        if kind.startswith("STATUSTEXT"):
            print(f"[sev {msg.severity}] {statustext_str(msg)}", flush=True)
        else:
            name = msg.name
            if isinstance(name, bytes):
                name = name.decode("ascii", "ignore")
            name = name.split("\x00", 1)[0]
            print(f"  {name}={msg.value:.2f}", flush=True)
    master.close()


def cmd_monitor(args) -> None:
    master = connect(args.device)
    print(f"STATUSTEXT {args.seconds}s...", flush=True)
    monitor_messages(master, args.seconds, args.grep)
    master.close()


def cmd_scan(args) -> None:
    if not I2C_SCAN_LUA.is_file():
        raise SystemExit(f"Missing {I2C_SCAN_LUA}")
    master = connect(args.device)
    ensure_scripts_dir(master)

    existing = ftp_list(master, REMOTE_SCRIPTS)
    names = {e.name for e in existing}
    if "bme280_leak_monitor.lua" in names:
        log.warning("bme280_leak_monitor.lua is on the card — both scripts will run. Remove it for a clean scan.")

    ftp_put(master, I2C_SCAN_LUA, REMOTE_SCAN)

    scr = param_get(master, "SCR_ENABLE")
    print(f"SCR_ENABLE = {scr}")
    if scr != 1.0:
        param_set(master, "SCR_ENABLE", 1.0)
        print("Scripting was off — rebooting FC. Re-run scan after it enumerates.")
        reboot(master)
        master.close()
        return

    scripting_restart(master)
    print("--- I2C scan STATUSTEXT ---", flush=True)
    lines = monitor_messages(master, args.seconds, None)
    if not lines:
        # catch late Lua output / non-STATUSTEXT names
        t_end = time.time() + 3
        while time.time() < t_end:
            msg = master.recv_match(blocking=True, timeout=0.4)
            if msg is None:
                continue
            if msg.get_type() in ("STATUSTEXT", "STATUSTEXT_LONG"):
                text = statustext_str(msg)
                print(f"[sev {getattr(msg, 'severity', '?')}] {text}", flush=True)
                lines.append(text)
    master.close()

    hits = [t for t in lines if "bus " in t.lower() or "i2c" in t.lower() or "bme" in t.lower() or "Lua" in t]
    print("--- summary ---")
    if hits:
        for t in hits:
            print(t)
    else:
        print("No I2C/Lua lines. Check SCR_HEAP_SIZE, SD card, and that only i2c_scan.lua is in APM/scripts.")


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Pixhawk 6C USB debug (MAVFTP / scripting / I2C scan)")
    p.add_argument("-d", "--device", help="COM18 or tcp:127.0.0.1:14551. Default: hub if up, else auto COM")
    p.add_argument("-v", "--verbose", action="store_true")
    sub = p.add_subparsers(dest="cmd", required=True)

    sub.add_parser("ports", help="List Pixhawk USB COM ports").set_defaults(func=cmd_ports)
    sub.add_parser("ping", help="Wait for heartbeat").set_defaults(func=cmd_ping)
    srv = sub.add_parser("serve", help="MAVProxy: USB -> QGC UDP 14550 + CLI TCP 14551")
    srv.add_argument("--builtin", action="store_true", help="Use the tiny mux instead of MAVProxy")
    srv.add_argument(
        "--tablet",
        metavar="IP:PORT",
        help="Also send MAVLink UDP to the tablet Router (e.g. 192.168.0.50:14540)",
    )
    srv.set_defaults(func=cmd_serve)

    ls = sub.add_parser("ls", help="MAVFTP list")
    ls.add_argument("path", nargs="?", default=None)
    ls.set_defaults(func=cmd_ls)

    put = sub.add_parser("put", help="MAVFTP upload")
    put.add_argument("local")
    put.add_argument("remote", nargs="?")
    put.set_defaults(func=cmd_put)

    rm = sub.add_parser("rm", help="MAVFTP remove")
    rm.add_argument("remote")
    rm.set_defaults(func=cmd_rm)

    tb = sub.add_parser("test-bme", help="Upload bme280_test.lua, remove scan/leak scripts, print T/RH")
    tb.add_argument("--seconds", type=float, default=20)
    tb.set_defaults(func=cmd_test_bme)

    pg = sub.add_parser("param-get")
    pg.add_argument("name")
    pg.set_defaults(func=cmd_param_get)
    ps = sub.add_parser("param-set")
    ps.add_argument("name")
    ps.add_argument("value", type=float)
    ps.set_defaults(func=cmd_param_set)

    sub.add_parser("enable", help="SCR_ENABLE=1 and reboot if needed").set_defaults(func=cmd_enable)
    sub.add_parser("restart", help="Reload Lua without reboot").set_defaults(func=cmd_restart)

    mon = sub.add_parser("monitor", help="Print STATUSTEXT")
    mon.add_argument("--seconds", type=float, default=20)
    mon.add_argument("--grep", default=None)
    mon.set_defaults(func=cmd_monitor)

    scan = sub.add_parser("scan", help="Upload i2c_scan.lua, restart scripting, print I2C results")
    scan.add_argument("--seconds", type=float, default=25)
    scan.set_defaults(func=cmd_scan)
    return p


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(levelname)s %(message)s",
    )
    args.func(args)


if __name__ == "__main__":
    main()
