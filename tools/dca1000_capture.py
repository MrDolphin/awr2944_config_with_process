#!/usr/bin/env python3
"""
Capture DCA1000 UDP ADC packets on a Raspberry Pi and save them locally.

This script is intentionally focused on a clean baseline capture:
  - listen on the DCA1000 data UDP port, usually 4098
  - strip the 10-byte DCA1000 packet header by default
  - write the ADC payload stream to a .bin file
  - write a companion metadata .json file for later Python/MATLAB analysis

Typical use on the Raspberry Pi:
  sudo python3 tools/dca1000_capture.py \
      --cfg Config/mimo_4tx_full.cfg \
      --duration 10 \
      --start-dca

If DCA1000 is already started by another tool, use:
  python3 tools/dca1000_capture.py --duration 10 --no-control

Notes:
  - DCA1000 data packets usually contain a 10-byte header:
      uint32 sequence number + uint48 byte count, followed by ADC payload.
  - The output .bin therefore matches the raw payload style expected by most
    offline ADC parsers, not a pcap file.
"""

from __future__ import annotations

import argparse
import datetime as _dt
import glob
import json
import os
import shutil
import socket
import struct
import sys
import time
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple


DCA_PACKET_HEADER_BYTES = 10
DEFAULT_DATA_PORT = 4098
DEFAULT_CONFIG_PORT = 4096
DEFAULT_DCA_IP = "192.168.33.180"

# Common DCA1000 command packet framing used by TI's CLI tool.
# The start/stop record commands are simple and useful when this script
# replaces DCA1000EVM_CLI_Record.exe as the UDP receiver.
DCA_CMD_HEADER = 0xA55A
DCA_CMD_FOOTER = 0xEEAA
DCA_CMD_RECORD_START = 0x05
DCA_CMD_RECORD_STOP = 0x06


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture DCA1000 raw ADC UDP stream to .bin on Raspberry Pi."
    )
    parser.add_argument("--cfg", help="Radar .cfg file used for this capture.")
    parser.add_argument(
        "--cf-json",
        default="mathlab/PostProc/cf.json",
        help="Optional DCA1000 cf.json to import IP/ports from.",
    )
    parser.add_argument("--listen-ip", default="0.0.0.0", help="Local IP to bind.")
    parser.add_argument("--data-port", type=int, default=None, help="DCA1000 UDP data port.")
    parser.add_argument("--dca-ip", default=None, help="DCA1000 board IP for control commands.")
    parser.add_argument("--config-port", type=int, default=None, help="DCA1000 UDP config port.")
    parser.add_argument(
        "--output-dir",
        default="auto",
        help="Capture directory. Use 'auto' to choose a large local disk.",
    )
    parser.add_argument(
        "--min-free-gb",
        type=float,
        default=2.0,
        help="Minimum free space required before capture starts.",
    )
    parser.add_argument("--prefix", default="adc_data", help="Output file prefix.")
    parser.add_argument("--duration", type=float, help="Capture duration in seconds.")
    parser.add_argument("--bytes", type=int, help="Stop after this many payload bytes.")
    parser.add_argument(
        "--frames",
        type=int,
        help="Stop after approximately this many frames. Requires --cfg for frame size estimate.",
    )
    parser.add_argument(
        "--raw-packets",
        action="store_true",
        help="Save full UDP payload including the 10-byte DCA1000 packet header.",
    )
    parser.add_argument(
        "--no-control",
        action="store_true",
        help="Do not send DCA1000 start/stop commands; listen only.",
    )
    parser.add_argument(
        "--start-dca",
        action="store_true",
        help="Send a DCA1000 record-start command before listening.",
    )
    parser.add_argument(
        "--stop-dca",
        action="store_true",
        help="Send a DCA1000 record-stop command when capture exits.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Parse CFG/DCA settings and print the capture plan without opening UDP or sending commands.",
    )
    parser.add_argument(
        "--socket-buffer-mb",
        type=int,
        default=128,
        help="Requested UDP receive buffer size in MB.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=3.0,
        help="Seconds without data before printing a warning.",
    )
    return parser.parse_args()


def load_dca_json(path: Optional[str]) -> Dict[str, object]:
    if not path:
        return {}
    p = Path(path)
    if not p.exists():
        return {}
    try:
        data = json.loads(p.read_text(encoding="utf-8"))
        return data.get("DCA1000Config", data)
    except Exception as exc:
        print(f"[WARN] Failed to read {path}: {exc}", file=sys.stderr)
        return {}


def ethernet_from_cf_json(data: Dict[str, object]) -> Tuple[Optional[str], Optional[int], Optional[int]]:
    eth = data.get("ethernetConfig") if isinstance(data, dict) else None
    if not isinstance(eth, dict):
        return None, None, None
    ip = eth.get("DCA1000IPAddress")
    cfg_port = eth.get("DCA1000ConfigPort")
    data_port = eth.get("DCA1000DataPort")
    return (
        str(ip) if ip else None,
        int(cfg_port) if cfg_port is not None else None,
        int(data_port) if data_port is not None else None,
    )


def count_bits(mask: int) -> int:
    return bin(mask).count("1")


def parse_radar_cfg(path: Optional[str]) -> Dict[str, object]:
    if not path:
        return {}
    cfg_path = Path(path)
    if not cfg_path.exists():
        return {"cfg_path": str(path), "error": "cfg file not found"}

    info: Dict[str, object] = {"cfg_path": str(cfg_path)}
    chirp_tx_masks: Dict[int, int] = {}

    for raw_line in cfg_path.read_text(encoding="utf-8", errors="ignore").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("%"):
            continue
        parts = line.split()
        cmd = parts[0]
        try:
            if cmd == "channelCfg" and len(parts) >= 3:
                rx_mask = int(parts[1], 0)
                tx_mask = int(parts[2], 0)
                info["rx_mask"] = rx_mask
                info["tx_mask"] = tx_mask
                info["num_rx"] = count_bits(rx_mask)
                info["num_tx"] = count_bits(tx_mask)
            elif cmd == "profileCfg" and len(parts) >= 12:
                info["start_freq_ghz"] = float(parts[2])
                info["idle_time_us"] = float(parts[3])
                info["adc_start_time_us"] = float(parts[4])
                info["ramp_end_time_us"] = float(parts[5])
                info["freq_slope_mhz_per_us"] = float(parts[8])
                info["num_adc_samples"] = int(float(parts[10]))
                info["sample_rate_ksps"] = float(parts[11])
            elif cmd == "chirpCfg" and len(parts) >= 9:
                chirp_start = int(parts[1])
                chirp_end = int(parts[2])
                tx_mask = int(parts[8], 0)
                for idx in range(chirp_start, chirp_end + 1):
                    chirp_tx_masks[idx] = tx_mask
            elif cmd == "frameCfg" and len(parts) >= 7:
                chirp_start = int(parts[1])
                chirp_end = int(parts[2])
                loops = int(parts[3])
                # AWR2944 MCU+ SDK profiles add numAdcSamples before the frame
                # periodicity, for example:
                #   frameCfg 0 3 24 0 256 50 1 0
                # Older profiles omit numAdcSamples. Preserve both layouts.
                if len(parts) >= 9:
                    info["frame_num_adc_samples"] = int(float(parts[5]))
                    frame_period_idx = 6
                else:
                    frame_period_idx = 5
                frame_period_ms = float(parts[frame_period_idx])
                info["frame_chirp_start"] = chirp_start
                info["frame_chirp_end"] = chirp_end
                info["num_loops"] = loops
                info["frame_period_ms"] = frame_period_ms
                info["num_chirps_per_loop"] = chirp_end - chirp_start + 1
                info["num_chirps_per_frame"] = (chirp_end - chirp_start + 1) * loops
            elif cmd == "adcCfg" and len(parts) >= 2:
                info["adc_output_fmt"] = int(parts[1])
            elif cmd == "adcbufCfg" and len(parts) >= 6:
                info["adcbuf_cfg"] = parts[1:]
            elif cmd == "lvdsStreamCfg":
                info["lvds_stream_cfg"] = parts[1:]
        except ValueError:
            continue

    if chirp_tx_masks:
        info["chirp_tx_masks"] = chirp_tx_masks

    num_rx = int(info.get("num_rx", 4))
    num_samples = int(info.get("num_adc_samples", 256))
    num_chirps = int(info.get("num_chirps_per_frame", 0))
    # adcCfg 2 in this project means complex samples. Each complex sample is
    # int16 I + int16 Q = 4 bytes per RX per chirp per ADC sample.
    bytes_per_sample_per_rx = 4
    if num_chirps:
        info["estimated_payload_bytes_per_frame"] = (
            num_samples * num_rx * num_chirps * bytes_per_sample_per_rx
        )
    return info


def candidate_output_dirs() -> Iterable[Path]:
    yield Path("/data/awr2944_adc")
    for pat in ("/mnt/*", "/media/*", "/media/pi/*", "/run/media/*"):
        for entry in glob.glob(pat):
            p = Path(entry)
            if p.is_dir():
                yield p / "awr2944_adc"
    yield Path("/home/pi/awr2944_adc")
    yield Path.cwd() / "captures"


def choose_output_dir(output_dir: str, min_free_gb: float) -> Path:
    if output_dir != "auto":
        path = Path(output_dir).expanduser()
        path.mkdir(parents=True, exist_ok=True)
        return path

    best: Optional[Tuple[int, Path]] = None
    for path in candidate_output_dirs():
        try:
            path.mkdir(parents=True, exist_ok=True)
            usage = shutil.disk_usage(path)
            if usage.free >= int(min_free_gb * 1024**3):
                if best is None or usage.free > best[0]:
                    best = (usage.free, path)
        except OSError:
            continue
    if best is None:
        raise RuntimeError(f"No output directory has at least {min_free_gb:.1f} GB free.")
    return best[1]


def make_capture_paths(base_dir: Path, prefix: str) -> Tuple[Path, Path]:
    stamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    session_dir = base_dir / stamp
    session_dir.mkdir(parents=True, exist_ok=True)
    return session_dir / f"{prefix}_{stamp}.bin", session_dir / f"{prefix}_{stamp}.json"


def dca_command_packet(command: int, payload: bytes = b"") -> bytes:
    return struct.pack("<HHH", DCA_CMD_HEADER, command, len(payload)) + payload + struct.pack(
        "<H", DCA_CMD_FOOTER
    )


def send_dca_command(ip: str, port: int, command: int, name: str, timeout: float = 0.5) -> None:
    packet = dca_command_packet(command)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.settimeout(timeout)
        sock.sendto(packet, (ip, port))
        try:
            response, _ = sock.recvfrom(2048)
            print(f"[DCA] {name}: response {response.hex(' ')}")
        except socket.timeout:
            print(f"[DCA] {name}: no response; continuing")


def open_data_socket(listen_ip: str, port: int, rcvbuf_mb: int) -> socket.socket:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    requested = max(1, rcvbuf_mb) * 1024 * 1024
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, requested)
    except OSError as exc:
        print(f"[WARN] Could not set SO_RCVBUF={requested}: {exc}", file=sys.stderr)
    sock.bind((listen_ip, port))
    sock.settimeout(0.25)
    actual = sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
    print(f"[UDP] Listening on {listen_ip}:{port}, SO_RCVBUF={actual / 1024 / 1024:.1f} MB")
    return sock


def parse_dca_data_header(packet: bytes) -> Tuple[Optional[int], Optional[int], bytes]:
    if len(packet) <= DCA_PACKET_HEADER_BYTES:
        return None, None, b""
    seq = struct.unpack_from("<I", packet, 0)[0]
    byte_count = int.from_bytes(packet[4:10], byteorder="little", signed=False)
    return seq, byte_count, packet[DCA_PACKET_HEADER_BYTES:]


def capture(args: argparse.Namespace) -> int:
    dca_json = load_dca_json(args.cf_json)
    json_ip, json_cfg_port, json_data_port = ethernet_from_cf_json(dca_json)
    dca_ip = args.dca_ip or json_ip or DEFAULT_DCA_IP
    config_port = args.config_port or json_cfg_port or DEFAULT_CONFIG_PORT
    data_port = args.data_port or json_data_port or DEFAULT_DATA_PORT
    radar_cfg = parse_radar_cfg(args.cfg)
    if args.cfg and "lvds_stream_cfg" not in radar_cfg:
        print(
            "[WARN] The cfg metadata does not contain lvdsStreamCfg. "
            "DCA1000 raw ADC capture usually requires LVDS streaming to be enabled "
            "in the radar profile, for example: lvdsStreamCfg -1 0 1 0",
            file=sys.stderr,
        )

    if args.dry_run:
        plan = {
            "status": "dry_run_only",
            "hardware_commands_executed": False,
            "udp_socket_opened": False,
            "dca_ip": dca_ip,
            "config_port": config_port,
            "data_port": data_port,
            "listen_ip": args.listen_ip,
            "duration_s": args.duration,
            "bytes": args.bytes,
            "frames": args.frames,
            "radar_cfg": radar_cfg,
            "cfg": str(Path(args.cfg).resolve()) if args.cfg else None,
            "cf_json": str(Path(args.cf_json).resolve()) if args.cf_json else None,
        }
        print(json.dumps(plan, ensure_ascii=False, indent=2))
        return 0

    out_base = choose_output_dir(args.output_dir, args.min_free_gb)
    bin_path, meta_path = make_capture_paths(out_base, args.prefix)
    free_gb = shutil.disk_usage(bin_path.parent).free / 1024**3
    print(f"[OUT] {bin_path}")
    print(f"[OUT] Free space: {free_gb:.1f} GB")

    estimated_frame_bytes = int(radar_cfg.get("estimated_payload_bytes_per_frame", 0) or 0)
    target_bytes = args.bytes
    if args.frames and estimated_frame_bytes:
        target_bytes = args.frames * estimated_frame_bytes
        print(f"[INFO] Frame target: {args.frames} frames ≈ {target_bytes:,} payload bytes")
    elif args.frames and not estimated_frame_bytes:
        print("[WARN] --frames ignored because frame size could not be estimated from --cfg")

    if args.start_dca and not args.no_control:
        send_dca_command(dca_ip, config_port, DCA_CMD_RECORD_START, "record_start")

    sock = open_data_socket(args.listen_ip, data_port, args.socket_buffer_mb)

    start_time = time.time()
    last_data_time = start_time
    last_print_time = start_time
    total_udp_bytes = 0
    total_payload_bytes = 0
    packet_count = 0
    short_packets = 0
    dropped_packets = 0
    first_sequence: Optional[int] = None
    last_sequence: Optional[int] = None
    first_byte_count: Optional[int] = None
    last_byte_count: Optional[int] = None
    stop_reason = "unknown"

    try:
        with bin_path.open("wb", buffering=1024 * 1024) as out:
            while True:
                now = time.time()
                if args.duration and now - start_time >= args.duration:
                    stop_reason = "duration"
                    break
                if target_bytes and total_payload_bytes >= target_bytes:
                    stop_reason = "bytes"
                    break

                try:
                    packet, _addr = sock.recvfrom(65535)
                except socket.timeout:
                    if now - last_data_time > args.timeout:
                        print(f"[WARN] No UDP data for {args.timeout:.1f}s")
                        last_data_time = now
                    continue

                last_data_time = now
                packet_count += 1
                total_udp_bytes += len(packet)

                seq, byte_count, payload = parse_dca_data_header(packet)
                if seq is None:
                    short_packets += 1
                    continue

                if first_sequence is None:
                    first_sequence = seq
                    first_byte_count = byte_count
                if last_sequence is not None:
                    expected = (last_sequence + 1) & 0xFFFFFFFF
                    if seq != expected:
                        dropped_packets += (seq - expected) & 0xFFFFFFFF
                last_sequence = seq
                last_byte_count = byte_count

                if args.raw_packets:
                    out.write(packet)
                    total_payload_bytes += len(packet)
                else:
                    out.write(payload)
                    total_payload_bytes += len(payload)

                if now - last_print_time >= 1.0:
                    elapsed = max(0.001, now - start_time)
                    mb = total_payload_bytes / 1024 / 1024
                    rate = mb / elapsed
                    print(
                        f"[CAP] packets={packet_count:,} payload={mb:.1f} MB "
                        f"rate={rate:.1f} MB/s drops={dropped_packets}"
                    )
                    last_print_time = now
    except KeyboardInterrupt:
        stop_reason = "keyboard_interrupt"
        print("\n[INFO] Capture interrupted by user.")
    finally:
        sock.close()
        if args.stop_dca and not args.no_control:
            send_dca_command(dca_ip, config_port, DCA_CMD_RECORD_STOP, "record_stop")

    elapsed = max(0.001, time.time() - start_time)
    metadata = {
        "created_at": _dt.datetime.now().isoformat(timespec="seconds"),
        "bin_file": str(bin_path),
        "raw_packets": bool(args.raw_packets),
        "payload_header_stripped": not bool(args.raw_packets),
        "listen_ip": args.listen_ip,
        "data_port": data_port,
        "dca_ip": dca_ip,
        "config_port": config_port,
        "duration_s": elapsed,
        "stop_reason": stop_reason,
        "packet_count": packet_count,
        "short_packets": short_packets,
        "dropped_packets_estimate": dropped_packets,
        "first_sequence": first_sequence,
        "last_sequence": last_sequence,
        "first_byte_count": first_byte_count,
        "last_byte_count": last_byte_count,
        "total_udp_bytes": total_udp_bytes,
        "total_saved_bytes": total_payload_bytes,
        "average_saved_mb_s": total_payload_bytes / 1024 / 1024 / elapsed,
        "radar_cfg": radar_cfg,
        "dca_cf_json": str(args.cf_json) if args.cf_json else None,
    }
    meta_path.write_text(json.dumps(metadata, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"[DONE] Saved {total_payload_bytes / 1024 / 1024:.1f} MB")
    print(f"[DONE] Metadata: {meta_path}")
    return 0


def main() -> int:
    args = parse_args()
    if not args.duration and not args.bytes and not args.frames:
        print("[WARN] No stop condition set; capture will run until Ctrl+C.")
    return capture(args)


if __name__ == "__main__":
    raise SystemExit(main())
