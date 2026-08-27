"""Pi-side runtime configuration and persistent serial-port selection."""

import configparser
from pathlib import Path


MODES = ("SINGLE", "DGPS", "RTK")
MESSAGE_COMMANDS = {
    "GGA": "GPGGA",
    "RMC": "GPRMC",
    "GSA": "GPGSA",
    "GSV": "GPGSV",
}


class PiRtkConfig:
    def __init__(self):
        self.work_mode = "RTK"
        self.config_path = ""

        self.serial_port = ""
        self.serial_baudrate = 115200
        self.serial_timeout = 1.0
        self.auto_on_start = True
        self.save_detected_port = True
        self.candidate_patterns = [
            "/dev/ttyUSB*",
            "/dev/ttyACM*",
            "/dev/ttyAMA*",
        ]
        self.exclude_ports = ["/dev/ttyACM0", "/dev/ttyACM1"]
        self.probe_timeout_s = 1.2
        self.detect_timeout_s = 10.0
        self.inject_serial_port = ""

        self.ntrip_enabled = True
        self.ntrip_host = ""
        self.ntrip_port = 2101
        self.ntrip_mountpoint = ""
        self.ntrip_username = ""
        self.ntrip_password = ""
        self.gga_fallback = (
            "$GNGGA,120000.00,3000.0000,N,12000.0000,E,"
            "1,8,1.0,10.0,M,0.0,M,,*49"
        )

        self.output_directory = "output"
        self.raw_log = True

        self.rover_profile = ""
        self.nmea_version = "V410"
        self.message_rates = {"GGA": 1, "RMC": 1, "GSA": 1, "GSV": 1}
        self.dgps_timeout_s = 300
        self.rtk_timeout_s = 600
        self.elevation_mask_deg = 15.0
        self.save_config = False

        self.fix_timeout_s = 25.0
        self.min_fixed_epochs = 8
        self.status_interval_s = 3.0
        self.duration_s = None

    @property
    def mode(self):
        return self.work_mode

    def inject_port(self):
        return self.inject_serial_port or self.serial_port

    def receiver_commands(self):
        """Build the UM982 initialization sequence for the selected mode."""
        profile = self.rover_profile.strip().upper()
        if profile:
            mode_cmd = "MODE ROVER {}".format(profile)
        else:
            mode_cmd = "MODE ROVER"

        commands = [
            mode_cmd,
            "CONFIG NMEA0183 {}".format(self.nmea_version),
        ]

        commands.extend(
            "{} {}".format(MESSAGE_COMMANDS[name], rate)
            for name, rate in self.message_rates.items()
        )

        # UM982 uses TIMEOUT 0 to disable an engine.
        dgps_timeout = 0 if self.work_mode == "SINGLE" else self.dgps_timeout_s
        rtk_timeout = self.rtk_timeout_s if self.work_mode == "RTK" else 0
        commands.append("CONFIG DGPS TIMEOUT {:d}".format(int(dgps_timeout)))
        commands.append("CONFIG RTK TIMEOUT {:d}".format(int(rtk_timeout)))
        commands.append("MASK {:.0f}".format(self.elevation_mask_deg))
        if self.save_config:
            commands.append("SAVECONFIG")
        return commands


def _get(config, section, option, default=None):
    if config.has_section(section) and config.has_option(section, option):
        return config.get(section, option).strip()
    return default


def _get_int(config, section, option, default):
    value = _get(config, section, option)
    if value is None:
        return default
    try:
        return int(value)
    except ValueError:
        return default


def _get_float(config, section, option, default):
    value = _get(config, section, option)
    if value is None:
        return default
    try:
        return float(value)
    except ValueError:
        return default


def _get_bool(config, section, option, default):
    value = _get(config, section, option)
    if value is None:
        return default
    return value.lower() in ("1", "true", "yes", "on")


def _get_list(config, section, option, default):
    value = _get(config, section, option)
    if value is None:
        return list(default)
    items = []
    for item in value.replace("\n", ",").split(","):
        item = item.strip()
        if item:
            items.append(item)
    return items


def resolve_output_dir(cfg):
    directory = Path(cfg.output_directory)
    if not directory.is_absolute():
        directory = Path(cfg.config_path).resolve().parent / directory
    return directory


def load_config(path):
    path = Path(path)
    parser = configparser.ConfigParser()
    if not parser.read(path, encoding="utf-8"):
        raise FileNotFoundError("configuration file not found: {}".format(path))

    cfg = PiRtkConfig()
    cfg.config_path = str(path)
    cfg.work_mode = _get(parser, "MODE", "type", cfg.work_mode).upper()
    if cfg.work_mode not in MODES:
        raise ValueError("MODE.type must be one of: {}".format(", ".join(MODES)))

    cfg.serial_port = _get(parser, "SERIAL", "port", cfg.serial_port)
    cfg.serial_baudrate = _get_int(
        parser, "SERIAL", "baudrate", cfg.serial_baudrate
    )
    cfg.serial_timeout = _get_float(
        parser, "SERIAL", "timeout", cfg.serial_timeout
    )
    cfg.auto_on_start = _get_bool(
        parser, "SERIAL", "auto_on_start", cfg.auto_on_start
    )
    cfg.save_detected_port = _get_bool(
        parser, "SERIAL", "save_detected_port", cfg.save_detected_port
    )
    cfg.candidate_patterns = _get_list(
        parser,
        "SERIAL",
        "candidate_patterns",
        cfg.candidate_patterns,
    )
    cfg.exclude_ports = _get_list(
        parser, "SERIAL", "exclude_ports", cfg.exclude_ports
    )
    cfg.probe_timeout_s = _get_float(
        parser, "SERIAL", "probe_timeout", cfg.probe_timeout_s
    )
    cfg.detect_timeout_s = _get_float(
        parser, "SERIAL", "detect_timeout", cfg.detect_timeout_s
    )
    cfg.inject_serial_port = _get(parser, "SERIAL", "injection_port", "")

    cfg.ntrip_enabled = _get_bool(
        parser, "NTRIP", "enabled", cfg.ntrip_enabled
    )
    cfg.ntrip_host = _get(parser, "NTRIP", "host", cfg.ntrip_host)
    cfg.ntrip_port = _get_int(parser, "NTRIP", "port", cfg.ntrip_port)
    cfg.ntrip_mountpoint = _get(
        parser, "NTRIP", "mountpoint", cfg.ntrip_mountpoint
    )
    cfg.ntrip_username = _get(parser, "NTRIP", "username", cfg.ntrip_username)
    cfg.ntrip_password = _get(parser, "NTRIP", "password", cfg.ntrip_password)
    cfg.gga_fallback = _get(parser, "NTRIP", "gga_fallback", cfg.gga_fallback)

    cfg.output_directory = _get(
        parser, "OUTPUT", "directory", cfg.output_directory
    )
    cfg.raw_log = _get_bool(parser, "OUTPUT", "raw_log", cfg.raw_log)

    cfg.rover_profile = _get(
        parser, "RECEIVER", "rover_profile", cfg.rover_profile
    )
    cfg.nmea_version = _get(
        parser, "RECEIVER", "nmea_version", cfg.nmea_version
    )
    for message, option in (
        ("GGA", "gga_rate"),
        ("RMC", "rmc_rate"),
        ("GSA", "gsa_rate"),
        ("GSV", "gsv_rate"),
    ):
        cfg.message_rates[message] = _get_int(
            parser, "RECEIVER", option, cfg.message_rates[message]
        )
    cfg.dgps_timeout_s = _get_int(
        parser, "RECEIVER", "dgps_timeout", cfg.dgps_timeout_s
    )
    cfg.rtk_timeout_s = _get_int(
        parser, "RECEIVER", "rtk_timeout", cfg.rtk_timeout_s
    )
    cfg.elevation_mask_deg = _get_float(
        parser, "RECEIVER", "elevation_mask", cfg.elevation_mask_deg
    )
    cfg.save_config = _get_bool(
        parser, "RECEIVER", "save_config", cfg.save_config
    )

    cfg.fix_timeout_s = _get_float(
        parser, "RTK", "fix_timeout", cfg.fix_timeout_s
    )
    cfg.min_fixed_epochs = _get_int(
        parser, "RTK", "min_fixed_epochs", cfg.min_fixed_epochs
    )
    cfg.status_interval_s = _get_float(
        parser, "SERVICE", "status_interval", cfg.status_interval_s
    )
    duration = _get_float(parser, "SERVICE", "duration", 0.0)
    cfg.duration_s = duration if duration > 0 else None

    raw_commands = _get(parser, "RECEIVER", "startup_commands", "")
    if raw_commands:
        cfg.startup_override = [
            line.strip() for line in raw_commands.splitlines() if line.strip()
        ]
        # Keep the public interface simple: override receiver_commands below.
        cfg.receiver_commands = lambda: list(cfg.startup_override)  # noqa: B010

    return cfg


def save_serial_port(path, port):
    """Persist only SERIAL.port, preserving comments and every other setting."""
    path = Path(path)
    lines = path.read_text(encoding="utf-8").splitlines()
    output = []
    in_serial = False
    replaced = False

    for line in lines:
        stripped = line.strip()
        if stripped == "[SERIAL]":
            in_serial = True
            output.append(line)
            continue
        if stripped.startswith("[") and stripped.endswith("]"):
            if in_serial and not replaced:
                output.append("port = {}".format(port))
                replaced = True
            in_serial = False
            output.append(line)
            continue

        option_name = stripped.split("=", 1)[0].strip().lower()
        if in_serial and not replaced and option_name == "port":
            output.append("port = {}".format(port))
            replaced = True
        else:
            output.append(line)

    if in_serial and not replaced:
        output.append("port = {}".format(port))
        replaced = True

    if not replaced:
        raise ValueError("[SERIAL] section missing")
    path.write_text("\n".join(output) + "\n", encoding="utf-8")
    return True
