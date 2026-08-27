"""Lightweight NMEA/Unicore sentence parser."""

def _latlon(value, direction):
    if not value:
        return None
    try:
        number = float(value)
        degrees = int(number // 100)
        minutes = number - degrees * 100
        result = degrees + minutes / 60.0
        if direction in ("S", "W"):
            result = -result
        return result
    except (TypeError, ValueError):
        return None


class NmeaParser:
    def parse(self, raw):
        if not raw:
            return None
        line = raw.strip().split("*", 1)[0]
        if line.startswith(("$GNGGA", "$GPGGA")):
            return self._parse_gga(line)
        if line.startswith(("$GNRMC", "$GPRMC")):
            return self._parse_rmc(line)
        if line.startswith("#UNIHEADINGA"):
            return self._parse_heading(line)
        return None

    @staticmethod
    def _parse_gga(line):
        fields = line.split(",")
        if len(fields) < 10:
            return None
        try:
            return {
                "type": "GGA",
                "gnss_time": fields[1],
                "latitude": _latlon(fields[2], fields[3]),
                "longitude": _latlon(fields[4], fields[5]),
                "quality": int(fields[6]),
                "satellites": int(fields[7] or 0),
                "hdop": float(fields[8]) if fields[8] else None,
                "altitude": float(fields[9]) if fields[9] else None,
            }
        except (TypeError, ValueError, IndexError):
            return None

    @staticmethod
    def _parse_rmc(line):
        fields = line.split(",")
        if len(fields) < 9:
            return None
        try:
            return {
                "type": "RMC",
                "status": fields[2],
                "latitude": _latlon(fields[3], fields[4]),
                "longitude": _latlon(fields[5], fields[6]),
                "speed": float(fields[7]) if fields[7] else None,
                "course": float(fields[8]) if fields[8] else None,
            }
        except (TypeError, ValueError, IndexError):
            return None

    @staticmethod
    def _parse_heading(line):
        if ";" not in line:
            return None
        fields = line.split(";", 1)[1].split(",")
        if len(fields) < 5:
            return None
        try:
            return {
                "type": "UNIHEADINGA",
                "sol_stat": fields[0],
                "pos_type": fields[1],
                "heading": float(fields[3]),
                "pitch": float(fields[4]),
            }
        except (TypeError, ValueError, IndexError):
            return None