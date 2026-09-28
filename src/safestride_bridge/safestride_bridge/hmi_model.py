"""ROS-independent display snapshot codec; little endian Pi/Terrain wire words."""
import math
import struct

PACKET_TYPE = 0x30
STATUS_PACKET_TYPE = 0x31
CAPABILITY = 1 << 12  # v3 adds packed ASCII intersection location
STATUS_FORMAT = struct.Struct('<BBHII')
BASE_WORDS = 16
LOCATION_WORDS = 10
LOCATION_BYTES = LOCATION_WORDS * 2
FORMAT = struct.Struct('<%dH' % (BASE_WORDS + LOCATION_WORDS))
UNKNOWN = 0xffff
SPEED, HANDS, CROSS, PITCH, TOF, WALKER = 1, 2, 4, 8, 16, 32
ARMED, DEADMAN, ESTOP, WATCHDOG, ENTRY_ALLOWED, URGENT, SIGNAL_VALID = (
    1, 2, 4, 8, 16, 32, 64)
SURFACE_VALID = 1 << 7
SURFACE_SHIFT = 8
SURFACE_CONFIDENCE_SHIFT = 11
SURFACE_CONFIDENCE_MAX = 31

# Revised Romanization without pronunciation assimilation. Common road-place
# suffixes are replaced first so the 20-byte LCD label stays recognizable.
_HANGUL_INITIAL = (
    'g', 'kk', 'n', 'd', 'tt', 'r', 'm', 'b', 'pp', 's', 'ss', '',
    'j', 'jj', 'ch', 'k', 't', 'p', 'h',
)
_HANGUL_MEDIAL = (
    'a', 'ae', 'ya', 'yae', 'eo', 'e', 'yeo', 'ye', 'o', 'wa',
    'wae', 'oe', 'yo', 'u', 'wo', 'we', 'wi', 'yu', 'eu', 'ui', 'i',
)
_HANGUL_FINAL = (
    '', 'k', 'k', 'ks', 'n', 'nj', 'nh', 't', 'l', 'lk', 'lm', 'lb',
    'ls', 'lt', 'lp', 'lh', 'm', 'p', 'ps', 't', 't', 'ng', 't', 't',
    'k', 't', 'p', 'h',
)
_LOCATION_TERMS = (
    ('주민센터', ' COMMUNITY CTR '),
    ('고등학교', ' HIGH SCHOOL '),
    ('중학교', ' MIDDLE SCHOOL '),
    ('초등학교', ' ELEM SCHOOL '),
    ('대학교', ' UNIV '),
    ('사거리', ' JCT '),
    ('삼거리', ' JCT '),
    ('네거리', ' JCT '),
    ('교차로', ' JCT '),
    ('우체국', ' POST OFFICE '),
    ('아파트', ' APT '),
    ('로터리', ' ROTARY '),
    ('입구', ' ENTRANCE '),
    ('병원', ' HOSP '),
    ('은행', ' BANK '),
    ('학교', ' SCHOOL '),
    ('초교', ' ELEM SCHOOL '),
    ('공원', ' PARK '),
    ('시장', ' MARKET '),
    ('구청', ' DISTRICT OFFICE '),
    ('역', ' STN '),
    ('앞', ' FRONT '),
)


def ascii_location(value):
    text = str(value or '').strip()
    for source, replacement in _LOCATION_TERMS:
        text = text.replace(source, replacement)
    output = []
    for character in text:
        code = ord(character)
        if 0xAC00 <= code <= 0xD7A3:
            offset = code - 0xAC00
            initial = offset // 588
            medial = (offset % 588) // 28
            final = offset % 28
            output.append(
                _HANGUL_INITIAL[initial]
                + _HANGUL_MEDIAL[medial]
                + _HANGUL_FINAL[final]
            )
        elif character.isascii() and (character.isalnum() or character in ' -'):
            output.append(character)
        else:
            output.append(' ')
    normalized = ' '.join(''.join(output).upper().split())
    return normalized[:LOCATION_BYTES].rstrip()


def encode_location_words(value):
    encoded = ascii_location(value).encode('ascii')
    encoded = encoded.ljust(LOCATION_BYTES, b'\0')
    return [
        (encoded[index] << 8) | encoded[index + 1]
        for index in range(0, LOCATION_BYTES, 2)
    ]


def finite(value):
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def scaled(value, factor=1):
    if not finite(value) or float(value) < 0:
        return UNKNOWN
    return min(65534, int(float(value) * factor + .5))


class Snapshot:
    def __init__(self, pitch_sign=1.0, pitch_offset=0.0):
        if pitch_sign not in (-1.0, 1.0) or not finite(pitch_offset):
            raise ValueError('Invalid pitch calibration')
        self.sign, self.offset = pitch_sign, pitch_offset
        self.samples = {}
        self.heartbeat = 0
        self.hazard = False

    def update(self, name, message, now):
        self.samples[name] = (message, now)

    def fresh(self, name, now, ttl):
        m, received = self.samples.get(name, (None, float('-inf')))
        if m is None or not 0 <= now - received < ttl:
            return None
        age = getattr(m, 'telemetry_age', 0.0)
        if not finite(age) or not 0 <= age + now - received < ttl:
            return None
        return m

    def words(self, now):
        self.heartbeat = (self.heartbeat + 1) & 0xffff
        w = [3,self.heartbeat,0,UNKNOWN,0,0,UNKNOWN,UNKNOWN,0,5,0,0,0,0,1,0]
        w.extend([0] * LOCATION_WORDS)
        status = self.fresh('walker', now, .6)
        if status is not None and status.link_ok:
            if 0 <= status.state <= 5:
                w[2] |= WALKER
                w[11], w[12], w[13] = status.state, int(status.braking), status.fault_bits & 0xffff
                w[15] = (int(status.armed) * ARMED | int(status.deadman) * DEADMAN |
                         int(status.estop) * ESTOP | int(status.watchdog_timeout) * WATCHDOG)
            if status.speed_valid and finite(status.measured_speed_kmh):
                w[2] |= SPEED
                w[3] = scaled(abs(status.measured_speed_kmh), 100)
        pressure = self.fresh('pressure', now, .6)
        # Pressure messages have no source-age field. Also require fresh Drive telemetry.
        if pressure is not None and pressure.calibrated and status is not None and status.link_ok:
            w[2] |= HANDS
            w[4] = int(pressure.left_present) | (int(pressure.right_present) << 1)
        cross = self.fresh('crosswalk', now, 1.5)
        # IDLE is deliberately N/A. Do NOT equate !entry_allowed with a red lamp.
        if cross is not None and cross.gps_valid and 0 <= cross.state <= 6:
            w[2] |= CROSS
            w[5] = cross.state
            w[15] |= (int(cross.signal_valid) * SIGNAL_VALID |
                      int(cross.urgent) * URGENT)
            if cross.signal_valid and cross.entry_allowed and cross.state == 3:
                w[15] |= ENTRY_ALLOWED
            if cross.state != 0:
                w[7] = scaled(cross.edge_distance_m, 10)
                w[BASE_WORDS:] = encode_location_words(
                    getattr(cross, 'intersection_name', '')
                )
                if cross.signal_valid:
                    w[6] = scaled(cross.signal_remaining_s)
        terrain = self.fresh('terrain', now, .6)
        if terrain is not None:
            if terrain.mpu_valid and finite(terrain.pitch_rad):
                deg = math.degrees((terrain.pitch_rad-self.offset)*self.sign)
                if abs(deg) <= 180:
                    w[2] |= PITCH
                    w[8] = int(round(deg*10)) & 0xffff
            if terrain.tof_valid and 0 <= terrain.tof_alert <= 4:
                w[2] |= TOF
                w[9] = terrain.tof_alert
                if terrain.terrain_hazard or terrain.tof_alert in (3,4):
                    self.hazard = True
                elif terrain.tof_alert == 0:
                    self.hazard = False
            elif terrain.terrain_hazard:
                self.hazard = True
        surface = self.fresh('surface', now, 2.5)
        if (surface is not None and surface.valid and
                1 <= surface.classification <= 6 and
                finite(surface.confidence) and
                0.0 <= surface.confidence <= 1.0):
            confidence = int(round(surface.confidence * SURFACE_CONFIDENCE_MAX))
            w[15] |= (SURFACE_VALID |
                      (int(surface.classification) << SURFACE_SHIFT) |
                      (confidence << SURFACE_CONFIDENCE_SHIFT))
        w[10] = int(self.hazard)
        return w

    def pack(self, now):
        return FORMAT.pack(*self.words(now))
