"""ROS-independent display snapshot codec; little endian Pi/Terrain wire words."""
import math
import struct

PACKET_TYPE = 0x30
CAPABILITY = 1 << 10
FORMAT = struct.Struct('<16H')
UNKNOWN = 0xffff
SPEED, HANDS, CROSS, PITCH, TOF, WALKER = 1, 2, 4, 8, 16, 32


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
        w = [1,self.heartbeat,0,UNKNOWN,0,0,UNKNOWN,UNKNOWN,0,5,0,0,0,0,1,0]
        status = self.fresh('walker', now, .6)
        if status is not None and status.link_ok:
            if 0 <= status.state <= 5:
                w[2] |= WALKER
                w[11], w[12], w[13] = status.state, int(status.braking), status.fault_bits & 0xffff
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
            if cross.state != 0:
                w[7] = scaled(cross.edge_distance_m, 10)
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
        w[10] = int(self.hazard)
        return w

    def pack(self, now):
        return FORMAT.pack(*self.words(now))
