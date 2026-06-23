"""tiny_bclibc_types — data classes for tiny_bclibc MicroPython natmod.

Usage:
    from tiny_bclibc_types import Shot, Request, Wind, Config, DRAG_G1, DRAG_G7, DRAG_CUSTOM
    import tiny_bclibc

    shot = Shot(bc=0.310, weight_grain=168.0, muzzle_velocity_fps=2750.0, ...)
    req  = Request(range_limit_ft=3000.0, range_step_ft=100.0)
    rows, reason = tiny_bclibc.integrate(shot.pack(), req.pack())

    # Streaming (no buffer allocation):
    total, reason = tiny_bclibc.integrate_stream(shot.pack(), req.pack(),
        lambda row: None)   # return truthy to stop early
"""

import uctypes
from micropython import const
from collections import namedtuple as _namedtuple

_NaN = float("nan")
_INF = 1e8  # TINY_BCLIBC_MAX_WIND_DIST_FT

DRAG_G1 = const(0)
DRAG_G7 = const(1)
DRAG_CUSTOM = const(2)

_TRAJ_FLAG_RANGE = const(8)  # mirrors TINY_BCLIBC_TRAJ_FLAG_RANGE

_MAX_WINDS = const(16)
_MAX_DRAG_PTS = const(128)

# Buffer sizes: little-endian layout, standard sizes, no padding (<-prefix guarantees this)
# Shot header: 17 scalars + 6 config floats + cMaxIterations + drag/wind counts
# 17*4 + 6*4 + 4 + 1 + 1 + 2 = 100 bytes
_SHOT_SIZE = const(100)
_WIND_SIZE = const(16)   # 4f
_DRAG_SIZE = const(8)    # ff

# Pre-compiled uctypes descriptors — avoid struct format-string re-parsing on each pack() call.
# Offsets are byte positions in the <-prefixed (little-endian, no-padding) binary layout.
# Nested sub-structs let pack() resolve s.props / s.cfg once as a local, then write fields.
_F32 = uctypes.FLOAT32
_I32 = uctypes.INT32
_U8  = uctypes.UINT8
_U16 = uctypes.UINT16

_REQ_DESC = {
    "range_limit_ft": _F32 | 0,
    "range_step_ft":  _F32 | 4,
    "time_step":      _F32 | 8,
    "filter_flags":   _I32 | 12,
}
_REQ_SIZE = const(16)

_SHOT_PROPS_DESC = {
    "bc":                   _F32 | 0,
    "weight_grain":         _F32 | 4,
    "diameter_inch":        _F32 | 8,
    "length_inch":          _F32 | 12,
    "muzzle_velocity_fps":  _F32 | 16,
    "sight_height_ft":      _F32 | 20,
    "twist_inch":           _F32 | 24,
    "temp_c":               _F32 | 28,
    "pressure_hpa":         _F32 | 32,
    "altitude_ft":          _F32 | 36,
    "humidity":             _F32 | 40,
    "look_angle_rad":       _F32 | 44,
    "barrel_elevation_rad": _F32 | 48,
    "barrel_azimuth_rad":   _F32 | 52,
    "cant_angle_rad":       _F32 | 56,
    "latitude_deg":         _F32 | 60,
    "azimuth_deg":          _F32 | 64,
}

_CFG_DESC = {
    "step_multiplier":       _F32 | 0,
    "zero_finding_accuracy": _F32 | 4,
    "minimum_velocity":      _F32 | 8,
    "maximum_drop":          _F32 | 12,
    "gravity_constant":      _F32 | 16,
    "minimum_altitude":      _F32 | 20,
}

_SHOT_DESC = {
    "props":          (0,  _SHOT_PROPS_DESC),
    "cfg":            (68, _CFG_DESC),
    "max_iterations": _I32 | 92,
    "drag_type":      _U8  | 96,
    "wind_count":     _U8  | 97,
    "drag_count":     _U16 | 98,
}

_WIND_DESC = {
    "velocity_fps":       _F32 | 0,
    "direction_from_rad": _F32 | 4,
    "until_distance_ft":  _F32 | 8,
    "max_distance_ft":    _F32 | 12,
}

_DRAG_DESC = {
    "mach": _F32 | 0,
    "cd":   _F32 | 4,
}


# Pure namedtuples — read-only data holders, no methods needed.
# Factory functions restore keyword-argument defaults that plain namedtuple lacks.
_Wind = _namedtuple("Wind", ("velocity_fps", "direction_from_rad", "until_distance_ft", "max_distance_ft"))
def Wind(velocity_fps=0.0, direction_from_rad=0.0, until_distance_ft=_INF, max_distance_ft=_INF):
    return _Wind(velocity_fps, direction_from_rad, until_distance_ft, max_distance_ft)

_Config = _namedtuple("Config", ("step_multiplier", "zero_finding_accuracy", "minimum_velocity",
                                  "maximum_drop", "max_iterations", "gravity_constant", "minimum_altitude"))
def Config(step_multiplier=0.5, zero_finding_accuracy=0.001, minimum_velocity=50.0,
           maximum_drop=-15000.0, max_iterations=50, gravity_constant=-32.17405,
           minimum_altitude=-1500.0):
    return _Config(step_multiplier, zero_finding_accuracy, minimum_velocity,
                   maximum_drop, max_iterations, gravity_constant, minimum_altitude)


# Shot is a namedtuple subclass: fields stored once as tuple elements (no self.x = x copy),
# pack() method kept on the subclass. For 22 fields the subclass is lighter than a plain
# class: measured 427 B vs 587 B per instance.
_ShotNT = _namedtuple("Shot", (
    "bc", "weight_grain", "diameter_inch", "length_inch",
    "muzzle_velocity_fps", "sight_height_ft", "twist_inch",
    "temp_c", "pressure_hpa", "altitude_ft", "humidity",
    "look_angle_rad", "barrel_elevation_rad", "barrel_azimuth_rad", "cant_angle_rad",
    "latitude_deg", "azimuth_deg",
    "drag_type", "drag_mach", "drag_cd", "winds", "config",
))

class Shot(_ShotNT):
    """Ballistic shot descriptor.  Call pack() to get the buffer for bclibc functions."""

    def __new__(
        cls,
        bc=0.0,
        weight_grain=0.0,
        diameter_inch=0.0,
        length_inch=0.0,
        muzzle_velocity_fps=0.0,
        sight_height_ft=0.0,
        twist_inch=0.0,
        temp_c=15.0,
        pressure_hpa=1013.25,
        altitude_ft=0.0,
        humidity=0.5,
        look_angle_rad=0.0,
        barrel_elevation_rad=0.0,
        barrel_azimuth_rad=0.0,
        cant_angle_rad=0.0,
        latitude_deg=_NaN,
        azimuth_deg=_NaN,
        drag_type=DRAG_G7,
        drag_mach=None,
        drag_cd=None,
        winds=None,
        config=None,
    ):
        return _ShotNT.__new__(
            cls,
            bc, weight_grain, diameter_inch, length_inch,
            muzzle_velocity_fps, sight_height_ft, twist_inch,
            temp_c, pressure_hpa, altitude_ft, humidity,
            look_angle_rad, barrel_elevation_rad, barrel_azimuth_rad, cant_angle_rad,
            latitude_deg, azimuth_deg,
            drag_type, drag_mach, drag_cd,
            winds if winds is not None else [],
            config if config is not None else Config(),
        )

    def pack(self):
        cfg = self.config
        wc = min(len(self.winds), _MAX_WINDS)
        dc = 0
        if self.drag_type == DRAG_CUSTOM and self.drag_mach and self.drag_cd:
            dc = min(len(self.drag_mach), len(self.drag_cd), _MAX_DRAG_PTS)

        buf = bytearray(_SHOT_SIZE + wc * _WIND_SIZE + dc * _DRAG_SIZE)
        base = uctypes.addressof(buf)
        s = uctypes.struct(base, _SHOT_DESC, uctypes.LITTLE_ENDIAN)
        p = s.props
        p.bc                   = self.bc
        p.weight_grain         = self.weight_grain
        p.diameter_inch        = self.diameter_inch
        p.length_inch          = self.length_inch
        p.muzzle_velocity_fps  = self.muzzle_velocity_fps
        p.sight_height_ft      = self.sight_height_ft
        p.twist_inch           = self.twist_inch
        p.temp_c               = self.temp_c
        p.pressure_hpa         = self.pressure_hpa
        p.altitude_ft          = self.altitude_ft
        p.humidity             = self.humidity
        p.look_angle_rad       = self.look_angle_rad
        p.barrel_elevation_rad = self.barrel_elevation_rad
        p.barrel_azimuth_rad   = self.barrel_azimuth_rad
        p.cant_angle_rad       = self.cant_angle_rad
        p.latitude_deg         = self.latitude_deg
        p.azimuth_deg          = self.azimuth_deg
        c = s.cfg
        c.step_multiplier       = cfg.step_multiplier
        c.zero_finding_accuracy = cfg.zero_finding_accuracy
        c.minimum_velocity      = cfg.minimum_velocity
        c.maximum_drop          = cfg.maximum_drop
        c.gravity_constant      = cfg.gravity_constant
        c.minimum_altitude      = cfg.minimum_altitude
        s.max_iterations        = int(cfg.max_iterations)
        s.drag_type             = self.drag_type
        s.wind_count            = wc
        s.drag_count            = dc
        off = _SHOT_SIZE
        for i in range(wc):
            w = self.winds[i]
            sw = uctypes.struct(base + off, _WIND_DESC, uctypes.LITTLE_ENDIAN)
            sw.velocity_fps = w.velocity_fps
            sw.direction_from_rad = w.direction_from_rad
            sw.until_distance_ft = w.until_distance_ft
            sw.max_distance_ft = w.max_distance_ft
            off += _WIND_SIZE
        for i in range(dc):
            sd = uctypes.struct(base + off, _DRAG_DESC, uctypes.LITTLE_ENDIAN)
            sd.mach = self.drag_mach[i]
            sd.cd = self.drag_cd[i]
            off += _DRAG_SIZE
        return bytes(buf)

    def __repr__(self):
        drag = {DRAG_G1: "G1", DRAG_G7: "G7", DRAG_CUSTOM: "custom"}.get(
            self.drag_type, "?"
        )
        return "Shot(bc={} mv={} fps drag={})".format(
            self.bc, self.muzzle_velocity_fps, drag
        )


class Request:
    """Trajectory integration request parameters."""

    def __init__(
        self,
        range_limit_ft=3000.0,
        range_step_ft=100.0,
        time_step=0.0,
        filter_flags=_TRAJ_FLAG_RANGE,
    ):
        self.range_limit_ft = range_limit_ft
        self.range_step_ft = range_step_ft
        self.time_step = time_step
        self.filter_flags = filter_flags

    def pack(self):
        buf = bytearray(_REQ_SIZE)
        s = uctypes.struct(uctypes.addressof(buf), _REQ_DESC, uctypes.LITTLE_ENDIAN)
        s.range_limit_ft = self.range_limit_ft
        s.range_step_ft  = self.range_step_ft
        s.time_step      = self.time_step
        s.filter_flags   = self.filter_flags
        return bytes(buf)

    def __repr__(self):
        return "Request(range={} ft step={} ft)".format(
            self.range_limit_ft, self.range_step_ft
        )
