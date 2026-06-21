"""tiny_bclibc_types — data classes for tiny_bclibc MicroPython natmod.

Usage:
    from tiny_bclibc_types import Shot, Request, Wind, Config, DRAG_G1, DRAG_G7, DRAG_CUSTOM
    import tiny_bclibc

    shot = Shot(bc=0.310, weight_grain=168.0, muzzle_velocity_fps=2750.0, ...)
    req  = Request(range_limit_ft=3000.0, range_step_ft=100.0)
    rows, reason = tiny_bclibc.integrate(shot.pack(), req.pack())
"""
import struct

_NaN = float('nan')
_INF = 1e8  # TINY_BCLIBC_MAX_WIND_DIST_FT

DRAG_G1     = 0
DRAG_G7     = 1
DRAG_CUSTOM = 2

# Shot header format: 17 scalars + 6 config floats + cMaxIterations + drag/wind counts
# Total: 17*4 + 6*4 + 4 + 1 + 1 + 2 = 100 bytes (little-endian, no padding)
_SHOT_FMT  = '<17f6fiBBH'
_SHOT_SIZE = struct.calcsize(_SHOT_FMT)  # 100

_WIND_FMT  = '<4f'         # 16 bytes
_WIND_SIZE = struct.calcsize(_WIND_FMT)

_DRAG_FMT  = '<ff'         # 8 bytes per drag point
_DRAG_SIZE = struct.calcsize(_DRAG_FMT)

_REQ_FMT   = '<3fi'        # 16 bytes


class Wind:
    def __init__(self, velocity_fps=0.0, direction_from_rad=0.0,
                 until_distance_ft=_INF, max_distance_ft=_INF):
        self.velocity_fps       = velocity_fps
        self.direction_from_rad = direction_from_rad
        self.until_distance_ft  = until_distance_ft
        self.max_distance_ft    = max_distance_ft

    def pack(self):
        return struct.pack(_WIND_FMT,
                           float(self.velocity_fps), float(self.direction_from_rad),
                           float(self.until_distance_ft), float(self.max_distance_ft))

    @staticmethod
    def unpack(buf, offset=0):
        v = struct.unpack_from(_WIND_FMT, buf, offset)
        return Wind(v[0], v[1], v[2], v[3])

    def __repr__(self):
        return 'Wind({:.1f} fps dir={:.3f} rad)'.format(
            self.velocity_fps, self.direction_from_rad)


class Config:
    def __init__(self, step_multiplier=0.5, zero_finding_accuracy=0.001,
                 minimum_velocity=50.0, maximum_drop=-15000.0,
                 max_iterations=50, gravity_constant=-32.17405,
                 minimum_altitude=-1500.0):
        self.step_multiplier       = step_multiplier
        self.zero_finding_accuracy = zero_finding_accuracy
        self.minimum_velocity      = minimum_velocity
        self.maximum_drop          = maximum_drop
        self.max_iterations        = max_iterations
        self.gravity_constant      = gravity_constant
        self.minimum_altitude      = minimum_altitude


class Shot:
    """Ballistic shot descriptor.  Call pack() to get the buffer for bclibc functions."""

    def __init__(self,
                 bc=0.0, weight_grain=0.0, diameter_inch=0.0,
                 length_inch=0.0, muzzle_velocity_fps=0.0,
                 sight_height_ft=0.0, twist_inch=0.0,
                 temp_c=15.0, pressure_hpa=1013.25,
                 altitude_ft=0.0, humidity=0.5,
                 look_angle_rad=0.0, barrel_elevation_rad=0.0,
                 barrel_azimuth_rad=0.0, cant_angle_rad=0.0,
                 latitude_deg=_NaN, azimuth_deg=_NaN,
                 drag_type=DRAG_G7, drag_mach=None, drag_cd=None,
                 winds=None, config=None):
        self.bc                   = bc
        self.weight_grain         = weight_grain
        self.diameter_inch        = diameter_inch
        self.length_inch          = length_inch
        self.muzzle_velocity_fps  = muzzle_velocity_fps
        self.sight_height_ft      = sight_height_ft
        self.twist_inch           = twist_inch
        self.temp_c               = temp_c
        self.pressure_hpa         = pressure_hpa
        self.altitude_ft          = altitude_ft
        self.humidity             = humidity
        self.look_angle_rad       = look_angle_rad
        self.barrel_elevation_rad = barrel_elevation_rad
        self.barrel_azimuth_rad   = barrel_azimuth_rad
        self.cant_angle_rad       = cant_angle_rad
        self.latitude_deg         = latitude_deg
        self.azimuth_deg          = azimuth_deg
        self.drag_type            = drag_type
        self.drag_mach            = drag_mach  # sequence of float or None for built-in
        self.drag_cd              = drag_cd    # sequence of float or None for built-in
        self.winds                = winds if winds is not None else []
        self.config               = config if config is not None else Config()

    def pack(self):
        cfg = self.config
        wc  = min(len(self.winds), 16)
        dc  = 0
        if self.drag_type == DRAG_CUSTOM and self.drag_mach and self.drag_cd:
            dc = min(len(self.drag_mach), len(self.drag_cd), 128)

        buf = bytearray(_SHOT_SIZE + wc * _WIND_SIZE + dc * _DRAG_SIZE)
        struct.pack_into(_SHOT_FMT, buf, 0,
            # 17 shot scalars
            float(self.bc), float(self.weight_grain), float(self.diameter_inch),
            float(self.length_inch), float(self.muzzle_velocity_fps),
            float(self.sight_height_ft), float(self.twist_inch),
            float(self.temp_c), float(self.pressure_hpa), float(self.altitude_ft),
            float(self.humidity),
            float(self.look_angle_rad), float(self.barrel_elevation_rad),
            float(self.barrel_azimuth_rad), float(self.cant_angle_rad),
            float(self.latitude_deg), float(self.azimuth_deg),
            # 6 config floats
            float(cfg.step_multiplier), float(cfg.zero_finding_accuracy),
            float(cfg.minimum_velocity), float(cfg.maximum_drop),
            float(cfg.gravity_constant), float(cfg.minimum_altitude),
            # cMaxIterations int32
            int(cfg.max_iterations),
            # drag_type uint8, wind_count uint8, drag_count uint16
            int(self.drag_type), wc, dc,
        )
        off = _SHOT_SIZE
        for i in range(wc):
            w = self.winds[i]
            struct.pack_into(_WIND_FMT, buf, off,
                             float(w.velocity_fps), float(w.direction_from_rad),
                             float(w.until_distance_ft), float(w.max_distance_ft))
            off += _WIND_SIZE
        for i in range(dc):
            struct.pack_into(_DRAG_FMT, buf, off,
                             float(self.drag_mach[i]), float(self.drag_cd[i]))
            off += _DRAG_SIZE
        return bytes(buf)

    @staticmethod
    def unpack(buf):
        v   = struct.unpack_from(_SHOT_FMT, buf, 0)
        wc  = v[25]
        dc  = v[26]
        off = _SHOT_SIZE
        winds = []
        for _ in range(wc):
            winds.append(Wind.unpack(buf, off))
            off += _WIND_SIZE
        drag_mach = drag_cd = None
        if dc:
            import array
            drag_mach = array.array('f')
            drag_cd   = array.array('f')
            for _ in range(dc):
                m, c = struct.unpack_from(_DRAG_FMT, buf, off)
                drag_mach.append(m)
                drag_cd.append(c)
                off += _DRAG_SIZE
        return Shot(
            bc=v[0], weight_grain=v[1], diameter_inch=v[2],
            length_inch=v[3], muzzle_velocity_fps=v[4],
            sight_height_ft=v[5], twist_inch=v[6],
            temp_c=v[7], pressure_hpa=v[8], altitude_ft=v[9], humidity=v[10],
            look_angle_rad=v[11], barrel_elevation_rad=v[12],
            barrel_azimuth_rad=v[13], cant_angle_rad=v[14],
            latitude_deg=v[15], azimuth_deg=v[16],
            drag_type=v[24], drag_mach=drag_mach, drag_cd=drag_cd,
            winds=winds,
            config=Config(
                step_multiplier=v[17], zero_finding_accuracy=v[18],
                minimum_velocity=v[19], maximum_drop=v[20],
                gravity_constant=v[21], minimum_altitude=v[22],
                max_iterations=v[23],
            ),
        )

    def __repr__(self):
        drag = {DRAG_G1: 'G1', DRAG_G7: 'G7', DRAG_CUSTOM: 'custom'}.get(self.drag_type, '?')
        return 'Shot(bc={} mv={} fps drag={})'.format(
            self.bc, self.muzzle_velocity_fps, drag)


class Request:
    """Trajectory integration request parameters."""

    def __init__(self, range_limit_ft=3000.0, range_step_ft=100.0,
                 time_step=0.0, filter_flags=8):  # 8 = TRAJ_FLAG_RANGE
        self.range_limit_ft = range_limit_ft
        self.range_step_ft  = range_step_ft
        self.time_step      = time_step
        self.filter_flags   = filter_flags

    def pack(self):
        return struct.pack(_REQ_FMT,
                           float(self.range_limit_ft), float(self.range_step_ft),
                           float(self.time_step), int(self.filter_flags))

    @staticmethod
    def unpack(buf):
        v = struct.unpack_from(_REQ_FMT, buf, 0)
        return Request(v[0], v[1], v[2], v[3])

    def __repr__(self):
        return 'Request(range={} ft step={} ft)'.format(
            self.range_limit_ft, self.range_step_ft)
