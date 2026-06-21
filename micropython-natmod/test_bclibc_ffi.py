"""
test_bclibc_ffi.py — mirrors test_bclibc.py using ffi to call libtiny_bclibc.so.

Run with:
    micropython test_bclibc_ffi.py

Requires x64 host (8-byte pointers, real_t = double).
"""

import sys, struct, math, array
import ffi
import uctypes

if struct.calcsize('P') != 8:
    print("SKIP: FFI test requires x64 (8-byte pointers)")
    sys.exit(0)

_HERE = __file__.rsplit("/", 1)[0] if "/" in __file__ else "."
sys.path.append(_HERE)

from tiny_bclibc_types import Shot, Request, DRAG_G1, DRAG_G7, DRAG_CUSTOM

# ── Constants ─────────────────────────────────────────────────────────────────
TRAJ_FLAG_RANGE = 8
INTERP_POS_X    = 2   # TINY_BCLIBC_KEY_POS_X

# ── Built-in drag tables (double) — same values as bclibc_mp.c ───────────────
_G7_MACH = array.array('d', [
    0.00, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45,
    0.50, 0.55, 0.60, 0.65, 0.70, 0.725, 0.75, 0.775, 0.80, 0.825,
    0.85, 0.875, 0.90, 0.925, 0.95, 0.975, 1.0, 1.025, 1.05, 1.075,
    1.10, 1.125, 1.15, 1.20, 1.25, 1.30, 1.35, 1.40, 1.50, 1.55,
    1.60, 1.65, 1.70, 1.75, 1.80, 1.85, 1.90, 1.95, 2.00, 2.05,
    2.10, 2.15, 2.20, 2.25, 2.30, 2.35, 2.40, 2.45, 2.50, 2.55,
    2.60, 2.65, 2.70, 2.75, 2.80, 2.90, 3.00, 3.10, 3.20, 3.30,
    3.40, 3.50, 3.60, 3.70, 3.80, 3.90, 4.00, 4.20, 4.40, 4.60,
    4.80, 5.00,
])
_G7_CD = array.array('d', [
    0.1198, 0.1197, 0.1196, 0.1194, 0.1193, 0.1194, 0.1194, 0.1194, 0.1193, 0.1193,
    0.1194, 0.1193, 0.1194, 0.1197, 0.1202, 0.1207, 0.1215, 0.1226, 0.1242, 0.1266,
    0.1306, 0.1368, 0.1464, 0.1660, 0.2054, 0.2993, 0.3803, 0.4015, 0.4043, 0.4034,
    0.4014, 0.3987, 0.3955, 0.3884, 0.3810, 0.3732, 0.3657, 0.3580, 0.3440, 0.3376,
    0.3315, 0.3260, 0.3209, 0.3160, 0.3117, 0.3078, 0.3042, 0.3010, 0.2980, 0.2951,
    0.2922, 0.2892, 0.2864, 0.2835, 0.2807, 0.2779, 0.2752, 0.2725, 0.2697, 0.2670,
    0.2643, 0.2615, 0.2588, 0.2561, 0.2534, 0.2481, 0.2429, 0.2379, 0.2330, 0.2283,
    0.2238, 0.2194, 0.2151, 0.2110, 0.2070, 0.2032, 0.1995, 0.1924, 0.1858, 0.1794,
    0.1732, 0.1672,
])
_G1_MACH = array.array('d', [
    0.00, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45,
    0.50, 0.55, 0.60, 0.70, 0.725, 0.75, 0.775, 0.80, 0.825, 0.85,
    0.875, 0.90, 0.925, 0.95, 0.975, 1.0, 1.025, 1.05, 1.075, 1.10,
    1.125, 1.15, 1.20, 1.25, 1.30, 1.35, 1.40, 1.45, 1.50, 1.55,
    1.60, 1.65, 1.70, 1.75, 1.80, 1.85, 1.90, 1.95, 2.00, 2.05,
    2.10, 2.15, 2.20, 2.25, 2.30, 2.35, 2.40, 2.45, 2.50, 2.60,
    2.70, 2.80, 2.90, 3.00, 3.10, 3.20, 3.30, 3.40, 3.50, 3.60,
    3.70, 3.80, 3.90, 4.00, 4.20, 4.40, 4.60, 4.80, 5.00,
])
_G1_CD = array.array('d', [
    0.2629, 0.2558, 0.2487, 0.2413, 0.2344, 0.2278, 0.2214, 0.2155, 0.2104, 0.2061,
    0.2032, 0.2020, 0.2034, 0.2165, 0.2230, 0.2313, 0.2417, 0.2546, 0.2706, 0.2901,
    0.3136, 0.3415, 0.3734, 0.4084, 0.4448, 0.4805, 0.5136, 0.5427, 0.5677, 0.5883,
    0.6053, 0.6191, 0.6393, 0.6518, 0.6589, 0.6621, 0.6625, 0.6607, 0.6573, 0.6528,
    0.6474, 0.6413, 0.6347, 0.6280, 0.6210, 0.6141, 0.6072, 0.6003, 0.5934, 0.5867,
    0.5804, 0.5743, 0.5685, 0.5630, 0.5577, 0.5527, 0.5481, 0.5438, 0.5397, 0.5325,
    0.5264, 0.5211, 0.5168, 0.5133, 0.5105, 0.5084, 0.5067, 0.5054, 0.5040, 0.5030,
    0.5022, 0.5016, 0.5010, 0.5006, 0.4998, 0.4995, 0.4992, 0.4990, 0.4988,
])

# ── Open library ──────────────────────────────────────────────────────────────
_SO = _HERE + "/../tiny_bclibc/build-shared/libtiny_bclibc.so.1.1.0"
_lib = ffi.open(_SO)

_f_build = _lib.func('i', 'tiny_bclibc_build_shot_props', 'PPP')
_f_integ  = _lib.func('i', 'tiny_bclibc_integrate',       'PPPiPPP')
_f_at     = _lib.func('i', 'tiny_bclibc_integrate_at',    'PidPP')
_f_apex   = _lib.func('i', 'tiny_bclibc_find_apex',       'PP')
_f_zero   = _lib.func('i', 'tiny_bclibc_find_zero_angle', 'PdP')
_f_err    = _lib.func('s', 'tiny_bclibc_last_error',      '')

# ── C struct sizes (x64, real_t = double) ─────────────────────────────────────
# TINY_BCLIBC_Shot:         11d + 2×ptr + i + pad + ptr + i + pad + 6d + Config(4d+i+pad+2d)
_SHOT_C_SIZE  = 232
_PROPS_C_SIZE = 512   # actual 376, 512 for safety
_TRAJ_C_SIZE  = 128   # 15d + i + pad
_BASE_C_SIZE  = 64    # 8d
_REQ_C_SIZE   = 32    # 3d + i + pad
_WIND_C_SIZE  = 32    # 4d
_CURVE_C_SIZE = 32    # 4d (PCHIP a,b,c,d)


def _ptr(buf):
    return uctypes.addressof(buf)


def _build_props(shot_py):
    """Parse Shot.pack() buffer, build C structs, call tiny_bclibc_build_shot_props.

    Mirrors bclibc_mp.c build_props_buf() field-by-field.
    Returns (props_c, holder) where holder keeps all pointer targets alive.
    """
    buf = shot_py.pack()

    # 17 shot floats at offsets 0..67 (matches <17f in _SHOT_FMT)
    s = struct.unpack_from('<17f', buf, 0)
    bc, wt, dia, length, mv, sh, tw, tc, phpa, alt, hum, la, be, baz, cant, lat, az = s

    # 6 config floats + cMaxIterations int32 at offsets 68..95
    step_mult, zero_acc, min_vel, max_drop, gravity, min_alt, max_iter = \
        struct.unpack_from('<6fi', buf, 68)

    drag_type  = buf[96]
    wind_count = buf[97]
    drag_count = struct.unpack_from('<H', buf, 98)[0]

    # Build double-precision drag arrays
    if drag_type == DRAG_G1:
        mach_d, cd_d = _G1_MACH, _G1_CD
    elif drag_type == DRAG_CUSTOM:
        wn_ = min(wind_count, 16)
        drag_off = 100 + wn_ * 16
        n = min(drag_count, 128)
        mach_d = array.array('d')
        cd_d   = array.array('d')
        for i in range(n):
            m, c = struct.unpack_from('<ff', buf, drag_off + i * 8)
            mach_d.append(float(m))
            cd_d.append(float(c))
    else:  # G7
        mach_d, cd_d = _G7_MACH, _G7_CD
    n_drag = len(mach_d)

    # Build C wind array (TINY_BCLIBC_Wind = 4 doubles = 32 bytes each)
    wn = min(wind_count, 16)
    winds_c = bytearray(max(wn, 1) * _WIND_C_SIZE)
    for i in range(wn):
        wf = struct.unpack_from('<4f', buf, 100 + i * 16)
        struct.pack_into('<4d', winds_c, i * _WIND_C_SIZE,
                         float(wf[0]), float(wf[1]), float(wf[2]), float(wf[3]))

    # Build TINY_BCLIBC_Shot C struct (232 bytes, x64 layout)
    shot_c = bytearray(_SHOT_C_SIZE)
    # 11 doubles at offsets 0..80: bc, wt, dia, length, mv, sh, tw, tc, phpa, alt, hum
    struct.pack_into('<11d', shot_c, 0,
        float(bc), float(wt), float(dia), float(length), float(mv),
        float(sh), float(tw), float(tc), float(phpa), float(alt), float(hum))
    # mach_data ptr at 88, cd_data ptr at 96
    struct.pack_into('<Q', shot_c,  88, _ptr(mach_d))
    struct.pack_into('<Q', shot_c,  96, _ptr(cd_d))
    # drag_table_size int32 at 104 (4 bytes padding at 108)
    struct.pack_into('<i', shot_c, 104, n_drag)
    # winds ptr at 112, wind_count int32 at 120 (4 bytes padding at 124)
    struct.pack_into('<Q', shot_c, 112, _ptr(winds_c) if wn > 0 else 0)
    struct.pack_into('<i', shot_c, 120, wn)
    # 6 doubles at 128..168: la, be, baz, cant, lat, az
    struct.pack_into('<6d', shot_c, 128,
        float(la), float(be), float(baz), float(cant), float(lat), float(az))
    # Config at 176: cStepMult, cZeroAcc, cMinVel, cMaxDrop (4 doubles)
    struct.pack_into('<4d', shot_c, 176,
        float(step_mult), float(zero_acc), float(min_vel), float(max_drop))
    # cMaxIterations int32 at 208 (4 bytes padding at 212)
    struct.pack_into('<i',  shot_c, 208, int(max_iter))
    # cGravityConstant, cMinimumAltitude at 216..231
    struct.pack_into('<2d', shot_c, 216, float(gravity), float(min_alt))

    # Curve buffer: n_drag × TINY_BCLIBC_CurvePoint (each 4 doubles = 32 bytes)
    curve_c = bytearray(n_drag * _CURVE_C_SIZE)

    # Output ShotProps
    props_c = bytearray(_PROPS_C_SIZE)

    rc = _f_build(shot_c, curve_c, props_c)
    if rc != 0:
        raise ValueError("build_shot_props rc={}: {}".format(rc, _f_err()))

    # holder keeps all pointer targets alive alongside props_c
    return props_c, (shot_c, curve_c, mach_d, cd_d, winds_c)


def _pack_req(req_py):
    """Build TINY_BCLIBC_TrajectoryRequest C struct (32 bytes)."""
    req_c = bytearray(_REQ_C_SIZE)
    struct.pack_into('<3d', req_c, 0,
        float(req_py.range_limit_ft),
        float(req_py.range_step_ft),
        float(req_py.time_step))
    struct.pack_into('<i', req_c, 24, int(req_py.filter_flags))
    return req_c


def _parse_traj(buf, idx=0):
    """Parse TINY_BCLIBC_TrajectoryData at slot idx. Returns 16-tuple matching traj_to_tuple."""
    off = idx * _TRAJ_C_SIZE
    v = struct.unpack_from('<15d', buf, off)
    flag = struct.unpack_from('<i', buf, off + 120)[0]
    return (v[0], v[1], v[2], v[3], v[4], v[5], v[6],
            v[7], v[8], v[9], v[10], v[11], v[12], v[13], v[14], flag)


def _parse_base(buf):
    """Parse TINY_BCLIBC_BaseTrajData. Returns 8-tuple matching base_traj_to_tuple."""
    return struct.unpack_from('<8d', buf, 0)


# ── Public API (mirrors bclibc natmod API) ────────────────────────────────────

def integrate(shot_py, req_py):
    props_c, holder = _build_props(shot_py)
    req_c = _pack_req(req_py)
    cap = int(req_py.range_limit_ft / max(req_py.range_step_ft, 1.0)) + 64
    traj_buf = bytearray(cap * _TRAJ_C_SIZE)
    out_w = bytearray(4)
    out_t = bytearray(4)
    out_r = bytearray(4)

    rc = _f_integ(props_c, req_c, traj_buf, cap, out_w, out_t, out_r)
    if rc == 6:  # ERR_BUF_TOO_SMALL
        total = struct.unpack('<i', out_t)[0]
        traj_buf = bytearray(total * _TRAJ_C_SIZE)
        rc = _f_integ(props_c, req_c, traj_buf, total, out_w, out_t, out_r)
    if rc != 0:
        raise ValueError("integrate rc={}: {}".format(rc, _f_err()))

    written = struct.unpack('<i', out_w)[0]
    reason  = struct.unpack('<i', out_r)[0]
    return [_parse_traj(traj_buf, i) for i in range(written)], reason


def find_zero_angle(shot_py, dist_ft):
    props_c, holder = _build_props(shot_py)
    out = bytearray(8)
    rc = _f_zero(props_c, float(dist_ft), out)
    if rc != 0:
        raise ValueError("find_zero_angle rc={}: {}".format(rc, _f_err()))
    return struct.unpack('<d', out)[0]


def find_apex(shot_py):
    props_c, holder = _build_props(shot_py)
    out = bytearray(_TRAJ_C_SIZE)
    rc = _f_apex(props_c, out)
    if rc != 0:
        raise ValueError("find_apex rc={}: {}".format(rc, _f_err()))
    return _parse_traj(out)


def integrate_at(shot_py, key, target):
    props_c, holder = _build_props(shot_py)
    raw_c  = bytearray(_BASE_C_SIZE)
    full_c = bytearray(_TRAJ_C_SIZE)
    rc = _f_at(props_c, int(key), float(target), raw_c, full_c)
    if rc != 0:
        raise ValueError("integrate_at rc={}: {}".format(rc, _f_err()))
    return _parse_base(raw_c), _parse_traj(full_c)


# Inline helpers — not exported from SO, compute locally
def calculate_energy(weight_grain, velocity_fps):
    return weight_grain * velocity_fps * velocity_fps / 450400.0

def get_correction(distance_ft, offset_ft):
    if distance_ft == 0.0:
        return 0.0
    return math.atan2(offset_ft, distance_ft)

def calculate_ogw(weight_grain, velocity_fps):
    return weight_grain * weight_grain * velocity_fps * velocity_fps * velocity_fps * 1.5e-12


# ── Test fixtures (same as test_bclibc.py) ───────────────────────────────────
G7_MACH = array.array('f', [
    0.00, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45,
    0.50, 0.55, 0.60, 0.65, 0.70, 0.725, 0.75, 0.775, 0.80, 0.825,
    0.85, 0.875, 0.90, 0.925, 0.95, 0.975, 1.0, 1.025, 1.05, 1.075,
    1.10, 1.125, 1.15, 1.20, 1.25, 1.30, 1.35, 1.40, 1.50, 1.55,
    1.60, 1.65, 1.70, 1.75, 1.80, 1.85, 1.90, 1.95, 2.00, 2.05,
    2.10, 2.15, 2.20, 2.25, 2.30, 2.35, 2.40, 2.45, 2.50, 2.55,
    2.60, 2.65, 2.70, 2.75, 2.80, 2.90, 3.00, 3.10, 3.20, 3.30,
    3.40, 3.50, 3.60, 3.70, 3.80, 3.90, 4.00, 4.20, 4.40, 4.60,
    4.80, 5.00,
])
G7_CD = array.array('f', [
    0.1198, 0.1197, 0.1196, 0.1194, 0.1193, 0.1194, 0.1194, 0.1194, 0.1193, 0.1193,
    0.1194, 0.1193, 0.1194, 0.1197, 0.1202, 0.1207, 0.1215, 0.1226, 0.1242, 0.1266,
    0.1306, 0.1368, 0.1464, 0.1660, 0.2054, 0.2993, 0.3803, 0.4015, 0.4043, 0.4034,
    0.4014, 0.3987, 0.3955, 0.3884, 0.3810, 0.3732, 0.3657, 0.3580, 0.3440, 0.3376,
    0.3315, 0.3260, 0.3209, 0.3160, 0.3117, 0.3078, 0.3042, 0.3010, 0.2980, 0.2951,
    0.2922, 0.2892, 0.2864, 0.2835, 0.2807, 0.2779, 0.2752, 0.2725, 0.2697, 0.2670,
    0.2643, 0.2615, 0.2588, 0.2561, 0.2534, 0.2481, 0.2429, 0.2379, 0.2330, 0.2283,
    0.2238, 0.2194, 0.2151, 0.2110, 0.2070, 0.2032, 0.1995, 0.1924, 0.1858, 0.1794,
    0.1732, 0.1672,
])

SHOT = Shot(
    bc=0.310, weight_grain=168.0, diameter_inch=0.308, length_inch=1.2,
    muzzle_velocity_fps=2750.0, sight_height_ft=0.125, twist_inch=11.0,
    temp_c=15.0, pressure_hpa=1013.25, altitude_ft=0.0, humidity=0.5,
    drag_type=DRAG_G7,
)
SHOT_CUSTOM = Shot(
    bc=0.310, weight_grain=168.0, diameter_inch=0.308, length_inch=1.2,
    muzzle_velocity_fps=2750.0, sight_height_ft=0.125, twist_inch=11.0,
    drag_type=DRAG_CUSTOM, drag_mach=G7_MACH, drag_cd=G7_CD,
)
REQUEST = Request(
    range_limit_ft=1500.0, range_step_ft=300.0, filter_flags=TRAJ_FLAG_RANGE,
)
ZERO_DIST_FT      = 300.0 * 3.28084
ZERO_DIST_100M_FT = 100.0 * 3.28084


def _pass(name):
    print("  PASS  " + name)

def _fail(name, msg):
    print("  FAIL  " + name + " — " + str(msg))


# ── Tests ─────────────────────────────────────────────────────────────────────
print("=== bclibc ffi test ===")

print("\n--- scalar helpers (computed locally — inline in SO) ---")

e = calculate_energy(168.0, 2750.0)
if abs(e - 2820.83) < 1.0:
    _pass("calculate_energy")
else:
    _fail("calculate_energy", e)

c = get_correction(300.0, -2.0)
if abs(c) < 0.1:
    _pass("get_correction (at zero)")
else:
    _fail("get_correction", c)

ogw = calculate_ogw(168.0, 2750.0)
if 800 < ogw < 1000:
    _pass("calculate_ogw")
else:
    _fail("calculate_ogw", ogw)

print("\n--- integrate (builtin G7, 1500 ft, step 300) ---")
try:
    rows, reason = integrate(SHOT, REQUEST)
    if len(rows) >= 2:
        _pass("integrate — {} rows, stop reason {}".format(len(rows), reason))
    else:
        _fail("integrate", "expected >=2 rows, got " + str(len(rows)))
    print("  {:>8s}  {:>8s}  {:>8s}  {:>8s}".format("dist_ft", "vel_fps", "height_ft", "mach"))
    for r in rows:
        print("  {:>8.0f}  {:>8.1f}  {:>8.3f}  {:>8.3f}".format(r[1], r[2], r[4], r[3]))
except Exception as ex:
    _fail("integrate", ex)

print("\n--- integrate (custom array G7, 1500 ft, step 300) ---")
try:
    rows2, reason2 = integrate(SHOT_CUSTOM, REQUEST)
    if len(rows2) >= 2:
        _pass("integrate custom — {} rows, stop reason {}".format(len(rows2), reason2))
    else:
        _fail("integrate custom", "expected >=2 rows, got " + str(len(rows2)))
except Exception as ex:
    _fail("integrate custom", ex)

print("\n--- find_zero_angle (300 m zero) ---")
elev = None
try:
    elev = find_zero_angle(SHOT, ZERO_DIST_FT)
    _pass("find_zero_angle elev_rad={:.6f}  ({:.4f} deg)".format(
        elev, math.degrees(elev)))
except Exception as ex:
    _fail("find_zero_angle", ex)

print("\n--- find_zero_angle (100 m zero) ---")
try:
    elev_100m = find_zero_angle(SHOT, ZERO_DIST_100M_FT)
    _pass("find_zero_angle 100m  elev_rad={:.6f}  ({:.4f} deg)".format(
        elev_100m, math.degrees(elev_100m)))
    if elev_100m <= 0:
        _fail("find_zero_angle 100m", "elevation must be positive, got {}".format(elev_100m))
except Exception as ex:
    _fail("find_zero_angle 100m", ex)

print("\n--- find_apex (zeroed shot) ---")
try:
    _elev = elev if elev is not None else 0.002442
    zeroed = Shot(
        bc=SHOT.bc, weight_grain=SHOT.weight_grain,
        diameter_inch=SHOT.diameter_inch, length_inch=SHOT.length_inch,
        muzzle_velocity_fps=SHOT.muzzle_velocity_fps,
        sight_height_ft=SHOT.sight_height_ft, twist_inch=SHOT.twist_inch,
        barrel_elevation_rad=_elev, drag_type=DRAG_G7,
    )
    apex = find_apex(zeroed)
    _pass("find_apex dist_ft={:.1f}  height_ft={:.1f}".format(apex[1], apex[4]))
except Exception as ex:
    _fail("find_apex", ex)

print("\n--- integrate_at (POS_X = 1000 ft) ---")
try:
    raw, full = integrate_at(SHOT, INTERP_POS_X, 1000.0)
    _pass("integrate_at dist_ft={:.1f}  vel_fps={:.1f}".format(full[1], full[2]))
except Exception as ex:
    _fail("integrate_at", ex)

print("\n--- RAM: integrate 3 km / 100 m step ---")
try:
    import gc
    REQ_3KM = Request(
        range_limit_ft=3000.0 * 3.28084,
        range_step_ft=100.0 * 3.28084,
        filter_flags=TRAJ_FLAG_RANGE,
    )
    gc.collect()
    mem_before = gc.mem_alloc()
    rows_3km, _ = integrate(SHOT, REQ_3KM)
    mem_after = gc.mem_alloc()
    gc.collect()
    mem_after_gc = gc.mem_alloc()
    _pass("integrate 3 km — {} rows  alloc={} B  alloc_after_gc={} B".format(
        len(rows_3km), mem_after - mem_before, mem_after_gc - mem_before))
    print("  mem_before={} B  mem_peak={} B  mem_after_gc={} B".format(
        mem_before, mem_after, mem_after_gc))
except Exception as ex:
    _fail("RAM integrate 3 km", ex)

print("\n=== done ===")
