#!/usr/bin/env python3
"""The bare WebAssembly module against the native library, through the same flat C ABI (bclibc_ffi.h).

    python tests/wasm_parity/parity.py --native build/libbclibc_ffi.so --wasm build/wasm/bclibc_wasm.wasm \\
        --backend wasmtime --backend node

Covers the integrate / zero / max-range / apex entry points, the adaptive integrators with per-call tolerances and step
statistics, dense output and its helpers, and the solver building blocks (error_at_distance, range_for_angle, Newton zero,
zero_init).

The same shots go through `libbclibc_ffi` (ctypes) and through the module (wasmhost: https://pypi.org/project/wasmhost,
`pip install --pre wasmhost wasmtime`). Every double of every result is compared bit for bit:

- everything computed with `+ - * / sqrt` must be identical (IEEE arithmetic is exact everywhere);
- the angle fields (`drop_angle_rad`, `windage_angle_rad`, `angle_rad`) come from `atan`/`atan2`, which differ between the
  native libm (glibc) and the module's (musl): up to --max-ulp (default 4) is allowed, 1 ulp is what is seen;
- a shot whose solve fails (a zero beyond the range) must return the same `BCLIBCFFI_ERR_*` status as the native
  library, on both toolchains (bclibc never throws, so neither wasi-sdk's nor zig's build ever traps on this).

Exit status: 0 if all of it holds, 1 otherwise. Layouts of the structs are computed here for both pointer sizes
(8 native, 4 wasm32) from one field list, and the native side is cross-checked against the C compiler's.
"""
import argparse
import ctypes
import math
import struct
import sys

import wasmhost

NATIVE: ctypes.CDLL
WASM: bytes

# ---- struct specs: list of (name, kind); kind: 'd' double, 'i' int32, 'p' pointer, ('a',n,kind) array, or a spec name
SPECS = {
 "Error": [("code","i"),("message",("a",512,"c")),("f0","d"),("f1","d"),("f2","d"),("i0","i")],
 "Config": [("cStepMultiplier","d"),("cZeroFindingAccuracy","d"),("cMinimumVelocity","d"),("cMaximumDrop","d"),("cMaxIterations","i"),("cGravityConstant","d"),("cMinimumAltitude","d")],
 "Wind": [("velocity_fps","d"),("direction_from_rad","d"),("until_distance_ft","d"),("max_distance_ft","d")],
 "Shot": [("bc","d"),("weight_grain","d"),("diameter_inch","d"),("length_inch","d"),("muzzle_velocity_fps","d"),
          ("sight_height_ft","d"),("twist_inch","d"),("temp_c","d"),("pressure_hpa","d"),("altitude_ft","d"),("humidity","d"),
          ("mach_data","p"),("cd_data","p"),("drag_table_size","i"),("winds","p"),("wind_count","i"),
          ("look_angle_rad","d"),("barrel_elevation_rad","d"),("barrel_azimuth_rad","d"),("cant_angle_rad","d"),
          ("latitude_deg","d"),("azimuth_deg","d"),("config","Config"),("method","i")],
 "Request": [("range_limit_ft","d"),("range_step_ft","d"),("time_step","d"),("filter_flags","i")],
 "TrajectoryData": [(n,"d") for n in "time distance_ft velocity_fps mach height_ft slant_height_ft drop_angle_rad windage_ft windage_angle_rad slant_distance_ft angle_rad density_ratio drag energy_ft_lb ogw_lb".split()] + [("flag","i")],
 "BaseTrajData": [(n,"d") for n in "time px py pz vx vy vz mach".split()],
 "MaxRange": [("max_range_ft","d"),("angle_at_max_rad","d")],
}
SPECS["Interception"] = [("raw","BaseTrajData"),("full","TrajectoryData")]
SPECS["ZeroPoint"] = [("angle_rad","d"),("point","TrajectoryData")]
SPECS["StepStats"] = [("accepted","i"),("rejected","i")]
SPECS["BaseTrajBuf"] = [("data","p"),("count","i"),("capacity","i")]
SPECS["ZeroInit"] = [("status","i"),("reserved","i"),("look_angle_rad","d"),("slant_range_ft","d"),("target_x_ft","d"),("target_y_ft","d"),("start_height_ft","d")]

def layout(name, P):
    """(size, align, [(field, offset, kind)]) for pointer size P."""
    off = 0; maxa = 1; out = []
    for f, k in SPECS[name]:
        if isinstance(k, tuple):
            n, kk = k[1], k[2]; size, al = n*{"c":1,"d":8,"i":4}[kk], {"c":1,"d":8,"i":4}[kk]
        elif k in SPECS:
            size, al, _ = layout(k, P)
        else:
            size = al = {"d":8,"i":4,"p":P}[k]
        off = (off + al - 1)//al*al
        out.append((f, off, k)); off += size; maxa = max(maxa, al)
    return (off + maxa - 1)//maxa*maxa, maxa, out

def ctype_struct(name):
    fields = []
    for f, k in SPECS[name]:
        if isinstance(k, tuple): t = ctypes.c_char * k[1]
        elif k in SPECS: t = ctype_struct(k)
        else: t = {"d":ctypes.c_double,"i":ctypes.c_int32,"p":ctypes.c_void_p}[k]
        fields.append((f, t))
    return type(name, (ctypes.Structure,), {"_fields_": fields})

CT = {n: ctype_struct(n) for n in SPECS}
for n in SPECS:  # the layout engine must agree with the C compiler's for the native pointer size
    assert layout(n, 8)[0] == ctypes.sizeof(CT[n]), (n, layout(n, 8)[0], ctypes.sizeof(CT[n]))


def doubles_of(buf, name, P, base=0):
    """every double in a struct image, as a list of (path, value_bits)"""
    size, _, fields = layout(name, P)
    out = []
    for f, off, k in fields:
        if k == "d": out.append((f, struct.unpack_from("<Q", buf, base+off)[0]))
        elif k == "i": out.append((f, struct.unpack_from("<i", buf, base+off)[0]))
        elif k in SPECS: out += [(f+"."+g, v) for g, v in doubles_of(buf, k, P, base+off)]
    return out

def pack(name, values, P):
    size, _, fields = layout(name, P)
    b = bytearray(size)
    for f, off, k in fields:
        v = values.get(f)
        if v is None: continue
        if k == "d": struct.pack_into("<d", b, off, v)
        elif k == "i": struct.pack_into("<i", b, off, v)
        elif k == "p": (struct.pack_into("<Q", b, off, v) if P == 8 else struct.pack_into("<I", b, off, v))
        elif k in SPECS: b[off:off+layout(k,P)[0]] = pack(k, v, P)
    return bytes(b)

# ---- wasm side
class Wasm:
    def __init__(self, backend):
        m = wasmhost.Module(WASM, backend=backend); stubs = {}
        for d in wasmhost.Module.imports(m):
            if d.kind == "function": stubs.setdefault(d.module, {})[d.name] = (lambda n: (lambda *a: (0,) * n if n > 1 else 0))(len(d.type.results))
        self.i = wasmhost.Instance(m, stubs or None); self.i.exports._initialize()
        self.e = self.i.exports; self.mem = self.e.memory
    def alloc(self, data):
        p = self.e.malloc(max(len(data), 8)); self.mem.write(p, data); return p
    def free(self, p): self.e.free(p)

MACH = [0.0,0.5,0.7,0.8,0.9,0.95,1.0,1.05,1.1,1.2,1.4,1.6,2.0,2.5,3.0,4.0]
CD   = [0.2629,0.2558,0.2310,0.2340,0.2790,0.3260,0.4805,0.5080,0.5190,0.5150,0.4820,0.4430,0.3600,0.3100,0.2740,0.2260]

def shot_values(method, **kw):
    v = dict(bc=0.4, weight_grain=168.0, diameter_inch=0.308, length_inch=1.24, muzzle_velocity_fps=2700.0,
             sight_height_ft=0.15, twist_inch=11.25, temp_c=15.0, pressure_hpa=1013.25, altitude_ft=0.0, humidity=0.5,
             drag_table_size=len(MACH), wind_count=1, look_angle_rad=0.0, barrel_elevation_rad=0.0, barrel_azimuth_rad=0.0,
             cant_angle_rad=0.0, latitude_deg=45.0, azimuth_deg=90.0,
             config=dict(cStepMultiplier=1.0,cZeroFindingAccuracy=0.000005,cMinimumVelocity=50.0,cMaximumDrop=-15000.0,cMaxIterations=60,cGravityConstant=-32.17405,cMinimumAltitude=-1500.0),
             method=method)
    v.update(kw); return v
WIND = dict(velocity_fps=14.7, direction_from_rad=math.pi/2, until_distance_ft=1e8, max_distance_ft=1e8)

def call_native(fn, shot_v, extra):
    md = (ctypes.c_double*len(MACH))(*MACH); cd = (ctypes.c_double*len(CD))(*CD); w = CT["Wind"](**WIND)
    v = dict(shot_v); v["mach_data"] = ctypes.addressof(md); v["cd_data"] = ctypes.addressof(cd); v["winds"] = ctypes.addressof(w)
    shot = CT["Shot"].from_buffer_copy(pack("Shot", v, 8))
    err = CT["Error"]()
    res = extra["native"](getattr(NATIVE, fn), shot, err)
    return res, bytes(err)

def call_wasm(wasm, fn, shot_v, extra):
    P = 4; keep = []
    md = wasm.alloc(struct.pack(f"<{len(MACH)}d", *MACH)); cd = wasm.alloc(struct.pack(f"<{len(CD)}d", *CD)); w = wasm.alloc(pack("Wind", WIND, P))
    v = dict(shot_v); v["mach_data"] = md; v["cd_data"] = cd; v["winds"] = w
    shot = wasm.alloc(pack("Shot", v, P)); err = wasm.alloc(bytes(layout("Error", P)[0]))
    res = extra["wasm"](wasm, fn, shot, err)
    e = bytes(wasm.mem.read(err, layout("Error", P)[0]))
    for p in (md, cd, w, shot, err): wasm.free(p)
    return res, e

# ---- the functions: an adapter for each, native and wasm
D, I, PT = ctypes.c_double, ctypes.c_int32, ctypes.c_void_p


def bind_native():
    def sig(name, *args):
        f = getattr(NATIVE, name); f.restype = I; f.argtypes = list(args)
    sig("BCLIBCFFI_find_apex_shot", PT, PT, PT); sig("BCLIBCFFI_find_max_range_shot", PT, D, D, PT, PT)
    sig("BCLIBCFFI_find_zero_angle_shot", PT, D, PT, PT); sig("BCLIBCFFI_find_zero_point_shot", PT, D, PT, PT)
    sig("BCLIBCFFI_integrate_shot", PT, PT, PT, PT, PT, PT); sig("BCLIBCFFI_integrate_at_shot", PT, I, D, PT, PT)
    NATIVE.BCLIBCFFI_free_trajectory.argtypes = [PT]; NATIVE.BCLIBCFFI_free_trajectory.restype = None
    NATIVE.BCLIBCFFI_free_base_trajectory.argtypes = [PT]; NATIVE.BCLIBCFFI_free_base_trajectory.restype = None

def structs(kind, image, P):
    return doubles_of(image, kind, P)

def out_struct_native(struct_name, call):
    def run(fn, shot, err):
        out = CT[struct_name](); r = call(fn, shot, out, err); return r, structs(struct_name, bytes(out), 8)
    return run

def scenario_apex():
    n = lambda fn, shot, err: (lambda out: (fn(ctypes.byref(shot), ctypes.byref(out), ctypes.byref(err)), structs("TrajectoryData", bytes(out), 8)))(CT["TrajectoryData"]())
    def w(wasm, name, shot, err):
        size = layout("TrajectoryData", 4)[0]; out = wasm.alloc(bytes(size)); r = getattr(wasm.e, name)(shot, out, err)
        img = bytes(wasm.mem.read(out, size)); wasm.free(out); return r, structs("TrajectoryData", img, 4)
    return dict(native=n, wasm=w)

def scenario_maxrange():
    n = lambda fn, shot, err: (lambda out: (fn(ctypes.byref(shot), 0.0, 90.0, ctypes.byref(out), ctypes.byref(err)), structs("MaxRange", bytes(out), 8)))(CT["MaxRange"]())
    def w(wasm, name, shot, err):
        size = layout("MaxRange", 4)[0]; out = wasm.alloc(bytes(size)); r = getattr(wasm.e, name)(shot, 0.0, 90.0, out, err)
        img = bytes(wasm.mem.read(out, size)); wasm.free(out); return r, structs("MaxRange", img, 4)
    return dict(native=n, wasm=w)

def scenario_zero_angle(dist):
    def n(fn, shot, err):
        out = D(); r = fn(ctypes.byref(shot), dist, ctypes.byref(out), ctypes.byref(err)); return r, [("angle", struct.unpack("<Q", struct.pack("<d", out.value))[0])]
    def w(wasm, name, shot, err):
        out = wasm.alloc(bytes(8)); r = getattr(wasm.e, name)(shot, dist, out, err); v = struct.unpack("<Q", bytes(wasm.mem.read(out, 8)))[0]; wasm.free(out); return r, [("angle", v)]
    return dict(native=n, wasm=w)

def scenario_zero_point(dist):
    def n(fn, shot, err):
        out = CT["ZeroPoint"](); r = fn(ctypes.byref(shot), dist, ctypes.byref(out), ctypes.byref(err)); return r, structs("ZeroPoint", bytes(out), 8)
    def w(wasm, name, shot, err):
        size = layout("ZeroPoint", 4)[0]; out = wasm.alloc(bytes(size)); r = getattr(wasm.e, name)(shot, dist, out, err)
        img = bytes(wasm.mem.read(out, size)); wasm.free(out); return r, structs("ZeroPoint", img, 4)
    return dict(native=n, wasm=w)

REQ = dict(range_limit_ft=3000.0, range_step_ft=300.0, time_step=0.0, filter_flags=31)
def scenario_integrate(req=REQ):
    def n(fn, shot, err):
        r = CT["Request"].from_buffer_copy(pack("Request", req, 8)); recs = PT(); cnt = I(); reason = I()
        rc = fn(ctypes.byref(shot), ctypes.byref(r), ctypes.byref(recs), ctypes.byref(cnt), ctypes.byref(reason), ctypes.byref(err))
        out = [("count", cnt.value), ("reason", reason.value)]
        if rc == 0:
            sz = ctypes.sizeof(CT["TrajectoryData"]); blob = ctypes.string_at(recs, sz*cnt.value)
            for i in range(cnt.value): out += [(f"[{i}].{p}", v) for p, v in structs("TrajectoryData", blob, 8)[0:0] or doubles_of(blob, "TrajectoryData", 8, i*sz)]
            NATIVE.BCLIBCFFI_free_trajectory(recs)
        return rc, out
    def w(wasm, name, shot, err):
        P = 4; r = wasm.alloc(pack("Request", req, P)); recs = wasm.alloc(bytes(4)); cnt = wasm.alloc(bytes(4)); reason = wasm.alloc(bytes(4))
        rc = getattr(wasm.e, name)(shot, r, recs, cnt, reason, err)
        c = struct.unpack("<i", bytes(wasm.mem.read(cnt, 4)))[0]; out = [("count", c), ("reason", struct.unpack("<i", bytes(wasm.mem.read(reason, 4)))[0])]
        if rc == 0:
            ptr = struct.unpack("<I", bytes(wasm.mem.read(recs, 4)))[0]; sz = layout("TrajectoryData", P)[0]; blob = bytes(wasm.mem.read(ptr, sz*c))
            for i in range(c): out += [(f"[{i}].{p}", v) for p, v in doubles_of(blob, "TrajectoryData", P, i*sz)]
            wasm.e.BCLIBCFFI_free_trajectory(ptr)
        for p in (r, recs, cnt, reason): wasm.free(p)
        return rc, out
    return dict(native=n, wasm=w)

def scenario_integrate_at(key, target):
    def n(fn, shot, err):
        out = CT["Interception"](); r = fn(ctypes.byref(shot), key, target, ctypes.byref(out), ctypes.byref(err)); return r, structs("Interception", bytes(out), 8)
    def w(wasm, name, shot, err):
        size = layout("Interception", 4)[0]; out = wasm.alloc(bytes(size)); r = getattr(wasm.e, name)(shot, key, target, out, err)
        img = bytes(wasm.mem.read(out, size)); wasm.free(out); return r, structs("Interception", img, 4)
    return dict(native=n, wasm=w)

METHODS = {"RK4":0,"EULER":1,"VERLET":2,"CASH_KARP":3,"DOPRI":4,"TSITOURAS":5}
def ulp(a, b):
    if a == b: return 0
    fa, fb = struct.unpack("<d", struct.pack("<Q", a & (2**64-1)))[0], struct.unpack("<d", struct.pack("<Q", b & (2**64-1)))[0]
    if math.isnan(fa) and math.isnan(fb): return 0
    ia = a if a < 2**63 else -(a - 2**63); ib = b if b < 2**63 else -(b - 2**63)
    return abs(ia - ib)

def bits_to_double(b):
    return struct.unpack("<d", struct.pack("<Q", b & (2**64 - 1)))[0]

def compare(label, nat, wa, max_ulp, rel_tol=None):
    """(ok, one line): the status must agree, integers must be equal, doubles identical except the angles by max_ulp.

    rel_tol (a case's own, see below) relaxes the doubles to a relative tolerance, for the one kind of result that
    amplifies the 1 ulp difference between the two libms (glibc, musl): a near-vertical lofted zero, where the miss changes
    2.5 ft per 1e-6 rad over a 50 s flight. It stays inside the solver's own angle tolerance (1e-7 rad)."""
    (rn, on), (rw, ow) = nat, wa
    if rn != rw:
        return False, f"{label:44} STATUS native={rn} wasm={rw}"
    if [p for p, _ in on] != [p for p, _ in ow]:
        return False, f"{label:44} FIELDS differ ({len(on)} vs {len(ow)})"
    ints = ("count", "reason", "flag", "code")
    bad, worst_angle, worst_rel = [], 0, 0.0
    for (p, a), (_, b) in zip(on, ow):
        if a == b:
            continue
        if rel_tol is not None and not p.endswith(ints):
            fa, fb = bits_to_double(a), bits_to_double(b)
            rel = abs(fa - fb) / max(abs(fa), abs(fb), 1e-300); worst_rel = max(worst_rel, rel)
            if not rel <= rel_tol:
                bad.append((p, f"relative difference {rel:.2e} > {rel_tol:.0e}"))
            continue
        if p.endswith(ints):
            bad.append((p, "integer"))
        elif p.endswith("angle_rad"):
            u = ulp(a, b); worst_angle = max(worst_angle, u)
            if u > max_ulp:
                bad.append((p, f"{u} ulp"))
        else:
            bad.append((p, f"{ulp(a, b)} ulp, must be identical"))
    if bad:
        return False, f"{label:44} {len(bad)} field(s) off, first: {bad[0][0]} ({bad[0][1]})"
    if worst_rel:
        return True, f"{label:44} ok ({len(on)} fields, within {worst_rel:.1e} relative)"
    return True, f"{label:44} ok ({len(on)} fields" + (f", angles within {worst_angle} ulp)" if worst_angle else ", identical)")


CASES = []
for mname, m in METHODS.items():
    sv = shot_values(m); sv2 = dict(sv)
    CASES += [(f"{mname:9} integrate 0..1000yd", "BCLIBCFFI_integrate_shot", sv, scenario_integrate()),
              (f"{mname:9} zero 300yd", "BCLIBCFFI_find_zero_angle_shot", sv, scenario_zero_angle(900.0)),
              (f"{mname:9} zero_point 300yd", "BCLIBCFFI_find_zero_point_shot", sv, scenario_zero_point(900.0))]
sv = shot_values(0, barrel_elevation_rad=math.radians(0.3))
CASES += [("RK4 apex", "BCLIBCFFI_find_apex_shot", shot_values(0, barrel_elevation_rad=math.radians(30)), scenario_apex()),
          ("RK4 max_range", "BCLIBCFFI_find_max_range_shot", sv, scenario_maxrange()),
          ("RK4 integrate_at time=0.5", "BCLIBCFFI_integrate_at_shot", sv, scenario_integrate_at(0, 0.5)),
          ("RK4 integrate_at pos_x=1500", "BCLIBCFFI_integrate_at_shot", sv, scenario_integrate_at(2, 1500.0)),
          ("RK4 altitude 5000ft, 5C, 850hPa", "BCLIBCFFI_integrate_shot", shot_values(0, altitude_ft=5000.0, temp_c=5.0, pressure_hpa=850.0, humidity=0.2, barrel_elevation_rad=0.004), scenario_integrate()),
          ("RK4 vacuum (pressure 0)", "BCLIBCFFI_integrate_shot", shot_values(0, pressure_hpa=0.0, barrel_elevation_rad=0.01), scenario_integrate()),
          ("RK4 no Coriolis (NaN lat)", "BCLIBCFFI_integrate_shot", shot_values(0, latitude_deg=float("nan"), azimuth_deg=float("nan"), barrel_elevation_rad=0.004), scenario_integrate()),
          ("RK4 cant 5deg, look 3deg", "BCLIBCFFI_integrate_shot", shot_values(0, cant_angle_rad=math.radians(5), look_angle_rad=math.radians(3), barrel_elevation_rad=0.004), scenario_integrate()),
          ("RK4 zero out of range 20000yd (error path)", "BCLIBCFFI_find_zero_angle_shot", shot_values(0), scenario_zero_angle(60000.0))]



# ---- the calls added for adaptive integrators, solver building blocks and dense output

def scenario_call(*args, shot=True, rel_tol=None):
    """Any call of the form f(shot, <args>, err) (f(<args>, err) with shot=False, the calls that take no shot). Each arg: ('d', x) double, ('i', n) int32, ('in', Struct, values) pointer
    to an input struct, ('arr', Struct, [values]) pointer to an input array, ('out', Struct) pointer to an output
    struct, ('outd',) pointer to an output double, ('null',) a NULL pointer. Every output is compared."""
    def fields_of(kind, image, P):
        if kind == "d": return [("value", struct.unpack("<Q", image[:8])[0])]
        return doubles_of(image, kind, P)
    def n(fn, shot_c, err):
        call, keep, outs = ([ctypes.byref(shot_c)] if shot else []), [], []
        for a in args:
            k = a[0]
            if k == "d": call.append(ctypes.c_double(a[1]))
            elif k == "i": call.append(ctypes.c_int32(a[1]))
            elif k == "in":
                o = CT[a[1]].from_buffer_copy(pack(a[1], a[2], 8)); keep.append(o); call.append(ctypes.byref(o))
            elif k == "arr":
                blob = b"".join(pack(a[1], v, 8) for v in a[2]); buf = ctypes.create_string_buffer(blob, len(blob)); keep.append(buf); call.append(ctypes.c_void_p(ctypes.addressof(buf)))
            elif k == "out":
                o = CT[a[1]](); outs.append((a[1], o)); call.append(ctypes.byref(o))
            elif k == "outd":
                o = D(); outs.append(("d", o)); call.append(ctypes.byref(o))
            elif k == "null": call.append(ctypes.c_void_p(None))
        rc = fn(*call, ctypes.byref(err))
        res = []
        for idx, (kind, o) in enumerate(outs):
            res += [(f"o{idx}.{p}", v) for p, v in fields_of(kind, bytes(o), 8)]
        return rc, res
    def w(wasm, name, shot_w, err):
        call, frees, outs = ([shot_w] if shot else []), [], []
        for a in args:
            k = a[0]
            if k == "d": call.append(float(a[1]))
            elif k == "i": call.append(int(a[1]))
            elif k in ("in", "arr"):
                vals = [a[2]] if k == "in" else a[2]
                ptr = wasm.alloc(b"".join(pack(a[1], v, 4) for v in vals)); frees.append(ptr); call.append(ptr)
            elif k in ("out", "outd"):
                kind = a[1] if k == "out" else "d"; size = layout(kind, 4)[0] if k == "out" else 8
                ptr = wasm.alloc(bytes(size)); frees.append(ptr); outs.append((kind, ptr, size)); call.append(ptr)
            elif k == "null": call.append(0)
        rc = getattr(wasm.e, name)(*call, err)
        res = []
        for idx, (kind, ptr, size) in enumerate(outs):
            res += [(f"o{idx}.{p}", v) for p, v in fields_of(kind, bytes(wasm.mem.read(ptr, size)), 4)]
        for ptr in frees: wasm.free(ptr)
        return rc, res
    return dict(native=n, wasm=w, rel_tol=rel_tol)

def scenario_integrate_dense(adaptive=None, req=REQ, stats=False):
    """BCLIBCFFI_integrate_adaptive_shot (adaptive=(method, rtol, atol)) or BCLIBCFFI_integrate_dense_shot (None), with the
    dense output requested: the records, the accepted steps and the step counts are all compared. stats=True passes an
    out_stats as well (BCLIBCFFI_integrate_dense_shot_stats with a fixed-step method): its StepStats is compared too."""
    use_stats = bool(adaptive) or stats
    def n(fn, shot, err):
        r = CT["Request"].from_buffer_copy(pack("Request", req, 8)); recs, dense = PT(), PT(); cnt, dcnt, reason = I(), I(), I(); st = CT["StepStats"]()
        head = [ctypes.byref(shot)] + ([ctypes.c_int32(adaptive[0]), ctypes.c_double(adaptive[1]), ctypes.c_double(adaptive[2])] if adaptive else []) + [ctypes.byref(r), ctypes.c_int32(1), ctypes.byref(recs), ctypes.byref(cnt), ctypes.byref(dense), ctypes.byref(dcnt), ctypes.byref(reason)]
        rc = fn(*head, *([ctypes.byref(st)] if use_stats else []), ctypes.byref(err))
        out = [("count", cnt.value), ("dcount", dcnt.value), ("reason", reason.value)]
        if use_stats: out += [("stats." + p, v) for p, v in doubles_of(bytes(st), "StepStats", 8)]
        if rc == 0:
            sz = ctypes.sizeof(CT["TrajectoryData"]); blob = ctypes.string_at(recs, sz * cnt.value)
            for i in range(cnt.value): out += [(f"[{i}].{p}", v) for p, v in doubles_of(blob, "TrajectoryData", 8, i * sz)]
            bs = ctypes.sizeof(CT["BaseTrajData"]); blob = ctypes.string_at(dense, bs * dcnt.value)
            for i in range(dcnt.value): out += [(f"d[{i}].{p}", v) for p, v in doubles_of(blob, "BaseTrajData", 8, i * bs)]
            NATIVE.BCLIBCFFI_free_trajectory(recs); NATIVE.BCLIBCFFI_free_base_trajectory(dense)
        return rc, out
    def w(wasm, name, shot, err):
        P = 4; r = wasm.alloc(pack("Request", req, P)); cells = [wasm.alloc(bytes(4)) for _ in range(5)]; recs, cnt, dense, dcnt, reason = cells
        stp = wasm.alloc(bytes(8))
        head = [shot] + (list(adaptive) if adaptive else []) + [r, 1, recs, cnt, dense, dcnt, reason]
        rc = getattr(wasm.e, name)(*head, *([stp] if use_stats else []), err)
        rd = lambda p: struct.unpack("<i", bytes(wasm.mem.read(p, 4)))[0]
        c, dc = rd(cnt), rd(dcnt); out = [("count", c), ("dcount", dc), ("reason", rd(reason))]
        if use_stats: out += [("stats." + p, v) for p, v in doubles_of(bytes(wasm.mem.read(stp, 8)), "StepStats", P)]
        if rc == 0:
            ptr, dptr = struct.unpack("<I", bytes(wasm.mem.read(recs, 4)))[0], struct.unpack("<I", bytes(wasm.mem.read(dense, 4)))[0]
            sz, bs = layout("TrajectoryData", P)[0], layout("BaseTrajData", P)[0]
            blob = bytes(wasm.mem.read(ptr, sz * c))
            for i in range(c): out += [(f"[{i}].{p}", v) for p, v in doubles_of(blob, "TrajectoryData", P, i * sz)]
            blob = bytes(wasm.mem.read(dptr, bs * dc))
            for i in range(dc): out += [(f"d[{i}].{p}", v) for p, v in doubles_of(blob, "BaseTrajData", P, i * bs)]
            wasm.e.BCLIBCFFI_free_trajectory(ptr); wasm.e.BCLIBCFFI_free_base_trajectory(dptr)
        for p in (r, stp, *cells): wasm.free(p)
        return rc, out
    return dict(native=n, wasm=w)

# a synthetic, time-ordered dense stream (plain + - * arithmetic: the same bits on both sides) and one step for the
# dense-to-record conversion
def _step(t): return dict(time=t, px=2700.0*t - 300.0*t*t, py=20.0*t - 16.1*t*t, pz=0.5*t, vx=2700.0 - 600.0*t, vy=20.0 - 32.2*t, vz=0.5, mach=2.4 - 0.3*t)
STREAM = [_step(i * 0.1) for i in range(8)]
STEP = dict(time=0.5, px=1300.0, py=3.2, pz=0.1, vx=2300.0, vy=-3.0, vz=0.2, mach=2.05)
D_ = lambda x: ("d", x)
I_ = lambda x: ("i", x)
RT, AT = 1e-8, 1e-8
D900 = 900.0  # 300 yd, in feet

NEW_CASES = []
for mname, m in (("CASH_KARP", 3), ("DOPRI", 4), ("TSITOURAS", 5)):
    sv = shot_values(0); sv_b = shot_values(0, barrel_elevation_rad=math.radians(0.3)); sv_a = shot_values(0, barrel_elevation_rad=math.radians(30))
    ad = (I_(m), D_(RT), D_(AT))
    NEW_CASES += [
        (f"{mname:9} integrate_adaptive+dense", "BCLIBCFFI_integrate_adaptive_shot", sv, scenario_integrate_dense((m, RT, AT))),
        (f"{mname:9} zero_angle_adaptive", "BCLIBCFFI_find_zero_angle_adaptive_shot", sv, scenario_call(*ad, D_(D900), ("outd",), ("out", "StepStats"))),
        (f"{mname:9} zero_point_adaptive", "BCLIBCFFI_find_zero_point_adaptive_shot", sv, scenario_call(*ad, D_(D900), ("out", "ZeroPoint"), ("out", "StepStats"))),
        (f"{mname:9} max_range_adaptive", "BCLIBCFFI_find_max_range_adaptive_shot", sv_b, scenario_call(*ad, D_(0.0), D_(90.0), ("out", "MaxRange"), ("out", "StepStats"))),
        (f"{mname:9} apex_adaptive", "BCLIBCFFI_find_apex_adaptive_shot", sv_a, scenario_call(*ad, ("out", "TrajectoryData"), ("out", "StepStats"))),
        (f"{mname:9} integrate_at_adaptive time=0.5", "BCLIBCFFI_integrate_at_adaptive_shot", sv_b, scenario_call(*ad, I_(0), D_(0.5), ("out", "Interception"), ("out", "StepStats"))),
        (f"{mname:9} zero_angle_lofted_adaptive low", "BCLIBCFFI_find_zero_angle_lofted_adaptive_shot", sv, scenario_call(*ad, D_(D900), I_(0), ("outd",), ("out", "StepStats"))),
        # near vertical: very sensitive to the last bits of the libm (see compare()), and so is the step count
        (f"{mname:9} zero_angle_lofted_adaptive high", "BCLIBCFFI_find_zero_angle_lofted_adaptive_shot", sv, scenario_call(*ad, D_(D900), I_(1), ("outd",), ("null",), rel_tol=1e-6)),
        (f"{mname:9} zero_point_lofted_adaptive low", "BCLIBCFFI_find_zero_point_lofted_adaptive_shot", sv, scenario_call(*ad, D_(D900), I_(0), ("out", "ZeroPoint"), ("out", "StepStats"))),
        (f"{mname:9} error_at_distance_adaptive", "BCLIBCFFI_error_at_distance_adaptive_shot", sv_b, scenario_call(*ad, D_(0.004), D_(D900), D_(0.0), ("outd",), ("out", "BaseTrajData"), ("out", "StepStats"))),
        (f"{mname:9} range_for_angle_adaptive", "BCLIBCFFI_range_for_angle_adaptive_shot", sv, scenario_call(*ad, D_(0.004), ("outd",), ("out", "StepStats"))),
        (f"{mname:9} zero_angle_newton_adaptive", "BCLIBCFFI_find_zero_angle_newton_adaptive_shot", sv, scenario_call(*ad, D_(D900), ("outd",), ("out", "StepStats"))),
        (f"{mname:9} zero_init_adaptive", "BCLIBCFFI_zero_init_adaptive_shot", sv, scenario_call(*ad, D_(D900), ("out", "ZeroInit"), ("out", "StepStats"))),
    ]
for mname, m in METHODS.items():
    if m > 2: continue  # the fixed-step methods
    sv = shot_values(m)
    NEW_CASES += [
        (f"{mname:9} integrate_dense", "BCLIBCFFI_integrate_dense_shot", sv, scenario_integrate_dense(None)),
        (f"{mname:9} integrate_dense_stats", "BCLIBCFFI_integrate_dense_shot_stats", sv, scenario_integrate_dense(None, stats=True)),
        (f"{mname:9} zero_angle_lofted low", "BCLIBCFFI_find_zero_angle_lofted_shot", sv, scenario_call(D_(D900), I_(0), ("outd",))),
        (f"{mname:9} zero_angle_lofted high", "BCLIBCFFI_find_zero_angle_lofted_shot", sv, scenario_call(D_(D900), I_(1), ("outd",))),
        (f"{mname:9} zero_point_lofted low", "BCLIBCFFI_find_zero_point_lofted_shot", sv, scenario_call(D_(D900), I_(0), ("out", "ZeroPoint"))),
        (f"{mname:9} error_at_distance", "BCLIBCFFI_error_at_distance_shot", shot_values(m, barrel_elevation_rad=math.radians(0.3)), scenario_call(D_(0.004), D_(D900), D_(0.0), ("outd",), ("out", "BaseTrajData"))),
        (f"{mname:9} error_at_distance, no hit out", "BCLIBCFFI_error_at_distance_shot", sv, scenario_call(D_(0.004), D_(D900), D_(0.0), ("outd",), ("null",))),
        (f"{mname:9} range_for_angle", "BCLIBCFFI_range_for_angle_shot", sv, scenario_call(D_(0.004), ("outd",))),
        (f"{mname:9} zero_angle_newton", "BCLIBCFFI_find_zero_angle_newton_shot", sv, scenario_call(D_(D900), ("outd",))),
        (f"{mname:9} zero_init", "BCLIBCFFI_zero_init_shot", sv, scenario_call(D_(D900), ("out", "ZeroInit"))),
    ]
sv = shot_values(0)
NEW_CASES += [
    ("zero_init very close shot", "BCLIBCFFI_zero_init_shot", sv, scenario_call(D_(1e-6), ("out", "ZeroInit"))),
    ("RK4 zero_angle_newton out of range (error path)", "BCLIBCFFI_find_zero_angle_newton_shot", sv, scenario_call(D_(60000.0), ("outd",))),
    # the trajectory ends before the target: the miss is measured at its last point (not an error any more), also as the hit
    ("RK4 error_at_distance, target never reached", "BCLIBCFFI_error_at_distance_shot", sv, scenario_call(D_(0.004), D_(1e8), D_(0.0), ("outd",), ("out", "BaseTrajData"))),
    ("TSITOURAS error_at_distance, target never reached", "BCLIBCFFI_error_at_distance_adaptive_shot", sv, scenario_call(I_(5), D_(RT), D_(AT), D_(0.004), D_(1e8), D_(0.0), ("outd",), ("out", "BaseTrajData"), ("out", "StepStats"))),
    ("dense_get_at time=0.35", "BCLIBCFFI_dense_get_at", sv, scenario_call(("arr", "BaseTrajData", STREAM), I_(len(STREAM)), I_(0), D_(0.35), D_(0.0), ("out", "BaseTrajData"), shot=False)),
    ("dense_get_at pos_x=1000", "BCLIBCFFI_dense_get_at", sv, scenario_call(("arr", "BaseTrajData", STREAM), I_(len(STREAM)), I_(2), D_(1000.0), D_(0.0), ("out", "BaseTrajData"), shot=False)),
    ("dense_get_at exact step", "BCLIBCFFI_dense_get_at", sv, scenario_call(("arr", "BaseTrajData", STREAM), I_(len(STREAM)), I_(0), D_(STREAM[3]["time"]), D_(0.0), ("out", "BaseTrajData"), shot=False)),
    ("dense_get_at outside the stream (error path)", "BCLIBCFFI_dense_get_at", sv, scenario_call(("arr", "BaseTrajData", STREAM), I_(len(STREAM)), I_(0), D_(50.0), D_(0.0), ("out", "BaseTrajData"), shot=False)),
    ("dense_get_at_slant_height 3 ft", "BCLIBCFFI_dense_get_at_slant_height", sv, scenario_call(("arr", "BaseTrajData", STREAM), I_(len(STREAM)), D_(0.0), D_(3.0), ("out", "BaseTrajData"), shot=False)),
    ("trajectory_data_from_step", "BCLIBCFFI_trajectory_data_from_step_shot", sv, scenario_call(("in", "BaseTrajData", STEP), I_(8), ("out", "TrajectoryData"))),
    ("base_traj_interpolate time=0.25", "BCLIBCFFI_base_traj_interpolate", sv, scenario_call(I_(0), D_(0.25), ("in", "BaseTrajData", STREAM[1]), ("in", "BaseTrajData", STREAM[2]), ("in", "BaseTrajData", STREAM[3]), ("out", "BaseTrajData"), shot=False)),
    ("base_traj_interpolate pos_x=900", "BCLIBCFFI_base_traj_interpolate", sv, scenario_call(I_(2), D_(900.0), ("in", "BaseTrajData", STREAM[1]), ("in", "BaseTrajData", STREAM[2]), ("in", "BaseTrajData", STREAM[3]), ("out", "BaseTrajData"), shot=False)),
    ("base_traj_interpolate equal keys (domain error path)", "BCLIBCFFI_base_traj_interpolate", sv, scenario_call(I_(0), D_(0.25), ("in", "BaseTrajData", STREAM[1]), ("in", "BaseTrajData", STREAM[1]), ("in", "BaseTrajData", STREAM[3]), ("out", "BaseTrajData"), shot=False)),
    ("base_traj_interpolate unknown key (invalid argument path)", "BCLIBCFFI_base_traj_interpolate", sv, scenario_call(I_(99), D_(0.25), ("in", "BaseTrajData", STREAM[1]), ("in", "BaseTrajData", STREAM[2]), ("in", "BaseTrajData", STREAM[3]), ("out", "BaseTrajData"), shot=False)),
    ("dense_get_at one step (domain error path)", "BCLIBCFFI_dense_get_at", sv, scenario_call(("arr", "BaseTrajData", STREAM[:1]), I_(1), I_(0), D_(0.0), D_(0.0), ("out", "BaseTrajData"), shot=False)),
]
# the zero search under the tight limits of issue #45 (cMaximumDrop = cMinimumAltitude = 0, "stop at muzzle level"): the
# widened limits, the last-point error and the Newton -> Ridder fallback, on every method, with and without a sight height
TIGHT = dict(cStepMultiplier=1.0, cZeroFindingAccuracy=0.000005, cMinimumVelocity=50.0, cMaximumDrop=0.0, cGravityConstant=-32.17405,
             cMaxIterations=60, cMinimumAltitude=0.0)
for mname, m in METHODS.items():
    for sight in (0.0, 2.0 / 12.0):
        tag = f"{mname:9} tight limits, sight {sight * 12:.0f} in"
        sv_t = shot_values(m, sight_height_ft=sight, config=TIGHT)
        NEW_CASES += [
            (f"{tag} zero 100yd", "BCLIBCFFI_find_zero_angle_shot", sv_t, scenario_zero_angle(300.0)),
            (f"{tag} zero 500yd", "BCLIBCFFI_find_zero_angle_shot", sv_t, scenario_zero_angle(1500.0)),
            (f"{tag} zero_point 200yd", "BCLIBCFFI_find_zero_point_shot", sv_t, scenario_zero_point(600.0)),
        ]
sv_t = shot_values(0, sight_height_ft=0.15, config=TIGHT)
NEW_CASES += [
    ("RK4 tight limits zero_angle_newton 200yd", "BCLIBCFFI_find_zero_angle_newton_shot", sv_t, scenario_call(D_(600.0), ("outd",))),
    ("RK4 tight limits zero_angle_lofted high 300ft", "BCLIBCFFI_find_zero_angle_lofted_shot", sv_t, scenario_call(D_(900.0), I_(1), ("outd",))),
    ("RK4 default limits zero_angle_lofted high 300ft", "BCLIBCFFI_find_zero_angle_lofted_shot", shot_values(0), scenario_call(D_(900.0), I_(1), ("outd",))),
    ("TSITOURAS tight limits zero_angle_adaptive 200yd", "BCLIBCFFI_find_zero_angle_adaptive_shot", sv_t, scenario_call(I_(5), D_(RT), D_(AT), D_(600.0), ("outd",), ("out", "StepStats"))),
    ("RK4 default limits zero 3500yd (altitude limit)", "BCLIBCFFI_find_zero_angle_shot", shot_values(0), scenario_zero_angle(10500.0)),
]
CASES += NEW_CASES


# BCLIBCFFI_get_layout() against the layouts computed here from the field lists (SPECS): the structs in the order they
# joined the ABI, each as its size and the offsets of its fields. Checked on both sides, with the pointer size of each
# (8 native, 4 wasm32), so every struct -- StepStats and ZeroInit, the newest, included -- is read back as the C
# compiler laid it out.
LAYOUT_ORDER = ["Config", "Wind", "Shot", "Request", "TrajectoryData", "MaxRange", "BaseTrajData", "Interception",
                "ZeroPoint", "Error", "StepStats", "ZeroInit", "BaseTrajBuf"]

def expected_layout(P):
    out = []
    for name in LAYOUT_ORDER:
        size, _, fields = layout(name, P)
        out += [size] + [off for _, off, _ in fields]
    return out

def check_layout(native_values, wasm_values):
    ok = True
    for side, got, P in (("native", native_values, 8), ("wasm", wasm_values, 4)):
        want = expected_layout(P)
        if got != want:
            first = next((i for i, (g, w) in enumerate(zip(got, want)) if g != w), min(len(got), len(want)))
            print(f"FAIL get_layout on {side}: {len(got)} values, expected {len(want)}; first difference at {first}")
            ok = False
        else:
            print(f"get_layout on {side:6} ok ({len(got)} values, {len(LAYOUT_ORDER)} structs)")
    return ok

def native_layout():
    NATIVE.BCLIBCFFI_get_layout.restype = I; NATIVE.BCLIBCFFI_get_layout.argtypes = [PT, I]
    buf = (I * 512)(); n = NATIVE.BCLIBCFFI_get_layout(ctypes.cast(buf, PT), 512)
    return list(buf[:n]) if n > 0 else []

def wasm_layout(wasm):
    buf = wasm.alloc(bytes(512 * 4)); n = wasm.e.BCLIBCFFI_get_layout(buf, 512)
    vals = list(struct.unpack(f"<{n}i", bytes(wasm.mem.read(buf, n * 4)))) if n > 0 else []
    wasm.free(buf); return vals

def main():
    global NATIVE, WASM
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--native", required=True, help="libbclibc_ffi.so / .dylib / .dll")
    ap.add_argument("--wasm", required=True, help="bclibc_wasm.wasm")
    ap.add_argument("--backend", action="append", help="wasmhost backend (repeatable; default wasmtime)")
    ap.add_argument("--max-ulp", type=int, default=4, help="allowed difference of the angle fields")
    args = ap.parse_args()
    NATIVE = ctypes.CDLL(args.native); bind_native()
    WASM = open(args.wasm, "rb").read()
    failures = 0
    for backend in args.backend or ["wasmtime"]:
        print(f"\n=== {args.wasm} on {backend} vs {args.native}")
        wasm = Wasm(backend)
        failures += not check_layout(native_layout(), wasm_layout(wasm))
        for label, fn, sv, extra in CASES:
            expect_error = "error path" in label
            try:
                nat = call_native(fn, sv, extra)
            except Exception as ex:  # the native library must not fail on these
                print(f"{label:44} NATIVE EXCEPTION {type(ex).__name__}: {ex}"); failures += 1; continue
            try:
                wa = call_wasm(wasm, fn, sv, extra)
            except Exception as ex:
                # bclibc never throws, so a trap here is always a bug, expected-error case or not.
                print(f"{label:44} WASM EXCEPTION {type(ex).__name__}: {str(ex).splitlines()[0][:80]}"); failures += 1
                wasm = Wasm(backend)
                continue
            ok, line = compare(label, (nat[0][0], nat[0][1]), (wa[0][0], wa[0][1]), args.max_ulp, extra.get("rel_tol"))
            if expect_error and nat[0][0] == 0:
                ok, line = False, f"{label:44} the shot was meant to fail natively"
            print(("" if ok else "FAIL ") + line)
            failures += not ok
    print(f"\n{'FAILED: ' + str(failures) + ' case(s)' if failures else 'all cases agree'}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
