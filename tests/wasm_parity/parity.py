#!/usr/bin/env python3
"""The bare WebAssembly module against the native library, through the same flat C ABI (bclibc_ffi.h).

    python tests/wasm_parity/parity.py --native build/libbclibc_ffi.so --wasm build/wasm/bclibc_wasm.wasm \\
        --backend wasmtime --backend node [--errors codes|trap]

The same shots go through `libbclibc_ffi` (ctypes) and through the module (wasmhost: https://pypi.org/project/wasmhost,
`pip install --pre wasmhost wasmtime`; the `node` backend needs Node with WebAssembly's final exception encoding when the
module uses exceptions). Every double of every result is compared bit for bit:

- everything computed with `+ - * / sqrt` must be identical (IEEE arithmetic is exact everywhere);
- the angle fields (`drop_angle_rad`, `windage_angle_rad`, `angle_rad`) come from `atan`/`atan2`, which differ between the
  native libm (glibc) and the module's (musl): up to --max-ulp (default 4) is allowed, 1 ulp is what is seen;
- a shot whose solve fails (a zero beyond the range) must return the same error status as the native library
  (`--errors codes`, the module built with exceptions) or must trap (`--errors trap`, zig's build, where a `throw` is a
  trap: a new instance is made after it).

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

def compare(label, nat, wa, max_ulp):
    """(ok, one line): the status must agree, integers must be equal, doubles identical except the angles by max_ulp."""
    (rn, on), (rw, ow) = nat, wa
    if rn != rw:
        return False, f"{label:44} STATUS native={rn} wasm={rw}"
    if [p for p, _ in on] != [p for p, _ in ow]:
        return False, f"{label:44} FIELDS differ ({len(on)} vs {len(ow)})"
    ints = ("count", "reason", "flag", "code")
    bad, worst_angle = [], 0
    for (p, a), (_, b) in zip(on, ow):
        if a == b:
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



def main():
    global NATIVE, WASM
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--native", required=True, help="libbclibc_ffi.so / .dylib / .dll")
    ap.add_argument("--wasm", required=True, help="bclibc_wasm.wasm")
    ap.add_argument("--backend", action="append", help="wasmhost backend (repeatable; default wasmtime)")
    ap.add_argument("--errors", choices=("codes", "trap"), default="codes", help="what a failed solve does in the module")
    ap.add_argument("--max-ulp", type=int, default=4, help="allowed difference of the angle fields")
    args = ap.parse_args()
    NATIVE = ctypes.CDLL(args.native); bind_native()
    WASM = open(args.wasm, "rb").read()
    failures = 0
    for backend in args.backend or ["wasmtime"]:
        print(f"\n=== {args.wasm} on {backend} vs {args.native}")
        wasm = Wasm(backend)
        for label, fn, sv, extra in CASES:
            expect_error = "error path" in label
            try:
                nat = call_native(fn, sv, extra)
            except Exception as ex:  # the native library must not fail on these
                print(f"{label:44} NATIVE EXCEPTION {type(ex).__name__}: {ex}"); failures += 1; continue
            try:
                wa = call_wasm(wasm, fn, sv, extra)
            except Exception as ex:
                if expect_error and args.errors == "trap":
                    print(f"{label:44} ok (native rc={nat[0][0]}, the module trapped as a throw does there)")
                    wasm = Wasm(backend)  # a trap leaves the shadow stack where it was: start over
                else:
                    print(f"{label:44} WASM EXCEPTION {type(ex).__name__}: {str(ex).splitlines()[0][:80]}"); failures += 1
                    wasm = Wasm(backend)
                continue
            if expect_error and args.errors == "trap":
                print(f"{label:44} the module returned {wa[0][0]}, it should have trapped"); failures += 1; continue
            ok, line = compare(label, (nat[0][0], nat[0][1]), (wa[0][0], wa[0][1]), args.max_ulp)
            if expect_error and nat[0][0] == 0:
                ok, line = False, f"{label:44} the shot was meant to fail natively"
            print(("" if ok else "FAIL ") + line)
            failures += not ok
    print(f"\n{'FAILED: ' + str(failures) + ' case(s)' if failures else 'all cases agree'}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
