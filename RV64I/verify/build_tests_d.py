"""
Test suite for the RV64D extension (double-precision floating point).
Same helper pattern as build_tests_f.py: build float constants via `li`
+ `fmv.d.x` (full 64-bit move, no NaN-boxing needed since a double already
occupies the whole FP register), and check results the same way in reverse.
"""
import os
import struct
import sys

sys.path.insert(0, os.path.dirname(__file__))
from build_tests64 import TestBuilder, build_and_run  # noqa: E402


def f64_bits(v):
    return struct.unpack("<Q", struct.pack("<d", v))[0]


def f32_bits(v):
    return struct.unpack("<I", struct.pack("<f", v))[0]


def f32_bits_signext(v):
    bits = f32_bits(v) if isinstance(v, float) else v
    return bits | 0xFFFFFFFF00000000 if (bits & 0x80000000) else bits


def dlit(freg, value, scratch="x30"):
    bits = f64_bits(value) if isinstance(value, float) else value
    return f"li {scratch}, {bits}\nfmv.d.x {freg}, {scratch}\n"


def check_double(t, freg, expected_value, scratch="x29"):
    t.asm(f"fmv.x.d {scratch}, {freg}")
    bits = f64_bits(expected_value) if isinstance(expected_value, float) else expected_value
    t.check_eq(scratch, bits)
    return t


def t_d_arith():
    t = TestBuilder("d_arith")
    t.asm(dlit("f1", 2.5) + dlit("f2", 4.0) + "fadd.d f3, f1, f2")
    check_double(t, "f3", 6.5)
    t.asm(dlit("f1", 6.0) + dlit("f2", 2.5) + "fsub.d f3, f1, f2")
    check_double(t, "f3", 3.5)
    t.asm(dlit("f1", 2.5) + dlit("f2", 4.0) + "fmul.d f3, f1, f2")
    check_double(t, "f3", 10.0)
    t.asm(dlit("f1", 7.0) + dlit("f2", 2.0) + "fdiv.d f3, f1, f2")
    check_double(t, "f3", 3.5)
    t.asm(dlit("f1", 4.0) + "fsqrt.d f3, f1")
    check_double(t, "f3", 2.0)
    t.asm(dlit("f1", 2.0) + dlit("f2", 3.0) + dlit("f3", 1.0) + "fmadd.d f4, f1, f2, f3")
    check_double(t, "f4", 7.0)
    return t


def t_d_precision():
    # A value that needs double's extra precision to represent exactly --
    # 1e18 has no exact float32 representation but does have an exact
    # float64 one, so this specifically demonstrates D isn't just F
    # zero-padded.
    t = TestBuilder("d_precision")
    t.asm(dlit("f1", 1e18) + dlit("f2", 1.0) + "fadd.d f3, f1, f2")
    check_double(t, "f3", 1e18 + 1.0)
    return t


def t_d_sign_minmax():
    t = TestBuilder("d_sign_minmax")
    t.asm(dlit("f1", 2.5) + dlit("f2", -1.0) + "fsgnj.d f3, f1, f2")
    check_double(t, "f3", -2.5)
    t.asm(dlit("f1", 2.5) + dlit("f2", -1.0) + "fmin.d f3, f1, f2")
    check_double(t, "f3", -1.0)
    t.asm(dlit("f1", 2.5) + dlit("f2", -1.0) + "fmax.d f3, f1, f2")
    check_double(t, "f3", 2.5)
    return t


def t_d_compare_class():
    t = TestBuilder("d_compare_class")
    t.asm(dlit("f1", 2.5) + dlit("f2", 2.5) + "feq.d x5, f1, f2").check_eq("x5", 1)
    t.asm(dlit("f1", -1.0) + dlit("f2", 2.5) + "flt.d x5, f1, f2").check_eq("x5", 1)
    t.asm(dlit("f1", 2.5) + "fclass.d x5, f1").check_eq("x5", 0x40)
    return t


def t_d_convert():
    t = TestBuilder("d_convert")
    t.asm(dlit("f1", 100.0) + "fcvt.w.d x5, f1").check_eq("x5", 100)
    t.asm(dlit("f1", -1.0) + "fcvt.w.d x5, f1").check_eq("x5", "0xFFFFFFFFFFFFFFFF")
    t.asm("li x1, 42\nfcvt.d.w f1, x1")
    check_double(t, "f1", 42.0)
    t.asm("li x1, 10000000000\nfcvt.d.l f1, x1")
    check_double(t, "f1", 10000000000.0)
    t.asm(dlit("f1", 10000000000.0) + "fcvt.l.d x5, f1").check_eq("x5", 10000000000)
    return t


def t_d_f_conversion():
    # Cross-format conversion: exact widening float32 -> double, and
    # rounding double -> float32.
    t = TestBuilder("d_f_conversion")
    t.asm("li x1, " + str(f32_bits(2.5)) + "\nfmv.w.x f1, x1\nfcvt.d.s f2, f1")
    check_double(t, "f2", 2.5)
    t.asm(dlit("f1", 2.5) + "fcvt.s.d f2, f1\nfmv.x.w x5, f2")
    t.check_eq("x5", f32_bits_signext(2.5))
    return t


def t_d_mem():
    t = TestBuilder("d_mem")
    t.asm(dlit("f1", 100.5) + """
li x2, 0
fsd f1, 0(x2)
fld f3, 0(x2)
""")
    check_double(t, "f3", 100.5)
    t.asm(dlit("f1", -2.5) + """
li x2, 16
fsd f1, 0(x2)
fld f3, 0(x2)
""")
    check_double(t, "f3", -2.5)
    # nonzero immediate offset (see build_tests_f.py's equivalent comment:
    # LOAD_FP/STORE_FP were missing from immediate generation entirely)
    t.asm(dlit("f1", 7.5) + """
li x2, 0
fsd f1, 24(x2)
fld f3, 24(x2)
""")
    check_double(t, "f3", 7.5)
    return t


def t_d_app_average():
    t = TestBuilder("d_app_average")
    vals = [10.0, 20.0, 30.0, 40.0]
    src = "".join(dlit(f"f{i+1}", v) for i, v in enumerate(vals))
    src += """
fadd.d f5, f1, f2
fadd.d f5, f5, f3
fadd.d f5, f5, f4
"""
    src += dlit("f6", 4.0)
    src += "fdiv.d f7, f5, f6\n"
    t.asm(src)
    check_double(t, "f7", sum(vals) / len(vals))
    return t


def main():
    results = []
    logs = {}

    def run(builder):
        line, log = build_and_run(builder)
        results.append(line)
        logs[builder.name] = log
        print(line)

    run(t_d_arith())
    run(t_d_precision())
    run(t_d_sign_minmax())
    run(t_d_compare_class())
    run(t_d_convert())
    run(t_d_f_conversion())
    run(t_d_mem())
    run(t_d_app_average())

    print("\n--- SUMMARY ---")
    passed = sum(1 for r in results if r.startswith("[PASS]"))
    total = len(results)
    print(f"{passed}/{total} passed")
    for r in results:
        print(r)


if __name__ == "__main__":
    main()
