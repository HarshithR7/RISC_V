"""
Test suite for the RV64F extension (single-precision floating point).
There's no "load float immediate" instruction in RV64F -- real compilers
build float constants by loading the bit pattern into an integer register
(`li`) and moving it into an FP register (`fmv.w.x`), which is exactly
what `flit` (a helper below, not a real instruction) expands to. Checking
a float *result* works the same way in reverse: `fmv.x.w` into a scratch
integer register, then the normal check_eq machinery -- FP registers can't
be used directly with `bne`. `fmv.x.w` sign-extends the 32-bit bit pattern
to 64 bits per spec, so the expected value passed to check_eq must be that
same sign-extended pattern, not the raw 32 bits.
"""
import math
import os
import struct
import sys

sys.path.insert(0, os.path.dirname(__file__))
from build_tests64 import TestBuilder, build_and_run  # noqa: E402


def f32_bits(v):
    return struct.unpack("<I", struct.pack("<f", v))[0]


def f32_bits_signext(v):
    bits = f32_bits(v) if isinstance(v, float) else v
    return bits | 0xFFFFFFFF00000000 if (bits & 0x80000000) else bits


def flit(freg, value, scratch="x30"):
    """Assembly text loading the float32 value `value` (a Python float, or
    already-encoded bit pattern if an int) into `freg`."""
    bits = f32_bits(value) if isinstance(value, float) else value
    return f"li {scratch}, {bits}\nfmv.w.x {freg}, {scratch}\n"


def check_float(t, freg, expected_value, scratch="x29"):
    """Appends fmv.x.w + check_eq for a float register's value."""
    t.asm(f"fmv.x.w {scratch}, {freg}")
    t.check_eq(scratch, f32_bits_signext(expected_value))
    return t


def t_f_arith():
    t = TestBuilder("f_arith")
    t.asm(flit("f1", 2.5) + flit("f2", 4.0) + "fadd.s f3, f1, f2")
    check_float(t, "f3", 6.5)
    t.asm(flit("f1", 6.0) + flit("f2", 2.5) + "fsub.s f3, f1, f2")
    check_float(t, "f3", 3.5)
    t.asm(flit("f1", 2.5) + flit("f2", 4.0) + "fmul.s f3, f1, f2")
    check_float(t, "f3", 10.0)
    t.asm(flit("f1", 7.0) + flit("f2", 2.0) + "fdiv.s f3, f1, f2")
    check_float(t, "f3", 3.5)
    t.asm(flit("f1", 4.0) + "fsqrt.s f3, f1")
    check_float(t, "f3", 2.0)
    # FMADD family: 2.0*3.0+1.0=7.0, 2.0*3.0-1.0=5.0
    t.asm(flit("f1", 2.0) + flit("f2", 3.0) + flit("f3", 1.0) + "fmadd.s f4, f1, f2, f3")
    check_float(t, "f4", 7.0)
    t.asm(flit("f1", 2.0) + flit("f2", 3.0) + flit("f3", 1.0) + "fmsub.s f4, f1, f2, f3")
    check_float(t, "f4", 5.0)
    return t


def t_f_sign_minmax():
    t = TestBuilder("f_sign_minmax")
    t.asm(flit("f1", 2.5) + flit("f2", -1.0) + "fsgnj.s f3, f1, f2")
    check_float(t, "f3", -2.5)
    t.asm(flit("f1", -2.5) + flit("f2", -1.0) + "fsgnjn.s f3, f1, f2")
    check_float(t, "f3", 2.5)
    t.asm(flit("f1", 2.5) + flit("f2", -1.0) + "fmin.s f3, f1, f2")
    check_float(t, "f3", -1.0)
    t.asm(flit("f1", 2.5) + flit("f2", -1.0) + "fmax.s f3, f1, f2")
    check_float(t, "f3", 2.5)
    return t


def t_f_compare_class():
    t = TestBuilder("f_compare_class")
    t.asm(flit("f1", 2.5) + flit("f2", 2.5) + "feq.s x5, f1, f2").check_eq("x5", 1)
    t.asm(flit("f1", -1.0) + flit("f2", 2.5) + "flt.s x5, f1, f2").check_eq("x5", 1)
    t.asm(flit("f1", 2.5) + flit("f2", -1.0) + "flt.s x5, f1, f2").check_eq("x5", 0)
    t.asm(flit("f1", 2.5) + flit("f2", 2.5) + "fle.s x5, f1, f2").check_eq("x5", 1)
    t.asm(flit("f1", 2.5) + "fclass.s x5, f1").check_eq("x5", 0x40)   # +normal
    t.asm(flit("f1", 0.0) + "fclass.s x5, f1").check_eq("x5", 0x10)   # +0
    t.asm(flit("f1", -0.0) + "fclass.s x5, f1").check_eq("x5", 0x8)   # -0
    return t


def t_f_convert():
    t = TestBuilder("f_convert")
    t.asm(flit("f1", 100.0) + "fcvt.w.s x5, f1").check_eq("x5", 100)
    t.asm(flit("f1", -1.0) + "fcvt.w.s x5, f1").check_eq("x5", "0xFFFFFFFFFFFFFFFF")
    t.asm(flit("f1", 100.0) + "fcvt.wu.s x5, f1").check_eq("x5", 100)
    t.asm("li x1, 42\nfcvt.s.w f1, x1")
    check_float(t, "f1", 42.0)
    t.asm("li x1, -1\nfcvt.s.w f1, x1")
    check_float(t, "f1", -1.0)
    # 64-bit conversions: value chosen to round-trip exactly through float32
    t.asm("li x1, 10000000000\nfcvt.s.l f1, x1")
    check_float(t, "f1", 10000000000.0)
    t.asm(flit("f1", 10000000000.0) + "fcvt.l.s x5, f1").check_eq("x5", 10000000000)
    return t


def t_f_move_mem():
    t = TestBuilder("f_move_mem")
    t.asm("li x1, 0xDEADBEEF\nfmv.w.x f1, x1\nfmv.x.w x5, f1").check_eq("x5", "0xFFFFFFFFDEADBEEF")
    # FLW/FSW round trip through data memory
    t.asm(flit("f1", 100.5) + """
li x2, 0
fsw f1, 0(x2)
flw f3, 0(x2)
""")
    check_float(t, "f3", 100.5)
    # a negative value (bit 31 set) must NOT be sign-extended by the load
    t.asm(flit("f1", -2.5) + """
li x2, 8
fsw f1, 0(x2)
flw f3, 0(x2)
""")
    check_float(t, "f3", -2.5)
    # a nonzero immediate offset on both FSW and FLW (caught a real decode
    # bug during development: LOAD_FP/STORE_FP were missing from the
    # immediate-generation case entirely, defaulting to offset 0 always --
    # every other test here happened to use offset 0 and would never have
    # caught it)
    t.asm(flit("f1", 42.0) + """
li x2, 0
fsw f1, 16(x2)
flw f3, 16(x2)
""")
    check_float(t, "f3", 42.0)
    return t


def t_f_app_average():
    # A small "application": average of 4 floats via FADD/FDIV.
    t = TestBuilder("f_app_average")
    vals = [10.0, 20.0, 30.0, 40.0]
    src = "".join(flit(f"f{i+1}", v) for i, v in enumerate(vals))
    src += """
fadd.s f5, f1, f2
fadd.s f5, f5, f3
fadd.s f5, f5, f4
"""
    src += flit("f6", 4.0)
    src += "fdiv.s f7, f5, f6\n"
    t.asm(src)
    check_float(t, "f7", sum(vals) / len(vals))
    return t


def main():
    results = []
    logs = {}

    def run(builder):
        line, log = build_and_run(builder)
        results.append(line)
        logs[builder.name] = log
        print(line)

    run(t_f_arith())
    run(t_f_sign_minmax())
    run(t_f_compare_class())
    run(t_f_convert())
    run(t_f_move_mem())
    run(t_f_app_average())

    print("\n--- SUMMARY ---")
    passed = sum(1 for r in results if r.startswith("[PASS]"))
    total = len(results)
    print(f"{passed}/{total} passed")
    for r in results:
        print(r)


if __name__ == "__main__":
    main()
