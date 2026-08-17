"""
Proof that "configurable vector width" is real, not just a parameter that
exists and does nothing: riscv64_processor's LANES parameter (default 4,
VLEN=128 -- what every other test in this repo uses) is explicitly
overridden to 8 (VLEN=256) via tb_core64_lanes8.v, and an 8-element
vadd.vv is checked end-to-end -- proving the *same* RTL genuinely computes
8 elements in one vector instruction when LANES=8, not just 4.

This is deliberately a single, focused test, not a parallel LANES=8 copy
of the whole RVV suite: the point is demonstrating the parameter is real
and load-bearing, not re-verifying every RVV instruction at a second
width (which the parameterization itself makes structurally certain to
behave the same way, since every width-dependent construct in
vector_alu.v/vector_register_file.v/riscv64_proc.v's mask logic is
generated generically off LANES, not hand-duplicated per width).
"""
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
import asm64 as asm  # noqa: E402
from build_tests64 import ROOT, RTL, GEN, RTL_FILES  # noqa: E402


def build_and_run_lanes8(name, src, dmem_words, max_cycles=20000):
    items = asm.assemble_to_mem(src)
    imem_path = os.path.join(GEN, f"{name}.mem")
    asm.write_imem_halfwords(imem_path, items)

    dmem_path = os.path.join(GEN, f"{name}_data.mem")
    asm.write_mem(dmem_path, dmem_words)

    wrapper_path = os.path.join(GEN, f"tb_{name}.v")
    with open(wrapper_path, "w") as f:
        f.write(f"""`timescale 1ns/1ps
module tb_{name};
    tb_core64_lanes8 #(
        .IMEM_FILE("{name}.mem"),
        .DMEM_FILE("{name}_data.mem"),
        .TEST_NAME("{name}"),
        .MAX_CYCLES({max_cycles})
    ) core();
endmodule
""")

    import subprocess
    vvp_path = os.path.join(GEN, f"{name}.vvp")
    compile_cmd = [
        "iverilog", "-g2012", "-o", vvp_path,
        os.path.join(ROOT, "tb_core64_lanes8.v"),
        wrapper_path,
    ] + [os.path.join(RTL, f) for f in RTL_FILES]
    r = subprocess.run(compile_cmd, cwd=GEN, capture_output=True, text=True)
    if r.returncode != 0:
        return f"[COMPILE ERROR] {name}\n{r.stdout}\n{r.stderr}", ""

    r2 = subprocess.run(["vvp", vvp_path], cwd=GEN, capture_output=True, text=True)
    out = r2.stdout + r2.stderr
    result_line = ""
    for line in out.splitlines():
        if line.startswith("[PASS]") or line.startswith("[FAIL]") or line.startswith("[TIMEOUT]"):
            result_line = line
    return result_line, out


def t_lanes8_vadd():
    a_base, b_base, y_base = 800, 896, 1000  # both 8-byte aligned
    a = [0, 1, 2, 3, 4, 5, 6, 7]
    b = [10, 20, 30, 40, 50, 60, 70, 80]
    expected = [a[i] + b[i] for i in range(8)]

    lines = []
    for i, v in enumerate(a):
        lines.append(f"li x5, {v}")
        lines.append(f"sw x5, {a_base + i * 4}(x0)")
    for i, v in enumerate(b):
        lines.append(f"li x5, {v}")
        lines.append(f"sw x5, {b_base + i * 4}(x0)")
    lines.append(f"""
li x11, 8
li x6, {a_base}
vsetvli x7, x11, e32,m1
vle32.v v1, (x6)
li x6, {b_base}
vle32.v v2, (x6)
vadd.vv v3, v1, v2
li x6, {y_base}
vse32.v v3, (x6)
""")

    fail_ids = []
    for i, val in enumerate(expected):
        cid = i + 1
        lines.append(f"lw x8, {i * 4}(x6)")
        lines.append(f"li x30, {val}")
        lines.append(f"bne x8, x30, fail{cid}")
        fail_ids.append(cid)
    lines.append("li x31, 0xFFFF0000")
    lines.append("ecall")
    for cid in fail_ids:
        lines.append(f"fail{cid}:")
        lines.append(f"li x31, {cid}")
        lines.append("ecall")

    src = "\n".join(lines)
    return build_and_run_lanes8("lanes8_vadd", src, [0] * 64)


def main():
    line, log = t_lanes8_vadd()
    print(line)
    if not line.startswith("[PASS]"):
        print(log[-3000:])
        sys.exit(1)
    print("1/1 passed")


if __name__ == "__main__":
    main()
