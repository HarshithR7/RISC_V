"""
Shared infrastructure for the RVV benchmark suite (bench_rvv.py):
- pack_i32_array / preload_mem: pack int32 arrays into data_memory.v's
  actual 64-bit-doubleword layout and preload them directly, so a
  benchmark's measured cycle count reflects only the loop under test,
  not an O(n) `li`+`sw` setup loop (each `li` alone is 8 instructions in
  this assembler's fixed-width 64-bit-constant encoding -- see asm64.py's
  `_li64_words` -- so a naive per-element setup would swamp any loop
  short enough to fit in a benchmark).
- build_and_run_bench: like build_tests64.build_and_run, but drives
  tb_bench64.v (which additionally counts dynamic vector-instruction and
  memory-operation retirements) instead of tb_core64.v, and parses those
  extra fields out of the [PASS] line.
"""
import os
import re
import subprocess
import sys

sys.path.insert(0, os.path.dirname(__file__))
import asm64 as asm  # noqa: E402
from build_tests64 import ROOT, RTL, GEN, RTL_FILES, TestBuilder  # noqa: E402


def write_dmem64(path, words64):
    """Like asm.write_mem, but writes the *real* 64-bit value on each
    line instead of truncating to 32 bits. asm.write_mem's truncation is
    an established, documented contract other tests rely on (see
    build_tests64.py's sum_array64 comment) -- not something to change
    out from under them -- so this benchmark suite, which genuinely needs
    to preload two packed 32-bit elements (nonzero upper half) per
    doubleword via pack_i32_array, uses its own writer instead of
    silently reinterpreting write_mem's behavior.
    """
    with open(path, "w") as f:
        for w in words64:
            f.write(f"{w & 0xFFFFFFFFFFFFFFFF:016x}\n")


def build_and_run_core(tb, dmem_words, max_cycles=40000):
    """Correctness-checking counterpart to build_and_run_bench: same
    program, driven by tb_core64.v (plain pass/fail, no instrumentation)
    instead of tb_bench64.v, and using write_dmem64 for the same reason
    build_and_run_bench does.
    """
    name = tb.name
    src = tb.source()
    items = asm.assemble_to_mem(src)
    imem_path = os.path.join(GEN, f"{name}.mem")
    asm.write_imem_halfwords(imem_path, items)

    dmem_path = os.path.join(GEN, f"{name}_data.mem")
    write_dmem64(dmem_path, dmem_words)

    wrapper_path = os.path.join(GEN, f"tb_{name}.v")
    with open(wrapper_path, "w") as f:
        f.write(f"""`timescale 1ns/1ps
module tb_{name};
    tb_core64 #(
        .IMEM_FILE("{name}.mem"),
        .DMEM_FILE("{name}_data.mem"),
        .TEST_NAME("{name}"),
        .MAX_CYCLES({max_cycles})
    ) core();
endmodule
""")

    vvp_path = os.path.join(GEN, f"{name}.vvp")
    compile_cmd = [
        "iverilog", "-g2012", "-o", vvp_path,
        os.path.join(ROOT, "tb_core64.v"),
        wrapper_path,
    ] + [os.path.join(RTL, f) for f in RTL_FILES]
    r = subprocess.run(compile_cmd, cwd=GEN, capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError(f"[COMPILE ERROR] {name}\n{r.stdout}\n{r.stderr}")

    r2 = subprocess.run(["vvp", vvp_path], cwd=GEN, capture_output=True, text=True)
    out = r2.stdout + r2.stderr
    for line in out.splitlines():
        if line.startswith("[PASS]") or line.startswith("[FAIL]") or line.startswith("[TIMEOUT]"):
            return line, out
    return "", out


def pack_i32_array(values):
    """Packs a list of 32-bit values into 64-bit doublewords, 2 per
    doubleword (low half = even index, high half = odd index) -- matches
    data_memory.v's `word_addr = mem_addr[...:3]` addressing (8-byte
    granularity), so a 4-byte-stride int32 array preloaded this way lands
    at the addresses both scalar `lw`/`sw` (any sub-doubleword offset)
    and vle32.v/vse32.v (doubleword-aligned) actually expect. Odd-length
    arrays are zero-padded to a whole doubleword.
    """
    padded = list(values) + [0] * (-len(values) % 2)
    return [(padded[i] & 0xFFFFFFFF) | ((padded[i + 1] & 0xFFFFFFFF) << 32)
            for i in range(0, len(padded), 2)]


def preload_mem(*arrays_at_base):
    """arrays_at_base: (base_addr, values) pairs, base_addr must be a
    multiple of 8 and arrays must be laid out with no gaps/overlaps for
    this simple flattener to produce a correct doubleword list -- callers
    are expected to lay out their own address map (this isn't a general
    allocator).
    """
    dmem = []
    for base_addr, values in arrays_at_base:
        assert base_addr % 8 == 0, "array base must be doubleword-aligned"
        idx = base_addr // 8
        packed = pack_i32_array(values)
        while len(dmem) < idx:
            dmem.append(0)
        dmem[idx:idx + len(packed)] = packed
    return dmem


def build_and_run_bench(tb, dmem_words, max_cycles=200000):
    name = tb.name
    src = tb.source()
    items = asm.assemble_to_mem(src)
    imem_path = os.path.join(GEN, f"{name}.mem")
    asm.write_imem_halfwords(imem_path, items)

    dmem_path = os.path.join(GEN, f"{name}_data.mem")
    write_dmem64(dmem_path, dmem_words)

    wrapper_path = os.path.join(GEN, f"tb_{name}.v")
    with open(wrapper_path, "w") as f:
        f.write(f"""`timescale 1ns/1ps
module tb_{name};
    tb_bench64 #(
        .IMEM_FILE("{name}.mem"),
        .DMEM_FILE("{name}_data.mem"),
        .TEST_NAME("{name}"),
        .MAX_CYCLES({max_cycles})
    ) core();
endmodule
""")

    vvp_path = os.path.join(GEN, f"{name}.vvp")
    compile_cmd = [
        "iverilog", "-g2012", "-o", vvp_path,
        os.path.join(ROOT, "tb_bench64.v"),
        wrapper_path,
    ] + [os.path.join(RTL, f) for f in RTL_FILES]
    r = subprocess.run(compile_cmd, cwd=GEN, capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError(f"[COMPILE ERROR] {name}\n{r.stdout}\n{r.stderr}")

    r2 = subprocess.run(["vvp", vvp_path], cwd=GEN, capture_output=True, text=True)
    out = r2.stdout + r2.stderr
    for line in out.splitlines():
        if line.startswith("[PASS]"):
            m = re.search(r"cycles=(\d+) instrs=(\d+) vec_instrs=(\d+) mem_ops=(\d+) bytes_moved=(\d+)", line)
            return {"status": "PASS", "cycles": int(m.group(1)), "instrs": int(m.group(2)),
                    "vec_instrs": int(m.group(3)), "mem_ops": int(m.group(4)),
                    "bytes_moved": int(m.group(5))}
        if line.startswith("[FAIL]") or line.startswith("[TIMEOUT]"):
            raise RuntimeError(f"{line}\n{out[-2000:]}")
    raise RuntimeError(f"no [PASS]/[FAIL] line found\n{out[-2000:]}")
