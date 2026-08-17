"""
Proves the "C -> real compiler -> RVV -> this hardware" pipeline actually
works, not just that hand-written RVV assembly runs (which every other
test in this suite already demonstrates).

vadd.c (a plain `c[i] = a[i] + b[i]` loop) is compiled with a real,
unmodified xPack GCC 15.2.0 (`riscv-none-elf-gcc -march=rv64gv_zicsr_zifencei
-O3 -ftree-vectorize`, C extension deliberately excluded for this first
integration so every instruction is a plain 4-byte word -- simplifies
splicing without needing compressed-instruction awareness here). GCC's
autovectorizer happened to emit exactly this scoped implementation's
supported subset (`vsetvli ...,e32,m1` / `vle32.v` / `vadd.vv` /
`vse32.v`, using the standard "vl = min(remaining, VLMAX)" stripmining
loop) -- not a coincidence engineered by this test, just what a real
compiler naturally generates for a simple int[] loop at -O3.

The compiled function's literal machine code (extracted via `objdump -d`,
copied verbatim -- not re-derived or hand-corrected) is spliced into an
otherwise hand-written test program via asm64.py's `.word` directive
(added for exactly this purpose), called with `jal ra` following the
standard RISC-V calling convention (a0=dest, a1=a, a2=b, a3=n), and its
result is checked against a Python-computed expected array. If this
passes, GCC's actual compiled output is running correctly on this
simulated hardware -- not just on paper.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from build_tests64 import TestBuilder, build_and_run  # noqa: E402

# Extracted once via:
#   riscv-none-elf-gcc -march=rv64gv_zicsr_zifencei -mabi=lp64d -O3 \
#       -ftree-vectorize -c vadd.c -o vadd.o
#   riscv-none-elf-objdump -d vadd.o
# from:
#   void vadd(int *c, const int *a, const int *b, int n) {
#       for (int i = 0; i < n; i++) c[i] = a[i] + b[i];
#   }
# Copied verbatim -- these are GCC's actual output words, not
# hand-derived encodings.
GCC_VADD_WORDS = [
    0x02d05863,  # blez  a3, .L5
    0x0d06f7d7,  # .L3: vsetvli a5, a3, e32, m1, ta, ma
    0x0205e107,  #      vle32.v v2, (a1)
    0x02066087,  #      vle32.v v1, (a2)
    0x00279713,  #      slli  a4, a5, 2
    0x40f686b3,  #      sub   a3, a3, a5
    0x00e585b3,  #      add   a1, a1, a4
    0x00e60633,  #      add   a2, a2, a4
    0x021100d7,  #      vadd.vv v1, v1, v2
    0x020560a7,  #      vse32.v v1, (a0)
    0x00e50533,  #      add   a0, a0, a4
    0xfc069ce3,  #      bnez  a3, .L3
    0x00008067,  # .L5: ret
]


def t_compiler_vadd():
    t = TestBuilder("compiler_vadd")
    a = [1, 2, 3, 4, 5, 6, 7, 8]
    b = [10, 20, 30, 40, 50, 60, 70, 80]
    n = len(a)
    a_base, b_base, c_base = 0, 32, 64

    lines = []
    for i, v in enumerate(a):
        lines.append(f"li x5, {v}")
        lines.append(f"sw x5, {a_base + i * 4}(x0)")
    for i, v in enumerate(b):
        lines.append(f"li x5, {v}")
        lines.append(f"sw x5, {b_base + i * 4}(x0)")
    lines.append(f"li a0, {c_base}")  # dest c
    lines.append(f"li a1, {a_base}")  # src a
    lines.append(f"li a2, {b_base}")  # src b
    lines.append(f"li a3, {n}")       # n
    lines.append("jal ra, vadd_func")
    t.asm("\n".join(lines))

    for i in range(n):
        t.asm(f"lw x8, {c_base + i * 4}(x0)")
        t.check_eq("x8", a[i] + b[i])

    word_lines = "\n".join(f".word {hex(w)}" for w in GCC_VADD_WORDS)
    t.asm(f"""
j after_vadd_func
vadd_func:
{word_lines}
after_vadd_func:
""")
    return t


def main():
    t = t_compiler_vadd()
    line, log = build_and_run(t)
    print(line)
    if not line.startswith("[PASS]"):
        print(log[-4000:])
        sys.exit(1)


if __name__ == "__main__":
    main()
