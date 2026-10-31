#!/usr/bin/env python3
"""Check that the ESP32 panic TX disconnect is executable with flash disabled."""

import argparse
from pathlib import Path
import re
import subprocess

SYMBOL = "_Z14quiesce_gdo_txv"


def check(disassembly):
    match = re.search(rf"^([0-9a-f]+) <{SYMBOL}>:", disassembly, re.M)
    if not match or not 0x40080000 <= int(match[1], 16) < 0x400C0000:
        raise ValueError("GDO TX disconnect must reside in ESP32 IRAM")
    # This routine is deliberately only register operations, with no calls to
    # GPIO/UART drivers, locks, allocators, logging, or outlined flash helpers.
    if re.search(r"\bcall(?:x)?(?:0|4|8|12)\b", disassembly):
        raise ValueError("GDO TX disconnect contains a function call")
    for address in re.findall(r"\b[0-9a-f]{8}\b", disassembly):
        value = int(address, 16)
        if 0x3F400000 <= value < 0x3F800000 or 0x400D0000 <= value < 0x40400000:
            raise ValueError(f"GDO TX disconnect references flash at {address}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--elf", type=Path, required=True)
    parser.add_argument("--toolchain", type=Path, required=True)
    args = parser.parse_args()
    tools = sorted((args.toolchain / "bin").glob("xtensa-*-objdump"))
    if not tools:
        raise ValueError("ESP32 objdump was not found")
    result = subprocess.run([str(tools[0]), "-d", f"--disassemble={SYMBOL}", str(args.elf)],
                            capture_output=True, text=True, check=True)
    check(result.stdout)
    print(f"{args.elf}: TX disconnect is in IRAM, with no calls or flash references")


if __name__ == "__main__":
    main()
