#!/usr/bin/env python3
"""Merge MCUboot and the signed application into a single merged.hex.

WHY THIS EXISTS: `west flash` programs the bootloader and the application as two
separate operations with a RESET BETWEEN THEM. On the nRF54L15 that reset lets
MCUboot run, and MCUboot configures SPU protection over the application slot as
part of its normal startup - so the debugger's second write to 0x10000 is
refused:

    Device error: Memory access error at 0x00010000.
    Probably a memory protection issue. Probe access is Secure (Generic)

One file programmed in one operation has no such window. It is also the right
artefact for production flashing: it cannot half-apply, leaving a board with a
bootloader and no application.

Deliberately NEVER fails the build. This is a convenience artefact, not a build
product - if intelhex is missing or an input has not been produced, it says so
and exits 0 so a working build is not broken by a packaging step.
"""
import os
import sys


def main():
    if len(sys.argv) != 4:
        print("merge_hex: usage: merge_hex.py <mcuboot.hex> <app.hex> <out.hex>")
        return 0

    boot, app, out = sys.argv[1:4]

    for p in (boot, app):
        if not os.path.isfile(p):
            print("merge_hex: skipped - %s not present" % p)
            return 0

    try:
        from intelhex import IntelHex
    except ImportError:
        print("merge_hex: skipped - the 'intelhex' module is not installed in "
              "%s\n           (pip install intelhex to enable merged.hex)"
              % sys.executable)
        return 0

    try:
        ih = IntelHex(boot)
        ap = IntelHex(app)

        # overlap='error' on purpose. The bootloader and the application must
        # occupy disjoint regions; if they ever overlap, the partition layout is
        # wrong and silently letting one overwrite the other would produce a
        # merged image that bricks the board in a way no test would catch.
        ih.merge(ap, overlap='error')
        ih.write_hex_file(out)
    except Exception as e:                                    # noqa: BLE001
        print("merge_hex: skipped - %s" % e)
        return 0

    print("merge_hex: %s  (0x%X-0x%X)" % (out, ih.minaddr(), ih.maxaddr()))
    return 0


if __name__ == "__main__":
    sys.exit(main())
