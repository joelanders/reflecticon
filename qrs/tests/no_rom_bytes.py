#!/usr/bin/env python3
"""Fail if a folder holds anything that looks like ROM bytes: a binary file, a hex string longer than a
SHA-256 (64 digits), a run of 32 or more byte literals (0x00-0xff), a run of 64 or more small integers
(0-255), or a base64-like run of 200+ characters. The plugins hold only sizes, hashes, short hash
prefixes and repair masks.

    python3 qrs/tests/no_rom_bytes.py [folder ...]     (default: qrs)
"""
import os
import re
import sys

LONG_HEX = re.compile(r"[0-9a-fA-F]{65,}")
BYTE_RUN = re.compile(r"(?:\b0x[0-9a-fA-F]{1,2}\b\s*,\s*){32,}")
INT_RUN = re.compile(r"(?:\b(?:25[0-5]|2[0-4]\d|1?\d?\d)\b\s*,\s*){64,}")
BASE64 = re.compile(r"[A-Za-z0-9+/]{200,}")


def check(path):
    data = open(path, "rb").read()
    try:
        text = data.decode("utf-8")
    except UnicodeDecodeError:
        return "not text (%d bytes)" % len(data)
    if "\0" in text:
        return "binary (%d bytes)" % len(data)
    for name, rx in (("a hex string over 64 digits", LONG_HEX), ("32+ byte literals in a row", BYTE_RUN),
                     ("64+ small integers in a row", INT_RUN), ("a base64-like run", BASE64)):
        m = rx.search(text)
        if m:
            line = text.count("\n", 0, m.start()) + 1
            return "%s at line %d" % (name, line)
    return None


def main():
    roots = sys.argv[1:] or ["qrs"]
    bad = 0
    for root in roots:
        for d, _, files in os.walk(root):
            for f in files:
                p = os.path.join(d, f)
                why = check(p)
                if why:
                    print("%s: %s" % (p, why))
                    bad += 1
    print("%s: %s" % (", ".join(roots), "%d file(s) look like ROM bytes" % bad if bad else "no ROM bytes"))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
