#!/usr/bin/env python3

"""
Replace strings like ${key} with a value
"""

import sys
from typing import Dict, List


def replace(s: str, d: Dict[str, str]) -> str:
    for k, v in d.items():
        long_k = '${' + k + '}'
        s = s.replace(long_k, v)
    return s


def parse_args(argv: List[str]) -> Dict[str, str]:
    d = {}
    for a in argv:
        p = a.split('=')
        if len(p) != 2:
            print('ignoring "%r"' % a, file=sys.stderr)
            continue
        d[p[0]] = p[1]
    return d


if len(sys.argv) < 2:
    print('usage: replace.py something.xml foo=1 bar=2.0 fee=string random="also a string"', file=sys.stderr)
    sys.exit(1)

f = open(sys.argv[1], 'r')
print(replace(f.read(), parse_args(sys.argv[2:])))
