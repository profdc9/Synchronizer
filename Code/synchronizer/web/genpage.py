#!/usr/bin/env python3
"""Turn web/page.html into src/webpage.c.

The page is served straight out of flash, so it lives in the firmware as a
C string.  Edit page.html and run this; never hand-edit the escaping.

    python3 web/genpage.py

Verify a round trip at any time with:

    python3 web/genpage.py --check
"""
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SRC = os.path.join(HERE, "page.html")
DST = os.path.join(ROOT, "src", "webpage.c")

LICENSE = """
/*
   Copyright (c) 2026 Daniel Marks

  This software is provided 'as-is', without any express or implied
  warranty. In no event will the authors be held liable for any damages
  arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not
   claim that you wrote the original software. If you use this software
   in a product, an acknowledgment in the product documentation would be
   appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and must not be
   misrepresented as being the original software.
  3. This notice may not be removed or altered from any source distribution.
*/
"""


def escape(s):
    out = []
    for ch in s:
        if ch == "\\":
            out.append("\\\\")
        elif ch == '"':
            out.append('\\"')
        elif ch == "\t":
            out.append("\\t")
        elif ch == "?":
            out.append("\\?")          # so no trigraph can ever form
        elif ord(ch) < 32:
            out.append("\\%03o" % ord(ch))
        elif ord(ch) > 126:
            out.append("".join("\\%03o" % b for b in ch.encode()))
        else:
            out.append(ch)
    return "".join(out)


def unescape(c_source):
    """Recover the page from the generated C, to prove the escaping is sound."""
    body = c_source.split("const char web_page[] =", 1)[1].rsplit(";", 1)[0]
    parts = re.findall(r'"((?:[^"\\]|\\.)*)"', body, re.S)
    out = bytearray()
    for p in parts:
        i = 0
        while i < len(p):
            if p[i] != "\\":
                out.append(ord(p[i])); i += 1
                continue
            n = p[i + 1]
            if n in "01234567":
                out.append(int(p[i + 1:i + 4], 8)); i += 4
            elif n == "t":
                out.append(9); i += 2
            elif n == "n":
                out.append(10); i += 2
            else:
                out.append(ord(n)); i += 2
    return out.decode("utf-8")


def build(html):
    lines = html.split("\n")
    body = "\n".join(
        '  "%s%s"' % (escape(l), "\\n" if i < len(lines) - 1 else "")
        for i, l in enumerate(lines)
    )
    return (
        "/* webpage.c - the web interface page.\n"
        "   GENERATED from web/page.html by web/genpage.py - do not edit by hand.\n"
        "   Edit the HTML and regenerate. */\n"
        + LICENSE
        + '\n#include <stdint.h>\n#include "webui.h"\n\n'
        "const char web_page[] =\n" + body + ";\n\n"
        "const uint32_t web_page_len = (uint32_t)(sizeof(web_page) - 1u);\n"
    )


def main():
    html = open(SRC, encoding="utf-8").read()
    out = build(html)

    if unescape(out) != html:
        sys.exit("escaping is not a faithful round trip - refusing to write")

    if "--check" in sys.argv:
        cur = open(DST, encoding="utf-8").read() if os.path.exists(DST) else ""
        if cur != out:
            sys.exit("%s is stale - run: python3 web/genpage.py" % DST)
        print("webpage.c matches page.html (%d bytes)" % len(html))
        return

    open(DST, "w", encoding="utf-8").write(out)
    print("wrote %s from %s (%d bytes of page)" % (DST, SRC, len(html)))


main()
