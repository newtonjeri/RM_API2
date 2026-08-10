"""Refresh error_codes.py + ERROR_CODES.md from RealMan's documentation.

The tables live on the web and change with firmware. This fetches them,
parses them, and rewrites ONLY the regions between the GENERATED markers
in `error_codes.py` — hand-written notes (SEEN_HERE, the module docstring,
the decode functions) survive untouched.

    python3 update_error_codes.py                 # report what would change
    python3 update_error_codes.py --apply         # rewrite
    python3 update_error_codes.py --md            # also rebuild the markdown

DESIGN RULES, each one earned:

  * NEVER overwrite with a worse table. A fetch that returns fewer rows
    than we already hold is treated as a failed parse, not as a deletion
    — a site redesign or a captive portal must not silently empty the
    reference we diagnose from.
  * REPORT the diff, always. New codes are the interesting output: a
    firmware release adding 0x1018 is exactly what this exists to catch.
  * stdlib only (urllib + html.parser). This runs on the lab laptop.
  * The network is optional. `--offline FILE` parses a saved page so the
    parser can be tested, and so a machine without internet can still
    update from a page someone else downloaded.
"""

import argparse
import html.parser
import pathlib
import re
import sys
import urllib.error
import urllib.request

HERE = pathlib.Path(__file__).resolve().parent
MODULE = HERE / "error_codes.py"
MARKDOWN = HERE.parent / "ERROR_CODES.md"
UA = "butterfli-error-code-sync/1.0 (+RM_API2 DualArmConcept)"
TIMEOUT = 30


class _Tables(html.parser.HTMLParser):
    """Collect every <table> as a list of rows of cell text."""

    def __init__(self):
        super().__init__()
        self.tables, self._t, self._row, self._cell = [], None, None, None

    def handle_starttag(self, tag, attrs):
        if tag == "table":
            self._t = []
        elif tag == "tr" and self._t is not None:
            self._row = []
        elif tag in ("td", "th") and self._row is not None:
            self._cell = []

    def handle_endtag(self, tag):
        if tag == "table" and self._t is not None:
            self.tables.append(self._t)
            self._t = None
        elif tag == "tr" and self._row is not None:
            if any(c.strip() for c in self._row):
                self._t.append(self._row)
            self._row = None
        elif tag in ("td", "th") and self._cell is not None:
            self._row.append(re.sub(r"\s+", " ", "".join(self._cell)).strip())
            self._cell = None

    def handle_data(self, data):
        if self._cell is not None:
            self._cell.append(data)


def fetch(url):
    req = urllib.request.Request(url, headers={"User-Agent": UA})
    with urllib.request.urlopen(req, timeout=TIMEOUT) as r:
        raw = r.read()
    for enc in ("utf-8", "gb18030", "latin-1"):
        try:
            return raw.decode(enc)
        except UnicodeDecodeError:
            continue
    return raw.decode("utf-8", "replace")


def parse_code_tables(page):
    """Rows keyed by a hex code found in ANY cell, not just the first.

    The real tables lead with a row NUMBER, so the hex sits in column 1:
        | 14 | 0x100D | 4109 | description | effect | handling | .. | 一般 |
    Requiring column 0 to be the code (the first version of this parser)
    silently matched nothing and would have wiped the table had the
    never-shrink guard not refused.

    Returns {int: {"dec", "text", "effect", "fix", "severity"}}.
    """
    p = _Tables()
    p.feed(page)
    tables = []
    for table in p.tables:
        out = {}
        for row in table:
            hexi = next((i for i, c in enumerate(row)
                         if re.fullmatch(r"0[xX][0-9A-Fa-f]{1,4}",
                                         c.strip())), None)
            if hexi is None:
                continue
            code = int(row[hexi], 16)
            rest = row[hexi + 1:]
            dec = None
            if rest and re.fullmatch(r"\d+", rest[0].strip()):
                dec = int(rest[0]); rest = rest[1:]
            cells = [c.strip() for c in rest]
            out[code] = {
                "dec": dec if dec is not None else code,
                "text": cells[0] if len(cells) > 0 else "",
                "effect": cells[1] if len(cells) > 1 else "",
                "fix": cells[2] if len(cells) > 2 else "",
                "severity": cells[-1] if len(cells) > 3 else "",
            }
        if out:
            tables.append(out)
    # Classify by CONTENT, not by order: both tables define 0x0000, and
    # merging them made the system table's "系统正常" become the joint
    # table's "关节正常". The joint table is the one carrying the step
    # warning bit; the system table carries the joint-comms code.
    system, joint = {}, {}
    for t in tables:
        if 0x4000 in t or 0x8000 in t:
            joint.update(t)
        elif 0x1001 in t or 0x100D in t:
            system.update(t)
    return system, joint


def parse_api2_returns(page):
    """The small-integer return values, with their handling suggestions.

    Scoped to rows whose FIRST cell is the code, which is how the API2
    table is laid out — and deliberately not applied to the JSON-protocol
    pages, whose leading row-number column would otherwise be read as a
    return value (that is how `1` once decoded as '0x0000').
    """
    p = _Tables()
    p.feed(page)
    out = {}
    for table in p.tables:
        for row in table:
            if not row or not re.fullmatch(r"-?\d{1,2}", row[0].strip()):
                continue
            rest = [c for c in row[1:]
                    if c.strip() and c.strip().lower() != "int"]
            # a JSON-protocol row would put a hex code here; not ours
            if rest and re.fullmatch(r"0[xX][0-9A-Fa-f]{1,4}", rest[0]):
                continue
            if not rest:
                continue
            out[int(row[0])] = (rest[0], rest[1] if len(rest) > 1 else "")
    return out


def parse_special_interfaces(page):
    """Function names in the WARNING box that opt out of the API2 codes.

    Scoped to the warning block: a page-wide scan also picked up
    `rm_set_modbustcp_mode()`, which appears in the -2 handling text and
    is NOT one of these interfaces.
    """
    low = page.lower()
    i = low.find("the following interfaces do not use the above error codes")
    if i < 0:
        i = low.find("warning")
    if i < 0:
        return ()
    block = page[i:i + 12000]
    end = block.lower().find("last update")
    if end > 0:
        block = block[:end]
    seen, out = set(), []
    for n in re.findall(r"\b(rm_[a-z0-9_]+)\s*\(\s*\)", block):
        if n not in seen:
            seen.add(n)
            out.append(n)
    return tuple(out)


# ── rendering the generated regions ──────────────────────────────────────
def _rec(d, indent="        "):
    """One field per line. The first version wrapped long lines by
    splitting on ", " — which also split INSIDE the quoted strings and
    produced a file that would not parse. Never reflow generated text;
    let repr() do the escaping and give each field its own line."""
    L = ["{"]
    L.append(f'{indent}"dec": {d["dec"]},')
    for k in ("zh", "en", "effect", "fix", "severity"):
        L.append(f'{indent}"{k}": {d[k]!r},')
    L.append(indent[:-4] + "}")
    return "\n".join(L)


def render_codes(name, d):
    lines = [f"{name} = {{"]
    for k in sorted(d):
        lines.append(f"    0x{k:04X}: " + _rec(d[k]) + ",")
    lines.append("}")
    return "\n".join(lines)


def render_system(d):
    return render_codes("SYSTEM_ERR", d)


def render_joint(bits, combined):
    return (render_codes("JOINT_ERR_BITS", bits) + "\n"
            + render_codes("JOINT_ERR_COMBINED", combined))


def render_api2(d):
    lines = ["API2_RETURN = {"]
    for k in sorted(d, key=lambda x: (x < 0, abs(x))):
        desc, fix = d[k]
        lines.append(f"    {k}: ({desc!r},")
        lines.append(f"        {fix!r}),")
    lines.append("}")
    return "\n".join(lines)


def render_special(names):
    lines = ["SPECIAL_INTERFACES = ("]
    for n in names:
        lines.append(f'    "{n}",')
    lines.append(")")
    return "\n".join(lines)


def splice(text, key, body):
    b = f"# --- GENERATED: {key}  BEGIN ---"
    e = f"# --- GENERATED: {key}  END ---"
    i, j = text.find(b), text.find(e)
    if i < 0 or j < 0:
        raise SystemExit(f"markers for {key!r} not found in {MODULE}")
    keep_head = text[:i + len(b)]
    keep_tail = text[j:]
    return keep_head + "\n" + body + "\n" + keep_tail


def _short(v):
    if isinstance(v, dict):
        return (v.get("en") or v.get("zh") or "")[:60]
    if isinstance(v, tuple):
        return str(v[0])[:60]
    return str(v)[:60]


def diff_keys(old, new, label):
    """Report added / removed / changed, printing only what a human reads."""
    added = sorted(set(new) - set(old))
    removed = sorted(set(old) - set(new))
    changed = [k for k in sorted(set(old) & set(new)) if old[k] != new[k]]
    if not (added or removed or changed):
        print(f"  {label:20s} unchanged ({len(new)} entries)")
        return False
    print(f"  {label:20s} {len(old)} -> {len(new)}")
    for k in added:
        kk = f"0x{k:04X}" if isinstance(k, int) and k > 255 else k
        print(f"      + {kk}  {_short(new[k])}")
    for k in removed:
        kk = f"0x{k:04X}" if isinstance(k, int) and k > 255 else k
        print(f"      - {kk}  {_short(old[k])}")
    for k in changed:
        kk = f"0x{k:04X}" if isinstance(k, int) and k > 255 else k
        print(f"      ~ {kk}  {_short(old[k])}  ->  {_short(new[k])}")
    return True


def build_markdown(mod):
    L = ["# RealMan error codes",
         "",
         "**Generated — do not hand-edit.** Refresh with "
         "`python3 src/update_error_codes.py --apply --md`.",
         "Hand-written notes belong in `src/error_codes.py` "
         "(`SEEN_HERE`), which survives regeneration.",
         "",
         "Sources:", ""]
    for k, v in sorted(mod.SOURCES.items()):
        L.append(f"- `{k}` — <{v}>")
    L += ["",
          "Three schemes, never interchangeable: an SDK call's **return "
          "value**, the controller's latched **system code**, and a "
          "per-joint **bitmask**.", "",
          "## API2 return values — what an SDK call returns", "",
          "| code | meaning | handling |", "|---|---|---|"]
    for k in sorted(mod.API2_RETURN, key=lambda x: (x < 0, abs(x))):
        d, f = mod.API2_RETURN[k]
        L.append(f"| `{k}` | {d} | {f or '—'} |")
    L += ["", "## System / arm codes — `rm_get_current_arm_state()"
          '["err"]["err"]`', "",
          "Reported as DECIMAL. The decimal column is what a log shows.",
          "**Bold** rows are ones this project has actually hit.", "",
          "| dec | hex | 中文 | English | effect | RealMan's remedy | sev |",
          "|---|---|---|---|---|---|---|"]
    for k in sorted(mod.SYSTEM_ERR):
        r = mod.SYSTEM_ERR[k]
        seen = ("system", k) in mod.SEEN_HERE
        en = f"**{r['en']}**" if seen and r["en"] else r["en"]
        L.append(f"| {r['dec']} | `0x{k:04X}` | {r['zh']} | {en} | "
                 f"{r['effect']} | {r['fix']} | {r['severity']} |")
    L += ["", "## Joint codes — `rm_get_joint_err_flag()`, a BITMASK", "",
          "Several bits can be set at once; decode bit by bit.", "",
          "| dec | hex | 中文 | English | effect | RealMan's remedy |",
          "|---|---|---|---|---|---|"]
    for tbl in (mod.JOINT_ERR_BITS, mod.JOINT_ERR_COMBINED):
        for k in sorted(tbl):
            r = tbl[k]
            L.append(f"| {r['dec']} | `0x{k:04X}` | {r['zh']} | {r['en']} | "
                     f"{r['effect']} | {r['fix']} |")
    L += ["", "## Interfaces that do NOT use the API2 return values", "",
          "Each documents its own codes. Passing one of their returns "
          "through the API2 decoder gives a confident wrong answer, which "
          "is why `describe_api2_return(code, func=...)` refuses.", ""]
    for n in mod.SPECIAL_INTERFACES:
        L.append(f"- `{n}()`")
    L += ["", "## Codes this project has actually hit", "",
          "| scheme | code | what it was |", "|---|---|---|"]
    for (kind, k), v in sorted(mod.SEEN_HERE.items()):
        L.append(f"| {kind} | `{k}` (`0x{k:04X}`) | {v} |")
    L.append("")
    return "\n".join(L)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--apply", action="store_true",
                    help="rewrite error_codes.py (default: report only)")
    ap.add_argument("--force", action="store_true",
                    help="write even if the parsed table is SMALLER than "
                         "what we hold (first population, or recovery from "
                         "a corrupted file). Off by default so a bad fetch "
                         "cannot delete the reference.")
    ap.add_argument("--md", action="store_true",
                    help="also rebuild ERROR_CODES.md")
    ap.add_argument("--offline", metavar="FILE", action="append", default=[],
                    help="parse a saved HTML page instead of fetching")
    args = ap.parse_args()

    sys.path.insert(0, str(HERE))
    import error_codes as mod

    pages = {}
    if args.offline:
        for f in args.offline:
            pages[f] = pathlib.Path(f).read_text(errors="replace")
    else:
        for key, url in sorted(set(mod.SOURCES.items()),
                               key=lambda kv: kv[1]):
            if url in pages:
                continue
            try:
                pages[url] = fetch(url)
                print(f"  fetched {url}  ({len(pages[url])} bytes)")
            except (urllib.error.URLError, OSError) as exc:
                print(f"  [WARN] {url}: {exc}")
    if not pages:
        print("\n  nothing fetched — no network? "
              "Save the pages and pass --offline FILE.")
        return 1

    # Parse each page for what it actually holds. Concatenating them
    # made the API2 parser read the JSON table's row-number column as a
    # return value.
    api2, special = {}, ()
    zh_rows, en_rows = {}, {}
    for url, page in pages.items():
        if "apierrorList" in url:
            api2.update(parse_api2_returns(page))
            got = parse_special_interfaces(page)
            if len(got) > len(special):
                special = got
        if "errorList" in url and "json" in url:
            sysr, jntr = parse_code_tables(page)
            tgt = en_rows if "/en/" in url else zh_rows
            tgt.setdefault("system", {}).update(sysr)
            tgt.setdefault("joint", {}).update(jntr)

    def merge(which):
        z, e = zh_rows.get(which, {}), en_rows.get(which, {})
        out = {}
        for k in set(z) | set(e):
            zz, ee = z.get(k, {}), e.get(k, {})
            out[k] = {
                "dec": (ee or zz).get("dec", k),
                "zh": zz.get("text", ""),
                "en": ee.get("text", ""),
                "effect": ee.get("effect") or zz.get("effect", ""),
                "fix": ee.get("fix") or zz.get("fix", ""),
                "severity": ee.get("severity") or zz.get("severity", ""),
            }
        return out

    system = merge("system")
    joint_all = merge("joint")
    joint_comb = {k: v for k, v in joint_all.items() if k == 0xF000}
    joint_bits = {k: v for k, v in joint_all.items() if k != 0xF000}

    print("\n  parsed from the documentation:")
    changed = False
    ok = True
    for label, got, have in (
            ("api2_return", api2, mod.API2_RETURN),
            ("system_err", system, mod.SYSTEM_ERR),
            ("joint_err", joint_bits, mod.JOINT_ERR_BITS)):
        if len(got) < len(have) and not args.force:
            print(f"  {label:20s} REFUSED: parsed {len(got)} rows but we "
                  f"already hold {len(have)}. A shrinking table is a failed "
                  "parse, not a deletion — leaving it alone.")
            ok = False
            continue
        changed |= diff_keys(have, got, label)
    if special and set(special) != set(mod.SPECIAL_INTERFACES):
        if len(special) < len(mod.SPECIAL_INTERFACES):
            print(f"  {'special_interfaces':20s} REFUSED: parsed "
                  f"{len(special)} < {len(mod.SPECIAL_INTERFACES)} held")
        else:
            print(f"  {'special_interfaces':20s} "
                  f"{len(mod.SPECIAL_INTERFACES)} -> {len(special)}")
            for n in sorted(set(special) - set(mod.SPECIAL_INTERFACES)):
                print(f"      + {n}()")
            changed = True
    else:
        print(f"  {'special_interfaces':20s} unchanged "
              f"({len(special or mod.SPECIAL_INTERFACES)} entries)")

    if not changed:
        print("\n  nothing to update.")
    elif not args.apply:
        print("\n  re-run with --apply to write these into error_codes.py")
    else:
        text = MODULE.read_text()
        if args.force or (ok and len(api2) >= len(mod.API2_RETURN)):
            text = splice(text, "api2_return", render_api2(api2))
        if args.force or (ok and len(system) >= len(mod.SYSTEM_ERR)):
            text = splice(text, "system_err", render_system(system))
        if args.force or (ok
                          and len(joint_bits) >= len(mod.JOINT_ERR_BITS)):
            text = splice(text, "joint_err",
                          render_joint(joint_bits,
                                       joint_comb or mod.JOINT_ERR_COMBINED))
        if args.force or len(special) >= len(mod.SPECIAL_INTERFACES):
            text = splice(text, "special_interfaces", render_special(special))
        MODULE.write_text(text)
        print(f"\n  wrote {MODULE}")

    if args.md:
        import importlib
        importlib.reload(mod)
        MARKDOWN.write_text(build_markdown(mod))
        print(f"  wrote {MARKDOWN}")
    return 0


if __name__ == "__main__":
    from log_utils import setup_log
    setup_log(__file__)
    sys.exit(main())
