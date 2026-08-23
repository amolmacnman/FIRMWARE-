# Smart Wagon — Coding Standard & Compliance (MISRA / static analysis)

## 1. Which standard applies
For railway on-board/control software the governing software-safety standard is
**EN 50128** (rolling-stock variant **EN 50657**), itself derived from **IEC 61508**.
It assigns a Software Safety Integrity Level (SIL 0–4) and, for the coding phase,
requires a **coding standard + static analysis + a defensive subset of C**. The
industry-standard C subset is **MISRA C:2012** (with Amendments/TC). So "compliance
match" for this project means:

1. Pick the SIL from the hazard analysis (a tracker gateway is typically low SIL,
   but the tender may state one).
2. Adopt **MISRA C:2012** as the coding standard.
3. Run static analysis, triage findings, and record **deviations** with rationale.
4. Keep the evidence (reports + deviation log) for the assessor.

Note: Zephyr RTOS itself is developed against MISRA C:2012 guidelines, so the base
is already close; most findings will be in application glue code and are advisory.

## 2. Tools

### Free (good for day-to-day)
| Tool | Purpose |
|------|---------|
| **Cppcheck** + MISRA addon | MISRA C:2012 rule checks (rule numbers; add rule-text file for descriptions) |
| **clang-tidy** / **scan-build** (clang analyzer) | bug patterns, CERT-C, portability |
| Compiler `-Wall -Wextra -Wconversion -Werror` | first line of defence |

### Commercial (needed for a certifiable EN 50128 report)
LDRA, MathWorks **Polyspace**, Synopsys **Coverity**, Parasoft C/C++test,
**PC-lint Plus**, Perforce **Helix QAC** (PRQA), Klocwork. These produce the formal
MISRA compliance matrix + deviation management that assessors expect.

## 3. How to run it (this project)

The reliable way is to check against the **real compile database** so the analyzer
sees the exact macros/includes Zephyr used.

```
# 1) build once, exporting compile_commands.json
west build -b nrf54l15dk/nrf54l15/cpuapp --sysbuild GATEWAY \
      -- -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# 2) MISRA + general static analysis
cppcheck --project=GATEWAY/build/compile_commands.json \
         --addon=compliance/misra.json \
         --enable=all --inline-suppr \
         --suppress=missingIncludeSystem \
         --output-file=GATEWAY-cppcheck.txt

# 3) clang-tidy (uses the same database)
clang-tidy -p GATEWAY/build GATEWAY/src/*.c > GATEWAY-clangtidy.txt 2>&1
```

`compliance/run_cppcheck.sh` (Linux/macOS) and `run_cppcheck.bat` (Windows) do all
three for a chosen project folder. Repeat with `SUBNODE`.

### Rule descriptions
Cppcheck ships the MISRA *checker* but not the MISRA *rule text* (it's copyrighted).
To get readable output, buy the MISRA C:2012 PDF, copy each rule's one-line headline
into `compliance/misra-rule-texts.txt`, and `misra.json` points the addon at it.
Without it you still get the rule numbers (e.g. `misra-c2012-10.4`).

### Compiler warnings as errors (in Zephyr)
Add to `prj.conf` for CI:
```
CONFIG_COMPILER_WARNINGS_AS_ERRORS=y
```
and extra flags in `CMakeLists.txt`:
```
zephyr_compile_options(-Wextra -Wconversion -Wshadow)
```

## 4. Interpreting results
A first run on this codebase reports ~322 Cppcheck-MISRA items, essentially all
**advisory** rules (e.g. 10.4 essential-type in comparisons, 12.1 parentheses,
14.4 controlling expression is boolean, 15.5 single point of exit). None are the
dangerous **mandatory** rules. The compliance workflow is:

1. Fix the cheap ones (add `U`/`bool` casts for 10.4, explicit parentheses for 12.1,
   `!= 0` / `!= NULL` for 14.4).
2. For the rest, either fix or write a **deviation** (rule id, location, why it's
   safe) — inline with `// cppcheck-suppress misra-c2012-15.5` or in a project
   deviation record. `compliance/misra-suppress.txt` is the central list.
3. Re-run until the report is empty or only documented deviations remain.
4. Archive the report + deviation log as the compliance evidence.

## 5. What to hand the assessor
- The MISRA compliance report (per file/rule), 0 open mandatory/required findings.
- The deviation record (each advisory deviation justified).
- The SIL determination and this coding-standard statement.
- Traceability from RDSO/EN 50128 requirements to code (the algorithm PDFs + the
  spec cross-reference already start this).
