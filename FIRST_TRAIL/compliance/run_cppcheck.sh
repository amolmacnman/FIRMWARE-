#!/usr/bin/env bash
# MISRA C:2012 + static analysis for a Smart Wagon project (GATEWAY or SUBNODE).
# Usage: compliance/run_cppcheck.sh GATEWAY   (run from the FIRST_TRIAL root)
set -e
PROJ="${1:-GATEWAY}"
BOARD="nrf54l15dk/nrf54l15/cpuapp"
HERE="$(cd "$(dirname "$0")" && pwd)"

echo "== 1) build $PROJ (export compile_commands.json) =="
west build -b "$BOARD" --sysbuild "$PROJ" -- -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

echo "== 2) cppcheck + MISRA =="
cppcheck --project="$PROJ/build/compile_commands.json" \
         --addon="$HERE/misra.json" \
         --enable=all --inline-suppr \
         --suppress=missingIncludeSystem \
         --suppressions-list="$HERE/misra-suppress.txt" \
         --output-file="$PROJ-cppcheck.txt" || true
echo "   -> $PROJ-cppcheck.txt"

echo "== 3) clang-tidy =="
clang-tidy -p "$PROJ/build" "$PROJ"/src/*.c > "$PROJ-clangtidy.txt" 2>&1 || true
echo "   -> $PROJ-clangtidy.txt"

echo "Done. Review the two report files."
