#!/usr/bin/env bash
# Build the golden-fixture harness against the original Java sources.
#
# Environment (all optional except where noted):
#   ORIGINAL_SRC  path to the 'source' directory of a clone of
#                 https://github.com/arc-l/optimal-mrppg-journal   (required)
#   GUROBI_JAR    path to gurobi.jar (default: $GUROBI_HOME/lib/gurobi.jar)
#   JAVAC         javac executable (default: javac on PATH)
#   OUT           output directory for the compiled classes
#                 (default: a fresh temporary directory)
#
# The original sources are copied to a temporary build directory and never modified.
# Gurobi >= 12 renamed the Java package 'gurobi' to 'com.gurobi.gurobi'; if the jar
# uses the new package, the import lines (and only those) are rewritten, in the
# original sources and in Harness.java alike.
#
# On success the last line printed is the classes directory.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
: "${ORIGINAL_SRC:?set ORIGINAL_SRC to the source directory of the original repository}"
if [ -z "${GUROBI_JAR:-}" ]; then
  if [ -n "${GUROBI_HOME:-}" ]; then GUROBI_JAR="$GUROBI_HOME/lib/gurobi.jar"; else
    echo "set GUROBI_JAR (or GUROBI_HOME)" >&2; exit 2; fi
fi
JAVAC="${JAVAC:-javac}"
[ -d "$ORIGINAL_SRC/projects/multipath" ] || { echo "ORIGINAL_SRC=$ORIGINAL_SRC has no projects/multipath" >&2; exit 2; }
[ -f "$GUROBI_JAR" ] || { echo "GUROBI_JAR=$GUROBI_JAR not found" >&2; exit 2; }

SRC_TMP="$(mktemp -d "${TMPDIR:-/tmp}/mppflow-golden-src.XXXXXX")"
trap 'rm -rf "$SRC_TMP"' EXIT
OUT="${OUT:-$(mktemp -d "${TMPDIR:-/tmp}/mppflow-golden-classes.XXXXXX")}"
mkdir -p "$OUT"

# Only the non-GUI ILP/advanced packages are needed (they still import java.awt, which
# is part of the JDK and works headless).
mkdir -p "$SRC_TMP/projects/multipath"
cp -r "$ORIGINAL_SRC/projects/multipath/advanced" "$SRC_TMP/projects/multipath/"
cp -r "$ORIGINAL_SRC/projects/multipath/ILP" "$SRC_TMP/projects/multipath/"
cp "$HERE/Harness.java" "$SRC_TMP/projects/multipath/ILP/Harness.java"

# Does the jar use the new package name?
jar_has_new_package() {
  if command -v unzip >/dev/null 2>&1; then
    unzip -l "$GUROBI_JAR" | grep -q "com/gurobi/gurobi/GRB.class"
  else
    python3 -c 'import sys, zipfile; sys.exit(0 if "com/gurobi/gurobi/GRB.class" in zipfile.ZipFile(sys.argv[1]).namelist() else 1)' "$GUROBI_JAR"
  fi
}
if jar_has_new_package; then
  # '-i.bak' (suffix attached) is the in-place form that GNU and BSD/macOS sed both accept.
  find "$SRC_TMP" -name "*.java" -exec sed -i.bak 's/^import gurobi\./import com.gurobi.gurobi./' {} +
  find "$SRC_TMP" -name "*.java.bak" -exec rm -f {} +
  echo "gurobi package: com.gurobi.gurobi (imports rewritten)" >&2
else
  echo "gurobi package: gurobi (no rewrite)" >&2
fi

rm -rf "$OUT/projects"
set +e
"$JAVAC" -nowarn -encoding UTF-8 -cp "$GUROBI_JAR" -d "$OUT" \
  $(find "$SRC_TMP" -name "*.java") > "$SRC_TMP/javac.log" 2>&1
rc=$?
set -e
grep -v -E "^Note: " "$SRC_TMP/javac.log" >&2 || true
[ $rc -eq 0 ] && [ -f "$OUT/projects/multipath/ILP/Harness.class" ] || { echo "compilation failed" >&2; exit 1; }
echo "$OUT"
