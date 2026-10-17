#!/usr/bin/env bash
# Record tests/data/core/{graph,problem,pathfinder,families}.json.gz from the original Java code.
#
# Environment: ORIGINAL_SRC and GUROBI_JAR (or GUROBI_HOME) as for ../build.sh (required);
#   JAVAC, JAVA   the JDK's javac / java (default: on PATH; JDK 17 was used)
#   OUT           output directory (default: tests/data/core of this repository)
#   WHAT          comma-separated parts (default: graph,problem,pathfinder,families)
#
# Builds the original classes with ../build.sh, compiles CoreAccess.java (package
# projects.multipath.advanced, for the package-private members) and CoreGolden.java next to
# them, runs CoreGolden in a temporary directory and gzips each JSON file into OUT.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../../.." && pwd)"
OUT="${OUT:-$REPO/tests/data/core}"
WHAT="${WHAT:-graph,problem,pathfinder,families}"
JAVAC="${JAVAC:-javac}"
JAVA="${JAVA:-java}"
if [ -z "${GUROBI_JAR:-}" ] && [ -n "${GUROBI_HOME:-}" ]; then GUROBI_JAR="$GUROBI_HOME/lib/gurobi.jar"; fi
: "${GUROBI_JAR:?set GUROBI_JAR (or GUROBI_HOME)}"
export GUROBI_JAR JAVAC

WORK="$(mktemp -d "${TMPDIR:-/tmp}/mppflow-core.XXXXXX")"
trap 'rm -rf "$WORK"' EXIT
CLASSES="$(OUT="$WORK/classes" bash "$HERE/../build.sh" | tail -n 1)"
"$JAVAC" -nowarn -encoding UTF-8 -cp "$CLASSES:$GUROBI_JAR" -d "$CLASSES" \
  "$HERE/CoreAccess.java" "$HERE/CoreGolden.java"
mkdir -p "$WORK/run" "$WORK/json"
(cd "$WORK/run" && "$JAVA" --add-opens java.base/java.lang=ALL-UNNAMED -cp "$CLASSES:$GUROBI_JAR" \
  projects.multipath.ILP.CoreGolden "$WORK/json" "$WORK/tmp" "$WHAT")
mkdir -p "$OUT"
for f in "$WORK"/json/*.json; do
  python3 -c 'import gzip, shutil, sys
with open(sys.argv[1], "rb") as src, gzip.GzipFile(sys.argv[2], "wb", mtime=0) as dst:
    shutil.copyfileobj(src, dst)' "$f" "$OUT/$(basename "$f").gz"
  echo "wrote $OUT/$(basename "$f").gz" >&2
done
