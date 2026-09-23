#!/bin/bash
##########################################################################
# Builds the abv_gnc Sphinx documentation locally.
#
# Usage:
#   ./build.sh          # build docs into build/html
#   ./build.sh --open   # build, then open build/html/index.html
#
# Must be run from the docs/ folder, or from anywhere via docs/build.sh.
##########################################################################

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

python3 -m pip install --quiet -r requirements.txt

# Diagram sources live in diagrams/*.puml and are committed pre-rendered to
# source/_static/, so a missing `plantuml` (e.g. on ReadTheDocs) just skips
# regeneration rather than failing the build.
if command -v plantuml >/dev/null 2>&1; then
    mkdir -p source/_static
    plantuml -tpng diagrams/*.puml -o "$SCRIPT_DIR/source/_static"
else
    echo "plantuml not found on PATH; skipping diagram regeneration (using committed images in source/_static/)."
fi

rm -rf build/html
sphinx-build -b html source build/html

echo ""
echo "Docs built: $SCRIPT_DIR/build/html/index.html"

if [[ "$1" == "--open" ]]; then
    xdg-open build/html/index.html 2>/dev/null || open build/html/index.html 2>/dev/null || \
        echo "Could not auto-open a browser; open build/html/index.html manually."
fi
