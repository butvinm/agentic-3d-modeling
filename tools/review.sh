#!/usr/bin/env bash
# Render a model's turntable views and ask Codex to critique them.
# usage: tools/review.sh <model-name> [extra instructions...]
set -euo pipefail

MODEL="${1:?usage: tools/review.sh <model-name> [extra instructions...]}"
shift
EXTRA="$*"

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

SRC="models/$MODEL.c"
[ -f "$SRC" ] || { echo "no such model: $SRC" >&2; exit 1; }

FRAMES="${FRAMES:-4}"
OUT="renders/$MODEL"

make -s "build/$MODEL"
rm -rf "$OUT"
"./build/$MODEL" --shots "$OUT" --frames "$FRAMES" >/dev/null

IMAGES=()
for shot in "$OUT"/*.png; do IMAGES+=(-i "$shot"); done
[ ${#IMAGES[@]} -gt 0 ] || { echo "no renders produced" >&2; exit 1; }

read -r -d '' PROMPT <<EOF || true
You are reviewing a 3D model built procedurally with raylib in C.

Source: $SRC
Attached: $FRAMES turntable renders of the same model, taken at evenly spaced
camera angles around it. The grid squares are 1 world unit.

Judge only what the renders and the source actually show. Report:
1. Geometry defects: self-intersections, pinching, gaps or seams, inverted or
   missing faces, faceting where the surface should read as smooth.
2. Form: proportion, silhouette readability, whether the shape reads as the
   thing it is meant to be from every angle.
3. Shading: normals that look wrong, banding, lighting that hides the form.

Be specific and concrete: name the angle and the region of the image. Rank
findings by how much they hurt the model. If something looks correct, say so
plainly instead of inventing problems. Do not write or rewrite code; describe
what is wrong and what should change. $EXTRA
EOF

# Prompt goes over stdin: --image is variadic and swallows a trailing positional as another image path.
printf '%s' "$PROMPT" | codex exec --sandbox read-only "${IMAGES[@]}"
