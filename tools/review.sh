#!/usr/bin/env bash
# Render a model's turntable views into the next renders/<model>/vN/ and ask Codex to critique them.
# usage: tools/review.sh <model-name> [extra instructions...]
# env:   FRAMES=4  PART=<part-name>
set -euo pipefail

MODEL="${1:?usage: tools/review.sh <model-name> [extra instructions...]}"
shift
EXTRA="$*"

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

SRC="models/$MODEL.c"
[ -f "$SRC" ] || { echo "no such model: $SRC" >&2; exit 1; }

FRAMES="${FRAMES:-4}"
PART="${PART:-}"

BASE="renders/$MODEL"
mkdir -p "$BASE"
N=1
while [ -e "$BASE/v$N" ]; do N=$((N + 1)); done
OUT="$BASE/v$N"

make -s "build/$MODEL"

SHOT_ARGS=(--shots "$OUT" --frames "$FRAMES")
SUBJECT="the whole model"
if [ -n "$PART" ]; then
    SHOT_ARGS+=(--part "$PART")
    SUBJECT="the \"$PART\" part in isolation, with the rest of the model hidden"
fi
"./build/$MODEL" "${SHOT_ARGS[@]}" >/dev/null

IMAGES=()
for shot in "$OUT"/*.png; do IMAGES+=(-i "$shot"); done
[ ${#IMAGES[@]} -gt 0 ] || { echo "no renders produced" >&2; exit 1; }

read -r -d '' PROMPT <<EOF || true
You are reviewing a 3D model built procedurally with raylib in C.

Source: $SRC
Attached: $FRAMES turntable renders of $SUBJECT, taken at evenly spaced camera
angles around it. The grid squares are 1 world unit.
$( [ -f "$OUT/description.txt" ] && echo "
What the model is meant to be:
$(cat "$OUT/description.txt")" )

Out of scope, do not report: lighting, exposure, contrast, shadow darkness,
colour washout, background, antialiasing and image resolution. All of these
come from a shared render harness that every model uses, not from this model,
and the author cannot fix them by changing the geometry.

In scope:
1. Geometry defects: self-intersections, pinching, gaps or seams, inverted or
   missing faces, parts that float apart or interpenetrate where they should meet.
2. Form: proportion, silhouette readability, whether the shape reads as the
   thing it is meant to be from every angle.
3. Shading only where it reveals a geometry defect: inconsistent or inverted
   normals, faceting caused by too few subdivisions, smoothing seams. Judge the
   mesh, not the lights.

Separate what you can prove from what you are inferring. For each finding say
whether it is a measurable geometry claim the author can verify in the source,
or a judgement about how the form reads. Be specific: name the angle and the
region of the image. Rank findings by how much they hurt the model. If something
looks correct, say so plainly instead of inventing problems. Do not write or
rewrite code; describe what is wrong and what should change. $EXTRA
EOF

printf '%s' "$PROMPT" | codex exec --sandbox read-only -o "$OUT/critique.md" "${IMAGES[@]}"
echo
echo "renders + critique: $OUT"
