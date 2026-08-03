#!/usr/bin/env bash
# Render a model's turntable views into the next renders/<model>/vN/ and ask Codex to critique them.
# usage: tools/review.sh <model-name> [extra instructions...]
# env:   FRAMES=4  PART=<part-name>  ANIM=1  PHASE=<0..1>  YAW=<degrees>
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
ANIM="${ANIM:-}"
YAW="${YAW:-}"
PHASE="${PHASE:-}"

if [ -n "$ANIM" ] && [ -n "$PHASE" ]; then
    echo "ANIM and PHASE contradict: one steps the pose, the other freezes it" >&2
    exit 1
fi

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

FRAMING="taken at evenly spaced camera angles around it"
if [ -n "$ANIM" ]; then
    SHOT_ARGS+=(--anim)
    FRAMING="taken from ONE fixed camera at evenly spaced steps through a single cycle of its motion, so what changes between frames is the pose and not the viewpoint.
Judge whether the parts stay connected as they move: a joint that separates, a part
that passes through another, or a travel that runs past its own guide will show up
in some frames and not others. A defect visible in only one frame is still a defect."
fi

if [ -n "$PHASE" ]; then
    SHOT_ARGS+=(--phase "$PHASE")
    FRAMING="taken at evenly spaced camera angles around it, with the pose frozen at one
single instant of its cycle, so every frame shows that same instant from a different
side. Judge the pose itself rather than the motion: a part that has travelled outside
the guide it runs in, or clear of the parent it is meant to stay buried in, is visible
from some angles and hidden from others."
fi

if [ -n "$YAW" ]; then
    SHOT_ARGS+=(--yaw "$YAW")
fi
"./build/$MODEL" "${SHOT_ARGS[@]}" >/dev/null

IMAGES=()
for shot in "$OUT"/*.png; do IMAGES+=(-i "$shot"); done
[ ${#IMAGES[@]} -gt 0 ] || { echo "no renders produced" >&2; exit 1; }

REFS=()
while IFS= read -r ref; do REFS+=("$ref"); IMAGES+=(-i "$ref"); done < <(
    find "references/$MODEL" -maxdepth 1 -type f \( -iname '*.png' -o -iname '*.jpg' -o -iname '*.jpeg' -o -iname '*.webp' \) 2>/dev/null | sort
)

REF_NOTE=""
if [ ${#REFS[@]} -gt 0 ]; then
    REF_NOTE="
The last ${#REFS[@]} attached image(s) are NOT renders. They are reference
photographs of the real object this model is copying. Compare the renders
against them and report where the model's shape departs from the real thing.
Trust the references over the description if they disagree."
else
    REF_NOTE="
No reference images were supplied. Do not invent details of the real object
from memory: if a fidelity question matters here, say which reference view the
author needs to obtain rather than asserting what the real thing looks like."
fi

PRIOR=""
for old in "$BASE"/v*/critique.md; do
    [ -e "$old" ] || continue
    [ "$old" = "$OUT/critique.md" ] && continue
    PRIOR="$PRIOR $old"
done

read -r -d '' PROMPT <<EOF || true
You are reviewing a 3D model built procedurally with raylib in C.

Source: $SRC
Attached: $FRAMES renders of $SUBJECT, $FRAMING
The grid squares are 1 world unit.
$( [ -f "$OUT/description.txt" ] && echo "
What the model is meant to be:
$(cat "$OUT/description.txt")" )

$REF_NOTE

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

State every finding physically before you interpret it. Open each one with a
plain sentence naming the parts involved and what they are visibly doing wrong,
as you would point it out to someone standing next to you: "the door sticks out
from behind the windscreen", "the mirror arm ends in mid-air short of the body",
"the rear wheel sits higher off the ground than the front". Only after that
sentence may you explain what it makes the model read as. A finding whose
headline is an impression rather than a physical description, such as "the
silhouette reads as a generic pickup" or "the form lacks character", is not
usable: the author cannot act on it. If that is genuinely all you see, name the
specific edges, panels or gaps that produce the impression.

Separate what you can prove from what you are inferring. Label each finding as
one of: a measurable claim about the mesh that the author can check in the
source; a fidelity claim about the real object being modelled, which the author
must check against a reference image; or a judgement about how the form reads.
Be specific: name the angle and the region of the image. Rank findings by how
much they hurt the model. If something looks correct, say so plainly instead of
inventing problems. Do not write or rewrite code; describe what is wrong and
what should change. $EXTRA
EOF

printf '%s' "$PROMPT" | codex exec --sandbox read-only -o "$OUT/critique.md" "${IMAGES[@]}"
echo
echo "renders + critique: $OUT"
[ ${#REFS[@]} -gt 0 ] && echo "compared against ${#REFS[@]} reference image(s) in references/$MODEL" \
                      || echo "NO reference images: run tools/reference.sh $MODEL <url> before trusting any fidelity claim"
if [ -n "$PRIOR" ]; then
    echo "earlier critiques:$PRIOR"
    echo "a finding that also appears in those must be fixed or refuted with evidence, not dismissed again"
fi
