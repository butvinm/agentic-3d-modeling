#!/usr/bin/env bash
# Montage a directory of harness PNGs into one labelled contact sheet.
# usage: tools/sheet.sh <dir-of-pngs> [out.png] [columns]
#
# Every inspection in this project ended up hand-writing the same magick montage
# call, so it lives here instead. Each tile keeps its filename as a label, which is
# what lets a finding name the frame it is about rather than "the third one".
set -euo pipefail

DIR="${1:?usage: tools/sheet.sh <dir-of-pngs> [out.png] [columns]}"
OUT="${2:-$DIR/sheet.png}"
COLS="${3:-}"

command -v magick >/dev/null || { echo "ImageMagick (magick) is not installed" >&2; exit 1; }
[ -d "$DIR" ] || { echo "no such directory: $DIR" >&2; exit 1; }

# Sorted so the tiles run in frame order, and never including a sheet from a previous run.
FILES=()
while IFS= read -r f; do FILES+=("$f"); done < <(find "$DIR" -maxdepth 1 -name '*.png' ! -name "$(basename "$OUT")" | sort)
[ ${#FILES[@]} -gt 0 ] || { echo "no PNGs in $DIR" >&2; exit 1; }

# Squarest grid that holds them all, unless the caller asked for a width.
if [ -z "$COLS" ]; then
    COLS=1
    while [ $((COLS * COLS)) -lt ${#FILES[@]} ]; do COLS=$((COLS + 1)); done
fi

# Tiles are capped rather than scaled: '>' only shrinks, so a sheet of small crops stays legible.
magick montage -label '%f' "${FILES[@]}" \
    -tile "${COLS}x" -geometry '620x620>+3+3' \
    -background '#16181c' -fill '#99a2ad' -pointsize 13 \
    "$OUT"

echo "$OUT"
echo "${#FILES[@]} tiles in ${COLS} columns, $(magick identify -format '%wx%h' "$OUT")"
