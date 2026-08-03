#!/usr/bin/env bash
# Save reference images for a model into references/<model>/ and record where they came from.
# usage: tools/reference.sh <model-name> <url> [url...]
#        tools/reference.sh <model-name> --list
set -euo pipefail

MODEL="${1:?usage: tools/reference.sh <model-name> <url> [url...]}"
shift

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
DIR="references/$MODEL"

ListImages() {
    # -iname, and .jpeg alongside .jpg: this script normalises what it downloads, but a reference
    # copied in by hand or salvaged from a worktree keeps whatever extension it arrived with, and
    # missing one here drops it from the review silently.
    find "$DIR" -maxdepth 1 -type f \( -iname '*.png' -o -iname '*.jpg' -o -iname '*.jpeg' -o -iname '*.webp' \) 2>/dev/null | sort
}

if [ "${1:-}" = "--list" ]; then
    found="$(ListImages)"
    [ -n "$found" ] || { echo "no reference images saved for $MODEL" >&2; exit 1; }
    echo "$found"
    exit 0
fi

[ $# -gt 0 ] || { echo "give at least one image URL" >&2; exit 1; }
mkdir -p "$DIR"

NextStem() {
    local n=1 stem
    while :; do
        stem="$DIR/ref_$(printf '%02d' "$n")"
        # A partially written .download must also reserve its slot.
        set -- "$stem".*
        # So must a stem sources.txt has already spoken for.
        # The images are git-ignored but sources.txt is tracked, so a fresh clone or worktree has the provenance lines without the files.
        # Numbering from the files alone would then hand out ref_01 a second time and leave two URLs claiming one name.
        if [ ! -e "$1" ] && ! grep -q "^ref_$(printf '%02d' "$n")\." "$DIR/sources.txt" 2>/dev/null; then
            echo "$stem"; return
        fi
        n=$((n + 1))
    done
}

saved=0
for url in "$@"; do
    stem="$(NextStem)"
    tmp="$stem.download"

    if ! curl -fsSL --max-time 30 -o "$tmp" "$url"; then
        echo "FAILED to fetch: $url" >&2
        rm -f "$tmp"
        continue
    fi

    # Trust the decoded content, not the URL suffix: many image URLs end in .php or a query
    # string, and a failed fetch often lands as an HTML error page with a 200 status.
    fmt="$(magick identify -format '%m' "$tmp" 2>/dev/null | head -1 | tr '[:upper:]' '[:lower:]' || true)"
    case "$fmt" in
        png|webp) ;;
        jpeg) fmt="jpg" ;;
        *) echo "NOT an image, discarded: $url" >&2; rm -f "$tmp"; continue ;;
    esac

    mv "$tmp" "$stem.$fmt"
    printf '%s\t%s\n' "$(basename "$stem.$fmt")" "$url" >> "$DIR/sources.txt"
    echo "$stem.$fmt"
    saved=$((saved + 1))
done

[ "$saved" -gt 0 ] || { echo "nothing saved" >&2; exit 1; }
echo "$saved reference image(s) in $DIR; provenance in $DIR/sources.txt"
echo "now Read them: saving is not the point, looking at them is"
