---
name: webp-animation
description: Turn a posed model's --anim frames into an animated WebP loop for the README. Use when asked for a gif, an animation, a loop, or a moving image of a model, or when adding a model to the README.
---

# Animated WebP from a posed model

Two steps: render the frames with `--anim`, then encode them. Everything hard about it is choosing the frame count and proving the result plays at the speed you claim.

## Use WebP, not GIF

GIF's 256-colour palette cannot carry this project's dark gradient background: every setting is a choice between banding and visible dither crosshatch. The same 60 frames came out 5.3 MB as a GIF against 1.6 MB as a WebP, and the WebP is indistinguishable from the source PNGs. GitHub has supported WebP since August 2025 (https://github.blog/changelog/2025-08-28-added-support-for-webp-images/), referenced as an ordinary `![](assets/x.webp)`.

If a GIF is ever unavoidable, pass `-gifflags -offsetting-transdiff`. Without it ffmpeg blanks unchanged pixels to transparency and crops each frame to its changed bounding box, so late frames are a bare subject on nothing and any viewer that reads frames individually shows them broken.

MP4 is smaller again (191 KB for the same frames) but GitHub only plays video from its own attachment URLs, not from a repo path, so it cannot be a committed file the way a WebP can.

## Render the frames

```sh
./build/<model> --shots renders/<model>/vN --anim --frames N --size 1200x900 --supersample 2
```

Render larger than the final loop and let ffmpeg downscale: that is a second antialiasing pass on top of `--supersample`. Add `--yaw DEG` to choose the angle without editing the model. To find a good angle cheaply, render a non-`--anim` turntable first: it orbits a frozen pose, so `--frames 24` is a 15-degree contact sheet in one run, and frame `i` is yaw `45 + 360*i/frames`.

## Choosing N

Three constraints, all of which have bitten:

**N must divide the total playback into whole milliseconds.** WebP stores per-frame delays as integer ms. A 1.25 s cycle over 60 frames wants 20.83 ms and silently rounds, drifting off the speed you intended; 50 frames at 25 ms is exact.

**N must land a frame on every brief event.** `--anim` samples N evenly spaced phases, so an event shorter than one interval is caught only by luck. `models/ak47.c` puts its three ignitions on fifths of the cycle, so any multiple of 5 catches all three and `--frames 8` catches only the first. Read the model's own note about where it put its events before picking.

**N must be fine enough to resolve the event.** `models/humvee.c` has a 0.19 s jump in a 4 s loop, so fewer than about 21 frames steps over it entirely.

## Playback speed

The honest total is the model's cycle in **real** seconds. Some models scale playback with their own constant: `models/ak47.c` has `SLOWDOWN`, and `CYCLE` is `TOTAL_T * SLOWDOWN`. Others derive everything from the cycle instead: `models/humvee.c` computes `SPEED` from `CYCLE`, so its 4 s is already real time. Read which before claiming a speed.

The README loops are deliberately **2x slower than real** so they can be followed. That is a presentation choice, not what the model does, and it means the README no longer matches the interactive window. Say so rather than letting a reader assume real time.

Keep every delay at 20 ms or more. Browsers clamp very short delays up to 100 ms, so a file that tries to be too fast plays far slower than intended.

## Encode

`-framerate` takes a rational, which is how you get delays that are not a whole number of fps:

```sh
ffmpeg -y -framerate 100/3 -i renders/<model>/vN/<model>_%02d.png \
  -vf "scale=900:-1:flags=lanczos" \
  -c:v libwebp_anim -lossless 0 -q:v 82 -compression_level 6 -loop 0 \
  renders/<model>/vN/<model>.webp
```

`100/3` gives 30 ms per frame. Frame delay in ms is `1000 * denominator / numerator`. The harness numbers files `_00`, `_01`, ... with two digits even past 100 frames, so `%02d` is right and `%03d` fails to open.

## Verify, do not trust

**ffmpeg cannot decode animated WebP.** It reports `image data not found` on a perfectly good file, which is its own decoder limitation and not evidence of a problem. Use PIL to look at frames, and read the delays out of the container:

```sh
python3 - <<'PY'
import struct, collections
p = 'renders/<model>/vN/<model>.webp'
b = open(p, 'rb').read(); i = 12; d = []
while i < len(b) - 8:
    f = b[i:i+4]; s = struct.unpack('<I', b[i+4:i+8])[0]
    if f == b'ANMF': d.append(struct.unpack('<I', b[i+20:i+23] + b'\x00')[0])
    i += 8 + s + (s & 1)
print(len(d), 'frames', dict(collections.Counter(d)), 'total', sum(d), 'ms')
PY
```

Check the total against the speed you are claiming, and extract at least one frame that should contain the brief event to confirm it survived sampling. A cheap way to prove an event landed is to count its pixels rather than to eyeball it: for a muzzle flash, count pixels with `R>200, G>120, B<160` per frame and print which frames exceed a threshold.

## Where the file goes

`renders/` is git-ignored, so a README pointing into it renders a broken image on GitHub. Copy the finished loop to `assets/<model>.webp` and reference that. The frames themselves stay under `renders/<model>/vN/` as usual.

Re-encoding at a different speed needs no re-render: the same PNGs with a different `-framerate` is the whole change.
