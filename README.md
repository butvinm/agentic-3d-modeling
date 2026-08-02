# Rendering

Models: `ak47`, `ak47_anim`, `crank_slider`, `humvee`, `humvee_v2`, `penguin`, `torus_knot`.

`crank_slider` and `ak47_anim` are posed; the rest are static.

## Build

```sh
git submodule update --init          # first clone only
make                                 # every models/*.c into build/<name>
make build/penguin                   # one model
```

## Render a model

```sh
./build/penguin --shots renders/penguin/v1 --frames 8 --size 1600x1200 --supersample 3
./build/penguin --list-parts
./build/penguin --part flippers --shots renders/penguin/v1_flippers --frames 4
```

## Render an animation

```sh
./build/crank_slider --shots renders/crank_slider/v1 --anim --frames 8 --size 1200x900 --supersample 3
./build/crank_slider --part conrod --anim --shots renders/crank_slider/v1_conrod --frames 8
```

`--anim` holds the camera and steps the pose through one cycle. Without it the frames orbit a frozen pose.

## Render and critique with Codex

```sh
tools/review.sh penguin
PART=flippers tools/review.sh penguin
FRAMES=8 tools/review.sh penguin "pay attention to the bill"
ANIM=1 tools/review.sh crank_slider
```

Writes into the next free `renders/<model>/vN/`.

## View interactively

```sh
./build/crank_slider                 # drag orbits, wheel zooms, space toggles spin, R resets, ESC quits
```
