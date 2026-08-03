# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this project is

An experiment in building 3D models procedurally with raylib, where Claude Code writes the geometry and Codex judges the result from rendered images. Claude cannot evaluate its own output the way a viewer would, so the loop is: write a model, render turntable views to PNG, hand those PNGs to Codex as a second pair of eyes, act on the critique.

Each model is a single self-contained C file in `models/`. A shared harness owns the window, camera, lighting and screenshot machinery, so a model file contains nothing but geometry.

## Skills

The detail lives in `.claude/skills/`, loaded when it applies:

- **modeling** covers writing or editing any `models/*.c`: the `Scene` contract, splitting a model into inspectable parts, and the raylib rules a mesh must obey. Its `posing.md` covers articulated models, and its `review.md` covers gathering reference images and acting on a Codex critique.
- **webp-animation** covers turning a posed model's `--anim` frames into a loop for the README.

## Commands

```sh
make                                 # build every models/*.c into build/<name>
make build/humvee                    # build one model
make clean                           # remove build/ and renders/
make raylib                          # build the vendored static lib (auto-run on first build)
make clean-raylib                    # force a full raylib rebuild

./build/humvee                       # interactive window: drag to orbit, wheel zooms, space toggles spin, [ and ] set playback speed, P pauses the pose, R resets
./build/humvee --list-parts          # print this model's part names
./build/humvee --part running_gear   # inspect one part, framed to its own bounds
./build/humvee --shots out --frames 6 --size 1600x1200 --supersample 3
./build/humvee --shots out --anim --frames 25        # step the pose instead of orbiting the camera
./build/humvee --shots out --yaw 140                 # choose the camera angle without editing the model

./tools/review.sh humvee             # build, render into the next renders/humvee/vN/, critique with Codex
PART=running_gear ./tools/review.sh humvee
FRAMES=8 ./tools/review.sh humvee "pay attention to the wheel arches"
ANIM=1 ./tools/review.sh humvee
```

`--shots` renders N evenly spaced turntable views and exits, printing each written path. Without it the binary opens a window.

`--anim` changes what those N frames vary: the camera holds still at `SCENE.animYaw` and the pose steps through one `SCENE.duration`. It is ignored, with a warning, on a scene that declares no `update`.

`--yaw` overrides both: the angle `--anim` holds, the angle a turntable starts from, and the window's opening angle. It exists so `SCENE.animYaw` stays the angle a review is judged at.

`[` and `]` step the window's playback speed down and up by 0.1, from 0.1x to 4.0x, and both auto-repeat when held. `P` pauses the pose where it is; the speed's bottom stop is 0.1x rather than zero, so stopping stays something you ask for explicitly. They belong to the window and change nothing that is rendered: `--anim` samples N evenly spaced phases of `SCENE.duration` regardless, so the same frames come out at any speed. The HUD shows the current speed and where the pose is in its cycle. A scene with no `update` gets neither the keys nor that line, since it has no pose to slow down.

The window opens at `SCENE.previewSpeed`, which `R` also returns to, and both models set it to 0.6 because both run too quickly to follow at real time. **Slowing a preview is what that field is for, and scaling `SCENE.duration` is not the way to do it**: `models/humvee.c:1771` derives the truck's road speed from `CYCLE`, so a longer cycle there is a truck driving slower, and it changes every rendered frame with it.

After a fresh clone: `git submodule update --init` then `make`.

## Renders belong to the working tree, never to a scratchpad

They are this project's output and its notebook, not temporary files, so write them under `renders/<model>/` and nowhere else. This overrides any general instruction to put working files in a session scratchpad directory: a render dropped in `/tmp` is invisible to the next version comparison and is deleted without anyone noticing. `tools/review.sh` already gets this right from any directory, because it resolves the repo root from its own path (`tools/review.sh:11`) and cds there before rendering. When you drive the binary yourself, pass `--shots renders/<model>/<something>` explicitly.

## Worktrees

Branch work may happen in a worktree under `.claude/worktrees/<name>/`. **Delete the worktree and its branch as soon as the branch is merged**, in the same session that merges it. The repo should sit at one worktree and one branch between tasks. Two abandoned worktrees accumulated ~190 MB here, mostly duplicated `vendor/raylib` build output, before anyone looked.

Salvage before removing, because `renders/` and `references/` are git-ignored and so do not travel with the merge. The commits are safe; the notebook and the downloaded reference photographs are not.

```sh
cp -rn <wt>/renders/<model> renders/<model>                 # review history exists only in the worktree
cp -n  <wt>/references/<model>/* references/<model>/         # every extension: .gitignore covers jpg, jpeg, png and webp
git worktree unlock <name>                                   # only if a session locked it
git worktree remove --force <wt>
git branch -d <branch>
```

`--force` is not optional here: plain `git worktree remove` fails with `fatal: working trees containing submodules cannot be moved or removed` because of `vendor/raylib`. Since `--force` also discards uncommitted work, check `git status --porcelain` inside the worktree first and stop if it prints anything.

Delete the branch with `git branch -d`, never `-D`. The lowercase form refuses to delete a branch that is not merged, so a successful `-d` is itself the proof that nothing is being orphaned.

If a worktree is locked, read `.git/worktrees/<name>/locked` before unlocking it: the lock records the session that owns it, and that session may still be running. Check the pid with `ps -p <pid>` and leave the worktree alone if it is alive.

## Conventions

- raylib is pinned at tag **5.5** (`vendor/raylib`, commit `c1ab645`). Chosen over 6.0 because 5.5 is what the cheatsheet, the examples in `vendor/raylib/examples/`, and model knowledge actually cover. `vendor/raylib/examples/` is the best local reference for raylib API usage.
- `build/` and `renders/` are generated and git-ignored. `assets/` is tracked, and holds the loops the README embeds.
