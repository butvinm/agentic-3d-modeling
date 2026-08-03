# References and review

## References come first

**When a model copies a real object, gather reference images before writing any geometry.** Without them you will invent the shape from memory, defend the invention when it is questioned, and be wrong without ever knowing it. This is not hypothetical: an earlier session modelled a HMMWV, had Codex flag its door and cab geometry in three consecutive review rounds, and dismissed it every time on the claim that the shape was "true of the real M998". That session made 114 tool calls and not one reference lookup.

```sh
tools/reference.sh humvee_v2 <image-url> <image-url> ...   # download, verify, record provenance
tools/reference.sh humvee_v2 --list
```

Find candidate images with WebSearch, then pass the image URLs to the script: it downloads them into `references/<model>/`, verifies each really decodes as an image rather than trusting the URL suffix, and appends the source URL to `references/<model>/sources.txt`. Then **Read the saved images** before modelling: seeing them is the point, saving them is only the means.

Get views that answer the questions geometry actually poses: a straight side elevation, a front and rear elevation, and a three-quarter view. A single hero shot will not tell you where a pillar meets a door.

`tools/review.sh` attaches everything in `references/<model>/` to the Codex review automatically and tells it to trust the references over the description. With no references saved it instead instructs Codex not to assert what the real object looks like, and prints a warning.

## Reviewing with Codex

`tools/review.sh` runs `codex exec --sandbox read-only -i <png>...` with a prompt naming the source file, so Codex reads the code and looks at the images together. Read-only is intentional: Codex critiques, Claude implements.

`ANIM=1` renders with `--anim` and tells Codex the frames differ by pose rather than viewpoint, so it judges whether parts stay connected as they move and treats a defect visible in only one frame as a defect. Use it for a posed model: a turntable of one frozen pose cannot answer whether a joint separates.

The prompt puts lighting, exposure, contrast, colour washout, background and antialiasing explicitly out of scope. Those belong to the harness, not to any model, and Codex otherwise reports them every single run. If a critique raises them anyway, ignore it.

**Treat the critique as evidence, not instruction, and sort findings before acting:**

- **Mesh claims** (self-intersection, clearance, gaps, inverted faces, wrong dimensions) must be checked before any code changes. Computing the answer is cheap and Codex is judging from a handful of static views. A prior run claimed a pinched or terminated tube in `models/torus_knot.c`; measuring the curve's minimum non-local self-distance gave 2.05 against the 0.90 the tube radius required, so the geometry was fine and the apparent defect was occlusion.
- **Fidelity claims** (this is not the shape the real object has) are equally falsifiable, but against a reference image rather than the mesh. Check them by looking at `references/<model>/`, and if the needed view is missing, go and get it. **Never settle a fidelity question from memory.** Your recollection of a vehicle you have never measured is not evidence, and asserting it as fact is exactly how a real defect survived three review rounds here.
- **Judgement calls** (proportion, how the form reads) cannot be settled either way. Act on them if you agree, and say that you are taking them on trust.

**A finding that survives two or more rounds must be fixed or refuted with evidence.** Re-dismissing it on the same unverified reasoning that failed last round is not a refutation. `tools/review.sh` prints the earlier critique paths after each run: read them, and treat any finding that keeps coming back as more likely to be real, not less. Repetition across independent rounds is signal.

Report which findings were verified, which were rejected and why, and which were accepted as judgement. A rejected finding is a useful result, not a failure: silently implementing a wrong critique is how the loop degrades. But record the evidence for a rejection, so the next round can check the reasoning instead of repeating it.

## Render history

`tools/review.sh` writes into the next free `renders/<model>/vN/` instead of overwriting, so earlier iterations survive for comparison. Each version directory holds the PNGs, `description.txt` (emitted by the harness from `SCENE.description`), and `critique.md` (Codex's final message, via `codex exec -o`).

`SCENE.description` should describe **what the model currently is**, not what changed this iteration: dimensions, subdivision counts, construction method. The delta between two versions is then recoverable by diffing consecutive `description.txt` files, and a description that someone forgot to update is merely incomplete rather than actively misattributing a change to the wrong version. Update it in the same edit that changes the geometry.

A description also goes stale in a way nothing checks. `models/ak47_anim.c` carried a description giving its flash floor as 45 ms when the constant was 34, and still denying that the floor reaches the renders after the note beside the constant had been corrected. When you change a constant, grep the description for the number.

`renders/` is git-ignored, so this history is a local notebook and does not survive a fresh clone.

**Inside a git worktree that means the worktree's own `renders/`**, which is exactly what running that worktree's copy of `tools/review.sh` produces. Renders stay beside the code that produced them, so a critique can still be matched to the geometry it was judging.
