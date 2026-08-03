# References and review

## References come first

**When a model copies a real object, gather reference images before writing any geometry.** Without them you will invent the shape from memory, defend the invention when it is questioned, and be wrong without ever knowing it. This is not hypothetical: an earlier session modelled a HMMWV, had Codex flag its door and cab geometry in three consecutive review rounds, and dismissed it every time on the claim that the shape was "true of the real M998". That session made 114 tool calls and not one reference lookup.

```sh
tools/reference.sh humvee <image-url> <image-url> ...   # download, verify, record provenance
tools/reference.sh humvee --list
```

Find candidate images with WebSearch, then pass the image URLs to the script: it downloads them into `references/<model>/`, verifies each really decodes as an image rather than trusting the URL suffix, and appends the source URL to `references/<model>/sources.txt`. Then **Read the saved images** before modelling: seeing them is the point, saving them is only the means.

Get views that answer the questions geometry actually poses: a straight side elevation, a front and rear elevation, and a three-quarter view. A single hero shot will not tell you where a pillar meets a door.

**Check the image can carry the measurement before making it.** Two shape claims here were wrong because they were read off a 406 px wide photograph, which cannot separate a gently steepening curve from a flat panel with a crease in it. Both were corrected by the user, and both had already reached a commit message and a `SCENE.description` by then. Quote the scale in pixels per metre before quoting a dimension, and if it is not enough to resolve the feature, go and find a better view rather than a better adjective.

**Scale a reference by a dimension you trust, then read a second one back to check it.** A side elevation of the truck came out at 403 px/m off the 3.30 m wheelbase, and reading the overall length back gave 4.54 m against a 4.57 m specification, so the scale was known good to under a percent before any claim rested on it. Overlaying a metric grid on the image with PIL and reading coordinates off that beats eyeballing proportions: it is what turned "the hood looks too long" into "1.46 m against the reference's 0.90".

**A fidelity reading can become a build-time check.** `CheckHoodProfile` (`models/humvee.c:2402`) holds six height readings taken off the reference and reports the model's deviation from them on every build. That turns the most argued-about class of finding into the same kind of measured claim as a clearance, and unlike a number in a critique it outlives the session that took it.

`tools/review.sh` attaches everything in `references/<model>/` to the Codex review automatically and tells it to trust the references over the description. With no references saved it instead instructs Codex not to assert what the real object looks like, and prints a warning.

## Reviewing with Codex

`tools/review.sh` runs `codex exec --sandbox read-only -i <png>...` with a prompt naming the source file, so Codex reads the code and looks at the images together. Read-only is intentional: Codex critiques, Claude implements.

`ANIM=1` renders with `--anim` and tells Codex the frames differ by pose rather than viewpoint, so it judges whether parts stay connected as they move and treats a defect visible in only one frame as a defect. Use it for a posed model: a turntable of one frozen pose cannot answer whether a joint separates.

The prompt puts lighting, exposure, contrast, colour washout, background and antialiasing explicitly out of scope. Those belong to the harness, not to any model, and Codex otherwise reports them every single run. If a critique raises them anyway, ignore it.

**Treat the critique as evidence, not instruction, and sort findings before acting:**

- **Mesh claims** (self-intersection, clearance, gaps, inverted faces, wrong dimensions) must be checked before any code changes. Computing the answer is cheap and Codex is judging from a handful of static views. It cuts both ways, and both directions have happened here. A run once claimed a pinched or terminated tube in a swept model; measuring the curve's minimum non-local self-distance gave 2.05 against the 0.90 the tube radius required, so the geometry was fine and the apparent defect was occlusion. The other way round, `renders/humvee/v1/critique.md` reported the half shafts detached from the hubs, and measuring showed the outer ends genuinely 0.110 m clear; `CheckHalfShaft` (`models/humvee.c:2377`) now walks the travel every build so that claim can never be argued about again.
- **Fidelity claims** (this is not the shape the real object has) are equally falsifiable, but against a reference image rather than the mesh. Check them by looking at `references/<model>/`, and if the needed view is missing, go and get it. **Never settle a fidelity question from memory.** Your recollection of a vehicle you have never measured is not evidence, and asserting it as fact is exactly how a real defect survived three review rounds here.
- **Judgement calls** (proportion, how the form reads) cannot be settled either way. Act on them if you agree, and say that you are taking them on trust.

## Three judges, each good at one thing

A build-time check settles geometry, because it computes the answer. Codex settles what reads from a rendered view, because it is a second pair of eyes on an image. The user settles taste: colour, how much dust is enough, whether a motion looks right. Both ways this loop has failed came from swapping them, and the two failures cost differently.

**Never settle an appearance question by sampling a rendered pixel.** A session judging thrown dust cropped a 60x40 patch, resized it to 1x1, read the mean colour, concluded the dust was grey rather than tan, and started re-deriving the base colour by inverting the gamma curve. The measurement could not carry that conclusion: the patch had grid lines running through it, the render is gamma-corrected on the way out (`.claude/skills/modeling/SKILL.md:79`), the supersample averages each puff against whatever is behind it, and at 0.1 alpha the sample is mostly background anyway. The session noted the contamination and acted on the number regardless. The user stopped it with "I think the dust color is ok". Make one attempt at an appearance quality, then render it and ask. That costs a single turn and is the only instrument that works.

**The two error directions do not cost the same.** A false positive burns a tuning loop the user has to interrupt. A false negative ships a defect that survives every round and is found by eye months later, or never. Spend effort on coverage (more of the model looked at, more of the travel sampled) before spending it on polish.

**A finding that survives two or more rounds must be fixed or refuted with evidence.** Re-dismissing it on the same unverified reasoning that failed last round is not a refutation. `tools/review.sh` prints the earlier critique paths after each run: read them, and treat any finding that keeps coming back as more likely to be real, not less. Repetition across independent rounds is signal.

Report which findings were verified, which were rejected and why, and which were accepted as judgement. A rejected finding is a useful result, not a failure: silently implementing a wrong critique is how the loop degrades. But record the evidence for a rejection, so the next round can check the reasoning instead of repeating it.

## Render history

`tools/review.sh` writes into the next free `renders/<model>/vN/` instead of overwriting, so earlier iterations survive for comparison. Each version directory holds the PNGs, `description.txt` (emitted by the harness from `SCENE.description`), and `critique.md` (Codex's final message, via `codex exec -o`).

`SCENE.description` should describe **what the model currently is**, not what changed this iteration: dimensions, subdivision counts, construction method. The delta between two versions is then recoverable by diffing consecutive `description.txt` files, and a description that someone forgot to update is merely incomplete rather than actively misattributing a change to the wrong version. Update it in the same edit that changes the geometry.

A description also goes stale in a way nothing checks. `models/ak47.c` carried a description giving its flash floor as 45 ms when the constant was 34, and still denying that the floor reaches the renders after the note beside the constant had been corrected. When you change a constant, grep the description for the number.

`renders/` is git-ignored, so this history is a local notebook and does not survive a fresh clone.

**Inside a git worktree that means the worktree's own `renders/`**, which is exactly what running that worktree's copy of `tools/review.sh` produces. Renders stay beside the code that produced them, so a critique can still be matched to the geometry it was judging.
