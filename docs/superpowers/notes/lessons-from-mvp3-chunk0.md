# Lessons from MVP-3 Chunk 0 — for future plan authors

> **Type**: Lessons-learned doc, not a spec or plan.
> **Date**: 2026-04-12
> **Audience**: any future plan author writing a path-migration plan in this repo (or any plan that touches multiple files for the same logical change).
> **Triggered by**: MVP-3 Engine Package Completion roadmap spec, Section 7 (γ) commitment.

## TL;DR

**Always run `grep -rn '<old_path>' software/ hardware/ Makefile docs/` before writing the Phase F file list of any path-migration plan.** Don't trust your memory of "what files reference this path". Don't trust the spec author's enumeration. Run the grep yourself, capture every hit, fold every operational hit into the plan's edit list, and exclude historical doc references (`docs/superpowers/.*YYYY-MM-DD`) and known-vendored references (e.g., `vendor/menagerie/VENDOR.md`). Bake the grep results into the plan, AND mandate that the implementer re-runs the grep at Phase A as a drift check.

This is the **(α) "grep first"** rule from the MVP-3 Engine Package Completion roadmap spec Section 7.

## Why this rule exists — the Chunk 0 incident

MVP-3 Chunk 0 was a directory restructure: promote `hardware/elrobot/simulation/` to a three-tier first-class layout (`mujoco/elrobot_follower/`, `manifests/norma/`, shared assets). The plan was 1422 lines, written by the same author as the spec, reviewed by spec-document-reviewer 4 times, and hand-checked at multiple stages. It was a mature plan.

The plan's Phase F ("Update path references in existing files") listed 9 files where the old `hardware/elrobot/simulation/elrobot_follower.scene.yaml` and `hardware/elrobot/simulation/menagerie_so_arm100.scene.yaml` paths needed to change. The implementer (gpt-5-class subagent, opus model) executed the plan, and at Phase G.8 — the final "no dangling references" grep gate that runs `grep -rn '<old_path>' software/ hardware/ Makefile docs/` — discovered **4 stale path references that were NOT in the plan's Phase F file list**:

1. `software/sim-server/tests/integration/test_menagerie_walking_skeleton.py:34` — hardcoded fixture path. **If the implementer hadn't fixed it, Phase G.7 (`pytest test_menagerie_walking_skeleton.py`) would have shown 6 *skipped* instead of 6 *passed*** — a silent failure that wouldn't have triggered the test count gate because the total number didn't change. This is the most dangerous kind of regression: a coverage drop that looks like coverage.
2. `software/sim-server/README.md:115`, `:142`, `:144` — three stale references in a single file (the plan listed only one, the "Scenario B command example" at line 30).
3. `hardware/elrobot/simulation/mujoco/elrobot_follower/measurements/menagerie_diff.md:8` — a self-reference inside a file that was being moved into the package (the file referenced its own old location).
4. `software/station/bin/station/station-sim-menagerie.yaml:13` — a comment that mentioned the old path (not just the operational `--manifest` line that the plan correctly listed).

The implementer correctly interpreted Phase G.8's instruction "if the grep finds any operational match outside `vendor/menagerie/VENDOR.md` and `docs/superpowers/`, those are real leaks that must be fixed" and folded all 4 fixes into the same atomic commit. The Chunk 0 commit `6ef605b` ended up with 23 files changed instead of the plan-predicted 21 — the +2 delta is the 4 fixes (some lived in the same files the plan already listed, others added new files to the touched set).

## Root cause

The plan author wrote Phase F by **recall**, not by **grep**. The spec listed 9 files; the plan author trusted the spec; the spec author trusted their memory of "what references this path"; nobody ran an exhaustive `grep -rn` until Phase G.8 at the very end.

Recall is unreliable for two reasons:

1. **Comments and docstrings are easy to forget.** They're not code; they don't show up when you mentally enumerate "what reads/writes this path". But they're operational from a "grep finds it" standpoint.
2. **Self-references inside files being moved are easy to forget.** When you think "I'm moving file X from A to B", you naturally think about "what other files reference X", not "what does X reference". But X might reference its own old location (e.g., a file path in a comment or a relative-link in a markdown file).

## The (α) rule, in operation

When writing a path-migration plan:

1. **Phase A pre-flight should require the implementer to run** the same grep the plan author ran at write time. This is the drift check — if the dependent file list changed between write time and execution time (e.g., another commit on main added a new reference), the implementer catches it before doing any work and escalates.

2. **The plan author MUST run the grep at write time** and bake every operational hit into Phase F. The exhaustive grep command (with the standard exclusions) lives in the plan's "Pre-flight grep results" section so reviewers can see the same scan happen.

3. **The exclusion list must be explicit**:
   - `docs/superpowers/.*YYYY-MM-DD` — historical specs and plans naturally reference old paths
   - `vendor/menagerie/VENDOR.md` — vendored upstream that has its own update flow
   - Anything else needs case-by-case justification

4. **Phase G.8 (the final grep gate) is "Plan completeness verification", not just "implementation verification".** When it finds matches that the plan didn't anticipate, that's a plan gap, not an implementation gap. The implementer should fix the leaks AND surface the gap as a `DONE_WITH_CONCERNS` so the plan template improves over time.

5. **Don't trust the spec's file list** if the spec author also didn't grep. Even mature, multi-reviewed specs have this gap — the Chunk 0 spec went through 4 spec-document-reviewer rounds and the gap survived.

## The (α') companion rule — "baseline first"

A related lesson surfaced during MVP-3 Chunk 1 plan writing: **don't hardcode absolute test counts in plan success criteria**. Use baseline-relative deltas for cross-repo totals (`make sim-test`). The reasoning is the same as (α) but for a different failure mode:

- Recall says "the baseline is 90 passed, 1 skipped"
- Reality says "the baseline depends on whether `mujoco.mjx` is installed in dev env, and on whether other commits on main added tests since the spec was written"
- Fragility = silent failure when reality drifts from recall

The (α') rule: every chunk plan must capture `BASELINE_PASSED` and `BASELINE_SKIPPED` from `make sim-test` at Phase A pre-flight, and assert deltas only for cross-repo totals. Package-local absolute counts (e.g., `pytest mujoco/elrobot_follower/tests/` returning 4 passed + 1 skipped) are fine because the spec owns the package's test count fully.

This rule is in the MVP-3 EPC roadmap spec Section 7 as "(α')".

## Concrete protections in the MVP-3 EPC chunk plans

Both Chunk 1 (and future Chunks 2, 3) plans embed these protections:

- **Phase A.3**: re-run the grep with explicit expected output (the plan author's bake-in). If the grep returns ANY hit not in the expected list, escalate. Don't silently extend.
- **Phase E.6 (or G.8 equivalent)**: the final grep gate, MUST return empty. If not empty, those are plan-author misses or new commits on main. Either way, escalate.
- **Phase A.2**: capture `BASELINE_PASSED` and `BASELINE_SKIPPED` before any work. Use those for delta assertions in Phase E.5.
- **Phase E.5**: assert delta = +N (specific predicted delta) against baseline, NOT absolute count.

## When NOT to apply (α)

The grep-first rule is for **path migrations**. Other types of plans don't need it:

- A plan that adds a new file but doesn't move or rename anything: no grep needed, the file simply doesn't exist anywhere yet.
- A plan that edits a single file in-place without changing its path: no grep needed.
- A plan that's purely additive (new tests, new docs): no grep needed.

The grep-first rule applies only when **a path is changing** (move, rename, or delete) and **other files might reference that path**.

## Cost vs. benefit

Running the grep takes ~1 second. Baking the results into the plan adds ~5-10 lines. Re-running the grep at Phase A takes another ~1 second. **Total cost: ~10 lines of plan + ~2 seconds of execution time.**

The benefit: zero `Phase G.8` surprises, zero silent coverage drops, zero "Why is the file count 23 instead of 21?" puzzles. For Chunk 0, the implementer caught 4 leaks at the very end of the plan; if any of them had been missed (especially the silent skip in `test_menagerie_walking_skeleton.py`), a regression would have shipped to main.

## See also

- MVP-3 EPC roadmap spec Section 7: `docs/superpowers/specs/2026-04-12-mvp3-foundation-roadmap-design.md` — the canonical (α)+(α')+(γ) rules
- MVP-3 Chunk 1 plan: `docs/superpowers/plans/2026-04-12-mvp3-chunk1-assets-urdf-move.md` — concrete application of (α) and (α')
- Chunk 0 commit `6ef605b`: the historical incident
- This doc itself is the (γ) deliverable from the spec's Section 7

*End of lessons doc.*
