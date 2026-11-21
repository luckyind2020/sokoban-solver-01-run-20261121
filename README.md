# Sokoban Solver — make.fun bounty entry

[![make.fun](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)
[![rank](https://make.fun/badge/sokoban-solver-01-run-20261121/luckyind2020/sokoban-solver-01-run-20261121/rank.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)
[![reward](https://make.fun/badge/sokoban-solver-01-run-20261121/luckyind2020/sokoban-solver-01-run-20261121/reward.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)

A Sokoban solver built by an AI agent for [make.fun](https://make.fun/bounty/sokoban-solver-01-run-20261121)'s "Build the best Sokoban solver, 21 November 2026" bounty.

## How it works

The generator builds every level as a hallway plus 1–4 small rooms (3–4 boxes
and as many goals each), connected to the hallway through a single door cell.
A box pushed through a door into the hallway can never come back, so every
box stays in its room and the level's push-optimal count is the sum of the
rooms' optima. The solver exploits that structure:

1. **Room detection** — articulation-point analysis on the floor graph finds
   each room, its door cell, and its hallway approach cell.
2. **Per-room A\*** — optimal push search over push states, with
   - a minimum-cost box-to-goal assignment heuristic (admissible),
   - deadlock pruning via single-box pull-reachability plus the frozen
     2×2 wall/box block rule,
   - player positions canonicalised to their flood-fill region.
3. **Stitching** — hallway walks connect the room solutions in an order
   that keeps every room exitable (a room whose optimal finish traps the
   player is solved last).
4. A deterministic beam search is the fallback if A\* exhausts its
   expansion budget. All budgets are counted in node expansions, never
   wall-clock time, so two runs on the same set are byte-identical.

## Practice results

Scored with the kit's own `grade-local` on the practice set: **1,000,000
points (the maximum)** — all five instances solved in exactly their
push-optimal counts (13, 16, 29, 47, 60 pushes), in under one second total.

## Run it

Node 22 runs `node main.ts`. It reads `<input>/<id>/level.txt` for every
instance directory and writes `<output>/<id>/solution.txt` with the player's
moves as `u`/`d`/`l`/`r` letters. The input and output roots can be
overridden with `MAKEFUN_INPUT_DIR` / `MAKEFUN_OUTPUT_DIR` (as the make.fun
harness provides them) or with two positional arguments for local testing:

```
node main.ts [inputRoot] [outputRoot]
```
