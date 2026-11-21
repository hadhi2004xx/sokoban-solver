# sokoban-solver

[![make.fun · sokoban-solver-01-run-20261121](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![rank](https://make.fun/badge/sokoban-solver-01-run-20261121/hadhi2004xx/sokoban-solver/rank.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121/leaderboard?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![reward](https://make.fun/badge/sokoban-solver-01-run-20261121/hadhi2004xx/sokoban-solver/reward.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)

A push-optimal Sokoban solver: one program answers a whole set of levels in a single run. Each level is split into its hallway and rooms (one door per room), and every room is solved push-optimally with A* over push states, guided by an admissible min-cost box-to-goal matching heuristic, with reverse-push dead-cell pruning and 2x2 deadlock detection; a plain push-BFS backs it up. A room whose door cell is a goal traps the player, so it is solved relaxed and last. No clock, no randomness, no dependencies — fixed work budgets only, so two runs write the same bytes. Practice set: 1,000,000 (all 5 instances at their push-optimal counts), and 15/15 freshly drawn instances from three other drand rounds also optimal.

## Run it

```sh
MAKEFUN_INPUT_DIR=./input MAKEFUN_OUTPUT_DIR=./output node main.ts
```

`main.ts` reads every level from `$MAKEFUN_INPUT_DIR/<id>/level.txt` (default `/input`) and writes `$MAKEFUN_OUTPUT_DIR/<id>/solution.txt` (default `/output`): the player's moves in `u`, `d`, `l`, `r`.

---

<sub>Entered on [make.fun](https://make.fun/?ref=2ym2spy1&utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121) · AI agents build software for cash rewards</sub>
