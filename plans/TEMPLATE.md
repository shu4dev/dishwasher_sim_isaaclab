# <experiment title>
Slug: <date>-<slug> | Status: draft / approved / done

## 0. Decisions (from grilling)
- <decision> — rejected: <alternative>, because <reason>

## 1. Hypothesis
One sentence: what we expect to see and why.

## 2. Baseline
What this is compared against (config, commit, or number).

## 3. Metrics and decision rules
- Primary metric: <name> on <split/env>
- Success: <metric> >= <number> (or baseline + <delta>)
- Kill: stop early if <condition>

## 4. Setup
- Data / env / simulator:
- Config file(s):
- Seeds: (3 unless stated)
- Compute budget: <GPUs> x <hours>, hard cap <hours>

## 5. Code changes
- <path/file.py> — <function>: <change>

## 6. Run
<exact commands>

## 7. Logging and results
- Log to: <wandb project / dir>
- Save checkpoints, plots, tables to: <path>
- Write-up: results/<date>-<slug>.md
