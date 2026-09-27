# Paper: spray-exposure objectives for dishwasher loading

LaTeX source for a robotics-conference paper (ICRA / IROS format, IEEE two-column) on the
HOTEC x Frigidaire rearrangement benchmark and its spray-exposure score.

## Status

| Section | File | State |
|---|---|---|
| Scoring methodology | `sections/scoring.tex` | draft 1 (2026-09-26) |
| Abstract, introduction, related work, problem, benchmark, experiments, conclusion | — | not started (commented out in `main.tex`) |

## Build

```bash
make          # latexmk -pdf into build/ (needs a TeX Live with IEEEtran, TikZ, booktabs)
make clean
```

Or upload this folder to Overleaf (it compiles with the default TeX Live there).
The corallab box has no TeX installed; nothing here has been compiled on it yet.

## Layout

```
main.tex              document class, section order, bibliography
macros.tex            packages + notation (one place to rename a symbol)
sections/<name>.tex   one file per section
figures/tikz_*.tex    TikZ sources (no binary figures yet)
bib/references.bib    BibTeX
notes/provenance.md   every number / rule in the text -> the code or record it comes from
```

## Conventions

- One sentence per line, so diffs show sentences.
- Labels: `sec:`, `eq:`, `fig:`, `tab:`, `prop:`, `rem:` + a short name.
- Notation lives in `macros.tex`; sections never redefine symbols.
- Every number in the text has a row in `notes/provenance.md`. Change the code, update both.
- Frame convention matches the repo: dishwasher frame, metres, z up.

## Git

This folder sits inside the `dishwasher_sim_isaaclab` repo. It can be tracked there as-is or
moved out and `git init`-ed as its own repo; `build/` and LaTeX by-products are ignored by
`.gitignore`.
