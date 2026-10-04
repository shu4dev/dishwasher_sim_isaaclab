# Scoring note

Short note on the spray-exposure score S: Section 1 the idea in five steps, Section 2 the math of each step,
each with one figure that draws its symbols.

- Build here: `make here` -> `build/scoring_note.pdf` (Tectonic on the 2 TB drive; VS Code LaTeX Workshop also builds on save).
- Build elsewhere: `make latexmk`, or upload this folder to Overleaf.
- Figures: `python3 figures/make_figures.py` (host) redraws Figs. 1-5 from `data/build/scoring_note/figdata.npz`
  at the repo root; recreate that data with `code/util/run_py.sh docs/scoring_note/figures/extract_figdata.py`
  (container; prints a base64 npz between NPZ_BEGIN / NPZ_END). Fig. 2 (b, c) are TikZ files in `figures/`.
- Source of every rule and number: `code/frigidaire/src/dishsim_frigidaire/exposure.py` (revision 5).
