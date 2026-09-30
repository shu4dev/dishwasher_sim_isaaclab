MAIN := scoring_note
TT   := /media/corallab-s1/2tbhdd/brianshu/dishsim/tools/tectonic

# corallab (no system TeX): Tectonic on the 2 TB drive
here:
	mkdir -p build && TECTONIC_CACHE_DIR=$(TT)/cache $(TT)/tectonic -X compile $(MAIN).tex --outdir build

# any machine with TeX Live
latexmk:
	latexmk -pdf -interaction=nonstopmode -halt-on-error -outdir=build $(MAIN).tex

.PHONY: here latexmk
