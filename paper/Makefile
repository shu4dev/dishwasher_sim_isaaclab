LATEXMK ?= latexmk
MAIN    := main

all:
	$(LATEXMK) -pdf -interaction=nonstopmode -halt-on-error -outdir=build $(MAIN).tex

clean:
	$(LATEXMK) -C -outdir=build $(MAIN).tex

.PHONY: all clean
