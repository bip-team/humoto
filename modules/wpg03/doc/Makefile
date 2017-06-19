NAME=documentation
TMPFOLDER=../doc-tmp

PSPDF_ARG=-dCompatibilityLevel=1.4 \
          -dPDFSETTINGS=/prepress \
          -dMAxSubsetPct=100 \
          -dSubsetFonts=true \
          -dEmbedAllFonts=true \
          -sPAPERSIZE=a4

ENV=env \
	TEXINPUTS="../../../doc/latex/Styles/:" \

build: clean
	cp -R ../doc ${TMPFOLDER}
	cd ${TMPFOLDER}; ${MAKE} buildhere
	cp ${TMPFOLDER}/${NAME}.pdf ./

clean:
	rm -Rf ${TMPFOLDER}
	rm -f *.pdf

buildhere: cleanhere
	${ENV} latex ${NAME}.tex
	${MAKE} bibtexhere
	${ENV} latex ${NAME}.tex
	${ENV} latex ${NAME}.tex
	${ENV} dvips ${DVIPS_ARG} ${NAME}
	ps2pdf ${PSPDF_ARG} ${NAME}.ps

bibtexhere:
	ls *.aux | xargs -n1 ${ENV} bibtex


cleanhere:
	rm -f *.log *.out *.aux *.pdf *.toc *.ps *.bbl *.blg *.dvi *.sta *.stp


# dummy targets
.PHONY: clean
