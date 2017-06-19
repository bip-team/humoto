NAME=documentation
PWD=latex
TMPFOLDER=../doc-tmp

PSPDF_ARG=-dCompatibilityLevel=1.4 \
          -dPDFSETTINGS=/prepress \
          -dMAxSubsetPct=100 \
          -dSubsetFonts=true \
          -dEmbedAllFonts=true \
          -sPAPERSIZE=a4

ENV=env \
	TEXINPUTS="./Styles/:" \


build: clean
	cp -R ../${PWD} ${TMPFOLDER}
	cd ${TMPFOLDER}; ${MAKE} buildhere
	cp ${TMPFOLDER}/${NAME}.pdf ./

clean:
	rm -Rf ${TMPFOLDER}
	rm -f *.pdf

buildhere: cleanhere
#	../../tools/find_modules_doc.sh
	${ENV} latex ${NAME}.tex
	${MAKE} bibtexhere
	${ENV} latex ${NAME}.tex
	${ENV} latex ${NAME}.tex
	${ENV} latex ${NAME}.tex
	${ENV} dvips ${DVIPS_ARG} ${NAME}
	ps2pdf ${PSPDF_ARG} ${NAME}.ps

cleanhere:
	rm -f *.log *.out *.aux *.pdf *.toc *.ps *.bbl *.blg *.dvi *.sta *.stp *-blx.bib *.xml
	rm -f *.glg* *.glo* *.gls* *.glsdefs *.ist *.tlg *.ilg *.ind *.mw

bibtexhere:
	ls *.aux | xargs -n1 ${ENV} bibtex


# dummy targets
.PHONY: clean
