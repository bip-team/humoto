EXPORTDIR?=./doc_export

all:
	cd doxygen; ${MAKE}
	cd latex; ${MAKE}
	cp latex/documentation.pdf doxygen/html/humoto_doc.pdf

export: all
	mkdir -p ${EXPORTDIR}
	rm -Rf ${EXPORTDIR}/*.html
	rm -Rf ${EXPORTDIR}/*.html
	rm -Rf ${EXPORTDIR}/*.png
	rm -Rf ${EXPORTDIR}/*.css
	rm -Rf ${EXPORTDIR}/*.js
	rm -Rf ${EXPORTDIR}/*.pdf
	rm -Rf ${EXPORTDIR}/search/
	cp -R doxygen/html/* ${EXPORTDIR}/
	rm -Rf ${EXPORTDIR}/*.md5
	rm -Rf ${EXPORTDIR}/*.map


clean: clean-latex clean-doxygen

clean-latex:
	cd latex; ${MAKE} clean;

clean-doxygen:
	cd doxygen; ${MAKE} clean;
