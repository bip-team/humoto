all:
	cd doxygen; ${MAKE}
	cd latex; ${MAKE}
	cp latex/documentation.pdf doxygen/html/humoto_doc.pdf

clean: clean-latex clean-doxygen

clean-latex:
	cd latex; ${MAKE} clean;

clean-doxygen:
	cd doxygen; ${MAKE} clean;
