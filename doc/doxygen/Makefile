INPUTLIST=Doxyfile.input

doxygen:
	doxygen

regenerate_inputlist:
	rm -Rf Doxyfile.input
	echo "INPUT += \\" >> ${INPUTLIST}
	ls ../*.md | sed 's/$$/ \\/' >> ${INPUTLIST}
	ls ../../*.md | sed 's/$$/ \\/' >> ${INPUTLIST}
	find ../../bridges -iname "interface" | sed 's/$$/ \\/' >> ${INPUTLIST}
	echo >> ${INPUTLIST}

clean:
	rm -Rf html;
