MAKE_FLAGS?=-j5


CMAKE_DIR=./cmake/
BUILD_DIR=./build/
ROOT_DIR=../../


# Toolchains
TC?=generic

OPTIONS?=default

# Build type as in CMAKE_BUILD_TYPE
TYPE?=Debug

TARGETS?=all

#----------------------------------------------
# Cleaning
#----------------------------------------------

clean:
	cd ./doc; ${MAKE} ${MAKE_FLAGS} clean;
	find modules/ -name "doc" | sed -e 's/^/cd /' -e "s/$$/ \&\& ${MAKE} ${MAKE_FLAGS} clean \&\& cd ..\/..\/..\//" | sh
	rm -Rf build;

forceclean: clean
	rm -Rf bridges/qpOASES/qpOASES/*
	rm -Rf bridges/googletest/googletest/*
	rm -Rf bridges/yaml_cpp/yamp-cpp/*
	rm -Rf bridges/rbdl/rbdl/*
	rm -Rf bridges/QuadProgpp/QuadProgpp/*


#----------------------------------------------
# Generic targets
#----------------------------------------------

BUILD_SUBDIR=${BUILD_DIR}/${TC}-${TYPE}-${OPTIONS}

build:
	mkdir -p ${BUILD_SUBDIR};
	cd ${BUILD_SUBDIR}; cmake 	-C ${ROOT_DIR}/${CMAKE_DIR}/options_${OPTIONS}.cmake \
								-DCMAKE_BUILD_TYPE=${TYPE} \
								-DCMAKE_TOOLCHAIN_FILE=${CMAKE_DIR}/toolchain_${TC}.cmake \
								${EXTRA_CMAKE_PARAM} \
								${ROOT_DIR};
	cd ${BUILD_SUBDIR}; ${MAKE} ${MAKE_FLAGS} ${TARGETS}

build-tests: build
	cd ${BUILD_SUBDIR}; ${MAKE} ${MAKE_FLAGS} test

# -------

FETCH_DIR=${BUILD_DIR}/fetch

fetch:
	mkdir -p ${FETCH_DIR};
	cd ${FETCH_DIR}; cmake -C ${ROOT_DIR}/${CMAKE_DIR}/options_all.cmake ${ROOT_DIR};
	cd ${FETCH_DIR}; ${MAKE} fetch-bridges;


#----------------------------------------------
# debug mode (all)
# Build & test
#----------------------------------------------

debug-all:
	${MAKE} build TC=${TC} TYPE=Debug OPTIONS=all TARGETS="${TARGETS}"

debug-all-tests:
	${MAKE} build-tests TC=${TC} TYPE=Debug OPTIONS=all TARGETS="${TARGETS}"

all: debug-all

tests: debug-all-tests

debug: debug-all


#----------------------------------------------
# debug mode (default)
# Build & test
#----------------------------------------------

debug-default:
	${MAKE} build TC=${TC} TYPE=Debug OPTIONS=default TARGETS="${TARGETS}"

debug-default-tests:
	${MAKE} build-tests TC=${TC} TYPE=Debug OPTIONS=default TARGETS="${TARGETS}"


#----------------------------------------------
# release mode
# Build & test
#----------------------------------------------

release-all:
	${MAKE} build TC=${TC} TYPE=Release OPTIONS=all TARGETS="${TARGETS}"

release-all-tests: release-all
	${MAKE} build-tests TC=${TC} TYPE=Release OPTIONS=all TARGETS="${TARGETS}"

release: release-all


#----------------------------------------------
# checks
#----------------------------------------------

check-doc:
	cd doc/doxygen/; ${MAKE}
	cd doc/latex/; ${MAKE}
	cd modules/wpg03/doc; ${MAKE}
	cd modules/wpg04/doc; ${MAKE}
	cd modules/pepper_mpc/doc; ${MAKE}
	cd modules/pepper_ik/doc; ${MAKE}


as-check-build: clean
	${MAKE}	debug-all-tests
	${MAKE}	debug-all-tests TC=gcc EXTRA_CMAKE_PARAM="${EXTRA_CMAKE_PARAM} -DHUMOTO_ENABLE_THREADS_FOR_LOGGING=OFF"

as-check: check-doc as-check-build


#----------------------------------------------
# other
#----------------------------------------------

update:
	git submodule update

cppcheck:
	cppcheck -q --inconclusive --enable=all `find ./core/ ./modules/ ./bridges/*/interface/ -iname "*.cpp" -or -iname "*.h"`

.PHONY: clean cmake build
