Dependencies and compilation    {#md_install}
============================

System dependencies
-------------------

- utilities
    - cmake
    - git and/or SVN to fetch bridges (3-rd party software)
    - C++98 compatible compiler

- libraries
    - Eigen 3               (MPL2 -- Mozilla Public License)
    - boost                 (Boost Software License)
        - shared_ptr.hpp
        - lexical_cast.hpp
        - detection of Eigen types in templates
            - utility/enable_if.hpp
            - mpl/has_xxx.hpp
            - mpl/and.hpp
        - [optional] with HUMOTO_ENABLE_THREADS_FOR_LOGGING=YES
            - thread.hpp


<br />
Dependencies which are fetched, compiled and installed by humoto
----------------------------------------------------------------

Humoto does not use system-wide versions the following packages to avoid
compatibility issues. However, they are installed with humoto -- be careful to
avoid conflicts.


| name          | purpose                   | importance                                | license               |
|---------------|---------------------------|-------------------------------------------|-----------------------|
| rbdl          | dynamics and kinematics   | mandatory for [pepper_ik](@ref pepper_ik) | zlib                  |
| yaml-cpp*     | YAML support              | optional                                  | MIT                   |
| googletest    | tests                     | optional                                  | 3-clause BSD          |
| qpOASES       | QP solver                 | optional                                  | LGPL 2.1              |
| QuadProgpp    | QP solver                 | optional                                  | MIT                   |
| LexLS         | hierarchical solver       | not distributed                           | non-public software   |
| eiQuadProg    | QP solver                 | not distributed                           | GPL                   |


(*) An old version of yaml-cpp is used in order to avoid dependency on C++11.



<br />
Dependencies included in the main distribution
----------------------------------------------

### cmake scripts

| name              | purpose                               | importance    | license       |
|-------------------|---------------------------------------|---------------|---------------|
| TargetArch.cmake  | detection of the target architecture  | mandatory     | 2-clause BSD  |
| FindEigen3.cmake  | searching for Eigen                   | mandatory     | 2-clause BSD  |


### Other

| name              | purpose                       | importance                                | license               |
|-------------------|-------------------------------|-------------------------------------------|-----------------------|
| pepper_`*`.urdf   | URDF models of Pepper robot   | mandatory for [pepper_ik](@ref pepper_ik) | Apache 2.0            |




<br />
Compilation & installation
--------------------------

Compilation of tests and / or installation of headers is performed using cmake.
While it is possible to use cmake directly it is recommended to use Makefile
which automates common tasks.


Makefile abides the following conventions:

    - Compilation is performed in subdirectories of 'build/'. Names of
      subdirectories consist of three parts: [toolchain]-[buildtype]-[options].

    - Toolchains indicate cmake presets (compilers, etc). Currently, two
      toolchains are provided: 'generic', which is completely empty, and 'gcc'
      which sets the compiler to gcc.

    - Build type directly corresponds to CMAKE_BUILD_TYPE, i.e., 'Debug',
      'Release', etc.

    - Options indicate presets of humoto. Currently there are two presets:
      'all', which enables all modules and brdiges, and 'default', which sets
      default options (LexLS disabled, RBDL disabled ...).


In order to compile humoto with particular settings it is necessary to call
    'make build TC=[toolchain] TYPE=[buildtype] OPTIONS=[options]'
or
    'make build-tests TC=[toolchain] TYPE=[buildtype] OPTIONS=[options]'
which runs regression tests after compilation.


For convenience, several shorthand targets are provided, for example

    - make all
        'make build TC=generic TYPE=Debug OPTIONS=all'

    - make tests
        'make build-tests TC=generic TYPE=Debug OPTIONS=all'

    - make release-all
        'make build TC=generic TYPE=Release OPTIONS=all'

    - make release-all-tests
        'make build-tests TC=generic TYPE=Release OPTIONS=all'

    - make debug-default
        'make build TC=generic TYPE=Debug OPTIONS=default'

    - make debug-default-tests
        'make build-tests TC=generic TYPE=Debug OPTIONS=default'

Please refer to the Makefile for complete list. Note that the toolchain can be
overriden for the specified targets, i.e. 'make all TC=gcc' is acceptable.


If you are working on a particular module you can save time by compiling
specific tests only. This can be achieved by adding `TARGETS` variable when
calling make, e.g.:

    make build TC=generic TYPE=Debug OPTIONS=all TARGETS=[test_id]

or

    make all TARGETS=[test_id]

where test_id is `[module]_regression_test_[test_number]` for regression tests
or '[module]_test_[test_number]` for other tests:

    make all TARGETS=wpg04_test_000

Alternative option is to go to

    build/[toolchain]-[buildtype]-[options]

and execute

    make [test_id]

for example:

    cd build/generic-Debug-all

    make wpg04_test_000
