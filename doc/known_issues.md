Known issues {#md_known_issues}
============

We had to introduce a number of hacks and workarounds to address limitations of
third-party software. Eventually, we should get rid of such hacks.



Doxygen
-------

- (**dox00**) Doxygen complains about template parameters of classes
  '@ref humoto::pepper_ik::TaskBaseCoMTracking'
  and
  '@ref humoto::pepper_ik::TaskBaseOrientation'.
  Currently, parameters are altered using 'HUMOTO_DOXYGEN_PROCESSING' macro
  in order to avoid warnings.

- (**dox01**) We use 'typedef' to define tasks ('@ref humoto::TaskALU',
  '@ref humoto::TaskGIB0', ...), but doxygen does not treat typedefs as
  classes and does not generate corresponding pages. For this reason, we
  define fake classes when the source code is processed by doxygen.



NAOqi, Aldebaran's Cross Compilation Toolkit
--------------------------------------------

- (**qi00**) '`cross-config.cmake`' file in the cross compilation toolkit
  overrides '`CMAKE_CXX_FLAGS`', hence all flags must be passed using
  '`CMAKE_C_FLAGS`'.

- (**qi01**) NAOqi adds wrappers for standard cmake functions to search for
  dependencies. If these wrappers are not used, e.g., in the bridges, it is
  necessary to provide hints for cmake's '`find_package`' manually. See
  '`BOOST_ROOT`' and '`EIGEN3_INCLUDE_DIR`' in CMakeLists.txt
