This is Humoto - a software framework for manipulation of linear least-squares
problems with (in)equality constraints. It supports both weighting and
lexicographic prioritization and can be characterized as a tool for goal
programming. However, the development was driven by works in other fields -
robotics, control, and numerical optimization; for this reason our terminology
and interpretations are different.


The core functionalities of Humoto are formulation of least-squares problems
and their resolution with the help of various third-party software using
uniform programming interfaces. However, due to our interest in robotic
applications, the framework also facilitates formulation and implementation of
optimization problems for control of robots, in particular, model predictive
control problems. For the same reason, we pay special attention to
computational performance in order to be able to employ the framework in
real-time scenarios.


In addition to the core components, the distribution of Humoto includes several
modules - implementations of specific controllers. The modules serve as
examples of using the framework, but can also be used in accordance with their
primary purpose.


@tableofcontents


Accessing Humoto    {#md_readme_access}
================
Humoto is distributed under the terms of Apache 2.0 license, see
[LICENSE](@ref LICENSE) for more information. All files are covered by the same
license except those which were borrowed from other projects as indicated in
[INSTALL.md](@ref md_install) file.


The source code can be obtained using git:
> * `git clone https://github.com/bip-team/humoto`



Learning Humoto     {#md_readme_learn}
===============

The framework is distributed with two kinds of documentation:

* [PDF manual](https://bip-team.github.io/humoto/humoto_doc.pdf) overviews the
  main concepts and contains mathematical derivations. If the precompiled
  version is not available, you have to go to '`doc/latex`' and execute
  '`make`' to compile it.

* Doxygen documentation for developers which incorporates a number of files in
  `markdown` format stored in the root and '`doc`' directories. It can be
  compiled by calling '`make`' in '`doc/doxygen`' folder.

Precompiled documentation is available at https://bip-team.github.io/humoto/.

The first step is to read the introductory chapter of LaTeX documentation,
then, depending on your interests, you may

1.  learn about
    - the **authors** from [AUTHORS.md](@ref md_authors)
    - the directory **layout** from [doc/directory_layout.md](@ref md_directory_layout)
    - **dependencies** and **compilation** of tests from file [INSTALL.md](@ref md_install)


2.  **use** one of the provided modules
    * you can find an example in [modules/wpg04/tests/test_000.cpp](@ref wpg04_example)
    * relevant mathematical derivations can be found in the respective chapters of
      LaTeX documentation


3.  **modify** the core or a module

    * you should first read the **coding policies** in
      [doc/coding_policies.md](@ref md_coding_policies)

    * then you can learn how to
        - implement new tasks
          [doc/howto_tasks.md](@ref md_howto_tasks)

        - add support of configuration files to your classes in
          [doc/howto_configurable.md](@ref md_howto_configurable)



FAQ     {#md_readme_faq}
===


What are the existing applications of this framework?   {#md_readme_faq_applications}
-----------------------------------------------------

* Early versions of Humoto were used for walking control of HRP-4 humanoid
  robot. See module [wpg03](@ref wpg03), its LaTeX documentation and references
  therein, and a [video](https://www.youtube.com/watch?v=VYwZU4_7sMA) of the
  robot during the trials.

* More recently, Humoto has been used for motion control of a humanoid robot on
  a wheeled base (Pepper). See modules [pepper_mpc](@ref pepper_mpc) and
  [pepper_ik](@ref pepper_ik) and their LaTeX documentation.



What is the difference between humoto and ...   {#md_readme_faq_comparison}
---------------------------------------------

* [**RobOptim**](http://roboptim.net/)?
    Humoto and RobOptim were developed with one similar goal in mind:
    interfacing various optimization solvers in robot control applications.
    There are, however, many differences. Humoto cannot deal with general
    nonlinear problems as RobOptim does, but, unlike RobOptim, Humoto supports
    hierarchical (lexicographic) optimization problems. Moreover, our framework
    standardizes and automates formulation of optimization problems using the
    concept of '[task](@ref md_howto_tasks)'. Finally, we pay more attention to
    performance, e.g., Humoto supports more hot-starting options and provides
    [functionality](@ref etools) for blockwise matrix operations with sparsity
    support.


* [**SOTH**](https://github.com/stack-of-tasks/soth) (SOT Hierarchical solver )?
    Humoto is a framework which is designed to provide common interface to
    various solvers, including hierarchical (lexicographic) solvers, while
    SOTH is a particular hierarchical solver.



More information    {#md_readme_faq_more}
================
* Todo: see [doxygen todo page](@ref todo) and [doc/known_issues.md](@ref md_known_issues)
