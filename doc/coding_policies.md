Coding Policies     {#md_coding_policies}
===============

[TOC]


Compatibility       {#md_coding_compat}
=============

* **C++ standard**: In order to avoid problems with portability we enforce
  C++98 standard with appropriate compilation flags. Note for developers: It
  appears that `clang++` is sloppier than `g++` and silently accepts code which
  results in a failure of the latter. Hence, it is recommended to verify the
  code with `g++` before pushing changes to the master branch.

* **cmake**: All cmake files and scripts should be compatible with version
  2.8.12 of `cmake`.



General policies    {#md_coding_general}
================

* Sometimes we use numbers in the names of various objects: modules, tests,
  etc. It is desirable to make these names unique not only currently,
  but also in the repository history. The simplest way to achieve this is to
  assign strictly incremental numbers. For example, if there are two tests
  `test_005` and `test_010` in a certain module, a newly added test must have
  name `test_011` and not `test_000` nor `test_006`. There still might be a
  collision with a deleted `test_011` -- such cases should also be avoided if
  possible.



Coding style        {#md_coding_style}
============
This section is loosely based on (CC-By 3.0 License)
https://google.github.io/styleguide/cppguide.html


General rules       {#md_coding_style_general}
-------------

### Tabulation

> * No tabs should be used. Indentation must include 4 spaces.


### Naming

> * Function names, variable names, and filenames should be descriptive; avoid
>   abbreviation. Types and variables should be nouns, while functions should
>   be "command" verbs.



Specific rules      {#md_coding_style_specific}
--------------

### Files

> * Filenames should be all lowercase and may include underscores `_`.
>
> * Extensions of C++ source and header files should be `.cpp` and `.h`
>   respectively.



### Type names

> Names of all types -- classes, structs, typedefs, and enums -- have the same
> naming convention:
>
> * Type names start with a capital letter and have a capital letter for each new
>   word, with no underscores: `MyExcitingClass`, `MyExcitingEnum`.



### Enumerations

> * All enumerations must be defined within some container class scope.
> * Names of the enumerators should be in upper case with underscores.



### Variables

> * Variable names are all lowercase, with underscores between words, i.e.,
>   `my_table_name`.
>
> * Global variables must be avoided. If it is not possible, their names must
>   have `g_` prefix, e.g., `g_my_global_variable`.



### Functions and methods

> * Functions should start with a lowercase letter and have a capital letter for
>   each new word without underscores.
>
> * Function names should typically be imperative, i.e., they should be commands:
>   `openFile()`.
>
> * Parameters, which do not serve as outputs, must be marked with `const`.
>
> * Output parameters must be gathered in the beginning of the list of
>   parameters:
>   `doSomething(output1, output2, input1, input2 = <default_value>)`.



### Classes

> * It is not allowed to mix definitions with different access modifiers, i.e.,
>   methods and members with the same access modifier should be gathered
>   together.
>
> * It is recommended to use access modifiers multiple times to visually separate
>   members from methods.
>
> * Names of member variables should have trailing underscores, i.e.,
>   `member_name_`.
>
> * Methods, which do not change the corresponding class, must always be marked
>   with `const`.
>
> * In general, destructors of base classes must be defined and must be
>   protected. If it is necessary to allow polymorphic destruction, the
>   destructor should be defined as public and virtual.
>
> * Constructors of base classes should be protected as well.



### Macro

> * Macro should be avoided when possible.
> * Macro name must be in all capitals with underscores.



### Namespaces

> * Names of namespaces should be in lower case with possible underscores.



### Templates

> * Names of template parameters should follow the same conventions as the
>   category they belong to (typenames, variables), but their names must include
>   `t_` prefix: `t_BaseClass`.



Visibility of symbols   {#md_coding_style_visibility}
---------------------

* We assume that implementation details should not be exposed when it is
  compiled into a library. Since it is default on many platforms, we introduced
  special macro: `HUMOTO_LOCAL`, which hides the corresponding symbol using
  appropriate compiler attributes. The opposite effect is achieved with
  `HUMOTO_API`.



Implementing tests      {#md_coding_tests}
==================
* It is preferrable to add a new regression test case to an existing related
  regression test instead of creating a new test and thus forcing creation of
  an additional binary.



Tips        {#md_coding_tips}
====

## Eigen

* In some cases, Eigen automatically creates temporary variables in order to
  avoid problems when the assigned variable participates in the expression on
  the right hand side of the assignment. This behavior can be suppressed with
  `noalias()`: `a.noalias() = b * c`. Note, however, that expressions like
  `a.noalias() = b * a` are potentially dangerous.
  See https://eigen.tuxfamily.org/dox/group__TopicAliasing.html for more
  information.
