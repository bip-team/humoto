Implementing tasks      {#md_howto_tasks}
==================

@tableofcontents


Types of tasks          {#md_howto_tasks_types}
==============

Tasks are composed of two main elements

> * body -- a matrix, which multiplies the vector of decision variables 'x';
> * bounds -- a vector(s) which constrain result of multiplication of the body
>   by 'x'.


Currently, there are four types of task bodies:

> * **A*x**   -- 'x' is multiplied by a dense matrix 'A';
> * **A*S*x** -- a continuous segment of 'x' is selected by matrix S and then
>   mupltiplied by a dense matrix 'A';
> * **G*I*x** -- a set of individual values of 'x' is selected by matrix 'I' and
>   then each of the values is multiplied by a corresponding scalar gain stored
>   in matrix 'G';
> * **I*x**   -- 'I' selects values of 'x', no weighting is performed, such tasks
>   are assumed to be satisifed exactly.

Note that the structured matrices 'S', 'G', 'I' are not stored or defined
explicitly. Refer to the documentation of tasks to learn how to specifiy them.


Bounds can also be of different types:
> * (L<=, <=U)  -- lower 'L' and upper 'U' bounds are defined;
> * (<=U)       -- only upper 'U' bound is defined;
> * (L<=)       -- only lower 'L' bound is defined;
> * (=B)        -- equality task: 'B = L = U';
> * (=0)        -- equality task with 'B = 0';


Various combinations of task bodies and bounds produce the following
[task types](@ref BaseTasks):

|           | Equality (=0)         | Equality (=B)        | Lower bounds (L<=)   | Upper bounds (<=U)   | Lower and upper bounds <br/> (L<=, <=U)
|-----------|-----------------------|----------------------|----------------------|----------------------|----------------------------------
| **A*x**   | @ref humoto::TaskAB0  | @ref humoto::TaskAB  | @ref humoto::TaskAL  | @ref humoto::TaskAU  | @ref humoto::TaskALU
| **A*S*x** | @ref humoto::TaskASB0 | @ref humoto::TaskASB | @ref humoto::TaskASL | @ref humoto::TaskASU | @ref humoto::TaskASLU
| **G*I*x** | @ref humoto::TaskGIB0 | @ref humoto::TaskGIB | @ref humoto::TaskGIL | @ref humoto::TaskGIU | @ref humoto::TaskGILU
| **I*x**   | @ref humoto::TaskIB0  | @ref humoto::TaskIB  | @ref humoto::TaskIL  | @ref humoto::TaskIU  | @ref humoto::TaskILU



Implementation      {#md_howto_tasks_impl}
==============

Each user-defined task must inherit from one of the task classes whose names
are given in the table in the previous section. It is also possible to derive
from one of [predefined tasks](@ref PredefinedTasks) instead.

Each user-defined task must have a unique string id (within a hierarchy) and
implement method @ref humoto::TaskBase::form(), which should

* initialize body and bounds of the task;
* multiply generated body and bounds by the task gain.



Performance     {#md_howto_tasks_perf}
===========

Choosing task type          {#md_howto_tasks_perf_type}
------------------

In order to attain the highest performance it is necessary to carefully choose
a parent class. While choosing a parent class the programmer should follow
several principles:

* Pick the most specific parent class -- in the table above the most specific
  classes are on the left and at the bottom.

* Do not perform any task transformations manually. In particular, do not
  convert upper bounds to lower bounds and vice versa, do not express two-sided
  inequalities as constraints with only lower or upper bounds. All
  transformations are performed automatically when necessary.

* If a task may change its form from one control iteration to another, pick the
  most specific parent class that represents all forms or implement multiple
  tasks.


Active set guessing         {#md_howto_task_perf_guess}
-------------------

For the sake of performance, inequality tasks may also implement
@ref humoto::TaskBase::guessActiveSet() method, which should generate guess of
active constraints. A default implementation of this method is provided for all
tasks and uses the following heuristics

* if the actual active set from the previous iteration is empty, mark all
  constraints as inactive;

* if the actual active set from the previous iteration is not empty, copy the
  actual active set from the previous iteration into guessed active set using
  methods @ref humoto::TaskBase::getActiveSetGuess() and
  @ref humoto::TaskBase::getActualActiveSet().

Alternatively, if you are working on an MPC problem, it is reasonable to shift
the active set from the previous iteration forward by the number of constraints
of the respective task in one interval, marking constraints of the last
interval as inactive.
