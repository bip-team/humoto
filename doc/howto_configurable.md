Implementing configurable classes   {#md_howto_configurable}
=================================

The framework provides functionality for serialization/deserialization of
classes to/from configuration files.


In order to implement a configurable class, it is necessary to take two steps:

* Inherit from [ConfigurableBase](@ref humoto::config::ConfigurableBase) class,
  which provides functions to work with configuration files, if they are
  enabled with corresponding cmake option.

* Implement pure virtual methods of
  [ConfigurableBase](@ref humoto::config::ConfigurableBase) class:

    - Some of these functions can be generated automatically using X-macro
      technique, if directive '`#include HUMOTO_CONFIG_DEFINE_ACCESSORS`' is added
      (it includes [define_accessors.h](@ref define_accessors.h) header).

    - Others must be implemented manually.


Optionally it is possible to

* Define additional constructors to initialize class from a configuration file
  using `HUMOTO_DEFINE_CONFIG_CONSTRUCTORS` macro.

* Postprocess class members after reading them from a configuration file -- in
  this case the programmer should provide appropriate implementation of
  @ref humoto::config::CommonConfigurableBase::finalize() virtual function.
