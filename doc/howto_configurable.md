Implementing configurable classes   {#md_howto_configurable}
=================================

The framework provides functionality for serialization/deserialization of
classes to/from configuration files.


In order to implement a configurable class, it is necessary to take two steps:

* Inherit from [ConfigurableBase](@ref humoto::config::ConfigurableBase) class,
  which provides functions to work with configuration files, if they are
  enabled with corresponding cmake option.

* Add '`#include HUMOTO_CONFIG_DEFINE_ACCESSORS`' directive inside a class
  which includes [define_accessors.h](@ref define_accessors.h) file. This
  automatically generates functions which are specific for each class or cannot
  be inherited for technical reasons. Three optional defines can precede this
  inclusion:

    - `HUMOTO_CONFIG_SECTION_ID` declares default name of the configuration
      section corresponding to this class. If it is not defined, the user must
      implement virtual function which provides this functionality.

    - `HUMOTO_CONFIG_CONSTRUCTOR` requests generation of additional class
      constructors for direct initialization from a configuration file.

    - `HUMOTO_CONFIG_ENTRIES` declares configuration file entries for this
      class. If not specified the user must implement
      `readConfigEntriesTemplate()` template function manually.



* Implement remaining virtual methods of @ref humoto::config::ConfigurableBase
  class:

    - User must provide definition of @ref humoto::config::CommonConfigurableBase::setDefaults()
      function.

    - Optionally it is possible to postprocess class members after reading them
      from a configuration file -- in this case the programmer should provide
      appropriate implementation of @ref humoto::config::CommonConfigurableBase::finalize()
      virtual function.
