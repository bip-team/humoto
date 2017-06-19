Directory layout    {#md_directory_layout}
================

* **humoto** -- root folder
    - **cmake** -- cmake scripts

    - **bridges** -- bridges (interfaces) and sources of 3-rd party software (mostly in external repositories)
        - **[bridge_name]** -- a particular software
            - **interface** -- interface layer between the software and humoto
            - **[bridge_name]** -- source code of the software

    - **core** -- core module
        - **config** -- default configuration files
        - **include** -- sources
        - **share** -- shared data
        - **tests** -- tests of the core functionality

    - **doc** -- documentation
        - **latex** -- LaTeX documentation
        - **doxygen** -- Doxygen documentation

    - **modules** -- modules (controllers)
        - **[module_name]** -- a particular controller
            - **config** -- configuration files
            - **doc** -- LaTeX documentation
            - **include** -- sources of the controller
            - **tests** -- tests of the controller
