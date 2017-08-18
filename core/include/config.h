/**
    @file
    @author  Alexander Sherikov
    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

namespace humoto
{
    /**
     * @brief Namespace of classes related to configuration handling
     */
    namespace config
    {
    }
}

#define HUMOTO_CONFIG_DEFINE_ACCESSORS  "humoto/config/define_accessors.h"

#ifdef HUMOTO_USE_CONFIG
    #define HUMOTO_DEFINE_CONFIG_WRITER(entries) \
            template <class t_Writer> \
                void writeConfigEntriesTemplate(t_Writer & writer) const \
            { \
                entries \
            }

    #define HUMOTO_CONFIG_WRITE_PARENT_CLASS(parent_class)  parent_class::writeConfigEntries(writer)
    #define HUMOTO_CONFIG_WRITE_MEMBER_CLASS(member, name)  member.writeNestedConfig(writer, name)

    #define HUMOTO_CONFIG_WRITE_COMPOUND_(entry)    writer.writeCompound(entry##_, #entry)
    #define HUMOTO_CONFIG_WRITE_COMPOUND(entry)     writer.writeCompound(entry, #entry)

    #define HUMOTO_CONFIG_WRITE_SCALAR_(entry)  writer.writeScalar(entry##_, #entry)
    #define HUMOTO_CONFIG_WRITE_SCALAR(entry)   writer.writeScalar(entry, #entry)

    #define HUMOTO_CONFIG_WRITE_ENUM_(entry)    writer.writeEnum(entry##_, #entry)
    #define HUMOTO_CONFIG_WRITE_ENUM(entry)     writer.writeEnum(entry, #entry)



    #define HUMOTO_DEFINE_CONFIG_READER(entries) \
            template <class t_Reader> \
                void readConfigEntriesTemplate( t_Reader & reader, \
                                        const bool crash_on_missing_entry = false) \
            { \
                entries \
                finalize(); \
            }

    #define HUMOTO_CONFIG_READ_PARENT_CLASS(parent_class)  parent_class::readConfigEntries(reader, crash_on_missing_entry)
    #define HUMOTO_CONFIG_READ_MEMBER_CLASS(member, name)  member.readNestedConfig(reader, crash_on_missing_entry, name)

    #define HUMOTO_CONFIG_READ_COMPOUND_(entry)     reader.readCompound(entry##_, #entry, crash_on_missing_entry)
    #define HUMOTO_CONFIG_READ_COMPOUND(entry)      reader.readCompound(entry, #entry, crash_on_missing_entry)

    #define HUMOTO_CONFIG_READ_SCALAR_(entry)   reader.readScalar(entry##_, #entry, crash_on_missing_entry)
    #define HUMOTO_CONFIG_READ_SCALAR(entry)    reader.readScalar(entry, #entry, crash_on_missing_entry)

    #define HUMOTO_CONFIG_READ_ENUM_(entry)     reader.readEnum(entry##_, #entry, crash_on_missing_entry)
    #define HUMOTO_CONFIG_READ_ENUM(entry)      reader.readEnum(entry, #entry, crash_on_missing_entry)


    // ----------------------------


    #define HUMOTO_CONFIG_DISABLED

    // If support for configuration files is enabled include appropriate
    // headers.
    #ifdef HUMOTO_BRIDGE_config_yaml
        #include "config_yaml/config_handling.h"
        #define HUMOTO_BRIDGE_config_yaml_NAMESPACE humoto::config::yaml
        #undef HUMOTO_CONFIG_DISABLED
    #endif //HUMOTO_BRIDGE_config_yaml


    #ifdef HUMOTO_CONFIG_DISABLED
        #error "Configuration is enabled, but there are no configuration bridges."
    #endif


    // ----------------------------


    namespace humoto
    {
        namespace config
        {
            /**
             * @brief Configurable base class.
             */
            template <bool t_crash_on_missing_entry>
                class HUMOTO_LOCAL CommonConfigurableBase
            {
                protected:
                    static const bool default_crash_on_missing_entry_ = t_crash_on_missing_entry;


                protected:
                    /**
                     * @brief Protected destructor: prevent destruction of the child
                     * classes through a base pointer.
                     */
                    ~CommonConfigurableBase() {}
                    CommonConfigurableBase() {}


                    ///@{
                    /**
                     * These functions must be defined in derived classes
                     *
                     * @attention Implementations of these methods are added
                     * automatically upon inclusion of define_accessors.h if
                     * HUMOTO_CONFIG_ENTRIES is defined.
                     */
                    #ifdef HUMOTO_BRIDGE_config_yaml
                        virtual void writeConfigEntries(HUMOTO_BRIDGE_config_yaml_NAMESPACE::Writer &) const = 0;
                        virtual void readConfigEntries(HUMOTO_BRIDGE_config_yaml_NAMESPACE::Reader &, const bool) = 0;
                    #endif
                    ///@}


                    /**
                     * @brief Set members to their default values.
                     */
                    virtual void setDefaults() = 0;


                    /**
                     * @brief Return the default name of a configuration node
                     * corresponding to this class
                     *
                     * @return the name
                     *
                     * @attention Implementation of this method is added
                     * automatically upon inclusion of define_accessors.h if
                     * HUMOTO_CONFIG_SECTION_ID is defined.
                     */
                    virtual const std::string & getConfigSectionID() const = 0;


                    /**
                     * @brief This function is called automaticaly after reading
                     * a configuration file. Does nothing by default.
                     */
                    virtual void finalize() {};


                public:
                    /**
                     * @brief Read nested configuration node.
                     *
                     * @param[in] reader
                     * @param[in] crash_on_missing_entry
                     * @param[in] node_name   node name, the default is used if empty
                     */
                    template <class t_Reader>
                        void readNestedConfig(  t_Reader            & reader,
                                                const bool          crash_on_missing_entry = default_crash_on_missing_entry_,
                                                const std::string   & node_name = "")
                    {
                        std::string name = node_name;
                        if (name.size() == 0)
                        {
                            name = getConfigSectionID();
                        }

                        try
                        {
                            setDefaults();
                            if (reader.descend(name))
                            {
                                readConfigEntries(reader, crash_on_missing_entry);
                                reader.ascend();
                            }
                            else
                            {
                                if (crash_on_missing_entry)
                                {
                                    HUMOTO_THROW_MSG(std::string("Configuration file does not contain entry '") + name + "'.");
                                }
                            }
                        }
                        catch(const std::exception &e)
                        {
                            HUMOTO_THROW_MSG(std::string("Failed to parse node <") + name + "> in the configuration file: " + e.what());
                        }
                    }


                    /**
                     * @brief Write nested configuration node
                     *
                     * @param[in,out] writer configuration writer
                     * @param[in] node_name   node name, the default is used if empty
                     */
                    template <class t_Writer>
                        void writeNestedConfig( t_Writer& writer,
                                                const std::string &node_name = "") const
                    {
                        std::string name = node_name;
                        if (name.size() == 0)
                        {
                            name = getConfigSectionID();
                        }

                        writer.descend(name);
                        writeConfigEntries(writer);
                        writer.ascend();
                    }


                    /**
                     * @brief Read configuration (assuming the configuration node
                     * to be in the root).
                     *
                     * @param[in] reader configuration reader
                     * @param[in] crash_on_missing_entry
                     * @param[in] node_name   node name, the default is used if empty
                     */
                    template <class t_Reader>
                        void readConfig(t_Reader            & reader,
                                        const bool          crash_on_missing_entry = default_crash_on_missing_entry_,
                                        const std::string   & node_name = "")
                    {
                        readNestedConfig(reader, crash_on_missing_entry, node_name);
                    }


                    /**
                     * @brief Write configuration
                     *
                     * @param[in,out] writer configuration writer
                     * @param[in] node_name   node name, the default is used if empty
                     */
                    template <class t_Writer>
                        void writeConfig(t_Writer& writer,
                                         const std::string &node_name = "") const
                    {
                        writeNestedConfig(writer, node_name);
                        writer.flush();
                    }


                    /**
                     * @brief Read configuration (assuming the configuration node
                     * to be in the root).
                     *
                     * @param[in] file_name file name
                     * @param[in] crash_on_missing_entry
                     * @param[in] node_name   node name, the default is used if empty
                     */
                    template <class t_Reader>
                        void readConfig(const std::string &file_name,
                                        const bool        crash_on_missing_entry = default_crash_on_missing_entry_,
                                        const std::string &node_name = "")
                    {
                        t_Reader reader(file_name);
                        readNestedConfig(reader, crash_on_missing_entry, node_name);
                    }


                    /**
                     * @brief Write configuration.
                     *
                     * @param[in] file_name file name
                     * @param[in] node_name   node name, the default is used if empty
                     */
                    template <class t_Writer>
                        void writeConfig(const std::string &file_name,
                                         const std::string &node_name = "") const
                    {
                        t_Writer writer(file_name);
                        writeConfig(writer, node_name);
                    }
            };



            /// Default configurable base is strict
            class ConfigurableBase : public humoto::config::CommonConfigurableBase<true>
            {
                protected:
                    /**
                     * @brief Protected destructor: prevent destruction of the child
                     * classes through a base pointer.
                     */
                    ~ConfigurableBase() {}
                    ConfigurableBase() {}
            };


            class StrictConfigurableBase : public humoto::config::CommonConfigurableBase<true>
            {
                protected:
                    /**
                     * @brief Protected destructor: prevent destruction of the child
                     * classes through a base pointer.
                     */
                    ~StrictConfigurableBase() {}
                    StrictConfigurableBase() {}
            };


            class RelaxedConfigurableBase : public humoto::config::CommonConfigurableBase<false>
            {
                protected:
                    /**
                     * @brief Protected destructor: prevent destruction of the child
                     * classes through a base pointer.
                     */
                    ~RelaxedConfigurableBase() {}
                    RelaxedConfigurableBase() {}
            };
        }
    }

#else

    namespace humoto
    {
        namespace config
        {
            // Some classes may inherit from this
            class HUMOTO_LOCAL ConfigurableBase
            {
                protected:
                    /**
                     * @brief Protected destructor: prevent destruction of the
                     * child classes through a base pointer.
                     */
                    ~ConfigurableBase() {}
            };


            // Some classes may inherit from this
            class HUMOTO_LOCAL StrictConfigurableBase
            {
                protected:
                    /**
                     * @brief Protected destructor: prevent destruction of the
                     * child classes through a base pointer.
                     */
                    ~StrictConfigurableBase() {}
            };


            // Some classes may inherit from this
            class HUMOTO_LOCAL RelaxedConfigurableBase
            {
                protected:
                    /**
                     * @brief Protected destructor: prevent destruction of the
                     * child classes through a base pointer.
                     */
                    ~RelaxedConfigurableBase() {}
            };
        }
    }

    #define HUMOTO_CONFIG_DISABLED

#endif //HUMOTO_USE_CONFIG
