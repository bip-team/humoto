/**
    @file
    @author  Alexander Sherikov
    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief Inclusion of this file results in generation of functions which
    read and write entries 'HUMOTO_CONFIG_ENTRIES' defined in the including
    header from / to a configuration file.
*/

#ifndef HUMOTO_DOXYGEN_PROCESSING

#ifdef HUMOTO_USE_CONFIG
// Generic stuff
    private:
        #ifdef HUMOTO_CONFIG_ENTRIES
            #define HUMOTO_CONFIG_COMPOUND_(entry)       HUMOTO_CONFIG_WRITE_COMPOUND_(entry)
            #define HUMOTO_CONFIG_COMPOUND(entry)        HUMOTO_CONFIG_WRITE_COMPOUND(entry)

            #define HUMOTO_CONFIG_SCALAR_(entry)       HUMOTO_CONFIG_WRITE_SCALAR_(entry)
            #define HUMOTO_CONFIG_SCALAR(entry)        HUMOTO_CONFIG_WRITE_SCALAR(entry)

            #define HUMOTO_CONFIG_ENUM_(entry)         HUMOTO_CONFIG_WRITE_ENUM_(entry)
            #define HUMOTO_CONFIG_ENUM(entry)          HUMOTO_CONFIG_WRITE_ENUM(entry)

            #define HUMOTO_CONFIG_PARENT_CLASS(entry)          HUMOTO_CONFIG_WRITE_PARENT_CLASS(entry)
            #define HUMOTO_CONFIG_MEMBER_CLASS(entry, name)    HUMOTO_CONFIG_WRITE_MEMBER_CLASS(entry, name)

            template <class t_Writer>
                void writeConfigEntriesTemplate(t_Writer & writer) const
            {
                HUMOTO_MACRO_SUBSTITUTE(HUMOTO_CONFIG_ENTRIES)
            }

            #undef HUMOTO_CONFIG_COMPOUND_
            #undef HUMOTO_CONFIG_COMPOUND

            #undef HUMOTO_CONFIG_SCALAR_
            #undef HUMOTO_CONFIG_SCALAR

            #undef HUMOTO_CONFIG_ENUM_
            #undef HUMOTO_CONFIG_ENUM

            #undef HUMOTO_CONFIG_PARENT_CLASS
            #undef HUMOTO_CONFIG_MEMBER_CLASS



            #define HUMOTO_CONFIG_COMPOUND_(entry)       HUMOTO_CONFIG_READ_COMPOUND_(entry)
            #define HUMOTO_CONFIG_COMPOUND(entry)        HUMOTO_CONFIG_READ_COMPOUND(entry)

            #define HUMOTO_CONFIG_SCALAR_(entry)       HUMOTO_CONFIG_READ_SCALAR_(entry)
            #define HUMOTO_CONFIG_SCALAR(entry)        HUMOTO_CONFIG_READ_SCALAR(entry)

            #define HUMOTO_CONFIG_ENUM_(entry)         HUMOTO_CONFIG_READ_ENUM_(entry)
            #define HUMOTO_CONFIG_ENUM(entry)          HUMOTO_CONFIG_READ_ENUM(entry)

            #define HUMOTO_CONFIG_PARENT_CLASS(entry)          HUMOTO_CONFIG_READ_PARENT_CLASS(entry)
            #define HUMOTO_CONFIG_MEMBER_CLASS(entry, name)    HUMOTO_CONFIG_READ_MEMBER_CLASS(entry, name)

            template <class t_Reader>
                void readConfigEntriesTemplate( t_Reader & reader,
                                        const bool crash_on_missing_entry = false)
            {
                HUMOTO_MACRO_SUBSTITUTE(HUMOTO_CONFIG_ENTRIES)
                finalize();
            }

            #undef HUMOTO_CONFIG_COMPOUND_
            #undef HUMOTO_CONFIG_COMPOUND

            #undef HUMOTO_CONFIG_SCALAR_
            #undef HUMOTO_CONFIG_SCALAR

            #undef HUMOTO_CONFIG_ENUM_
            #undef HUMOTO_CONFIG_ENUM

            #undef HUMOTO_CONFIG_PARENT_CLASS
            #undef HUMOTO_CONFIG_MEMBER_CLASS
        #endif


    protected:
        #ifdef HUMOTO_CONFIG_SECTION_ID
            const std::string & getConfigSectionID() const
            {
                static const std::string name(HUMOTO_CONFIG_SECTION_ID);
                return (name);
            }
        #endif


        #ifdef HUMOTO_CONFIG_ENTRIES
            #define HUMOTO_CONFIG_COMPOUND_(entry)       +1
            #define HUMOTO_CONFIG_COMPOUND(entry)        +1

            #define HUMOTO_CONFIG_SCALAR_(entry)       +1
            #define HUMOTO_CONFIG_SCALAR(entry)        +1

            #define HUMOTO_CONFIG_ENUM_(entry)         +1
            #define HUMOTO_CONFIG_ENUM(entry)          +1

            #define HUMOTO_CONFIG_PARENT_CLASS(entry)          +entry::getNumberOfEntries()
            #define HUMOTO_CONFIG_MEMBER_CLASS(entry, name)    +1

            std::size_t getNumberOfEntries() const
            {
                static const std::size_t    num_entries = 0 HUMOTO_MACRO_SUBSTITUTE(HUMOTO_CONFIG_ENTRIES);
                return(num_entries);
            }

            #undef HUMOTO_CONFIG_COMPOUND_
            #undef HUMOTO_CONFIG_COMPOUND

            #undef HUMOTO_CONFIG_SCALAR_
            #undef HUMOTO_CONFIG_SCALAR

            #undef HUMOTO_CONFIG_ENUM_
            #undef HUMOTO_CONFIG_ENUM

            #undef HUMOTO_CONFIG_PARENT_CLASS
            #undef HUMOTO_CONFIG_MEMBER_CLASS
        #endif


    public:
        #ifdef HUMOTO_CONFIG_CONSTRUCTOR
            /**
             * Define constructors for the given class.
             */
            template <class t_Reader>
                explicit HUMOTO_CONFIG_CONSTRUCTOR(
                        t_Reader &reader,
                        const std::string &node_name,
                        const bool crash_on_missing_entry = default_crash_on_missing_entry_)
            {
                readConfig(reader, node_name, crash_on_missing_entry);
            }

            template <class t_Reader>
                explicit HUMOTO_CONFIG_CONSTRUCTOR(
                        t_Reader &reader,
                        const bool crash_on_missing_entry = default_crash_on_missing_entry_)
            {
                readConfig(reader, crash_on_missing_entry);
            }
        #endif


// Format-specific stuff
    #ifdef HUMOTO_USE_CONFIG_YAML
        private:
            HUMOTO_CONFIG_YAML_PRIVATE_DATA

        protected:
            void writeConfigEntries(HUMOTO_CONFIG_YAML_NAMESPACE::Writer & writer) const
            {
                writeConfigEntriesTemplate(writer);
            }
            void readConfigEntries(HUMOTO_CONFIG_YAML_NAMESPACE::Reader & reader, const bool crash_flag)
            {
                readConfigEntriesTemplate(reader, crash_flag);
            }
    #endif

    #ifdef HUMOTO_USE_CONFIG_MSGPACK
        private:
            HUMOTO_CONFIG_MSGPACK_PRIVATE_DATA

        protected:
            void writeConfigEntries(HUMOTO_CONFIG_MSGPACK_NAMESPACE::Writer & writer) const
            {
                writeConfigEntriesTemplate(writer);
            }
            void readConfigEntries(HUMOTO_CONFIG_MSGPACK_NAMESPACE::Reader & reader, const bool crash_flag)
            {
                readConfigEntriesTemplate(reader, crash_flag);
            }
    #endif

#endif //HUMOTO_USE_CONFIG

#undef HUMOTO_CONFIG_SECTION_ID
#undef HUMOTO_CONFIG_CONSTRUCTOR
#undef HUMOTO_CONFIG_ENTRIES
#endif
