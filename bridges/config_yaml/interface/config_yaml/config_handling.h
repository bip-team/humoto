/**
    @file
    @author Jan Michalczyk
    @author Alexander Sherikov
    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include "yaml-cpp/yaml.h"


namespace humoto
{
    namespace config
    {
        /**
         * @brief Configuration reader class
         */
        class HUMOTO_LOCAL Reader
        {
            private:
                /// input file stream
                std::ifstream config_ifs_;

                /// instance of YAML parser
                YAML::Parser  parser_;

                /// root node
                YAML::Node    root_node_;

                /// Stack of nodes.
                std::stack<const YAML::Node *>    node_stack_;


            private:
                /**
                 * @brief open configuration file
                 *
                 * @param[in] file_name
                 */
                void openFile(const std::string& file_name)
                {
                    config_ifs_.open(file_name.c_str());
                    if (!config_ifs_.good())
                    {
                        std::string file_name_default = std::string(HUMOTO_DEFAULT_CONFIG_PREFIX) + file_name;
                        config_ifs_.open(file_name_default.c_str());
                    }
                    if (!config_ifs_.good())
                    {
                        HUMOTO_THROW_MSG(std::string("Could not open configuration file: ") +  file_name.c_str());
                    }

                    parser_.Load(config_ifs_),
                    parser_.GetNextDocument(root_node_);
                    node_stack_.push(&root_node_);
                }


                /**
                 * @brief Get current node
                 *
                 * @return pointer to the current node
                 */
                const YAML::Node * getCurrentNode()
                {
                    return(node_stack_.top());
                }


            public:
                /**
                 * @brief Constructor
                 *
                 * @param[in] file_name
                 */
                Reader(const std::string& file_name)
                {
                    openFile(file_name);
                }


                /**
                 * @brief Default constructor
                 */
                Reader()
                {
                }



                /**
                 * @brief Descend to the entry with the given name
                 *
                 * @param[in] child_name child node name
                 *
                 * @return true if successful.
                 */
                bool descend(const std::string & child_name)
                {
                    const YAML::Node * child = getCurrentNode()->FindValue(child_name);

                    if (child == NULL)
                    {
                        return(false);
                    }
                    else
                    {
                        node_stack_.push(child);
                        return(true);
                    }
                }


                /**
                 * @brief Ascend from the current entry to its parent.
                 */
                void ascend()
                {
                    node_stack_.pop();
                }


                /**
                 * @brief Read configuration entry (vector)
                 *
                 * @tparam t_Scalar Eigen template parameter
                 * @tparam t_rows   Eigen template parameter
                 * @tparam t_flags  Eigen template parameter
                 *
                 * @param[out] entry     configuration parameter
                 * @param[in] entry_name name of the configuration parameter
                 * @param[in] crash_on_missing_entry
                 */
                template <  typename t_Scalar,
                            int t_rows,
                            int t_flags>
                    void readCompound(Eigen::Matrix<t_Scalar, t_rows, 1, t_flags> &entry,
                                      const std::string & entry_name,
                                      const bool crash_on_missing_entry = false)
                {
                    if (descend(entry_name))
                    {
                        HUMOTO_ASSERT(  (getCurrentNode()->Type() == YAML::NodeType::Sequence),
                                        "[Config] Entry is not a sequence.");

                        if (Eigen::Dynamic == t_rows)
                        {
                            entry.resize(getCurrentNode()->size());
                        }
                        else
                        {
                            HUMOTO_ASSERT(  (static_cast<int>(getCurrentNode()->size()) == t_rows),
                                            "[Config] Wrong entry sequence size.");
                        }

                        for(humoto::EigenIndex i = 0; i < (Eigen::Dynamic == t_rows ? entry.rows() : t_rows); ++i)
                        {
                            (*getCurrentNode())[i] >> entry(i);
                        }

                        ascend();
                    }
                    else
                    {
                        if (crash_on_missing_entry)
                        {
                            HUMOTO_THROW_MSG(std::string("Configuration file does not contain entry '") + entry_name + "'.");
                        }
                    }
                }



                /**
                 * @brief Read a configuration entry (matrix)
                 *
                 * @tparam t_Scalar Eigen template parameter
                 * @tparam t_rows   Eigen template parameter
                 * @tparam t_cols   Eigen template parameter
                 * @tparam t_flags  Eigen template parameter
                 *
                 * @param[out] entry      data
                 * @param[in]  entry_name name
                 * @param[in] crash_on_missing_entry
                 */
                template <  typename t_Scalar,
                            int t_rows,
                            int t_cols,
                            int t_flags>
                    void readCompound(  Eigen::Matrix<t_Scalar, t_rows, t_cols, t_flags> &entry,
                                        const std::string& entry_name,
                                        const bool crash_on_missing_entry = false)
                {
                    if (descend(entry_name))
                    {
                        humoto::EigenIndex num_rows;
                        humoto::EigenIndex num_cols;

                        readScalar(num_rows, "rows", true);
                        readScalar(num_cols, "cols", true);


                        Eigen::VectorXd v;
                        readCompound(v, "data", true);

                        HUMOTO_ASSERT(  v.rows() == num_rows*num_cols,
                                        std::string("Inconsistent configuration file entry: ") + entry_name);

                        Eigen::Map< Eigen::Matrix<  double,
                                                    Eigen::Dynamic,
                                                    Eigen::Dynamic,
                                                    Eigen::RowMajor> >  map(v.data(),
                                                                            num_rows,
                                                                            num_cols);
                        entry = map;

                        ascend();
                    }
                    else
                    {
                        if (crash_on_missing_entry)
                        {
                            HUMOTO_THROW_MSG(std::string("Configuration file does not contain entry '") + entry_name + "'.");
                        }
                    }
                }


                /**
                 * @brief Read configuration entry (std::vector)
                 *
                 * @tparam t_VectorEntryType type of the entry of std::vector
                 *
                 * @param[out] entry      configuration parameter
                 * @param[in]  entry_name name of the configuration parameter
                 * @param[in]  crash_on_missing_entry
                 */
                template <typename t_VectorEntryType>
                    void readCompound(  std::vector<t_VectorEntryType> & entry,
                                        const std::string              & entry_name,
                                        const bool crash_on_missing_entry = false)
                {
                    if (descend(entry_name))
                    {
                        HUMOTO_ASSERT(  (getCurrentNode()->Type() == YAML::NodeType::Sequence),
                                        "[Config] Entry is not a sequence.");

                        entry.resize(getCurrentNode()->size());

                        for(std::size_t i = 0; i < getCurrentNode()->size(); ++i)
                        {
                            (*getCurrentNode())[i] >> entry[i];
                        }

                        ascend();
                    }
                    else
                    {
                        if (crash_on_missing_entry)
                        {
                            HUMOTO_THROW_MSG(std::string("Configuration file does not contain entry '") + entry_name + "'.");
                        }
                    }
                }


                /**
                 * @brief Read configuration entry (std::vector<std::vector<double> >)
                 *
                 * @param[out] entry      configuration parameter
                 * @param[in]  entry_name name of the configuration parameter
                 * @param[in]  crash_on_missing_entry
                 */
                void readCompound(  std::vector<std::vector<double> >      & entry,
                                    const std::string                      & entry_name,
                                    const bool crash_on_missing_entry = false)
                {
                    if (descend(entry_name))
                    {
                        HUMOTO_ASSERT(  (getCurrentNode()->Type() == YAML::NodeType::Sequence),
                                        "[Config] Entry is not a sequence.");

                        entry.resize(getCurrentNode()->size());

                        for(std::size_t i = 0; i < getCurrentNode()->size(); ++i)
                        {
                            (*getCurrentNode())[i] >> entry[i];
                        }

                        ascend();
                    }
                    else
                    {
                        if (crash_on_missing_entry)
                        {
                            HUMOTO_THROW_MSG(std::string("Configuration file does not contain entry '") + entry_name + "'.");
                        }
                    }
                }

                /**
                 * @brief Read configuration entry (std::vector<std::vector<std::string> >)
                 *
                 * @param[out] entry      configuration parameter
                 * @param[in]  entry_name name of the configuration parameter
                 * @param[in]  crash_on_missing_entry
                 */
                void readCompound(  std::vector<std::vector<std::string> > & entry,
                                    const std::string                      & entry_name,
                                    const bool crash_on_missing_entry = false)
                {
                    if (descend(entry_name))
                    {
                        HUMOTO_ASSERT(  (getCurrentNode()->Type() == YAML::NodeType::Sequence),
                                        "[Config] Entry is not a sequence.");

                        entry.resize(getCurrentNode()->size());

                        for(std::size_t i = 0; i < getCurrentNode()->size(); ++i)
                        {
                            (*getCurrentNode())[i] >> entry[i];
                        }

                        ascend();
                    }
                    else
                    {
                        if (crash_on_missing_entry)
                        {
                            HUMOTO_THROW_MSG(std::string("Configuration file does not contain entry '") + entry_name + "'.");
                        }
                    }
                }


                /**
                 * @brief Read configuration entry (scalar template)
                 *
                 * @tparam t_EntryType type of the entry
                 *
                 * @param[out] entry     configuration parameter
                 * @param[in] entry_name name of the configuration parameter
                 * @param[in] crash_on_missing_entry
                 */
                template <typename t_EntryType>
                    void readScalar(t_EntryType        & entry,
                                    const std::string  & entry_name,
                                    const bool crash_on_missing_entry = false)
                {
                    if (descend(entry_name))
                    {
                        *getCurrentNode() >> entry;
                        ascend();
                    }
                    else
                    {
                        if (crash_on_missing_entry)
                        {
                            HUMOTO_THROW_MSG(std::string("Configuration file does not contain entry '") + entry_name + "'.");
                        }
                    }
                }


                /**
                 * @brief Read configuration entry (an enum). This method
                 * is added since an explicit casting to integer is needed.
                 *
                 * @tparam t_EnumerationType enumeration type
                 *
                 * @param[out] entry     configuration parameter
                 * @param[in] entry_name name of the configuration parameter
                 * @param[in] crash_on_missing_entry
                 */
                template <typename t_EnumerationType>
                    void readEnum(  t_EnumerationType  & entry,
                                    const std::string  & entry_name,
                                    const bool crash_on_missing_entry = false)
                {
                    if (descend(entry_name))
                    {
                        int tmp_value = 0;
                        *getCurrentNode() >> tmp_value;

                        entry = static_cast<t_EnumerationType> (tmp_value);
                        ascend();
                    }
                    else
                    {
                        if (crash_on_missing_entry)
                        {
                            HUMOTO_THROW_MSG(std::string("Configuration file does not contain entry '") + entry_name + "'.");
                        }
                    }
                }
        };



        /**
         * @brief Configuration writer class
         */
        class HUMOTO_LOCAL Writer
        {
            private:
                /// output file stream
                std::ofstream   config_ofs_;

                /// instance of YAML emitter, is destroyed and reinitialized by flush()
                YAML::Emitter   *emitter_;


            private:
                /**
                 * @brief Initialize emitter
                 */
                void initEmitter()
                {
                    emitter_ = new YAML::Emitter;
                    emitter_->SetDoublePrecision(std::numeric_limits<double>::digits10);
                    if (config_ofs_.tellp() != 0)
                    {
                        *emitter_ << YAML::Newline;
                    }
                    *emitter_ << YAML::BeginMap;
                }


                /**
                 * @brief Destroy emitter
                 */
                void destroyEmitter()
                {
                    *emitter_ << YAML::EndMap;
                    config_ofs_ << emitter_->c_str();
                    delete emitter_;
                }


            public:
                /**
                 * @brief Constructor
                 *
                 * @param[in] file_name
                 */
                Writer(const std::string& file_name) :
                    config_ofs_(file_name.c_str())
                {
                    initEmitter();
                }


                /**
                 * @brief Destructor
                 */
                ~Writer()
                {
                    delete emitter_;
                }


                /**
                 * @brief Starts a nested map in the configuration file
                 *
                 * @param[in] map_name name of the map
                 */
                void descend(const std::string &map_name)
                {
                    *emitter_ << YAML::Key << map_name;
                    *emitter_ << YAML::Value;
                    *emitter_ << YAML::BeginMap;
                }


                /**
                 * @brief Ends a nested map in the configuration file
                 */
                void ascend()
                {
                    *emitter_ << YAML::EndMap;
                }


                /**
                 * @brief Write a configuration entry (vector)
                 *
                 * @tparam t_Derived Eigen template parameter
                 *
                 * @param[in] entry      data
                 * @param[in] entry_name name
                 */
                template <  typename t_Scalar,
                            int t_rows,
                            int t_flags>
                    void writeCompound( const Eigen::Matrix<t_Scalar, t_rows, 1, t_flags> &entry,
                                        const std::string & entry_name)
                {
                    // this is
                    *emitter_ << YAML::Key << entry_name;
                    *emitter_ << YAML::Value << YAML::Flow;
                    *emitter_ << YAML::BeginSeq;

                    for(humoto::EigenIndex i = 0; i < entry.rows(); ++i)
                    {
                        *emitter_ << entry(i);
                    }

                    *emitter_ << YAML::EndSeq;
                }



                /**
                 * @brief Write a configuration entry (matrix)
                 *
                 * @tparam t_Scalar Eigen template parameter
                 * @tparam t_rows   Eigen template parameter
                 * @tparam t_cols   Eigen template parameter
                 * @tparam t_flags  Eigen template parameter
                 *
                 * @param[in] entry      data
                 * @param[in]  entry_name name
                 */
                template <  typename t_Scalar,
                            int t_rows,
                            int t_cols,
                            int t_flags>
                    void writeCompound( const Eigen::Matrix<t_Scalar, t_rows, t_cols, t_flags> &entry,
                                        const std::string& entry_name)
                {
                    descend(entry_name);

                    writeScalar(entry.cols(), "cols");
                    writeScalar(entry.rows(), "rows");


                    *emitter_ << YAML::Key << "data";
                    *emitter_ << YAML::Value << YAML::Flow;
                    *emitter_ << YAML::BeginSeq;

                    for(humoto::EigenIndex i = 0; i < entry.cols(); ++i)
                    {
                        for(humoto::EigenIndex j = 0; j < entry.rows(); ++j)
                        {
                            *emitter_ << entry(j, i);
                        }
                    }

                    *emitter_ << YAML::EndSeq;

                    ascend();
                }


                /**
                 * @brief Write configuration entry (std::vector<std::vector<double>>)
                 *
                 * @param[in] entry      configuration parameter
                 * @param[in] entry_name name of the configuration parameter
                 */
                void writeCompound( const std::vector<std::vector<double> >      & entry,
                                    const std::string                            & entry_name) const
                {
                    *emitter_ << YAML::Key << entry_name;
                    *emitter_ << YAML::Value << YAML::Flow;
                    *emitter_ << YAML::BeginSeq;

                    for(std::size_t i = 0; i < entry.size(); ++i)
                    {
                        for(std::size_t j = 0; j < entry[i].size(); ++j)
                        {
                            *emitter_ << entry[i][j];
                        }
                    }

                    *emitter_ << YAML::EndSeq;
                }

                /**
                 * @brief Write configuration entry (std::vector<std::vector<std::string>>)
                 *
                 * @param[in] entry      configuration parameter
                 * @param[in] entry_name name of the configuration parameter
                 */
                void writeCompound( const std::vector<std::vector<std::string> > & entry,
                                    const std::string                            & entry_name) const
                {
                    *emitter_ << YAML::Key << entry_name;
                    *emitter_ << YAML::Value << YAML::Flow;
                    *emitter_ << YAML::BeginSeq;

                    for(std::size_t i = 0; i < entry.size(); ++i)
                    {
                        for(std::size_t j = 0; j < entry[i].size(); ++j)
                        {
                            *emitter_ << entry[i][j];
                        }
                    }

                    *emitter_ << YAML::EndSeq;
                }


                /**
                 * @brief Read configuration entry (std::vector)
                 *
                 * @tparam t_VectorEntryType type of the entry of std::vector
                 *
                 * @param[in] entry      data
                 * @param[in] entry_name name
                 */
                template <typename t_VectorEntryType>
                    void writeCompound( const std::vector<t_VectorEntryType> & entry,
                                        const std::string                    & entry_name)
                {
                    *emitter_ << YAML::Key << entry_name;
                    *emitter_ << YAML::Value << YAML::Flow;
                    *emitter_ << YAML::BeginSeq;

                    for(std::size_t i = 0; i < entry.size(); ++i)
                    {
                        *emitter_ << entry[i];
                    }

                    *emitter_ << YAML::EndSeq;
                }


                /**
                 * @brief Write a configuration entry (enum)
                 *
                 * @tparam t_EnumType type of the enum
                 *
                 * @param[in] entry      data
                 * @param[in] entry_name name
                 */
                template <typename t_EnumType>
                    void writeEnum( const t_EnumType  entry,
                                    const std::string  & entry_name)
                {
                    *emitter_ << YAML::Key << entry_name << YAML::Value << entry;
                }


                /**
                 * @brief Write a configuration entry (scalar template)
                 *
                 * @tparam t_EntryType type of the entry
                 *
                 * @param[in] entry_name name
                 * @param[in] entry      data
                 */
                template <typename t_EntryType>
                    void writeScalar(   const t_EntryType  entry,
                                        const std::string  & entry_name)
                {
                    *emitter_ << YAML::Key << entry_name << YAML::Value << entry;
                }


                /**
                 * @brief Flush the configuration to the file
                 */
                void flush()
                {
                    destroyEmitter();
                    initEmitter();
                }
        };



        /**
         * @brief Common configurable base.
         *
         * @attention Do not inherit from this class -- inherit from
         * @ref humoto::config::ConfigurableBase, or, if you need more control,
         * from @ref humoto::config::StrictConfigurableBase or
         * @ref humoto::config::RelaxedConfigurableBase.
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
                virtual void writeConfigEntries(humoto::config::Writer &) const = 0;
                virtual void readConfigEntries(humoto::config::Reader &, const bool) = 0;
                ///@}

                /**
                 * @brief Set members to their default values.
                 */
                virtual void setDefaults() = 0;


                /**
                 * @brief This function is called automaticaly after reading
                 * a configuration file. Does nothing by default.
                 */
                virtual void finalize() {};


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


            public:
                /**
                 * @brief Read nested configuration node.
                 *
                 * @param[in] reader
                 * @param[in] crash_on_missing_entry
                 * @param[in] node_name   node name, the default is used if empty
                 */
                void readNestedConfig(  humoto::config::Reader      & reader,
                                        const bool                  crash_on_missing_entry = t_crash_on_missing_entry,
                                        const std::string           & node_name = "")
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
                void writeNestedConfig( humoto::config::Writer& writer,
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
                 * @param[in] file_name file name
                 * @param[in] crash_on_missing_entry
                 * @param[in] node_name   node name, the default is used if empty
                 */
                void readConfig(const std::string &file_name,
                                const bool        crash_on_missing_entry = t_crash_on_missing_entry,
                                const std::string &node_name = "")
                {
                    humoto::config::Reader reader(file_name);
                    readNestedConfig(reader, crash_on_missing_entry, node_name);
                }


                /**
                 * @brief Read configuration (assuming the configuration node
                 * to be in the root).
                 *
                 * @param[in] reader configuration reader
                 * @param[in] crash_on_missing_entry
                 * @param[in] node_name   node name, the default is used if empty
                 */
                void readConfig(humoto::config::Reader          & reader,
                                const bool                      crash_on_missing_entry = t_crash_on_missing_entry,
                                const std::string               & node_name = "")
                {
                    readNestedConfig(reader, crash_on_missing_entry, node_name);
                }



                /**
                 * @brief Write configuration.
                 *
                 * @param[in] file_name file name
                 * @param[in] node_name   node name, the default is used if empty
                 */
                void writeConfig(const std::string &file_name,
                                 const std::string &node_name = "") const
                {
                    humoto::config::Writer writer(file_name);
                    writeConfig(writer, node_name);
                }


                /**
                 * @brief Write configuration
                 *
                 * @param[in,out] writer configuration writer
                 * @param[in] node_name   node name, the default is used if empty
                 */
                void writeConfig(humoto::config::Writer& writer,
                                 const std::string &node_name = "") const
                {
                    writeNestedConfig(writer, node_name);
                    writer.flush();
                }
        };


        /**
         * @brief An interface of a configurable object, a couple of default
         * constructors for inherited classes can be added with
         * #HUMOTO_DEFINE_CONFIG_CONSTRUCTORS
         */
        class HUMOTO_LOCAL RelaxedConfigurableBase : public CommonConfigurableBase<false>
        {
            protected:
                /**
                 * @brief Protected destructor: prevent destruction of the child
                 * classes through a base pointer.
                 */
                ~RelaxedConfigurableBase() {}
                RelaxedConfigurableBase() {}
        };


        /**
         * @brief The same as @ref humoto::config::RelaxedConfigurableBase, but
         * requires all configuration entries to be set.
         */
        class HUMOTO_LOCAL StrictConfigurableBase : public CommonConfigurableBase<true>
        {
            protected:
                /**
                 * @brief Protected destructor: prevent destruction of the child
                 * classes through a base pointer.
                 */
                ~StrictConfigurableBase() {}
                StrictConfigurableBase() {}
        };


        /// Default configurable base is strict
        typedef StrictConfigurableBase  ConfigurableBase;
    }
}


/**
 * Define constructors of ConfigurableBase and ConfigurableBase::getConfigSectionID()
 * for the given class 'class_name'.
 */
#define HUMOTO_DEFINE_CONFIG_CONSTRUCTORS(class_name)    \
                                class_name( const std::string &file_name,\
                                            const bool crash_on_missing_entry = default_crash_on_missing_entry_, \
                                            const std::string &node_name = "")  \
                                { \
                                    readConfig(file_name, crash_on_missing_entry, node_name); \
                                } \
                                class_name( humoto::config::Reader &reader, \
                                            const bool crash_on_missing_entry = default_crash_on_missing_entry_, \
                                            const std::string &node_name = "")  \
                                { \
                                    readConfig(reader, crash_on_missing_entry, node_name); \
                                }


#define HUMOTO_DEFINE_CONFIG_WRITER(entries) \
                                void writeConfigEntries(humoto::config::Writer& writer) const \
                                { \
                                    entries \
                                }

#define HUMOTO_DEFINE_CONFIG_SECTION_ID(id) \
                                const std::string & getConfigSectionID() const \
                                { \
                                    static const std::string name(id); \
                                    return (name); \
                                }

#define HUMOTO_DEFINE_CONFIG_READER(entries) \
                                void readConfigEntries( humoto::config::Reader& reader, \
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


#define HUMOTO_CONFIG_WRITE_PARENT_CLASS(parent_class)  parent_class::writeConfigEntries(writer)
#define HUMOTO_CONFIG_WRITE_MEMBER_CLASS(member, name)  member.writeNestedConfig(writer, name)

#define HUMOTO_CONFIG_WRITE_COMPOUND_(entry)    writer.writeCompound(entry##_, #entry)
#define HUMOTO_CONFIG_WRITE_COMPOUND(entry)     writer.writeCompound(entry, #entry)

#define HUMOTO_CONFIG_WRITE_SCALAR_(entry)  writer.writeScalar(entry##_, #entry)
#define HUMOTO_CONFIG_WRITE_SCALAR(entry)   writer.writeScalar(entry, #entry)

#define HUMOTO_CONFIG_WRITE_ENUM_(entry)    writer.writeEnum(entry##_, #entry)
#define HUMOTO_CONFIG_WRITE_ENUM(entry)     writer.writeEnum(entry, #entry)
