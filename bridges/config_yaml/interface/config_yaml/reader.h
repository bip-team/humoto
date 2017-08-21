/**
    @file
    @author Jan Michalczyk
    @author Alexander Sherikov
    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

namespace humoto
{
    namespace config
    {
        namespace yaml
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
        }
    }
}
