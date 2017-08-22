/**
    @file
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
        namespace msgpack
        {
            /**
             * @brief Configuration reader class
             */
            class HUMOTO_LOCAL Reader
            {
                private:
                    std::string     buffer_;

                    std::vector< boost::shared_ptr< ::msgpack::object_handle >  >           handles_;

                    /// Stack of nodes.
                    std::stack< const ::msgpack::object * >      node_stack_;


                private:
                    /**
                     * @brief open configuration file
                     *
                     * @param[in] file_name
                     */
                    void openFile(const std::string& file_name)
                    {
                        std::ifstream config_ifs_;

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


                        std::stringstream   str_stream;
                        str_stream << config_ifs_.rdbuf();
                        buffer_ = str_stream.str();


                        std::size_t     buffer_offset = 0;

                        handles_.clear();
                        try
                        {
                            while (buffer_offset != buffer_.size())
                            {
                                handles_.push_back(boost::shared_ptr< ::msgpack::object_handle >(new ::msgpack::object_handle));

                                unpack(*handles_[handles_.size()-1], buffer_.data(), buffer_.size(), buffer_offset);
                            }
                        }
                        catch(const std::exception &e)
                        {
                            HUMOTO_THROW_MSG(std::string("Failed to parse the configuration file: ") + e.what());
                        }
                    }


                    /**
                     * @brief Get current node
                     *
                     * @return pointer to the current node
                     */
                    const ::msgpack::object * getCurrentNode() const
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
                        if (node_stack_.size() == 0)
                        {
                            for (std::size_t i = 0; i < handles_.size(); ++i)
                            {
                                if (::msgpack::type::MAP == handles_[i]->get().type)
                                {
                                    if (child_name == handles_[i]->get().via.map.ptr[0].key.as<std::string>())
                                    {
                                        if (::msgpack::type::MAP == handles_[i]->get().via.map.ptr[0].val.type)
                                        {
                                            node_stack_.push( &(handles_[i]->get().via.map.ptr[0].val) );
                                            return(true);
                                        }
                                    }
                                }
                            }
                        }
                        else
                        {
                            if (::msgpack::type::MAP == getCurrentNode()->type)
                            {
                                for (std::size_t i = 0; i < getCurrentNode()->via.map.size; ++i)
                                {
                                    if (child_name == getCurrentNode()->via.map.ptr[i].key.as<std::string>())
                                    {
                                        node_stack_.push( &(getCurrentNode()->via.map.ptr[i].val) );
                                        return(true);
                                    }
                                }
                            }
                        }

                        return (false);
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
                            HUMOTO_ASSERT(  (::msgpack::type::ARRAY == getCurrentNode()->type),
                                            "[Config] Entry is not an array.");

                            if (Eigen::Dynamic == t_rows)
                            {
                                entry.resize(getCurrentNode()->via.array.size);
                            }
                            else
                            {
                                HUMOTO_ASSERT(  (static_cast<int>(getCurrentNode()->via.array.size) == t_rows),
                                                "[Config] Wrong entry sequence size.");
                            }

                            for(humoto::EigenIndex i = 0; i < (Eigen::Dynamic == t_rows ? entry.rows() : t_rows); ++i)
                            {
                                getCurrentNode()->via.array.ptr[i] >> entry[i];
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
                            HUMOTO_ASSERT(  (::msgpack::type::ARRAY == getCurrentNode()->type),
                                            "[Config] Entry is not an array.");

                            entry.resize(getCurrentNode()->via.array.size);

                            for(std::size_t i = 0; i < entry.size(); ++i)
                            {
                                getCurrentNode()->via.array.ptr[i] >> entry[i];
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
                            HUMOTO_ASSERT(  (::msgpack::type::ARRAY == getCurrentNode()->type),
                                            "[Config] Entry is not an array.");

                            entry.resize(getCurrentNode()->via.array.size);

                            for(std::size_t i = 0; i < entry.size(); ++i)
                            {
                                getCurrentNode()->via.array.ptr[i] >> entry[i];
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
