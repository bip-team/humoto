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
             * @brief Configuration writer class
             */
            class HUMOTO_LOCAL Writer
            {
                private:
                    /// output file stream
                    std::ofstream   config_ofs_;

                    ::msgpack::packer< std::ofstream > *packer_;


                public:
                    /**
                     * @brief Constructor
                     *
                     * @param[in] file_name
                     */
                    explicit Writer(const std::string& file_name) 
                    {
                        config_ofs_.open(file_name.c_str());

                        if (!config_ofs_.good())
                        {
                            HUMOTO_THROW_MSG(std::string("Could not open configuration file for writing: ") +  file_name.c_str());
                        }

                        packer_ = new ::msgpack::packer< std::ofstream >(config_ofs_);
                    }


                    /**
                     * @brief Destructor
                     */
                    ~Writer()
                    {
                        delete packer_;
                    }


                    /**
                     * @brief Starts a nested map in the configuration file
                     *
                     * @param[in] map_name name of the map
                     * @param[in] num_entries number of child entries
                     */
                    void descend(const std::string &map_name, const std::size_t num_entries)
                    {
                        packer_->pack(map_name);
                        packer_->pack_map(num_entries);
                    }


                    /**
                     * @brief Starts a nested map in the configuration file
                     */
                    void initRoot()
                    {
                        packer_->pack_map(1);
                    }


                    /**
                     * @brief Ends a nested map in the configuration file
                     */
                    void ascend()
                    {
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
                        HUMOTO_ASSERT(entry.size() <= std::numeric_limits<uint32_t>::max(), "Vector is too long.");

                        packer_->pack(entry_name);
                        packer_->pack_array(entry.size());
                        for (humoto::EigenIndex i = 0; i < entry.rows(); ++i)
                        {
                            packer_->pack(entry[i]);
                        }
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
                        HUMOTO_ASSERT(entry.size() <= std::numeric_limits<uint32_t>::max(), "Matrix is too large.");

                        descend(entry_name, 3);

                        writeScalar(entry.cols(), "cols");
                        writeScalar(entry.rows(), "rows");


                        packer_->pack("data");
                        packer_->pack_array(entry.size());

                        for (humoto::EigenIndex i = 0; i < entry.size(); ++i)
                        {
                            packer_->pack(entry.data()[i]);
                        }

                        ascend();
                    }


                    /**
                     * @brief Write configuration entry (std::vector<std::vector<std::string>>)
                     *
                     * @tparam t_VectorEntryType type of the entry of std::vector
                     *
                     * @param[in] entry      configuration parameter
                     * @param[in] entry_name name of the configuration parameter
                     */
                    template <typename t_VectorEntryType>
                        void writeCompound( const std::vector< std::vector<t_VectorEntryType> > & entry,
                                            const std::string                                   & entry_name) const
                    {
                        HUMOTO_ASSERT(entry.size() <= std::numeric_limits<uint32_t>::max(), "Vector is too long.");

                        packer_->pack(entry_name);
                        packer_->pack_array(entry.size());
                        for(std::size_t i = 0; i < entry.size(); ++i)
                        {
                            packer_->pack(entry[i]);
                        }
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
                        packer_->pack(entry_name);
                        packer_->pack(entry);
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
                        packer_->pack(entry_name);
                        packer_->pack_int64(entry);
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
                        packer_->pack(entry_name);
                        packer_->pack(entry);
                    }


                    /**
                     * @brief Flush the configuration to the file
                     */
                    void flush()
                    {
                        config_ofs_.flush();
                    }
            };
        }
    }
}
