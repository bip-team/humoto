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
        }
    }
}
