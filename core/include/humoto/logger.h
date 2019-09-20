/**
    @file
    @author  Alexander Sherikov
    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include <boost/algorithm/string.hpp>

namespace humoto
{
    #ifndef LOGGER_FORMATTER
    #define LOGGER_FORMATTER OctaveFormatter
    #endif

    /**
     * @brief A collection of functions which format data to be parsable by
     * python.
     */
    class HUMOTO_LOCAL PythonFormatter
    {
        public:
            /**
             * @brief Returns format description for Eigen matrices.
             *
             * @return Eigen IO format
             */
            static const Eigen::IOFormat& getEigenMatrixFormat()
            {
                static const Eigen::IOFormat    eigen_output_format(Eigen::FullPrecision, 0, ", ", "],[", "", "", "[", "];");
                return (eigen_output_format);
            }


            static std::string getPreviousPrefix(std::string & name, std::string & prefix)
            {
                std::vector<std::string> result;
                boost::split(result, name, boost::is_any_of("\n"));
                if(result.size() > 1)
                {
                    prefix = result.back().erase(0,1);
                    return "nothing";
                }
                else
                {
                    prefix = result.back();
                    return "if not \"" + prefix + "\" in globals():" + prefix;
                }
            }

            /**
             * @brief Add a part to the given log entry name.
             *
             * @param[in] name      name
             * @param[in] addition  subname
             */
            static void appendEntryName(std::string & name,
                                        const std::string & addition)
            {
                if ((name.size() > 0) && (addition.size() > 0))
                {
                    std::string prefix;
                    std::string result = getPreviousPrefix(name, prefix);
                    if(result!="nothing")
                        name = result;
                    name += " = {}\nif \""+addition+"\" not in "+prefix+".keys():\n\t"+prefix+"[\""+addition+"\"]"; 
                    //std::cout << name << std::endl << std::endl;
                }
            }


            /**
             * @brief Add a part to the given log entry name.
             *
             * @param[in] name      name
             * @param[in] addition  subname
             */
            static void appendEntryName(std::string & name,
                                        const char * addition)
            {
                if ((name.size() > 0) && ('\0' != addition[0]))
                {
                    std::string prefix;
                    std::string result = getPreviousPrefix(name, prefix);
                    if(result!="nothing")
                        name = result;
                    name += " = {}\nif \""+std::string(addition)+"\" not in "+prefix+".keys():\n\t"+prefix+"[\""+addition+"\"]"; 
                    //std::cout << name << std::endl << std::endl;
                }
            }


            /**
             * @brief Add a part to the given log entry name.
             *
             * @param[in] name      name
             * @param[in] index     index of a subentry
             */
            static void appendEntryName(std::string & name,
                                        const std::size_t index)
            {
                std::string prefix;
                std::string result = getPreviousPrefix(name, prefix);
                if(result!="nothing")
                    name = result;
                std::string addition = boost::lexical_cast<std::string>(index);
                name += " = {}\nif \""+addition+"\" not in "+prefix+".keys():\n\t"+prefix+"[\""+addition+"\"]"; 
                //std::cout << name << std::endl << std::endl;
            }

            /**
             * @brief Format and output a log entry (std::vector)
             *
             * @tparam t_Scalar type of vector elements
             * @param[in,out] out   output stream
             * @param[in] name      name of the log entry
             * @param[in] vector    data vector
             */
            template <typename t_Scalar>
                static void  formatAndOutputEntry(  std::ostream *out,
                                                    const std::string & name,
                                                    const std::vector<t_Scalar> &vector)
            {
                *out << name << " = " << "[";
                for (std::size_t i = 0; i < vector.size(); ++i)
                {
                    *out << vector[i];
                    if (i + 1 != vector.size())
                    {
                        *out << ",";
                    }
                }
                *out << "];\n";
            }


            /**
             * @brief Format and output a log entry (Eigen matrix)
             *
             * @tparam t_Scalar     (Eigen) scalar type
             * @tparam t_rows       (Eigen) number of rows
             * @tparam t_cols       (Eigen) number of columns
             * @tparam t_flags      (Eigen) flags
             *
             * @param[in,out] out   output stream
             * @param[in] name      name of the log entry
             * @param[in] matrix    matrix
             */
            template<typename t_Scalar, int t_rows, int t_cols, int t_flags>
                static void  formatAndOutputEntry(  std::ostream *out,
                                                    const std::string & name,
                                                    const Eigen::Matrix<t_Scalar, t_rows, t_cols, t_flags> &matrix)
            {
                *out << name << " = \\\n" << matrix.format(getEigenMatrixFormat()) << "\n";
            }

            /**
             * @brief Format and output a log entry (Eigen vector)
             *
             * @tparam t_Scalar     (Eigen) scalar type
             * @tparam t_rows       (Eigen) number of rows
             * @tparam t_flags      (Eigen) flags
             *
             * @param[in,out] out   output stream
             * @param[in] name      name of the log entry
             * @param[in] matrix    vector
             */
            template<typename t_Scalar, int t_rows, int t_cols, int t_flags>
                static void  formatAndOutputEntry(  std::ostream *out,
                                                    const std::string & name,
                                                    const Eigen::VectorXd &vector)
            {
                *out << name << " = \\\n" << vector.format(getEigenMatrixFormat()) << "\n";
            }

            /**
             * @brief Format and output a log entry (std::string)
             *
             * @tparam t_Data       data type
             * @param[in,out] out   output stream
             * @param[in] name      name of the log entry
             * @param[in] string    string
             */
            static void  formatAndOutputEntry(  std::ostream *out,
                                                const std::string & name,
                                                const std::string &string)
            {

                *out << name << " = '" << string << "';\n";
            }

            /**
             * @brief Format and output a log entry (double)
             *
             * @tparam t_Data       data type
             * @param[in,out] out   output stream
             * @param[in] name      name of the log entry
             * @param[in] data      data
             */
                static void  formatAndOutputEntry(  std::ostream *out,
                                                    const std::string & name,
                                                    const double & data)
            {
            	if(data==NAN)
            	{
                	*out << name << " = '" << "None" << "';\n";
                }
                else
                {
                	*out << name << " = '" << data << "';\n";
                }
            }

            /**
             * @brief Format and output a log entry (default)
             *
             * @tparam t_Data       data type
             * @param[in,out] out   output stream
             * @param[in] name      name of the log entry
             * @param[in] data      data
             */
            template <typename t_Data>
                static void  formatAndOutputEntry(  std::ostream *out,
                                                    const std::string & name,
                                                    const t_Data & data)
            {
                *out << name << " = '" << data << "';\n";
            }
    };



    /**
     * @brief A collection of functions which format data to be parsable by
     * MATLAB/Octave.
     */
    class HUMOTO_LOCAL OctaveFormatter
    {
        public:
            /**
             * @brief Returns format description for Eigen matrices.
             *
             * @return Eigen IO format
             */
            static const Eigen::IOFormat& getEigenMatrixFormat()
            {
                static const Eigen::IOFormat    eigen_output_format(Eigen::FullPrecision, 0, ", ", ";\n", "", "", "[", "];");
                return (eigen_output_format);
            }




            /**
             * @brief Add a part to the given log entry name.
             *
             * @param[in] name      name
             * @param[in] addition  subname
             */
            static void appendEntryName(std::string & name,
                                        const std::string & addition)
            {
                if ((name.size() > 0) && (addition.size() > 0))
                {
                    name += ".";
                }
                name += addition;
            }


            /**
             * @brief Add a part to the given log entry name.
             *
             * @param[in] name      name
             * @param[in] addition  subname
             */
            static void appendEntryName(std::string & name,
                                        const char * addition)
            {
                if ((name.size() > 0) && ('\0' != addition[0]))
                {
                    name += ".";
                }
                name += addition;
            }


            /**
             * @brief Add a part to the given log entry name.
             *
             * @param[in] name      name
             * @param[in] index     index of a subentry
             */
            static void appendEntryName(std::string & name,
                                        const std::size_t index)
            {
                name += "{";
                name += boost::lexical_cast<std::string>(index + 1);
                name += "}";
            }


            /**
             * @brief Format and output a log entry (std::vector)
             *
             * @tparam t_Scalar type of vector elements
             * @param[in,out] out   output stream
             * @param[in] name      name of the log entry
             * @param[in] vector    data vector
             */
            template <typename t_Scalar>
                static void  formatAndOutputEntry(  std::ostream *out,
                                                    const std::string & name,
                                                    const std::vector<t_Scalar> &vector)
            {
                *out << name << " = " << "[";
                for (std::size_t i = 0; i < vector.size(); ++i)
                {
                    *out << vector[i];
                    if (i + 1 != vector.size())
                    {
                        *out << ";";
                    }
                }
                *out << "];\n";
            }


            /**
             * @brief Format and output a log entry (Eigen matrix)
             *
             * @tparam t_Scalar     (Eigen) scalar type
             * @tparam t_rows       (Eigen) number of rows
             * @tparam t_cols       (Eigen) number of columns
             * @tparam t_flags      (Eigen) flags
             *
             * @param[in,out] out   output stream
             * @param[in] name      name of the log entry
             * @param[in] matrix    matrix
             */
            template<typename t_Scalar, int t_rows, int t_cols, int t_flags>
                static void  formatAndOutputEntry(  std::ostream *out,
                                                    const std::string & name,
                                                    const Eigen::Matrix<t_Scalar, t_rows, t_cols, t_flags> &matrix)
            {
                *out << name << " = ...\n" << matrix.format(getEigenMatrixFormat()) << "\n";
            }


            /**
             * @brief Format and output a log entry (std::string)
             *
             * @tparam t_Data       data type
             * @param[in,out] out   output stream
             * @param[in] name      name of the log entry
             * @param[in] string    string
             */
            static void  formatAndOutputEntry(  std::ostream *out,
                                                const std::string & name,
                                                const std::string &string)
            {
                *out << name << " = '" << string << "';\n";
            }


            /**
             * @brief Format and output a log entry (default)
             *
             * @tparam t_Data       data type
             * @param[in,out] out   output stream
             * @param[in] name      name of the log entry
             * @param[in] data      data
             */
            template <typename t_Data>
                static void  formatAndOutputEntry(  std::ostream *out,
                                                    const std::string & name,
                                                    const t_Data & data)
            {
                *out << name << " = " << data << ";\n";
            }
    };


    using Formatter = LOGGER_FORMATTER;

    /**
     * @brief Represents log entry name
     */
    template <class t_Formatter>
    class HUMOTO_LOCAL LogEntryNameTemplate
    {
        private:
            std::string     entry_name_;


        public:
            /**
             * @brief Default constructor
             */
            LogEntryNameTemplate()
            {
            }


            /**
             * @brief Constructor with name initialization
             *
             * @param[in] name
             */
            LogEntryNameTemplate (const std::string & name) : entry_name_(name)
            {
            }


            /**
             * @brief Constructor with name initialization
             *
             * @param[in] name
             */
            LogEntryNameTemplate (const char * name) : entry_name_(name)
            {
            }


            /**
             * @brief Constructor with name initialization
             *
             * @param[in] name
             */
            LogEntryNameTemplate (const LogEntryNameTemplate<t_Formatter> & name) : entry_name_(name.entry_name_)
            {
            }


            /**
             * @brief Returns entry name as a string
             *
             * @return string
             */
            const std::string& getAsString() const
            {
                return(entry_name_);
            }


            /**
             * @brief extends entry name with a subname
             *
             * @param[in] name      subname
             *
             * @return this
             */
            LogEntryNameTemplate<t_Formatter>& add(const char * name)
            {
                t_Formatter::appendEntryName(entry_name_, name);
                return (*this);
            }


            /**
             * @brief extends entry name with a subname
             *
             * @param[in] name      subname
             *
             * @return this
             */
            LogEntryNameTemplate<t_Formatter>& add(const std::string & name)
            {
                t_Formatter::appendEntryName(entry_name_, name);
                return (*this);
            }


            /**
             * @brief extends entry name with a subname
             *
             * @param[in] index     integer index
             *
             * @return this
             */
            LogEntryNameTemplate<t_Formatter>& add(const std::size_t index)
            {
                t_Formatter::appendEntryName(entry_name_, index);
                return (*this);
            }


            /**
             * @brief Writes the log entry name to a stream
             *
             * @param[in,out] out output stream
             * @param[in] log_entry_name
             *
             * @return output stream
             */
            friend std::ostream& operator<< (std::ostream& out, const LogEntryNameTemplate<t_Formatter> & log_entry_name)
            {
                out << log_entry_name.entry_name_;
                return(out);
            }
    };

    using LogEntryName = LogEntryNameTemplate<Formatter>; // Default formatter is Octave/Matlab

    /**
     * @brief Logger base class (stream handling)
     */
    class HUMOTO_LOCAL LoggerBase
    {
        private:
            /// Set if file stream is open
            bool                file_stream_is_open_;

            /// Output file stream (not used if a stream given to constructor)
            std::ofstream       output_file_stream_;


        private:
            /**
             * @brief Initialize output parameters
             */
            void setDefaults()
            {
                *output_stream_ << std::setprecision(std::numeric_limits<double>::digits10);
            }


            /**
             * @brief Cleanup before destruction or reinitialization.
             */
            void clean()
            {
                output_stream_ = NULL;

                if (file_stream_is_open_)
                {
                    output_file_stream_.close();
                }
            }



            /**
             * @brief Stream insertion operator
             *
             * @tparam t_Data output data type
             *
             * @param[in] data data
             *
             * @return Logger
             */
            template<typename t_Data>
                LoggerBase & operator<< (const t_Data & data)
            {
                HUMOTO_ASSERT(output_stream_ != NULL, "Logger is not properly initialized.");
                *output_stream_ << data;
                return (*this);
            }


            /**
             * @brief Stream manipulation
             *
             * @param[in] manipulator_function stream manipulation function
             *
             * @return Logger
             */
            LoggerBase& operator<< (std::ostream& (*manipulator_function)(std::ostream&))
            {
                HUMOTO_ASSERT(output_stream_ != NULL, "Logger is not properly initialized.");
                manipulator_function(*output_stream_);
                return (*this);
            }


        protected:
            /// Output stream
            std::ostream        *output_stream_;


        protected:
            /**
             * @brief Default constructor
             */
            LoggerBase()
            {
                file_stream_is_open_ = false;
                output_stream_ = NULL;
            }


            /**
             * @brief Constructor
             *
             * @param[in,out] output_stream output stream
             */
            explicit LoggerBase(std::ostream & output_stream)
            {
                initialize(output_stream);
            }


            /**
             * @brief Constructor
             *
             * @param[in] output_filename name of the log file
             */
            explicit LoggerBase(const std::string &output_filename)
            {
                initialize(output_filename);
            }


            /**
             * @brief Destructor
             */
            ~LoggerBase()
            {
                clean();
            }



        public:
            /**
             * @brief initialize
             *
             * @param[in,out] output_stream output stream
             */
            void initialize(std::ostream & output_stream)
            {
                file_stream_is_open_ = false;
                output_stream_ = &output_stream;

                setDefaults();
            }


            /**
             * @brief initialize
             *
             * @param[in] output_filename name of the log file
             */
            void initialize(const std::string &output_filename)
            {
                file_stream_is_open_ = true;

                output_file_stream_.open(output_filename.c_str());
                if (output_file_stream_.fail())
                {
                    HUMOTO_THROW_MSG(std::string("Could not open log file: ") + output_filename);
                }
                output_stream_ = &output_file_stream_;

                setDefaults();
            }
    };



#ifdef HUMOTO_USE_THREADS_FOR_LOGGING
    /**
     * @brief Virtual base class for representation of an enqueued log message.
     */
    class HUMOTO_LOCAL LogMessageBase
    {
        public:
            /**
             * @brief Destructor should be virtual
             */
            virtual ~LogMessageBase() {}


            /**
             * @brief Write message to a stream. This method must be
             * implemented in derived classes.
             *
             * @param[in] out output stream
             */
            virtual void write(std::ostream * out) = 0;
    };


    /**
     * @brief Standard (with a name) log message
     *
     * @tparam t_Data message data type
     */
    template <typename t_Data, class t_Formatter>
        class HUMOTO_LOCAL LogMessage : public LogMessageBase
    {
        public:
            LogEntryName    entry_name_;
            t_Data          data_;


        public:
            /**
             * @brief Construct message
             *
             * @param[in] entry_name
             * @param[in] data
             */
            LogMessage( const LogEntryName & entry_name,
                        const t_Data &data) : entry_name_(entry_name), data_(data)
            {
            }


            /**
             * @brief Default destructor
             */
            ~LogMessage() {}


            /// @copydoc humoto::LogMessageBase::write
            virtual void write(std::ostream * out)
            {
                t_Formatter::formatAndOutputEntry(out, entry_name_.getAsString(), data_);
            }
    };


    /**
     * @brief Raw log message (without a name)
     *
     * @tparam t_Data message data type
     */
    template <typename t_Data>
        class HUMOTO_LOCAL LogMessageRaw : public LogMessageBase
    {
        protected:
            t_Data  data_;


        public:
            /**
             * @brief Construct message
             *
             * @param[in] data
             */
            explicit LogMessageRaw(const t_Data &data) : data_(data)
            {
            }


            /**
             * @brief Default destructor
             */
            ~LogMessageRaw() {}


            /// @copydoc humoto::LogMessageBase::write
            virtual void write(std::ostream * out)
            {
                *out << data_ << "\n";
            }
    };



    /**
     * @brief Threaded logger: any data sent to this logger is wrapped in a
     * message and pushed to a queue, the queue is processed in a separate
     * thread. Hence, time consuming output does not delay execution of the
     * main thread(s).
     *
     * @attention A high frequency controller may easily flood the logger with
     * data.
     *
     * @attention This logger uses mutexes, so it should be possible to use a
     * single logger in multiple threads, however, this is not recommended.
     *
     * @todo Consider using a nonblocking queue, e.g., <boost/lockfree/queue.hpp>.
     */
    template <class t_Formatter>
    	class HUMOTO_LOCAL LoggerTemplate : public LoggerBase
    {
        public:
            std::size_t                     thread_sleep_ms_;

            std::queue<LogMessageBase *>    message_queue_;

            boost::mutex                    message_queue_mutex_;
            boost::thread                   processor_thread_;

            bool                            interrupted_;

            static const std::size_t        default_thread_sleep_ms_ = 2;


        private:
            /**
             * @brief Push message to a queue
             *
             * @param[in] msg message
             */
            void pushMessage(LogMessageBase * msg)
            {
                message_queue_mutex_.lock();
                message_queue_.push(msg);
                message_queue_mutex_.unlock();
            }


            /**
             * @brief This function is running in a separate thread.
             */
            void processQueue()
            {
                bool stop_thread = false;

                for (;;)
                {
                    std::size_t num_msg = 0;


                    if (interrupted_)
                    {
                        stop_thread = true;
                    }

                    message_queue_mutex_.lock();
                    num_msg = message_queue_.size();
                    message_queue_mutex_.unlock();


                    if (0 == num_msg)
                    {
                        if (false == stop_thread)
                        {
                            boost::this_thread::sleep_for(boost::chrono::milliseconds(thread_sleep_ms_));
                            continue;
                        }
                    }
                    else
                    {
                        for(std::size_t i = 0; i < num_msg; ++i)
                        {
                            LogMessageBase * msg = NULL;

                            message_queue_mutex_.lock();
                            msg = message_queue_.front();
                            message_queue_.pop();
                            message_queue_mutex_.unlock();

                            msg->write(output_stream_);
                            delete msg;
                        }

                        output_stream_->flush();
                    }

                    if (stop_thread)
                    {
                        return;
                    }
                }
            }


            /**
             * @brief Stop the logging thread if necessary.
             */
            void interrupt()
            {
                if (! interrupted_)
                {
                    interrupted_ = true;
                    processor_thread_.join();
                }
            }


            /**
             * @brief Start the thread.
             */
            void start()
            {
                interrupted_ = false;
                processor_thread_ = boost::thread(&humoto::LoggerTemplate<t_Formatter>::processQueue, this);
            }


        public:
            /**
             * @brief Constructor
             *
             * @param[in] thread_sleep_ms   how long the queue processing
             *                              thread should sleep if there is no data
             */
            explicit LoggerTemplate(const std::size_t thread_sleep_ms = default_thread_sleep_ms_)
            {
                interrupted_ = true;
                thread_sleep_ms_ = thread_sleep_ms;
            }


            /**
             * @brief Constructor
             *
             * @param[in,out] output_stream output stream
             * @param[in] thread_sleep_ms   how long the queue processing
             *                              thread should sleep if there is no data
             */
            LoggerTemplate( std::ostream & output_stream,
                    const std::size_t thread_sleep_ms = default_thread_sleep_ms_) : LoggerBase(output_stream)
            {
                thread_sleep_ms_ = thread_sleep_ms;
                start();
            }


            /**
             * @brief Constructor
             *
             * @param[in] output_filename   name of the log file
             * @param[in] thread_sleep_ms   how long the queue processing
             *                              thread should sleep if there is no data
             */
            LoggerTemplate( const std::string &output_filename,
                    const std::size_t thread_sleep_ms = default_thread_sleep_ms_) : LoggerBase(output_filename)
            {
                thread_sleep_ms_ = thread_sleep_ms;
                start();
            }


            /**
             * @brief Destructor
             */
            ~LoggerTemplate()
            {
                interrupt();
            }


            /**
             * @brief initialize
             *
             * @param[in,out] output_stream output stream
             * @param[in] thread_sleep_ms   how long the queue processing
             *                              thread should sleep if there is no data
             */
            void initialize(std::ostream & output_stream,
                            const std::size_t thread_sleep_ms = default_thread_sleep_ms_)
            {
                interrupt();

                LoggerBase::initialize(output_stream);
                thread_sleep_ms_ = thread_sleep_ms;

                start();
            }


            /**
             * @brief initialize
             *
             * @param[in] output_filename name of the log file
             * @param[in] thread_sleep_ms   how long the queue processing
             *                              thread should sleep if there is no data
             */
            void initialize(const std::string &output_filename,
                            const std::size_t thread_sleep_ms = default_thread_sleep_ms_)
            {
                interrupt();

                LoggerBase::initialize(output_filename);
                thread_sleep_ms_ = thread_sleep_ms;

                start();
            }



            /**
             * @brief Log raw data
             *
             * @tparam t_Data data type
             *
             * @param[in] data
             * @param[in] dummy dummy parameter for enabling/disabling of the template
             *
             * @attention This template relies on Boost to filter out Eigen
             * types, which cannot be treated by it safely -- the data might
             * not be copied properly.
             */
            template <typename t_Data>
                void log(   const t_Data &data,
                            EIGENTOOLS_EIGENTYPE_DISABLER_TYPE(t_Data) * dummy = NULL)
            {
                pushMessage(new LogMessageRaw<t_Data>(data));
            }


            /**
             * @brief Log entry (Eigen matrix)
             *
             * @tparam t_Derived    Eigen template parameter
             * @param[in] matrix    matrix
             */
            template<typename t_Derived>
                void  log(const Eigen::DenseBase<t_Derived> &matrix)
            {
                pushMessage(  new LogMessageRaw< Eigen::Matrix<
                                typename Eigen::DenseBase<t_Derived>::Scalar,
                                Eigen::DenseBase<t_Derived>::RowsAtCompileTime,
                                Eigen::DenseBase<t_Derived>::ColsAtCompileTime,
                                ((Eigen::DenseBase<t_Derived>::RowsAtCompileTime == Eigen::Dynamic)
                                    || (Eigen::DenseBase<t_Derived>::ColsAtCompileTime == Eigen::Dynamic))
                                        ? Eigen::AutoAlign
                                            | (Eigen::DenseBase<t_Derived>::IsRowMajor
                                                ? Eigen::RowMajor
                                                : Eigen::ColMajor)
                                        : EIGENTOOLS_CONSTANT_SIZE_ALIGN_TYPE
                                            | (Eigen::DenseBase<t_Derived>::IsRowMajor
                                                ? Eigen::RowMajor
                                                : Eigen::ColMajor)> >(matrix.eval())  );
            }


            /**
             * @brief Log raw data (char string)
             *
             * @param[in] string
             */
            void log(const char * string)
            {
                pushMessage(new LogMessageRaw<std::string>(string));
            }



            /**
             * @brief Log data (fundamental types)
             *
             * @tparam t_Data data type
             *
             * @param[in] name
             * @param[in] data
             * @param[in] dummy dummy parameter for enabling/disabling of the template
             *
             * @attention This template relies on Boost to filter out Eigen
             * types, which cannot be treated by it safely -- the data might
             * not be copied properly.
             */
            template <typename t_Data>
                void  log(  const LogEntryName & name,
                            const t_Data & data,
                            EIGENTOOLS_EIGENTYPE_DISABLER_TYPE(t_Data) * dummy = NULL)
            {
                pushMessage(new LogMessage<t_Data, t_Formatter>(name, data));
            }


            /**
             * @brief Log entry (Eigen matrix)
             *
             * @tparam t_Derived    Eigen template parameter
             * @param[in] name      name of the log entry
             * @param[in] matrix    matrix
             */
            template<typename t_Derived>
                void  log(  const LogEntryName & name,
                            const Eigen::DenseBase<t_Derived> &matrix)
            {
                pushMessage(  new LogMessage< Eigen::Matrix<
                                typename Eigen::DenseBase<t_Derived>::Scalar,
                                Eigen::DenseBase<t_Derived>::RowsAtCompileTime,
                                Eigen::DenseBase<t_Derived>::ColsAtCompileTime,
                                ((Eigen::DenseBase<t_Derived>::RowsAtCompileTime == Eigen::Dynamic)
                                    || (Eigen::DenseBase<t_Derived>::ColsAtCompileTime == Eigen::Dynamic))
                                        ? Eigen::AutoAlign
                                            | (Eigen::DenseBase<t_Derived>::IsRowMajor
                                                ? Eigen::RowMajor
                                                : Eigen::ColMajor)
                                        : EIGENTOOLS_CONSTANT_SIZE_ALIGN_TYPE
                                            | (Eigen::DenseBase<t_Derived>::IsRowMajor
                                                ? Eigen::RowMajor
                                                : Eigen::ColMajor)>, t_Formatter >(name, matrix.eval())  );
            }


            /**
             * @brief Log data (char string)
             *
             * @param[in] name
             * @param[in] string
             */
            void  log(  const LogEntryName & name,
                        const char *string)
            {
                pushMessage(new LogMessage<std::string, t_Formatter>(name, string));
            }
    };
#else
    /**
     * @brief Basic logger.
     *
     * @attention NOT thread safe!
     */
    template <class t_Formatter>
    class HUMOTO_LOCAL LoggerTemplate : public LoggerBase
    {
        public:
            /**
             * @brief Default constructor
             */
            LoggerTemplate()
            {
            }


            /**
             * @brief Constructor
             *
             * @param[in,out] output_stream output stream
             */
            explicit LoggerTemplate( std::ostream & output_stream) : LoggerBase(output_stream)
            {
            }


            /**
             * @brief Constructor
             *
             * @param[in] output_filename name of the log file
             */
            explicit LoggerTemplate( const std::string &output_filename) : LoggerBase(output_filename)
            {
            }


            /**
             * @brief Log raw data
             *
             * @tparam t_Data data type
             *
             * @param[in] data
             */
            template <typename t_Data>
                void log(const t_Data &data)
            {
                *output_stream_ << data << "\n";
                output_stream_->flush();
            }



            /**
             * @brief Log entry (Eigen matrix)
             *
             * @tparam t_Derived    Eigen template parameter
             * @param[in] name      name of the log entry
             * @param[in] matrix    matrix
             */
            template<typename t_Derived>
                void  log(  const LogEntryName & name,
                            const Eigen::DenseBase<t_Derived> &matrix)
            {
                t_Formatter::formatAndOutputEntry(output_stream_, name.getAsString(), matrix.eval());
                output_stream_->flush();
            }



            /**
             * @brief Log data (fundamental types)
             *
             * @tparam t_Data data type
             *
             * @param[in] name
             * @param[in] data
             * @param[in] dummy dummy parameter for enabling/disabling of the template
             *
             * @attention This template relies on Boost to filter out Eigen
             * types, which cannot be treated by it safely.
             */
            template <typename t_Data>
                void  log(  const LogEntryName & name,
                            const t_Data & data,
                            EIGENTOOLS_EIGENTYPE_DISABLER_TYPE(t_Data) * dummy = NULL)
            {
                t_Formatter::formatAndOutputEntry(output_stream_, name.getAsString(), data);
                output_stream_->flush();
            }
    }; 
#endif // HUMOTO_USE_THREADS_FOR_LOGGING

    using HUMOTO_LOCAL Logger = HUMOTO_LOCAL LoggerTemplate<Formatter>;

}


#ifdef HUMOTO_GLOBAL_LOGGER_ENABLED

namespace humoto
{
    /**
     * @brief Global logger variable.
     */
    extern humoto::Logger g_logger;
}

/// Name of the global logger
#define HUMOTO_GLOBAL_LOGGER                    humoto::g_logger

#define HUMOTO_GLOBAL_LOGGER_IF_DEFINED         = HUMOTO_GLOBAL_LOGGER

/// Initialize logger (must be called in the global scope)
#define HUMOTO_INITIALIZE_GLOBAL_LOGGER(output) humoto::Logger HUMOTO_GLOBAL_LOGGER(output)

/// Log message
#define HUMOTO_LOG_RAW(message)                 HUMOTO_GLOBAL_LOGGER.log(message)

/// Log an data with a name
#define HUMOTO_LOG(name, data)                  HUMOTO_GLOBAL_LOGGER.log(name, data)

#else //HUMOTO_GLOBAL_LOGGER_ENABLED


#define HUMOTO_GLOBAL_LOGGER_IF_DEFINED
#define HUMOTO_INITIALIZE_GLOBAL_LOGGER(output)
#define HUMOTO_INITIALIZE_LOGGER(stream)
#define HUMOTO_LOG_RAW(message)
#define HUMOTO_LOG(name, data)


#endif //HUMOTO_GLOBAL_LOGGER_ENABLED
