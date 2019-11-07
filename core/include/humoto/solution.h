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
     * @brief Return status of a solver
     */
    class SolverStatus
    {
        public:
            enum Status
            {
                UNDEFINED = 0,
                OK        = 1,
                MAX_ITER  = 2,
                OTHER     = 3
            };
    };


    /**
     * @brief Analog of 'sol_structure' struct in Octave code. [determine_solution_structure.m]
     */
    class HUMOTO_LOCAL SolutionStructure
    {
        protected:
            std::size_t                         number_of_variables_;
            std::map<std::string, Location>     variable_map_;


        public:
            /**
             * @brief Virtual destructor
             */
            virtual ~SolutionStructure() {}


            /**
             * @brief Default constructor
             */
            SolutionStructure()
            {
                reset();
            }


            /**
             * @brief Reset
             */
            void reset()
            {
                number_of_variables_ = 0;
                variable_map_.clear();
            }


            /**
             * @brief Add part of the solution
             *
             * @param[in] id string id of the solution part
             * @param[in] length length of the solution part
             */
            void addSolutionPart(   const std::string   &id,
                                    const std::size_t  length)
            {
                Location location(number_of_variables_, length);

                variable_map_.insert(std::pair< std::string, Location > (id, location));

                number_of_variables_ += length;
            }


            /**
             * @brief Checks if the structure is empty or not.
             *
             * @return true / false
             */
            bool isNonEmpty() const
            {
                if (variable_map_.size() == 0)
                {
                    return (false);
                }
                else
                {
                    return (true);
                }
            }


            /**
             * @brief Get total number of variables in the solution vector.
             *
             * @return number of variables.
             *
             * @attention This method should be called only after addition of
             * all solution parts.
             */
            std::size_t getNumberOfVariables() const
            {
                return (number_of_variables_);
            }


            /**
             * @brief Get location of a data in the solution vector
             *
             * @param[in] id id of the data block
             *
             * @return Location
             */
            Location getSolutionPartLocation(const std::string &id) const
            {
                std::map<std::string, Location>::const_iterator it;

                it = variable_map_.find(id);

                if (it == variable_map_.end())
                {
                    return (Location(0,0));
                }
                else
                {
                    return( it->second );
                }
            }


            /**
             * @brief Get part of a matrix corresponding to given part of the solution.
             *
             * @param[in] id id of the data block
             * @param[in] matrix matrix
             *
             * @return
             */
            Eigen::Block<Eigen::MatrixXd> getMatrixPart(const std::string &id, Eigen::MatrixXd & matrix) const
            {
                Location loc_var = getSolutionPartLocation(id);

                return (matrix.block(0, loc_var.offset_, matrix.rows(), loc_var.length_));
            }


            /**
             * @brief Get part of a matrix corresponding to given part of the solution.
             *
             * @param[in] id id of the data block
             * @param[in] matrix matrix
             *
             * @return
             */
            const Eigen::Block<const Eigen::MatrixXd> getMatrixPart(const std::string &id, const Eigen::MatrixXd & matrix) const
            {
                Location loc_var = getSolutionPartLocation(id);

                return (matrix.block(0, loc_var.offset_, matrix.rows(), loc_var.length_));
            }
    };



    /**
     * @brief Container of the solution
     */
    class HUMOTO_LOCAL Solution : public SolutionStructure, public humoto::config::ConfigurableBase
    {
        #define HUMOTO_CONFIG_SECTION_ID "Solution"
        #define HUMOTO_CONFIG_CONSTRUCTOR Solution
        #define HUMOTO_CONFIG_ENTRIES \
                HUMOTO_CONFIG_COMPOUND_(x)
        #include HUMOTO_CONFIG_DEFINE_ACCESSORS


        public:
            Eigen::VectorXd             x_;
            Eigen::VectorXd             lambda_;
            SolverStatus::Status  return_status_;


        public:
            Solution()
            {
                return_status_ = SolverStatus::UNDEFINED;
            }


            /**
             * @brief Virtual destructor
             */
            virtual ~Solution() {}


            /**
             * @brief No defaults
             */
            void setDefaults() {}


            /**
             * @brief Initialize the solution vector
             *
             * @param[in] sol_structure structure of the solution
             */
            void initialize(const SolutionStructure &sol_structure)
            {
                reset();
                SolutionStructure::operator=(sol_structure);
                x_.resize(getNumberOfVariables());
            }



            /**
             * @brief Initialize the solution vector
             *
             * @param[in] sol_structure structure of the solution
             * @param[in] value
             */
            void initialize(const SolutionStructure &sol_structure,
                            const double value)
            {
                reset();
                SolutionStructure::operator=(sol_structure);
                if (value == 0.0)
                {
                    x_.setZero(getNumberOfVariables());
                }
                else
                {
                    x_.setConstant(getNumberOfVariables(), value);
                }
            }


            /**
             * @brief Initialize the solution vector using an old solution
             *
             * @param[in] sol_structure structure of the solution
             * @param[in] old_solution old solution
             */
            void initialize(const SolutionStructure &sol_structure,
                            const Solution          &old_solution)
            {
                initialize(sol_structure, 0.0);


                for (std::map<std::string, Location>::const_iterator i = variable_map_.begin();
                     i != variable_map_.end();
                     ++i)
                {
                    Location old_part = old_solution.getSolutionPartLocation(i->first);
                    Location guess_part = i->second;


                    if (old_part.length_ == guess_part.length_)
                    {
                        getData(guess_part) = old_solution.getData(old_part);
                    }
                }
            }



            /**
             * @brief Get part of the solution by its id
             *
             * @param[in] id            id of the solution part
             *
             * @return part of the solution (not a copy!)
             */
            Eigen::VectorBlock<Eigen::VectorXd> getSolutionPart(const std::string &id)
            {
                return (  getData( getSolutionPartLocation(id) )  );
            }


            /**
             * @brief Get data from solution vector.
             *
             * @param[in] location location of the data
             *
             * @return data (not a copy!)
             */
            Eigen::VectorBlock<Eigen::VectorXd> getData(const Location &location)
            {
                HUMOTO_ASSERT(location.checkLength(x_.rows()), "Incorrect data length or offset.")

                return (x_.segment(location.offset_, location.length_));
            }


            /**
             * @brief Get part of the solution by its id
             *
             * @param[in] id            id of the solution part
             *
             * @return part of the solution
             */
            Eigen::VectorXd getSolutionPart(const std::string &id) const
            {
                return (  getData( getSolutionPartLocation(id) )  );
            }



            /**
             * @brief Get data from solution vector.
             *
             * @param[in] location location of the data
             *
             * @return data
             */
            Eigen::VectorXd getData(const Location &location) const
            {
                HUMOTO_ASSERT(location.checkLength(x_.rows()), "Incorrect data length or offset.")

                return (x_.segment(location.offset_, location.length_));
            }



            /**
             * @brief Returns solution vector
             *
             * @return Solution vector (not a copy!)
             */
            const Eigen::VectorXd & get_x() const
            {
                return(x_);
            }



            /**
             * @brief Log a QP problem
             *
             * @param[in,out] logger logger
             * @param[in] parent parent
             * @param[in] name name
             */
            virtual void log(   humoto::Logger &logger HUMOTO_GLOBAL_LOGGER_IF_DEFINED,
                                const LogEntryName & parent = LogEntryName(),
                                const std::string & name = "solution") const
            {
                LogEntryName subname = parent;
                subname.add(name);

                logger.log(LogEntryName(subname).add("x"), x_);
                logger.log(LogEntryName(subname).add("status"), return_status_);
            }



            /**
             * @brief Status description
             *
             * @return description of the status if available
             */
            virtual std::string getStatusDescription() const
            {
                std::string description("Status description is not available.");
                return (description);
            }
    };
}
