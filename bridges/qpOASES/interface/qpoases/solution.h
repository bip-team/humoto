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
    namespace qpoases
    {
        /**
         * @brief Solution of a QP.
         */
        class HUMOTO_LOCAL Solution : public humoto::Solution
        {
            public:
                /// Number of iterations made by the solver
                int     number_of_iterations_;

                /// The return value of qpOASES.
                qpOASES::returnValue    qpoases_return_value_;


            public:
                /**
                 * @brief Status description
                 *
                 * @return description of the status if available
                 */
                std::string getStatusDescription() const
                {
                    std::string description = qpOASES::MessageHandling::getErrorCodeMessage(qpoases_return_value_);

                    std::stringstream description_stream;
                    description_stream << "(CODE: " << qpoases_return_value_ << ") " << description;

                    return (description_stream.str());
                }


                /**
                 * @brief Log a QP problem
                 *
                 * @param[in,out] logger logger
                 * @param[in] parent parent
                 * @param[in] name name
                 */
                void log(   humoto::Logger &logger HUMOTO_GLOBAL_LOGGER_IF_DEFINED,
                            const LogEntryName &parent = LogEntryName(),
                            const std::string &name = "solution") const
                {
                    LogEntryName subname = parent; subname.add(name);

                    humoto::Solution::log(logger, subname, "");

                    logger.log(LogEntryName(subname).add("number_of_iterations"), number_of_iterations_);
                    logger.log(LogEntryName(subname).add("qpoases_return_value"), qpoases_return_value_);
                }
        };
    }
}
