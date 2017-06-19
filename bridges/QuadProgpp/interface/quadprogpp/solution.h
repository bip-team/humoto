/**
    @file
    @author  Alexander Sherikov

    @brief
*/


#pragma once


namespace humoto
{
    namespace quadprogpp
    {
        /**
         * @brief Solution of a QP.
         */
        class HUMOTO_LOCAL Solution : public humoto::Solution
        {
            public:
                /// The return value of quadprogpp.
                double  quadprogpp_return_value_;


            public:
                /**
                 * @brief Status description
                 *
                 * @return description of the status if available
                 */
                std::string getStatusDescription() const
                {
                    if (quadprogpp_return_value_ == std::numeric_limits<double>::infinity())
                    {
                        return ("quadprogpp: an infeasible or degenerate problem.");
                    }
                    else
                    {
                        return ("quadprogpp: Ok.");
                    }
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

                    logger.log(LogEntryName(subname).add("quadprogpp_return_value"), quadprogpp_return_value_);
                }
        };
    }
}
