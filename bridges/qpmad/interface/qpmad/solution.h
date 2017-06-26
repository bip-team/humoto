/**
    @file
    @author  Alexander Sherikov

    @brief
*/


#pragma once


namespace humoto
{
    namespace qpmad
    {
        /**
         * @brief Solution of a QP.
         */
        class HUMOTO_LOCAL Solution : public humoto::Solution
        {
            public:
                /// The return value of qpmad.
                ::qpmad::Solver::ReturnStatus qpmad_return_value_;


            public:
                /**
                 * @brief Status description
                 *
                 * @return description of the status if available
                 */
                std::string getStatusDescription() const
                {
                    switch(qpmad_return_value_)
                    {
                        case ::qpmad::Solver::OK:
                            return ("qpmad: Ok.");
                        default:
                            return ("qpmad: Some error.");
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

                    logger.log(LogEntryName(subname).add("qpmad_return_value"), qpmad_return_value_);
                }
        };
    }
}
