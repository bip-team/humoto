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
    namespace lexls
    {
        /**
         * @brief Solution of a hierarchy.
         */
        class HUMOTO_LOCAL Solution : public humoto::Solution
        {
            public:
                std::ptrdiff_t number_of_activations_;
                std::ptrdiff_t number_of_deactivations_;
                std::ptrdiff_t number_of_factorizations_;
                std::ptrdiff_t cycling_counter_;


                /// The return value of LexLS.
                LexLS::TerminationStatus    lexls_termination_status_;


            public:
                /**
                 * @brief Log solution
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

                    logger.log(LogEntryName(subname).add("number_of_activations"), number_of_activations_);
                    logger.log(LogEntryName(subname).add("number_of_deactivations"), number_of_deactivations_);
                    logger.log(LogEntryName(subname).add("number_of_factorizations"), number_of_factorizations_);
                    logger.log(LogEntryName(subname).add("cycling_counter"), cycling_counter_);
                }
        };
    }
}
