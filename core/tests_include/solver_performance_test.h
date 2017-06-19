/**
    @file
    @author  Alexander Sherikov
    @author  Jan Michalczyk
    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

namespace humoto_tests
{
    class SolverPerformanceSingleRunBase
    {
        protected:
            // time logging
            humoto::Timer                       timer_global_;
            humoto::Timer                       timer_form_problem_;
            humoto::Timer                       timer_solve_problem_;

        public:
            virtual void run(std::vector<Eigen::Vector3d> & measures) = 0;
            virtual void initialize (const std::string & config_path) = 0;
    };


    template    <class t_SolverPerformanceSingleRun>
    class SolverPerformanceMultiRun
    {
        public:
            static  void run(   humoto::Logger & logger,
                                const std::size_t n_of_simulations,
                                const std::string &config_path,
                                const humoto::LogEntryName & log_prefix,
                                const std::string &hotstart_type)
            {
                std::vector<Eigen::MatrixXd>    all_measures;

                all_measures.resize(n_of_simulations);
                for (std::size_t i = 0; i < n_of_simulations; ++i)
                {
                    t_SolverPerformanceSingleRun    single_run;
                    single_run.initialize(config_path);

                    std::vector<Eigen::Vector3d>    measures;
                    single_run.run(measures);

                    all_measures[i].resize(3, measures.size());
                    for (std::size_t j = 0; j < measures.size(); ++j)
                    {
                        all_measures[i].col(j) = measures[j];
                    }

                    std::cout << "." << std::flush;
                }


                Eigen::MatrixXd     all_measures_sum = all_measures[0];
                for (std::size_t i = 1; i < n_of_simulations; ++i)
                {
                    all_measures_sum += all_measures[i];
                }


                double nfactor = 1./static_cast<double>(n_of_simulations);

                logger.log(humoto::LogEntryName(log_prefix).add(hotstart_type).add("average_time_to_form_the_problem") , nfactor * all_measures_sum.row(0));
                logger.log(humoto::LogEntryName(log_prefix).add(hotstart_type).add("average_time_to_solve_the_problem"), nfactor * all_measures_sum.row(1));
                logger.log(humoto::LogEntryName(log_prefix).add(hotstart_type).add("average_time_to_run_full_loop")    , nfactor * all_measures_sum.row(2));

                std::cout << std::endl;
            }
    };
}
