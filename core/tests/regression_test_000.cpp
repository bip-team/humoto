/**
    @file
    @author  Alexander Sherikov
    @author  Jan Michalczyk
    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#include <iostream>
#include <limits>
#include <iomanip>

#define HUMOTO_GLOBAL_LOGGER_ENABLED

// Enable YAML configuration files (must be first)
#include "humoto/config_yaml.h"
#include "humoto/config_msgpack.h"
// common & abstract classes
#include "humoto/humoto.h"
// walking-related classes
#include "humoto/walking.h"

//testing
#include "gtest/gtest.h"

HUMOTO_INITIALIZE_GLOBAL_LOGGER(std::cout);

//#undef HUMOTO_GLOBAL_LOGGER_ENABLED


class ConfigInterfaceTest : public ::testing::Test
{
    protected:
        template<class t_Reader, class t_Writer>
            void test()
        {
            {
                humoto::walking::StanceFSMParameters stance_fsm_parameters;

                t_Writer writer("stance_fsm_config1.cfg");
                stance_fsm_parameters.writeConfig(writer);
            }

            {
                humoto::walking::StanceFSMParameters stance_fsm_parameters;

                t_Reader reader("stance_fsm_config1.cfg");
                stance_fsm_parameters.readConfig(reader);
            }

            // --------------------------------

            {
                humoto::walking::StanceFSMParameters stance_fsm_parameters;
                stance_fsm_parameters.writeConfig<t_Writer>("stance_fsm_config2.cfg");
            }

            {
                humoto::walking::StanceFSMParameters stance_fsm_parameters;
                stance_fsm_parameters.readConfig<t_Reader>("stance_fsm_config2.cfg");
            }
        }
};


class ConfigMatchTest : public ::testing::Test
{
    protected:
        void    initialize(humoto::walking::StanceFSMParameters &stance_fsm_parameters)
        {
            stance_fsm_parameters.first_ss_type_            = humoto::walking::StanceType::TDS;
            stance_fsm_parameters.first_stance_             = humoto::walking::StanceType::RSS;
            stance_fsm_parameters.last_stance_              = humoto::walking::StanceType::LSS;
            stance_fsm_parameters.ss_duration_ms_           = 666;
            stance_fsm_parameters.tds_duration_ms_          = 777;
            stance_fsm_parameters.first_stance_duration_ms_ = 888;
            stance_fsm_parameters.last_stance_duration_ms_  = 999;
            stance_fsm_parameters.num_steps_                = 555;
        }


        void    compare(const humoto::walking::StanceFSMParameters &stance_fsm_parameters_out,
                        const humoto::walking::StanceFSMParameters &stance_fsm_parameters_in)
        {
            EXPECT_EQ(stance_fsm_parameters_out.first_ss_type_,            stance_fsm_parameters_in.first_ss_type_);
            EXPECT_EQ(stance_fsm_parameters_out.first_stance_,             stance_fsm_parameters_in.first_stance_);
            EXPECT_EQ(stance_fsm_parameters_out.last_stance_,              stance_fsm_parameters_in.last_stance_);
            EXPECT_EQ(stance_fsm_parameters_out.ss_duration_ms_,           stance_fsm_parameters_in.ss_duration_ms_);
            EXPECT_EQ(stance_fsm_parameters_out.tds_duration_ms_,          stance_fsm_parameters_in.tds_duration_ms_);
            EXPECT_EQ(stance_fsm_parameters_out.first_stance_duration_ms_, stance_fsm_parameters_in.first_stance_duration_ms_);
            EXPECT_EQ(stance_fsm_parameters_out.last_stance_duration_ms_,  stance_fsm_parameters_in.last_stance_duration_ms_);
            EXPECT_EQ(stance_fsm_parameters_out.num_steps_,                stance_fsm_parameters_in.num_steps_);
        }


        template<class t_Reader, class t_Writer>
            void testSimple()
        {
            humoto::walking::StanceFSMParameters stance_fsm_parameters_out;
            initialize(stance_fsm_parameters_out);
            stance_fsm_parameters_out.writeConfig<t_Writer>("stance_fsm_config_match_simple.cfg");

            // -------

            humoto::walking::StanceFSMParameters stance_fsm_parameters_in;
            stance_fsm_parameters_in.readConfig<t_Reader>("stance_fsm_config_match_simple.cfg");
            compare(stance_fsm_parameters_out, stance_fsm_parameters_in);
        }


        template<class t_Reader, class t_Writer>
            void testMulti()
        {
            humoto::walking::StanceFSMParameters stance_fsm_parameters_out;
            initialize(stance_fsm_parameters_out);
            {
                t_Writer writer("stance_fsm_config_match_multi.cfg");
                stance_fsm_parameters_out.writeConfig(writer, "node1");
                stance_fsm_parameters_out.writeConfig(writer, "node2");
            }

            // -------

            humoto::walking::StanceFSMParameters stance_fsm_parameters_in1;
            humoto::walking::StanceFSMParameters stance_fsm_parameters_in2;
            t_Reader reader("stance_fsm_config_match_multi.cfg");
            stance_fsm_parameters_in1.readConfig(reader, "node1");
            stance_fsm_parameters_in2.readConfig(reader, "node2");
            compare(stance_fsm_parameters_out, stance_fsm_parameters_in1);
            compare(stance_fsm_parameters_out, stance_fsm_parameters_in2);
        }
};



class ConfigHierarchyMatchTest : public ::testing::Test
{
    protected:
        template<class t_Reader, class t_Writer>
            void test()
        {
            humoto::ConfigurableOptimizationProblem             opt_problem_out;

            humoto::TaskSharedPointer   task1 (new humoto::TaskInfeasibleInequality(1.0, "task1"));
            humoto::TaskSharedPointer   task2 (new humoto::TaskInfeasibleInequality(1.0, "task2"));
            humoto::TaskSharedPointer   task3 (new humoto::TaskZeroVariables(1.0, "task3"));
            humoto::TaskSharedPointer   task4 (new humoto::TaskInfeasibleInequality(1.0, "task4"));

            opt_problem_out.reset(3);

            opt_problem_out.pushTask(task1, 0, "TaskInfeasibleInequality");
            opt_problem_out.pushTask(task2, 0, "TaskInfeasibleInequality");

            opt_problem_out.pushTask(task3, 1, "TaskZeroVariables");

            opt_problem_out.pushTask(task4, 2, "TaskInfeasibleInequality");

            opt_problem_out.writeConfig<t_Writer>("hierarchy_match.cfg");

            // -------

            humoto::ConfigurableOptimizationProblem             opt_problem_in;
            opt_problem_in.readConfig<t_Reader>("hierarchy_match.cfg");

            // -------

            EXPECT_EQ(opt_problem_out.getNumberOfLevels(),  opt_problem_in.getNumberOfLevels());
            if (opt_problem_out.getNumberOfLevels() == opt_problem_in.getNumberOfLevels())
            {
                for (std::size_t i = 0; i < opt_problem_in.getNumberOfLevels(); ++i)
                {
                    EXPECT_EQ(opt_problem_out[i].getNumberOfConstraints(),  opt_problem_in[i].getNumberOfConstraints());
                    EXPECT_EQ(opt_problem_out[i].tasks_.size(),             opt_problem_in[i].tasks_.size());

                    if (opt_problem_out[i].tasks_.size() == opt_problem_in[i].tasks_.size())
                    {
                        std::list<humoto::HierarchyLevel::TaskInfo>::const_iterator it_out = opt_problem_out[i].tasks_.begin();
                        std::list<humoto::HierarchyLevel::TaskInfo>::const_iterator it_in = opt_problem_in[i].tasks_.begin();

                        for (; it_out != opt_problem_out[i].tasks_.end(); ++it_out, ++it_in)
                        {
                            EXPECT_EQ(it_out->ptr_->getDescription(), it_in->ptr_->getDescription());
                        }
                    }
                }
            }
        }
};


TEST_F(ConfigInterfaceTest, ConfigInterfaceYAML)
{
    test<humoto::config::yaml::Reader, humoto::config::yaml::Writer>();
}


TEST_F(ConfigInterfaceTest, ConfigInterfaceMSGPACK)
{
    test<humoto::config::msgpack::Reader, humoto::config::msgpack::Writer>();
}


//---------------


TEST_F(ConfigMatchTest, ConfigMatchSimpleYAML)
{
    testSimple<humoto::config::yaml::Reader, humoto::config::yaml::Writer>();
}


TEST_F(ConfigMatchTest, ConfigMatchSimpleMSGPACK)
{
    testSimple<humoto::config::msgpack::Reader, humoto::config::msgpack::Writer>();
}


TEST_F(ConfigMatchTest, ConfigMatchMultiYAML)
{
    testMulti<humoto::config::yaml::Reader, humoto::config::yaml::Writer>();
}


TEST_F(ConfigMatchTest, ConfigMatchMultiMSGPACK)
{
    testMulti<humoto::config::msgpack::Reader, humoto::config::msgpack::Writer>();
}


//---------------

TEST_F(ConfigHierarchyMatchTest, ConfigMatchHierarchyYAML)
{
    test<humoto::config::yaml::Reader, humoto::config::yaml::Writer>();
}


TEST_F(ConfigHierarchyMatchTest, ConfigMatchHierarchyMSGPACK)
{
    test<humoto::config::msgpack::Reader, humoto::config::msgpack::Writer>();
}



/**
 * @brief main
 *
 * @param[in] argc number of args
 * @param[in] argv args
 *
 * @return status
 */
int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
