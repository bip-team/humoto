/**
    @file
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
// common & abstract classes
#include "humoto/humoto.h"
// walking-related classes
#include "humoto/walking.h"

//testing
#include "gtest/gtest.h"

HUMOTO_INITIALIZE_GLOBAL_LOGGER(std::cout);

//#undef HUMOTO_GLOBAL_LOGGER_ENABLED


class ConfigTest : public ::testing::Test
{
    protected:
        /**
         * @brief Config write and read
        */
        void ConfigWriteAndRead()
        {
            {
                humoto::walking::StanceFSMParameters stance_fsm_parameters;

                humoto::config::Writer writer("stance_fsm_config1.yaml");
                stance_fsm_parameters.writeConfig(writer);
            }

            {
                humoto::walking::StanceFSMParameters stance_fsm_parameters;

                humoto::config::Reader reader("stance_fsm_config1.yaml");
                stance_fsm_parameters.readConfig(reader);
            }

            // --------------------------------

            {
                humoto::walking::StanceFSMParameters stance_fsm_parameters;
                stance_fsm_parameters.writeConfig("stance_fsm_config2.yaml");
            }

            {
                humoto::walking::StanceFSMParameters stance_fsm_parameters;
                stance_fsm_parameters.readConfig("stance_fsm_config2.yaml");
            }
        }
};


TEST_F(ConfigTest, ConfigWriteAndRead)
{
    ConfigWriteAndRead();
}

TEST_F(ConfigTest, ConfigWriteMatchesRead)
{
    humoto::walking::StanceFSMParameters stance_fsm_parameters_out;

    stance_fsm_parameters_out.first_ss_type_            = humoto::walking::StanceType::TDS;
    stance_fsm_parameters_out.first_stance_             = humoto::walking::StanceType::RSS;
    stance_fsm_parameters_out.last_stance_              = humoto::walking::StanceType::LSS;
    stance_fsm_parameters_out.ss_duration_ms_           = 666;
    stance_fsm_parameters_out.tds_duration_ms_          = 777;
    stance_fsm_parameters_out.first_stance_duration_ms_ = 888;
    stance_fsm_parameters_out.last_stance_duration_ms_  = 999;
    stance_fsm_parameters_out.num_steps_                = 555;

    {
        humoto::config::Writer writer("stance_fsm_config3.yaml");
        stance_fsm_parameters_out.writeConfig(writer);
    }

    humoto::walking::StanceFSMParameters stance_fsm_parameters_in;
    humoto::config::Reader reader("stance_fsm_config3.yaml");
    stance_fsm_parameters_in.readConfig(reader);

    EXPECT_EQ(stance_fsm_parameters_out.first_ss_type_,            stance_fsm_parameters_in.first_ss_type_);
    EXPECT_EQ(stance_fsm_parameters_out.first_stance_,             stance_fsm_parameters_in.first_stance_);
    EXPECT_EQ(stance_fsm_parameters_out.last_stance_,              stance_fsm_parameters_in.last_stance_);
    EXPECT_EQ(stance_fsm_parameters_out.ss_duration_ms_,           stance_fsm_parameters_in.ss_duration_ms_);
    EXPECT_EQ(stance_fsm_parameters_out.tds_duration_ms_,          stance_fsm_parameters_in.tds_duration_ms_);
    EXPECT_EQ(stance_fsm_parameters_out.first_stance_duration_ms_, stance_fsm_parameters_in.first_stance_duration_ms_);
    EXPECT_EQ(stance_fsm_parameters_out.last_stance_duration_ms_,  stance_fsm_parameters_in.last_stance_duration_ms_);
    EXPECT_EQ(stance_fsm_parameters_out.num_steps_,                stance_fsm_parameters_in.num_steps_);
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
