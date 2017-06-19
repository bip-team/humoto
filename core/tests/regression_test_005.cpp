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

//testing
#include "gtest/gtest.h"

#include "utilities_regression.h"

HUMOTO_INITIALIZE_GLOBAL_LOGGER(std::cout);


namespace humoto
{
    class CondensingTestTimeInvariant : public ::testing::Test, public humoto::MPC, public humoto::config::ConfigurableBase
    {
        protected:
            const double      timestep_;
            const std::size_t preview_horizon_length_;

            Eigen::MatrixXd U_;
            Eigen::MatrixXd S_;


        protected:
            #define HUMOTO_CONFIG_SECTION_ID "USTimeInvariant"
            #define HUMOTO_CONFIG_ENTRIES \
                HUMOTO_CONFIG_COMPOUND_(U);\
                HUMOTO_CONFIG_COMPOUND_(S);
            #include HUMOTO_CONFIG_DEFINE_ACCESSORS


            /**
             * @brief Set defaults
             */
            void setDefaults()
            {
                etools::unsetMatrix(S_);
                etools::unsetMatrix(U_);
            }


            /**
             * @brief Constructor
             */
            CondensingTestTimeInvariant() : timestep_(0.1), preview_horizon_length_(10)
            {
                setDefaults();
                readConfig(g_ref_filename);
            }


        public:
            // need to override to inherit from humoto::MPC
            void log(humoto::Logger&, const humoto::LogEntryName&, const std::string&) const
            {
            }
    };



    /**
     * @brief Check output of condenseTimeInvariant function
     */
    TEST_F(CondensingTestTimeInvariant, CondenseTimeInvariant)
    {
        etools::Matrix3 A;
        etools::Vector3 B;

        Eigen::MatrixXd U;
        Eigen::MatrixXd S;

        A = humoto::rigidbody::TripleIntegrator::getAJerk<1>(timestep_);
        B = humoto::rigidbody::TripleIntegrator::getBJerk<1>(timestep_);

        // condense
        condenseTimeInvariant(S, U, preview_horizon_length_, A, B);

        ASSERT_TRUE(U.isApprox(U_, 1e-8));
        ASSERT_TRUE(S.isApprox(S_, 1e-8));
    }


    class CondensingTestTimeVariant : public ::testing::Test, public humoto::MPC, public humoto::config::ConfigurableBase
    {
        protected:
            #define HUMOTO_CONFIG_SECTION_ID "USTimeVariant"
            #define HUMOTO_CONFIG_ENTRIES \
                HUMOTO_CONFIG_COMPOUND_(U);\
                HUMOTO_CONFIG_COMPOUND_(S);
            #include HUMOTO_CONFIG_DEFINE_ACCESSORS

            Eigen::MatrixXd U_;
            Eigen::MatrixXd S_;

            std::vector<double> timesteps_;


        protected:
            /**
             * @brief Set defaults
             */
            void setDefaults()
            {
                etools::unsetMatrix(S_);
                etools::unsetMatrix(U_);
            }


            /**
             * @brief Constructor
             */
            CondensingTestTimeVariant()
            {
                setDefaults();

                readConfig(g_ref_filename);

                // variable timesteps
                timesteps_.push_back(0.10);
                timesteps_.push_back(0.30);
                timesteps_.push_back(0.15);
                timesteps_.push_back(0.25);
                timesteps_.push_back(0.40);
                timesteps_.push_back(0.45);
                timesteps_.push_back(0.70);
                timesteps_.push_back(0.50);
                timesteps_.push_back(0.90);
                timesteps_.push_back(0.75);
            }


        public:
            // need to override to inherit from humoto::MPC
            void log(humoto::Logger&, const humoto::LogEntryName&, const std::string&) const
            {
            }
    };


    /**
     * @brief Check output of condense function
     */
    TEST_F(CondensingTestTimeVariant, CondenseFirstOverload)
    {
        etools::Matrix3 A;
        etools::Vector3 B;

        etools::LeftLowerTriangularBlockMatrix<3,1> U;
        etools::GenericBlockMatrix<3,3>             S;

        std::vector<etools::Matrix3> A_matrices;
        std::vector<etools::Vector3> B_matrices;
        for(std::size_t i = 0; i < timesteps_.size(); ++i)
        {
            A = humoto::rigidbody::TripleIntegrator::getAJerk<1>(timesteps_[i]);
            B = humoto::rigidbody::TripleIntegrator::getBJerk<1>(timesteps_[i]);

            A_matrices.push_back(A);
            B_matrices.push_back(B);
        }

        // condense - first overload
        condense(S, U, A_matrices, B_matrices);

        ASSERT_TRUE(U.getRaw().isApprox(U_, 1e-8));
        ASSERT_TRUE(S.getRaw().isApprox(S_, 1e-8));
    }


    /**
     * @brief Check output of condense function
     */
    TEST_F(CondensingTestTimeVariant, CondenseSecondOverload)
    {
        etools::Matrix3 A;
        etools::Vector3 B;

        Eigen::MatrixXd U;
        Eigen::MatrixXd S;

        std::vector<etools::Matrix3> A_matrices;
        std::vector<etools::Vector3> B_matrices;
        for(std::size_t i = 0; i < timesteps_.size(); ++i)
        {
            A = humoto::rigidbody::TripleIntegrator::getAJerk<1>(timesteps_[i]);
            B = humoto::rigidbody::TripleIntegrator::getBJerk<1>(timesteps_[i]);

            A_matrices.push_back(A);
            B_matrices.push_back(B);
        }

        // condense - second overload
        condense(S, U, A_matrices, B_matrices);

        ASSERT_TRUE(U.isApprox(U_, 1e-8));
        ASSERT_TRUE(S.isApprox(S_, 1e-8));
    }
}


HUMOTO_DEFINE_REGRESSION_TEST_MAIN()
