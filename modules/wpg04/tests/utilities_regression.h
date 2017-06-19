/**
    @file
    @author  Alexander Sherikov
    @author  Jan Michalczyk
    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#define HUMOTO_TEST_MODULE_NAME     wpg04
#include "regression_test_helpers.h"

#include "utilities.h"

namespace humoto_tests
{
    namespace wpg04
    {
        class VerificationData
        {
            public:
                humoto::Solution            solution_;
                humoto::wpg04::ModelState   state_;


                VerificationData(){}

                VerificationData(   const humoto::Solution & solution,
                                    const humoto::wpg04::ModelState & state)
                {
                    solution_ = solution;
                    state_ = state;
                }
        };
    }
}


#define HUMOTO_TEST_COMPARE_WITH_REFERENCE(data_or_call, reference, tolerance)  \
        humoto_tests::wpg04::VerificationData data = data_or_call; \
        \
        ASSERT_TRUE(reference.solution_.x_.isApprox(data.solution_.x_, tolerance));\
        \
        ASSERT_TRUE(reference.state_.com_state_.position_.isApprox(data.state_.com_state_.position_, tolerance));\
        ASSERT_TRUE(reference.state_.com_state_.velocity_.isApprox(data.state_.com_state_.velocity_, tolerance));\
        ASSERT_TRUE(reference.state_.com_state_.acceleration_.isApprox(data.state_.com_state_.acceleration_, tolerance));\
        \
        ASSERT_TRUE(reference.state_.feet_.getLeft().position_.isApprox(data.state_.feet_.getLeft().position_, tolerance));\
        ASSERT_TRUE(reference.state_.feet_.getLeft().velocity_.isApprox(data.state_.feet_.getLeft().velocity_, tolerance));\
        ASSERT_TRUE(reference.state_.feet_.getLeft().acceleration_.isApprox(data.state_.feet_.getLeft().acceleration_, tolerance));\
        ASSERT_TRUE(reference.state_.feet_.getLeft().rpy_.isApprox(data.state_.feet_.getLeft().rpy_, tolerance));\
        ASSERT_TRUE(reference.state_.feet_.getLeft().angular_velocity_.isApprox(data.state_.feet_.getLeft().angular_velocity_, tolerance));\
        ASSERT_TRUE(reference.state_.feet_.getLeft().angular_acceleration_.isApprox(data.state_.feet_.getLeft().angular_acceleration_, tolerance));\
        \
        \
        ASSERT_TRUE(reference.state_.feet_.getRight().position_.isApprox(data.state_.feet_.getRight().position_, tolerance));\
        ASSERT_TRUE(reference.state_.feet_.getRight().velocity_.isApprox(data.state_.feet_.getRight().velocity_, tolerance));\
        ASSERT_TRUE(reference.state_.feet_.getRight().acceleration_.isApprox(data.state_.feet_.getRight().acceleration_, tolerance));\
        ASSERT_TRUE(reference.state_.feet_.getRight().rpy_.isApprox(data.state_.feet_.getRight().rpy_, tolerance));\
        ASSERT_TRUE(reference.state_.feet_.getRight().angular_velocity_.isApprox(data.state_.feet_.getRight().angular_velocity_, tolerance));\
        ASSERT_TRUE(reference.state_.feet_.getRight().angular_acceleration_.isApprox(data.state_.feet_.getRight().angular_acceleration_, tolerance));\
        \
        ASSERT_EQ(reference_.state_.stance_type_, data.state_.stance_type_);\
        ASSERT_EQ(reference_.state_.next_stance_type_, data.state_.next_stance_type_);
