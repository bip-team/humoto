/**
    @file
    @author  Alexander Sherikov
    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once


/**
 * @brief Hierarchy 0: basic
 *
 * @param[out] opt_problem       hierarchy
 */
void setupHierarchy_v0( humoto::OptimizationProblem &opt_problem )
{
    humoto::TaskSharedPointer   base_com (new humoto::pepper_ik::TaskBaseCoMTracking<MODEL_FEATURES>);
    humoto::TaskSharedPointer   body_com (new humoto::pepper_ik::TaskBodyCoMTracking<MODEL_FEATURES>);
    humoto::TaskSharedPointer   base_orient (new humoto::pepper_ik::TaskBaseOrientation<MODEL_FEATURES>);

    humoto::TaskSharedPointer   joints_ref (new humoto::pepper_ik::TaskJointsReference<MODEL_FEATURES>);
//    humoto::TaskSharedPointer   root_regularization (new humoto::TaskZeroVariables(2.0, "Minimize_Root_Motion", humoto::pepper_ik::ROOT_VARIABLES_ID));

    opt_problem.reset(2);

    opt_problem.pushTask(base_com, 0);
    opt_problem.pushTask(body_com, 0);
    opt_problem.pushTask(base_orient, 0);

    opt_problem.pushTask(joints_ref, 1);
//    opt_problem.pushTask(root_regularization, 1);
}


/**
 * @brief Hierarchy 1: base motion on the second level
 *
 * @param[out] opt_problem       hierarchy
 */
void setupHierarchy_v1( humoto::OptimizationProblem &opt_problem )
{
    humoto::TaskSharedPointer   base_com (new humoto::pepper_ik::TaskBaseCoMTracking<MODEL_FEATURES>);
    humoto::TaskSharedPointer   body_com (new humoto::pepper_ik::TaskBodyCoMTracking<MODEL_FEATURES>);
    humoto::TaskSharedPointer   base_orient (new humoto::pepper_ik::TaskBaseOrientation<MODEL_FEATURES>);

    humoto::TaskSharedPointer   joints_ref (new humoto::pepper_ik::TaskJointsReference<MODEL_FEATURES>);

    opt_problem.reset(2);

    opt_problem.pushTask(body_com, 0);

    opt_problem.pushTask(base_com, 1);
    opt_problem.pushTask(base_orient, 1);
    opt_problem.pushTask(joints_ref, 1);
}


/**
 * @brief Hierarchy 2: fixed arms + fixed head, no reference configuration
 *
 * @param[out] opt_problem      hierarchy
 */
void setupHierarchy_v2(humoto::OptimizationProblem     &opt_problem)
{
    humoto::IndexVector     fixed_joints;

    fixed_joints.resize(9);

    fixed_joints[0] = humoto::pepper_ik::ModelDescription<MODEL_FEATURES>::LShoulderRoll ;
    fixed_joints[1] = humoto::pepper_ik::ModelDescription<MODEL_FEATURES>::LElbowYaw     ;
    fixed_joints[2] = humoto::pepper_ik::ModelDescription<MODEL_FEATURES>::LElbowRoll    ;
    fixed_joints[3] = humoto::pepper_ik::ModelDescription<MODEL_FEATURES>::LWristYaw     ;

    fixed_joints[4] = humoto::pepper_ik::ModelDescription<MODEL_FEATURES>::RShoulderRoll ;
    fixed_joints[5] = humoto::pepper_ik::ModelDescription<MODEL_FEATURES>::RElbowYaw     ;
    fixed_joints[6] = humoto::pepper_ik::ModelDescription<MODEL_FEATURES>::RElbowRoll    ;
    fixed_joints[7] = humoto::pepper_ik::ModelDescription<MODEL_FEATURES>::RWristYaw     ;

    fixed_joints[8] = humoto::pepper_ik::ModelDescription<MODEL_FEATURES>::HeadYaw     ;


    etools::Vector3     head_orientation;
    head_orientation << 0.0, -0.15, 0.0;


    humoto::TaskSharedPointer   base_com (new humoto::pepper_ik::TaskBaseCoMTracking<MODEL_FEATURES>);
    humoto::TaskSharedPointer   base_orient (new humoto::pepper_ik::TaskBaseOrientation<MODEL_FEATURES>);
    humoto::TaskSharedPointer   fix_joints (new humoto::TaskZeroSelectedVariables(fixed_joints));
    //humoto::TaskSharedPointer   regularization (new humoto::TaskZeroVariables(1e-8));
    humoto::TaskSharedPointer   joints_ref (new humoto::pepper_ik::TaskJointsReference<MODEL_FEATURES>(2.23606797749979e-04, 0.1));

    humoto::TaskSharedPointer   body_com_yz (new humoto::pepper_ik::TaskBodyCoMTracking<MODEL_FEATURES>(0.707106781186548, 0.1, humoto::AxisIndex::FLAG_Y | humoto::AxisIndex::FLAG_Z, "TaskBodyCoMTracking_YZ"));
    humoto::TaskSharedPointer   body_com_x (new humoto::pepper_ik::TaskBodyCoMTracking<MODEL_FEATURES>(0.0223606797749979, 0.1, humoto::AxisIndex::FLAG_X, "TaskBodyCoMTracking_X"));
    humoto::TaskSharedPointer   head_orient (new humoto::pepper_ik::TaskTagOrientation<MODEL_FEATURES>("Head", head_orientation, 0.223606797749979));



    opt_problem.reset(2);

    opt_problem.pushTask(base_com, 0);
    opt_problem.pushTask(base_orient, 0);
    opt_problem.pushTask(fix_joints, 0);

    opt_problem.pushTask(body_com_x, 1);
    opt_problem.pushTask(body_com_yz, 1);
    opt_problem.pushTask(joints_ref, 1);
    opt_problem.pushTask(head_orient, 1);
    //opt_problem.pushTask(regularization, 1);
}
