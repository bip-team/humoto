/**
    @file
    @author  Alexander Sherikov
    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief

    @todo This code is overcomplicated, find a better approach.
*/

#pragma once

namespace humoto
{
    namespace pepper_ik
    {
        /**
         * @brief Model features are used to define different versions of the model.
         */
        class HUMOTO_LOCAL ModelFeatures
        {
            public:
                /**
                 * @brief Binary map
                 *
                 * 000  000
                 * root wheels
                 */
                enum    Features
                {
                    UNDEFINED       = 0,

                    FIXED_WHEELS    = 1,

                    ROOT_DEFAULT    = 8,
                    ROOT_TORSO      = 8,
                    ROOT_TIBIA      = 16,
                    ROOT_PLANAR     = 32
                };
        };



        /**
         * @brief Base model description class
         *
         * @tparam t_features Model features.
         */
        template <int t_features>
            class HUMOTO_LOCAL ModelDescriptionBase;


        /**
         * @brief Specific model description classes
         *
         * @tparam t_features Model features.
         */
        template <int t_features> class HUMOTO_LOCAL ModelDescription;



        /**
         * @brief Base description class of models with fixed wheels
         */
        template <>
            class HUMOTO_LOCAL ModelDescriptionBase<ModelFeatures::FIXED_WHEELS>
        {
            public:
                enum WheelIds
                {
                    WHEEL_FRONT_LEFT = 0,
                    WHEEL_FRONT_RIGHT = 1,
                    WHEEL_BACK = 2
                };


                /**
                 * @brief Get string ids of the bodies comprising the base.
                 *
                 * @param[out] ids ids
                 */
                static void getBaseLinksStringIds(std::vector<std::string> &ids)
                {
                    ids.clear();
                    /*
                    ids.push_back("WheelB_link");
                    ids.push_back("WheelFL_link");
                    ids.push_back("WheelFR_link");
                    */
                    ids.push_back("Tibia");
                }


                /**
                 * @brief Get string ids of the links comprising the upper body.
                 *
                 * @param[out] ids ids
                 */
                static void getBodyLinksStringIds(std::vector<std::string> &ids)
                {
                    ids.clear();

                    ids.push_back("Head");
                    ids.push_back("Hip");
                    ids.push_back("LBicep");
                    ids.push_back("LElbow");
                    /*
                    ids.push_back("LFinger11_link");
                    ids.push_back("LFinger12_link");
                    ids.push_back("LFinger13_link");
                    ids.push_back("LFinger21_link");
                    ids.push_back("LFinger22_link");
                    ids.push_back("LFinger23_link");
                    ids.push_back("LFinger31_link");
                    ids.push_back("LFinger32_link");
                    ids.push_back("LFinger33_link");
                    ids.push_back("LFinger41_link");
                    ids.push_back("LFinger42_link");
                    ids.push_back("LFinger43_link");
                    */
                    ids.push_back("LForeArm");
                    ids.push_back("LShoulder");
                    /*
                    ids.push_back("LThumb1_link");
                    ids.push_back("LThumb2_link");
                    */
                    ids.push_back("Neck");
                    ids.push_back("Pelvis");
                    ids.push_back("RBicep");
                    ids.push_back("RElbow");
                    /*
                    ids.push_back("RFinger11_link");
                    ids.push_back("RFinger12_link");
                    ids.push_back("RFinger13_link");
                    ids.push_back("RFinger21_link");
                    ids.push_back("RFinger22_link");
                    ids.push_back("RFinger23_link");
                    ids.push_back("RFinger31_link");
                    ids.push_back("RFinger32_link");
                    ids.push_back("RFinger33_link");
                    ids.push_back("RFinger41_link");
                    ids.push_back("RFinger42_link");
                    ids.push_back("RFinger43_link");
                    */
                    ids.push_back("RForeArm");
                    ids.push_back("RShoulder");
                    /*
                    ids.push_back("RThumb1_link");
                    ids.push_back("RThumb2_link");
                    ids.push_back("l_gripper");
                    */
                    ids.push_back("l_wrist");
                    //ids.push_back("r_gripper");
                    ids.push_back("r_wrist");
                    ids.push_back("torso"); // torso
                }


                /**
                 * @brief Get id of the body which is used to control orientation of the base.
                 *
                 * @param[out] id id
                 */
                static void getBaseOrientationLinkStringId(std::string &id)
                {
                    id = "base_footprint";
                }


                /**
                 * @brief Get id of the body which is used to control position of the base.
                 *
                 * @param[out] id id
                 */
                static void getBasePositionLinkStringId(std::string &id)
                {
                    id = "base_footprint";
                }


                /**
                 * @brief Returns matrix which maps velocities of the wheels to
                 * translational and rotational motion of the base.
                 *
                 * @return 3x3 matrix
                 */
                static etools::Matrix3  getRootMotionToWheelsTransform()
                {
                    // ------------------------------------------------
                    /// @todo This code is based on the URDF file -- may be
                    /// there is a way to avoid hardcoding it.
                    etools::Vector3     front_left_wheel_rpy(0, 0, 1.07519348146676);
                    etools::Vector3     front_right_wheel_rpy(0, 0, -1.07519348146676);
                    etools::Vector3     back_wheel_rpy(0, 0, -humoto::g_pi);


                    etools::Vector3     front_left_wheel_position(0.09, 0.155, 0);
                    etools::Vector3     front_right_wheel_position(0.09, -0.155, 0);
                    etools::Vector3     back_wheel_position(-0.17, 0, 0);

                    etools::Vector3     z_axis(0, 0, 1);

                    double  wheel_radius = 0.07;


                    etools::Vector3     front_left_wheel_tangent = -convertEulerAnglesToMatrix( front_left_wheel_rpy,
                                                                                                rigidbody::EulerAngles::RPY).col(1);
                    etools::Vector3     front_right_wheel_tangent = -convertEulerAnglesToMatrix(front_right_wheel_rpy,
                                                                                                rigidbody::EulerAngles::RPY).col(1);
                    etools::Vector3     back_wheel_tangent = -convertEulerAnglesToMatrix(back_wheel_rpy,
                                                                                         rigidbody::EulerAngles::RPY).col(1);
                    // ------------------------------------------------


                    // ------------------------------------------------
                    etools::Matrix1x3   front_left_wheel_transform;
                    etools::Matrix1x3   front_right_wheel_transform;
                    etools::Matrix1x3   back_wheel_transform;


                    front_left_wheel_transform <<   front_left_wheel_tangent.segment(0, 2).transpose(),
                                                    -front_left_wheel_tangent.transpose()*(front_left_wheel_position.cross(z_axis));

                    front_right_wheel_transform <<  front_right_wheel_tangent.segment(0, 2).transpose(),
                                                    -front_right_wheel_tangent.transpose()*(front_right_wheel_position.cross(z_axis));

                    back_wheel_transform << back_wheel_tangent.segment(0, 2).transpose(),
                                            -back_wheel_tangent.transpose()*(back_wheel_position.cross(z_axis));
                    // ------------------------------------------------


                    // ------------------------------------------------
                    etools::Matrix3 result;

                    result <<   front_left_wheel_transform / wheel_radius,
                                front_right_wheel_transform / wheel_radius,
                                back_wheel_transform / wheel_radius;

                    return (result);
                }


                /**
                 * @brief Use joint order to reorder columns of the given matrix
                 *
                 * @tparam t_features_out   features of the model corresponding to output
                 * @tparam t_features_in    features of the model corresponding to input
                 *
                 * @param[in,out] matrix
                 */
                template <int t_features_out, int t_features_in>
                    static void reorderJointsColumns(Eigen::MatrixXd & matrix)
                {
                    const int out_index_offset = ModelDescription<t_features_out>::ROOT_DOF_NUMBER;
                    const int in_index_offset = ModelDescription<t_features_in>::ROOT_DOF_NUMBER;

                    Eigen::MatrixXd reordered_matrix;

                    reordered_matrix.resize(matrix.rows(), ModelDescription<t_features_out>::DOF_NUMBER);
                    etools::unsetMatrix(reordered_matrix);

                    reordered_matrix.col(out_index_offset + ModelDescription<t_features_out>::KneePitch     ) = matrix.col(in_index_offset + ModelDescription<t_features_in>::KneePitch     );
                    reordered_matrix.col(out_index_offset + ModelDescription<t_features_out>::HipPitch      ) = matrix.col(in_index_offset + ModelDescription<t_features_in>::HipPitch      );
                    reordered_matrix.col(out_index_offset + ModelDescription<t_features_out>::HipRoll       ) = matrix.col(in_index_offset + ModelDescription<t_features_in>::HipRoll       );
                    reordered_matrix.col(out_index_offset + ModelDescription<t_features_out>::HeadYaw       ) = matrix.col(in_index_offset + ModelDescription<t_features_in>::HeadYaw       );
                    reordered_matrix.col(out_index_offset + ModelDescription<t_features_out>::HeadPitch     ) = matrix.col(in_index_offset + ModelDescription<t_features_in>::HeadPitch     );
                    reordered_matrix.col(out_index_offset + ModelDescription<t_features_out>::LShoulderPitch) = matrix.col(in_index_offset + ModelDescription<t_features_in>::LShoulderPitch);
                    reordered_matrix.col(out_index_offset + ModelDescription<t_features_out>::LShoulderRoll ) = matrix.col(in_index_offset + ModelDescription<t_features_in>::LShoulderRoll );
                    reordered_matrix.col(out_index_offset + ModelDescription<t_features_out>::LElbowYaw     ) = matrix.col(in_index_offset + ModelDescription<t_features_in>::LElbowYaw     );
                    reordered_matrix.col(out_index_offset + ModelDescription<t_features_out>::LElbowRoll    ) = matrix.col(in_index_offset + ModelDescription<t_features_in>::LElbowRoll    );
                    reordered_matrix.col(out_index_offset + ModelDescription<t_features_out>::LWristYaw     ) = matrix.col(in_index_offset + ModelDescription<t_features_in>::LWristYaw     );
                    reordered_matrix.col(out_index_offset + ModelDescription<t_features_out>::RShoulderPitch) = matrix.col(in_index_offset + ModelDescription<t_features_in>::RShoulderPitch);
                    reordered_matrix.col(out_index_offset + ModelDescription<t_features_out>::RShoulderRoll ) = matrix.col(in_index_offset + ModelDescription<t_features_in>::RShoulderRoll );
                    reordered_matrix.col(out_index_offset + ModelDescription<t_features_out>::RElbowYaw     ) = matrix.col(in_index_offset + ModelDescription<t_features_in>::RElbowYaw     );
                    reordered_matrix.col(out_index_offset + ModelDescription<t_features_out>::RElbowRoll    ) = matrix.col(in_index_offset + ModelDescription<t_features_in>::RElbowRoll    );
                    reordered_matrix.col(out_index_offset + ModelDescription<t_features_out>::RWristYaw     ) = matrix.col(in_index_offset + ModelDescription<t_features_in>::RWristYaw     );

                    matrix = reordered_matrix;
                }


                /**
                 * @brief Returns maximal velocites, which can be executed by the wheels
                 *
                 * @param[out] max_velocities
                 *
                 * @todo RBDL must load this from an URDF file!
                 */
                static void getMaxWheelVelocities(etools::Vector3 & max_velocities)
                {
                    max_velocities << 6.28319, 6.28319, 6.28319;
                }



            protected:
                /**
                 * @brief Returns joint position bounds.
                 *
                 * @tparam t_features these features identify model description
                 *
                 * @param[out] min_bounds
                 * @param[out] max_bounds
                 *
                 * @todo RBDL must load this from an URDF file!
                 */
                template <int t_features>
                    static void getJointPositionBounds(EIGENTOOLS_CONSTANT_SIZE_VECTOR(ModelDescription<t_features>::JOINTS_DOF_NUMBER) &    min_bounds,
                                                    EIGENTOOLS_CONSTANT_SIZE_VECTOR(ModelDescription<t_features>::JOINTS_DOF_NUMBER) &    max_bounds)
                {
                    min_bounds(ModelDescription<t_features>::KneePitch     ) = -0.51487200000000000;
                    min_bounds(ModelDescription<t_features>::HipPitch      ) = -1.03847000000000000;
                    min_bounds(ModelDescription<t_features>::HipRoll       ) = -0.51487200000000000;
                    min_bounds(ModelDescription<t_features>::HeadYaw       ) = -2.08566999999999991;
                    min_bounds(ModelDescription<t_features>::HeadPitch     ) = -0.70685799999999999;
                    min_bounds(ModelDescription<t_features>::LShoulderPitch) = -2.08566999999999991;
                    min_bounds(ModelDescription<t_features>::LShoulderRoll ) =  0.00872665000000000;
                    min_bounds(ModelDescription<t_features>::LElbowYaw     ) = -2.08566999999999991;
                    min_bounds(ModelDescription<t_features>::LElbowRoll    ) = -1.56207000000000007;
                    min_bounds(ModelDescription<t_features>::LWristYaw     ) = -1.82387000000000010;
                    min_bounds(ModelDescription<t_features>::RShoulderPitch) = -2.08566999999999991;
                    min_bounds(ModelDescription<t_features>::RShoulderRoll ) = -1.56207000000000007;
                    min_bounds(ModelDescription<t_features>::RElbowYaw     ) = -2.08566999999999991;
                    min_bounds(ModelDescription<t_features>::RElbowRoll    ) =  0.00872665000000000;
                    min_bounds(ModelDescription<t_features>::RWristYaw     ) = -1.82387000000000010;

                    max_bounds(ModelDescription<t_features>::KneePitch     ) =  0.51487200000000000;
                    max_bounds(ModelDescription<t_features>::HipPitch      ) =  1.03847000000000000;
                    max_bounds(ModelDescription<t_features>::HipRoll       ) =  0.51487200000000000;
                    max_bounds(ModelDescription<t_features>::HeadYaw       ) =  2.08566999999999991;
                    max_bounds(ModelDescription<t_features>::HeadPitch     ) =  0.63704499999999997;
                    max_bounds(ModelDescription<t_features>::LShoulderPitch) =  2.08566999999999991;
                    max_bounds(ModelDescription<t_features>::LShoulderRoll ) =  1.56207000000000007;
                    max_bounds(ModelDescription<t_features>::LElbowYaw     ) =  2.08566999999999991;
                    max_bounds(ModelDescription<t_features>::LElbowRoll    ) = -0.00872665000000000;
                    max_bounds(ModelDescription<t_features>::LWristYaw     ) =  1.82387000000000010;
                    max_bounds(ModelDescription<t_features>::RShoulderPitch) =  2.08566999999999991;
                    max_bounds(ModelDescription<t_features>::RShoulderRoll ) = -0.00872665000000000;
                    max_bounds(ModelDescription<t_features>::RElbowYaw     ) =  2.08566999999999991;
                    max_bounds(ModelDescription<t_features>::RElbowRoll    ) =  1.56207000000000007;
                    max_bounds(ModelDescription<t_features>::RWristYaw     ) =  1.82387000000000010;
                }


                /**
                 * @brief Returns bounds on joint velocities.
                 *
                 * @tparam t_features these features identify model description
                 *
                 * @param[out] joint_vel_bounds
                 *
                 * @todo RBDL must load this from an URDF file!
                 */
                template <int t_features>
                    static void getJointVelocityBounds(EIGENTOOLS_CONSTANT_SIZE_VECTOR(ModelDescription<t_features>::JOINTS_DOF_NUMBER) & joint_vel_bounds)
                {
                    joint_vel_bounds(ModelDescription<t_features>::KneePitch     ) = 2.93276;
                    joint_vel_bounds(ModelDescription<t_features>::HipPitch      ) = 2.93276;
                    joint_vel_bounds(ModelDescription<t_features>::HipRoll       ) = 2.27032;
                    joint_vel_bounds(ModelDescription<t_features>::HeadYaw       ) = 7.33998;
                    joint_vel_bounds(ModelDescription<t_features>::HeadPitch     ) = 9.22756;
                    joint_vel_bounds(ModelDescription<t_features>::LShoulderPitch) = 7.33998;
                    joint_vel_bounds(ModelDescription<t_features>::LShoulderRoll ) = 9.22756;
                    joint_vel_bounds(ModelDescription<t_features>::LElbowYaw     ) = 7.33998;
                    joint_vel_bounds(ModelDescription<t_features>::LElbowRoll    ) = 9.22756;
                    joint_vel_bounds(ModelDescription<t_features>::LWristYaw     ) = 17.3835;
                    joint_vel_bounds(ModelDescription<t_features>::RShoulderPitch) = 7.33998;
                    joint_vel_bounds(ModelDescription<t_features>::RShoulderRoll ) = 9.22756;
                    joint_vel_bounds(ModelDescription<t_features>::RElbowYaw     ) = 7.33998;
                    joint_vel_bounds(ModelDescription<t_features>::RElbowRoll    ) = 9.22756;
                    joint_vel_bounds(ModelDescription<t_features>::RWristYaw     ) = 17.3835;
                }


                /**
                 * @brief Sets joint angles in the given vector to their default values.
                 *
                 * @tparam t_features these features identify model description
                 *
                 * @param[out] joint_angles  vector, where the joint angles are set
                 */
                template <int t_features>
                    static void getDefaultJointAngles(EIGENTOOLS_CONSTANT_SIZE_VECTOR(ModelDescription<t_features>::JOINTS_DOF_NUMBER) & joint_angles)
                {
                    /*
                    joint_angles(ModelDescription<t_features>::HeadYaw       ) = 0.0;
                    joint_angles(ModelDescription<t_features>::HeadPitch     ) = -0.2;

                    joint_angles(ModelDescription<t_features>::HipRoll       ) = 0.0;
                    joint_angles(ModelDescription<t_features>::HipPitch      ) = -0.04;
                    joint_angles(ModelDescription<t_features>::KneePitch     ) = -0.01;

                    joint_angles(ModelDescription<t_features>::LShoulderPitch) = 1.57;
                    joint_angles(ModelDescription<t_features>::LShoulderRoll ) = 0.12;
                    joint_angles(ModelDescription<t_features>::LElbowYaw     ) = -1.2217007;
                    joint_angles(ModelDescription<t_features>::LElbowRoll    ) = -0.52345699;
                    joint_angles(ModelDescription<t_features>::LWristYaw     ) = 0.0;

                    joint_angles(ModelDescription<t_features>::RShoulderPitch) = 1.57;
                    joint_angles(ModelDescription<t_features>::RShoulderRoll ) = -0.12;
                    joint_angles(ModelDescription<t_features>::RElbowYaw     ) = 1.2217007;
                    joint_angles(ModelDescription<t_features>::RElbowRoll    ) = 0.52345699;
                    joint_angles(ModelDescription<t_features>::RWristYaw     ) = 0.0;
                    */

                    joint_angles(ModelDescription<t_features>::KneePitch     ) = -0.1975011835036236;
                    joint_angles(ModelDescription<t_features>::HipPitch      ) =  0.4160023626190753;
                    joint_angles(ModelDescription<t_features>::HipRoll       ) =  0.0;

                    joint_angles(ModelDescription<t_features>::HeadYaw       ) =  0.0;
                    joint_angles(ModelDescription<t_features>::HeadPitch     ) =  0.06850114982656237;

                    joint_angles(ModelDescription<t_features>::LShoulderPitch) =  1.639454123287812;
                    joint_angles(ModelDescription<t_features>::LShoulderRoll ) =  0.12;
                    joint_angles(ModelDescription<t_features>::LElbowYaw     ) =  -1.2217007;
                    joint_angles(ModelDescription<t_features>::LElbowRoll    ) =  -0.52345699;
                    joint_angles(ModelDescription<t_features>::LWristYaw     ) =  0.0;

                    joint_angles(ModelDescription<t_features>::RShoulderPitch) =  1.639454123287804;
                    joint_angles(ModelDescription<t_features>::RShoulderRoll ) =  -0.12;
                    joint_angles(ModelDescription<t_features>::RElbowYaw     ) =  1.2217007;
                    joint_angles(ModelDescription<t_features>::RElbowRoll    ) =  0.52345699;
                    joint_angles(ModelDescription<t_features>::RWristYaw     ) =  0.0;
                }
        };


#define HUMOTO_PARENT_CLASS_SHORTHAND   ModelDescriptionBase<ModelFeatures::FIXED_WHEELS>
        /**
         * @brief Specific model description (default model)
         */
        template <>
            class HUMOTO_LOCAL ModelDescription<ModelFeatures::FIXED_WHEELS | ModelFeatures::ROOT_TORSO> :
                public HUMOTO_PARENT_CLASS_SHORTHAND
        {
            private:
                static const int   features_ = ModelFeatures::FIXED_WHEELS | ModelFeatures::ROOT_TORSO;


            public:
                /**
                 * @brief Ids of root DoF.
                 */
                enum RootDoFIds
                {
                    ROOT_TRANSLATION_X  =   0,
                    ROOT_TRANSLATION_Y  =   1,
                    ROOT_TRANSLATION_Z  =   2,

                    ROOT_ORIENTATION_ROLL  =   3,
                    ROOT_ORIENTATION_PITCH  =   4,
                    ROOT_ORIENTATION_YAW  =   5,

                    ROOT_DOF_NUMBER = 6
                };


                /**
                 * @brief Ids of joints DoF.
                 */
                enum JointsDoFIds
                {
                    HeadYaw         = 0,
                    HeadPitch       = 1,

                    HipRoll         = 2,
                    HipPitch        = 3,
                    KneePitch       = 4,

                    LShoulderPitch  = 5,
                    LShoulderRoll   = 6,
                    LElbowYaw       = 7,
                    LElbowRoll      = 8,
                    LWristYaw       = 9,

                    RShoulderPitch  = 10,
                    RShoulderRoll   = 11,
                    RElbowYaw       = 12,
                    RElbowRoll      = 13,
                    RWristYaw       = 14,

                    JOINTS_DOF_NUMBER = 15,

                    DOF_NUMBER = ROOT_DOF_NUMBER + JOINTS_DOF_NUMBER
                };

                typedef EIGENTOOLS_CONSTANT_SIZE_VECTOR(JOINTS_DOF_NUMBER) JointAnglesVector;

            public:
                /**
                 * @brief Get default generalized coordinates
                 *
                 * @param[out] joint_angles
                 * @param[out] root_pose
                 */
                static void getDefaultGeneralizedCoordinates(JointAnglesVector & joint_angles,
                                                             etools::Vector6 & root_pose)
                {
                    root_pose.segment(ROOT_TRANSLATION_X, 3) << 0.0135554392293566, 0.0, 0.819715156461154;
                    root_pose.segment(ROOT_ORIENTATION_ROLL, 3) << 0.0, 0.05, 0.0;

                    HUMOTO_PARENT_CLASS_SHORTHAND::getDefaultJointAngles<features_>(joint_angles);
                }


                /**
                 * @brief Returns joint position bounds.
                 *
                 * @param[out] min_bounds
                 * @param[out] max_bounds
                 */
                static void getJointPositionBounds(JointAnglesVector &    min_bounds,
                                                JointAnglesVector &    max_bounds)
                {
                    HUMOTO_PARENT_CLASS_SHORTHAND::getJointPositionBounds<features_>(min_bounds, max_bounds);
                }


                /**
                 * @brief Returns bounds on joint velocities.
                 *
                 * @param[out] joint_vel_bounds
                 */
                static void getJointVelocityBounds(JointAnglesVector & joint_vel_bounds)
                {
                    HUMOTO_PARENT_CLASS_SHORTHAND::getJointVelocityBounds<features_>(joint_vel_bounds);
                }


                /**
                 * @brief Get default joint angles (with zero offset)
                 *
                 * @param[out] joint_angles
                 */
                static void getDefaultJointAngles (JointAnglesVector & joint_angles)
                {
                    HUMOTO_PARENT_CLASS_SHORTHAND::getDefaultJointAngles<features_>(joint_angles);
                }



                /**
                 * @brief Set root position to the position given vector
                 *
                 * @param[out] root_pose
                 * @param[in] position
                 */
                static void setRootPosition(etools::Vector6 & root_pose,
                                            const etools::Vector3 & position)
                {
                    root_pose.segment(ROOT_TRANSLATION_X, 3) = position;
                }


                /**
                 * @brief Set root orientation given RPY Euler angles.
                 *
                 * @param[out] root_pose
                 * @param[in] rpy
                 */
                static void setRootOrientation( etools::Vector6 & root_pose,
                                                const etools::Vector3 & rpy)
                {
                    root_pose.segment(ROOT_ORIENTATION_ROLL, 3) =
                        rigidbody::convertEulerAngles(  rpy,
                                                        rigidbody::EulerAngles::RPY,
                                                        rigidbody::EulerAngles::YPR);
                }


                /**
                 * @brief Get root position
                 *
                 * @param[out] position
                 * @param[in] root_pose
                 */
                static void getRootPosition(etools::Vector3 & position,
                                            const etools::Vector6 & root_pose)
                {
                    position = root_pose.segment(ROOT_TRANSLATION_X, 3);
                }


                /**
                 * @brief Get root orientation given RPY Euler angles.
                 *
                 * @param[out] rpy
                 * @param[in] root_pose
                 */
                static void getRootOrientation( etools::Vector3 & rpy,
                                                const etools::Vector6 & root_pose)
                {
                    rpy = rigidbody::convertEulerAngles(root_pose.segment(ROOT_ORIENTATION_ROLL, 3),
                                                        rigidbody::EulerAngles::YPR,
                                                        rigidbody::EulerAngles::RPY);
                }


                /**
                 * @brief Location of the torso root
                 *
                 * @param[in] parent_link_string_id
                 * @param[in] position
                 */
                static void getTorsoRootLocation(   std::string &parent_link_string_id,
                                                    etools::Vector3 & position)
                {
                    parent_link_string_id = "torso";
                    position << 0.0, 0.0, 0.0;
                }
        };



        /**
         * @brief Specific model description (relocated root)
         */
        template <>
            class HUMOTO_LOCAL ModelDescription<ModelFeatures::FIXED_WHEELS | ModelFeatures::ROOT_TIBIA> :
                public HUMOTO_PARENT_CLASS_SHORTHAND
        {
            private:
                static const int   features_ = ModelFeatures::FIXED_WHEELS | ModelFeatures::ROOT_TIBIA;

            public:
                /**
                 * @brief Ids of root DoF.
                 */
                enum RootDoFIds
                {
                    ROOT_TRANSLATION_X  =   0,
                    ROOT_TRANSLATION_Y  =   1,
                    ROOT_TRANSLATION_Z  =   2,

                    ROOT_ORIENTATION_ROLL  =   3,
                    ROOT_ORIENTATION_PITCH  =   4,
                    ROOT_ORIENTATION_YAW  =   5,

                    ROOT_DOF_NUMBER = 6
                };


                /**
                 * @brief Ids of joints DoF.
                 */
                enum JointsDoFIds
                {
                    KneePitch       = 0,
                    HipPitch        = 1,
                    HipRoll         = 2,

                    HeadYaw         = 3,
                    HeadPitch       = 4,

                    LShoulderPitch  = 5,
                    LShoulderRoll   = 6,
                    LElbowYaw       = 7,
                    LElbowRoll      = 8,
                    LWristYaw       = 9,

                    RShoulderPitch  = 10,
                    RShoulderRoll   = 11,
                    RElbowYaw       = 12,
                    RElbowRoll      = 13,
                    RWristYaw       = 14,

                    JOINTS_DOF_NUMBER = 15,

                    DOF_NUMBER = ROOT_DOF_NUMBER + JOINTS_DOF_NUMBER
                };

                typedef EIGENTOOLS_CONSTANT_SIZE_VECTOR(JOINTS_DOF_NUMBER) JointAnglesVector;


            public:
                /**
                 * @brief Get default generalized coordinates
                 *
                 * @param[out] joint_angles
                 * @param[out] root_pose
                 */
                static void getDefaultGeneralizedCoordinates(JointAnglesVector & joint_angles,
                                                             etools::Vector6 & root_pose)
                {
                    root_pose.setZero();
                    HUMOTO_PARENT_CLASS_SHORTHAND::getDefaultJointAngles<features_>(joint_angles);
                }


                /**
                 * @brief Returns joint position bounds.
                 *
                 * @param[out] min_bounds
                 * @param[out] max_bounds
                 */
                static void getJointPositionBounds(JointAnglesVector &    min_bounds,
                                                JointAnglesVector &    max_bounds)
                {
                    HUMOTO_PARENT_CLASS_SHORTHAND::getJointPositionBounds<features_>(min_bounds, max_bounds);
                }


                /**
                 * @brief Returns bounds on joint velocities.
                 *
                 * @param[out] joint_vel_bounds
                 */
                static void getJointVelocityBounds(JointAnglesVector & joint_vel_bounds)
                {
                    HUMOTO_PARENT_CLASS_SHORTHAND::getJointVelocityBounds<features_>(joint_vel_bounds);
                }


                /**
                 * @brief Get default joint angles (with zero offset)
                 *
                 * @param[out] joint_angles
                 */
                static void getDefaultJointAngles (JointAnglesVector & joint_angles)
                {
                    HUMOTO_PARENT_CLASS_SHORTHAND::getDefaultJointAngles<features_>(joint_angles);
                }


                /**
                 * @brief Set root position to the position given vector
                 *
                 * @param[out] root_pose
                 * @param[in] position
                 */
                static void setRootPosition(etools::Vector6 & root_pose,
                                            const etools::Vector3 & position)
                {
                    root_pose.segment(ROOT_TRANSLATION_X, 3) = position;
                }


                /**
                 * @brief Set root orientation given RPY Euler angles.
                 *
                 * @param[out] root_pose
                 * @param[in] rpy
                 */
                static void setRootOrientation( etools::Vector6 & root_pose,
                                                const etools::Vector3 & rpy)
                {
                    root_pose.segment(ROOT_ORIENTATION_ROLL, 3) =
                        rigidbody::convertEulerAngles(  rpy,
                                                        rigidbody::EulerAngles::RPY,
                                                        rigidbody::EulerAngles::YPR);
                }


                /**
                 * @brief Get root position
                 *
                 * @param[out] position
                 * @param[in] root_pose
                 */
                static void getRootPosition(etools::Vector3 & position,
                                            const etools::Vector6 & root_pose)
                {
                    position = root_pose.segment(ROOT_TRANSLATION_X, 3);
                }


                /**
                 * @brief Location of the torso root
                 *
                 * @param[in] parent_link_string_id
                 * @param[in] position
                 */
                static void getTorsoRootLocation(   std::string &parent_link_string_id,
                                                    etools::Vector3 & position)
                {
                    parent_link_string_id = "torso";
                    position << -2e-05, 0, 0.139;
                }


                /**
                 * @brief Get root orientation given RPY Euler angles.
                 *
                 * @param[out] rpy
                 * @param[in] root_pose
                 */
                static void getRootOrientation( etools::Vector3 & rpy,
                                                const etools::Vector6 & root_pose)
                {
                    rpy = rigidbody::convertEulerAngles(root_pose.segment(ROOT_ORIENTATION_ROLL, 3),
                                                        rigidbody::EulerAngles::YPR,
                                                        rigidbody::EulerAngles::RPY);
                }
        };



        /**
         * @brief Specific model description (planar root joint)
         */
        template <>
            class HUMOTO_LOCAL ModelDescription<ModelFeatures::FIXED_WHEELS | ModelFeatures::ROOT_PLANAR> :
                public HUMOTO_PARENT_CLASS_SHORTHAND
        {
            private:
                static const int   features_ = ModelFeatures::FIXED_WHEELS | ModelFeatures::ROOT_PLANAR;

            public:
                /**
                 * @brief Ids of root DoF.
                 */
                enum RootDoFIds
                {
                    ROOT_TRANSLATION_X  =   0,
                    ROOT_TRANSLATION_Y  =   1,
                    ROOT_ORIENTATION_YAW  =   2,

                    ROOT_DOF_NUMBER = 3
                };


                /**
                 * @brief Ids of joints DoF.
                 */
                enum JointsDoFIds
                {
                    KneePitch       = 0,
                    HipPitch        = 1,
                    HipRoll         = 2,

                    HeadYaw         = 3,
                    HeadPitch       = 4,

                    LShoulderPitch  = 5,
                    LShoulderRoll   = 6,
                    LElbowYaw       = 7,
                    LElbowRoll      = 8,
                    LWristYaw       = 9,

                    RShoulderPitch  = 10,
                    RShoulderRoll   = 11,
                    RElbowYaw       = 12,
                    RElbowRoll      = 13,
                    RWristYaw       = 14,

                    JOINTS_DOF_NUMBER = 15,

                    DOF_NUMBER = ROOT_DOF_NUMBER + JOINTS_DOF_NUMBER
                };

                typedef EIGENTOOLS_CONSTANT_SIZE_VECTOR(JOINTS_DOF_NUMBER) JointAnglesVector;


            public:
                /**
                 * @brief Get default generalized coordinates
                 *
                 * @param[out] joint_angles
                 * @param[out] root_pose
                 */
                static void getDefaultGeneralizedCoordinates(JointAnglesVector & joint_angles,
                                                             etools::Vector3 & root_pose)
                {
                    root_pose.segment(ROOT_TRANSLATION_X, 2) << 0.0, 0.0;
                    root_pose(ROOT_ORIENTATION_YAW) = 0.0;

                    HUMOTO_PARENT_CLASS_SHORTHAND::getDefaultJointAngles<features_>(joint_angles);
                }


                /**
                 * @brief Returns joint position bounds.
                 *
                 * @param[out] min_bounds
                 * @param[out] max_bounds
                 */
                static void getJointPositionBounds( JointAnglesVector &    min_bounds,
                                                    JointAnglesVector &    max_bounds)
                {
                    HUMOTO_PARENT_CLASS_SHORTHAND::getJointPositionBounds<features_>(min_bounds, max_bounds);
                }


                /**
                 * @brief Returns bounds on joint velocities.
                 *
                 * @param[out] joint_vel_bounds
                 */
                static void getJointVelocityBounds(JointAnglesVector & joint_vel_bounds)
                {
                    HUMOTO_PARENT_CLASS_SHORTHAND::getJointVelocityBounds<features_>(joint_vel_bounds);
                }


                /**
                 * @brief Get default joint angles (with zero offset)
                 *
                 * @param[out] joint_angles
                 */
                static void getDefaultJointAngles (JointAnglesVector & joint_angles)
                {
                    HUMOTO_PARENT_CLASS_SHORTHAND::getDefaultJointAngles<features_>(joint_angles);
                }


                /**
                 * @brief Set root position to the position given vector
                 *
                 * @param[out] root_pose
                 * @param[in] position
                 */
                static void setRootPosition(etools::Vector3 & root_pose,
                                            const etools::Vector3 & position)
                {
                    HUMOTO_ASSERT(  0.0 == position.z(),
                                    "Translation along the Z axis is not supported.");
                    root_pose(ROOT_TRANSLATION_X) = position.x();
                    root_pose(ROOT_TRANSLATION_Y) = position.y();
                }


                /**
                 * @brief Set root orientation given RPY Euler angles.
                 *
                 * @param[out] root_pose
                 * @param[in] rpy
                 */
                static void setRootOrientation( etools::Vector3 & root_pose,
                                                const etools::Vector3 & rpy)
                {
                    HUMOTO_ASSERT(  (0.0 == rpy.x()) && (0.0 == rpy.y()),
                                    "Only rotation about the Z axis is supported.");
                    root_pose(ROOT_ORIENTATION_YAW) = rpy.z();
                }


                /**
                 * @brief Get root position
                 *
                 * @param[out] position
                 * @param[in] root_pose
                 */
                static void getRootPosition(etools::Vector3 & position,
                                            const etools::Vector3 & root_pose)
                {
                    position.x() = root_pose(ROOT_TRANSLATION_X);
                    position.y() = root_pose(ROOT_TRANSLATION_Y);
                    position.z() = 0.0;
                }


                /**
                 * @brief Get root orientation given RPY Euler angles.
                 *
                 * @param[out] rpy
                 * @param[in] root_pose
                 */
                static void getRootOrientation( etools::Vector3 & rpy,
                                                const etools::Vector3 & root_pose)
                {
                    rpy <<  0.0, 0.0, root_pose(ROOT_ORIENTATION_YAW);
                }


                /**
                 * @brief Location of the torso root
                 *
                 * @param[in] parent_link_string_id
                 * @param[in] position
                 */
                static void getTorsoRootLocation(   std::string &parent_link_string_id,
                                                    etools::Vector3 & position)
                {
                    parent_link_string_id = "torso";
                    position << -2e-05, 0, 0.139;
                }
        };
#undef HUMOTO_PARENT_CLASS_SHORTHAND
    }
}
