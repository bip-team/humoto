/**
    @file
    @author  Alexander Sherikov
    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

// Enable YAML configuration files (must be first)
#include "humoto/config_yaml.h"
// common & abstract classes
#include "humoto/humoto.h"
#include "humoto/pepper_ik.h"

//testing
#include "gtest/gtest.h"
#include "utilities_regression.h"


namespace humoto_tests
{
    namespace pepper_ik
    {
#define MODEL_FEATURES humoto::pepper_ik::ModelFeatures::FIXED_WHEELS | humoto::pepper_ik::ModelFeatures::ROOT_DEFAULT
        class Fixture_FixedWheelsRoot : public ::testing::Test, public humoto::config::ConfigurableBase
        {
            #define HUMOTO_CONFIG_SECTION_ID "TestFixture"
            #define HUMOTO_CONFIG_ENTRIES \
                HUMOTO_CONFIG_COMPOUND_(com_reference00); \
                HUMOTO_CONFIG_COMPOUND_(com_reference01); \
                HUMOTO_CONFIG_COMPOUND_(com_jacobian_reference00); \
                HUMOTO_CONFIG_COMPOUND_(base_com_jacobian_reference00); \
                HUMOTO_CONFIG_COMPOUND_(body_com_jacobian_reference00); \
                HUMOTO_CONFIG_COMPOUND_(wrist_orientation_reference00); \
                HUMOTO_CONFIG_COMPOUND_(wrist_rotation_jacobian_reference00);
            #include HUMOTO_CONFIG_DEFINE_ACCESSORS

            protected:
                etools::Vector3     com_reference00_;
                etools::Vector3     com_reference01_;

                etools::Matrix3     wrist_orientation_reference00_;
                Eigen::MatrixXd     wrist_rotation_jacobian_reference00_;

                Eigen::MatrixXd     com_jacobian_reference00_;
                Eigen::MatrixXd     base_com_jacobian_reference00_;
                Eigen::MatrixXd     body_com_jacobian_reference00_;

                humoto::pepper_ik::Model<MODEL_FEATURES>                   model_;
                humoto::pepper_ik::GeneralizedCoordinates<MODEL_FEATURES>              generalized_coordinates_;


            protected:
                void setDefaults() {}


                Fixture_FixedWheelsRoot()
                {
                    model_.loadParameters(g_config_path + "pepper_fixedwheels.urdf");
                    bool crash_on_missing_entries = true;
                    readConfig<humoto::config::yaml::Reader>(g_ref_filename, crash_on_missing_entries);
                    generalized_coordinates_.readConfig<humoto::config::yaml::Reader>(g_config_path + "initial_state_pepper_ik_torso_default.yaml");
                    model_.updateState(generalized_coordinates_);

                    /*
                    joints = [  JOINT_HeadYaw, JOINT_HeadPitch, ...
                                JOINT_HipRoll, JOINT_HipPitch, JOINT_KneePitch, ...
                                JOINT_LShoulderPitch, JOINT_LShoulderRoll, JOINT_LElbowYaw, ...
                                JOINT_LElbowRoll, JOINT_LWristYaw, ...
                                JOINT_RShoulderPitch, JOINT_RShoulderRoll, JOINT_RElbowYaw, ...
                                JOINT_RElbowRoll, JOINT_RWristYaw];
                    dofs = [BODY_POSITION,BODY_ORIENTATION,joints];

                    J = model.CoM.J;

                    % select nonfixed dofs
                    J = J(:,dofs);

                    dlmwrite('./jac.txt', J(1:3, :)'(:)')
                    */

                    /*
                    ids = [ ...
                    FRAME_Tibia, FRAME_WheelB_link, FRAME_WheelFL_link, FRAME_WheelFR_link];

                    ids = [...
                    FRAME_torso, FRAME_Neck, FRAME_Head, ...
                    FRAME_Hip, FRAME_Pelvis, ...
                    FRAME_LShoulder, FRAME_LBicep, FRAME_LElbow, ...
                    FRAME_LForeArm, FRAME_l_wrist, FRAME_l_gripper, ...
                    FRAME_LFinger21_link, FRAME_LFinger22_link, FRAME_LFinger23_link, ...
                    FRAME_LFinger11_link, FRAME_LFinger12_link, FRAME_LFinger13_link, ...
                    FRAME_LFinger41_link, FRAME_LFinger42_link, FRAME_LFinger43_link, ...
                    FRAME_LFinger31_link, FRAME_LFinger32_link, ...
                    FRAME_LFinger33_link, ...
                    FRAME_LThumb1_link, FRAME_LThumb2_link, ...
                    FRAME_RShoulder, FRAME_RBicep, FRAME_RElbow, ...
                    FRAME_RForeArm, FRAME_r_wrist, FRAME_r_gripper, ...
                    FRAME_RFinger41_link, FRAME_RFinger42_link, FRAME_RFinger43_link, ...
                    FRAME_RFinger31_link, FRAME_RFinger32_link, FRAME_RFinger33_link, ...
                    FRAME_RFinger21_link, FRAME_RFinger22_link, FRAME_RFinger23_link, ...
                    FRAME_RFinger11_link, FRAME_RFinger12_link, FRAME_RFinger13_link, ...
                    FRAME_RThumb1_link, FRAME_RThumb2_link];

                    J = [];
                    bodies_mass = [];
                    for i = 1:numel(ids)
                        bodies_mass = [bodies_mass, model.DP(ids(i)).m];
                        if (i == 1)
                            J = model.DP(ids(i)).J * bodies_mass(i);
                        else
                            J = J + model.DP(ids(i)).J * bodies_mass(i);
                        end
                    end
                    J = J / sum(bodies_mass);
                    dlmwrite('./jac.txt', J(1:3, dofs)'(:)')


                    bodies_mass = [];
                    c = 0;
                    for i = 1:numel(ids)
                        bodies_mass = [bodies_mass, model.DP(ids(i)).m];
                        c = c + model.DP(ids(i)).c * bodies_mass(i);
                    end
                    c = c / sum(bodies_mass);
                    */
                }


                void changeState01()
                {
                    etools::Vector3  root_position;
                    root_position.x() = 3.0;
                    root_position.y() = 2.0;
                    root_position.z() = 1.0;
                    generalized_coordinates_.setRootPosition(root_position);

                    etools::Vector3  root_orient;
                    root_orient.x() = 1.0;
                    root_orient.y() = 2.0;
                    root_orient.z() = 3.0;
                    generalized_coordinates_.setRootOrientation(root_orient);

                    model_.updateState(generalized_coordinates_);
                }
        };


        TEST_F(Fixture_FixedWheelsRoot, ComputeCoM00_00)
        {
            etools::Vector3 com = model_.getCoM();
            double mass                 = model_.getMass();

            etools::Vector3 com_rbdl;
            double mass_rbdl;

            mass_rbdl = model_.rbdl_model_.getCoMRBDL(com_rbdl);


            EXPECT_LE(std::abs(mass_rbdl - mass), 1e-12);
            ASSERT_TRUE(com.isApprox(com_rbdl, 1e-12));
        }


        TEST_F(Fixture_FixedWheelsRoot, ComputeCoM00_01)
        {
            etools::Vector3 com = model_.getCoM();
            double mass                 = model_.getMass();

            etools::Vector3 com_bb;

            etools::Vector3 base_com = model_.getBaseCoM();
            etools::Vector3 body_com = model_.getBodyCoM();
            double base_mass    = model_.getBaseMass();
            double body_mass    = model_.getBodyMass();

            com_bb = (base_com*base_mass + body_com*body_mass)/(body_mass+base_mass);


            EXPECT_LE(std::abs(base_mass + body_mass - mass), 1e-12);
            ASSERT_TRUE(com.isApprox(com_bb, 1e-12));
        }


        TEST_F(Fixture_FixedWheelsRoot, ComputeCoM00_02)
        {
            etools::Vector3 com = model_.getCoM();

            ASSERT_TRUE(com.isApprox(com_reference00_, 1e-12));
        }


        TEST_F(Fixture_FixedWheelsRoot, ComputeCoM01_00)
        {
            changeState01();

            etools::Vector3 com = model_.getCoM();
            double mass                 = model_.getMass();

            etools::Vector3 com_rbdl;
            double mass_rbdl;

            mass_rbdl = model_.rbdl_model_.getCoMRBDL(com_rbdl);


            EXPECT_LE(std::abs(mass_rbdl - mass), 1e-12);
            ASSERT_TRUE(com.isApprox(com_rbdl, 1e-12));
        }


        TEST_F(Fixture_FixedWheelsRoot, ComputeCoM01_01)
        {
            changeState01();

            etools::Vector3 com = model_.getCoM();
            double mass                 = model_.getMass();

            etools::Vector3 com_bb;

            etools::Vector3 base_com = model_.getBaseCoM();
            etools::Vector3 body_com = model_.getBodyCoM();
            double base_mass    = model_.getBaseMass();
            double body_mass    = model_.getBodyMass();

            com_bb = (base_com*base_mass + body_com*body_mass)/(body_mass+base_mass);


            EXPECT_LE(std::abs(base_mass + body_mass - mass), 1e-12);
            ASSERT_TRUE(com.isApprox(com_bb, 1e-12));
        }


        TEST_F(Fixture_FixedWheelsRoot, ComputeCoM01_02)
        {
            changeState01();

            etools::Vector3 com;

            model_.rbdl_model_.getCoMRBDL(com);

            ASSERT_TRUE(com.isApprox(com_reference01_, 1e-12));
        }


        TEST_F(Fixture_FixedWheelsRoot, ComputeCoMJacobian00_00)
        {
            Eigen::MatrixXd com_jacobian;

            model_.getCoMJacobian(com_jacobian);


            ASSERT_TRUE(com_jacobian.block(
                        0,
                        6,
                        com_jacobian.rows(),
                        com_jacobian.cols() - 6
                        ).isApprox(com_jacobian_reference00_.block(
                                0,
                                6,
                                com_jacobian.rows(),
                                com_jacobian.cols() - 6
                                ), 1e-12));

            ASSERT_TRUE(com_jacobian.block(
                        0,
                        0,
                        com_jacobian.rows(),
                        3
                        ).isApprox(com_jacobian_reference00_.block(
                                0,
                                0,
                                com_jacobian.rows(),
                                3
                                ), 1e-12));
        }


        TEST_F(Fixture_FixedWheelsRoot, ComputeCoMJacobian00_01)
        {
            Eigen::MatrixXd base_com_jacobian;

            model_.getBaseCoMJacobian(base_com_jacobian);


            ASSERT_TRUE(base_com_jacobian.block(
                        0,
                        6,
                        base_com_jacobian.rows(),
                        base_com_jacobian.cols() - 6
                        ).isApprox(base_com_jacobian_reference00_.block(
                                0,
                                6,
                                base_com_jacobian.rows(),
                                base_com_jacobian.cols() - 6
                                ), 1e-12));

            ASSERT_TRUE(base_com_jacobian.block(
                        0,
                        0,
                        base_com_jacobian.rows(),
                        3
                        ).isApprox(base_com_jacobian_reference00_.block(
                                0,
                                0,
                                base_com_jacobian.rows(),
                                3
                                ), 1e-12));
        }


        TEST_F(Fixture_FixedWheelsRoot, ComputeCoMJacobian00_02)
        {
            Eigen::MatrixXd body_com_jacobian;

            model_.getBodyCoMJacobian(body_com_jacobian);


            ASSERT_TRUE(body_com_jacobian.block(
                        0,
                        6,
                        body_com_jacobian.rows(),
                        body_com_jacobian.cols() - 6
                        ).isApprox(body_com_jacobian_reference00_.block(
                                0,
                                6,
                                body_com_jacobian.rows(),
                                body_com_jacobian.cols() - 6
                                ), 1e-12));

            ASSERT_TRUE(body_com_jacobian.block(
                        0,
                        0,
                        body_com_jacobian.rows(),
                        3
                        ).isApprox(body_com_jacobian_reference00_.block(
                                0,
                                0,
                                body_com_jacobian.rows(),
                                3
                                ), 1e-12));
        }


        TEST_F(Fixture_FixedWheelsRoot, ComputeCoMJacobian01_00)
        {
            Eigen::MatrixXd com_jacobian;
            Eigen::MatrixXd com_jacobian_rbdl;

            model_.getCoMJacobian(com_jacobian);
            model_.rbdl_model_.getCoMJacobianRBDL(com_jacobian_rbdl);

            ASSERT_TRUE(com_jacobian.isApprox(com_jacobian_rbdl, 1e-12));
        }


        TEST_F(Fixture_FixedWheelsRoot, ComputeTagOrientation00_00)
        {
            etools::Matrix3 orientation;

            orientation = model_.rbdl_model_.getTagOrientation(model_.getLinkTag("l_wrist"));

            ASSERT_TRUE(orientation.isApprox(wrist_orientation_reference00_, 1e-12));
        }


        TEST_F(Fixture_FixedWheelsRoot, ComputeTagRotationJacobian00_00)
        {
            Eigen::MatrixXd     jacobian;

            humoto::rbdl::TagLinkPtr   tag_ = model_.rbdl_model_.getLinkTag("l_wrist");

            model_.rbdl_model_.getTagJacobian<humoto::rbdl::SpatialType::ROTATION>(jacobian, tag_);

            ASSERT_TRUE(jacobian.isApprox(wrist_rotation_jacobian_reference00_, 1e-12));
        }

#undef MODEL_FEATURES
// ===========================================================================
// ===========================================================================
// ===========================================================================
#define MODEL_FEATURES humoto::pepper_ik::ModelFeatures::FIXED_WHEELS | humoto::pepper_ik::ModelFeatures::ROOT_TIBIA

        class Fixture_FixedWheelsRootTibiaModel : public ::testing::Test, public humoto::config::ConfigurableBase
        {
            #define HUMOTO_CONFIG_SECTION_ID "TestFixture"
            #define HUMOTO_CONFIG_ENTRIES \
                HUMOTO_CONFIG_COMPOUND_(com_reference00); \
                HUMOTO_CONFIG_COMPOUND_(com_reference01); \
                HUMOTO_CONFIG_COMPOUND_(com_jacobian_reference00); \
                HUMOTO_CONFIG_COMPOUND_(base_com_jacobian_reference00); \
                HUMOTO_CONFIG_COMPOUND_(body_com_jacobian_reference00); \
                HUMOTO_CONFIG_COMPOUND_(wrist_orientation_reference00);
            #include HUMOTO_CONFIG_DEFINE_ACCESSORS

            protected:
                etools::Vector3     com_reference00_;
                etools::Vector3     com_reference01_;

                etools::Matrix3     wrist_orientation_reference00_;

                Eigen::MatrixXd    com_jacobian_reference00_;
                Eigen::MatrixXd    base_com_jacobian_reference00_;
                Eigen::MatrixXd    body_com_jacobian_reference00_;

                humoto::pepper_ik::Model<MODEL_FEATURES>                   model_;
                humoto::pepper_ik::GeneralizedCoordinates<MODEL_FEATURES>              generalized_coordinates_;


            protected:
                void setDefaults() {}


                Fixture_FixedWheelsRootTibiaModel()
                {
                    model_.loadParameters(g_config_path + "pepper_fixedwheels_roottibia.urdf");
                    bool crash_on_missing_entries = true;
                    readConfig<humoto::config::yaml::Reader>(g_ref_filename, crash_on_missing_entries);
                    generalized_coordinates_.readConfig<humoto::config::yaml::Reader>(g_config_path + "initial_state_pepper_ik_tibia_default.yaml");
                    model_.updateState(generalized_coordinates_);

                    reorderJacobian(com_jacobian_reference00_);
                    reorderJacobian(base_com_jacobian_reference00_);
                    reorderJacobian(body_com_jacobian_reference00_);
                }


                void reorderJacobian(Eigen::MatrixXd & jacobian)
                {
                    humoto::pepper_ik::ModelDescription<MODEL_FEATURES>::reorderJointsColumns<MODEL_FEATURES, humoto::pepper_ik::ModelFeatures::FIXED_WHEELS|humoto::pepper_ik::ModelFeatures::ROOT_DEFAULT>(jacobian);
                }


                void changeState01()
                {
                    etools::Vector3  root_position;
                    root_position.x() = 3.31252546523508;
                    root_position.y() = 1.25900632786912;
                    root_position.z() = 1.15928872874887;
                    generalized_coordinates_.setRootPosition(root_position);

                    etools::Vector3  root_orient;
                    root_orient.x() = -2.0436092612995971;
                    root_orient.y() = 1.1665487679847131 ;
                    root_orient.z() = -0.0344640732770108;
                    generalized_coordinates_.setRootOrientation(root_orient);

                    model_.updateState(generalized_coordinates_);
                }
        };


        TEST_F(Fixture_FixedWheelsRootTibiaModel, ComputeCoM00_00)
        {
            etools::Vector3 com = model_.getCoM();
            double mass                 = model_.getMass();

            etools::Vector3 com_rbdl;
            double mass_rbdl;

            mass_rbdl = model_.rbdl_model_.getCoMRBDL(com_rbdl);


            EXPECT_LE(std::abs(mass_rbdl - mass), 1e-12);
            ASSERT_TRUE(com.isApprox(com_rbdl, 1e-12));
        }


        TEST_F(Fixture_FixedWheelsRootTibiaModel, ComputeCoM00_01)
        {
            etools::Vector3 com = model_.getCoM();
            double mass                 = model_.getMass();

            etools::Vector3 com_bb;

            etools::Vector3 base_com = model_.getBaseCoM();
            etools::Vector3 body_com = model_.getBodyCoM();
            double base_mass    = model_.getBaseMass();
            double body_mass    = model_.getBodyMass();

            com_bb = (base_com*base_mass + body_com*body_mass)/(body_mass+base_mass);


            EXPECT_LE(std::abs(base_mass + body_mass - mass), 1e-12);
            ASSERT_TRUE(com.isApprox(com_bb, 1e-12));
        }


        TEST_F(Fixture_FixedWheelsRootTibiaModel, ComputeCoM00_02)
        {
            etools::Vector3 com = model_.getCoM();

            ASSERT_TRUE(com.isApprox(com_reference00_, 1e-12));
        }


        TEST_F(Fixture_FixedWheelsRootTibiaModel, ComputeCoM01_00)
        {
            changeState01();

            etools::Vector3 com = model_.getCoM();
            double mass                 = model_.getMass();

            etools::Vector3 com_rbdl;
            double mass_rbdl;

            mass_rbdl = model_.rbdl_model_.getCoMRBDL(com_rbdl);


            EXPECT_LE(std::abs(mass_rbdl - mass), 1e-12);
            ASSERT_TRUE(com.isApprox(com_rbdl, 1e-12));
        }


        TEST_F(Fixture_FixedWheelsRootTibiaModel, ComputeCoM01_01)
        {
            changeState01();

            etools::Vector3 com = model_.getCoM();
            double mass                 = model_.getMass();

            etools::Vector3 com_bb;

            etools::Vector3 base_com = model_.getBaseCoM();
            etools::Vector3 body_com = model_.getBodyCoM();
            double base_mass    = model_.getBaseMass();
            double body_mass    = model_.getBodyMass();

            com_bb = (base_com*base_mass + body_com*body_mass)/(body_mass+base_mass);


            EXPECT_LE(std::abs(base_mass + body_mass - mass), 1e-12);
            ASSERT_TRUE(com.isApprox(com_bb, 1e-12));
        }


        TEST_F(Fixture_FixedWheelsRootTibiaModel, ComputeCoM01_02)
        {
            changeState01();

            etools::Vector3 com;

            model_.rbdl_model_.getCoMRBDL(com);


            ASSERT_TRUE(com.isApprox(com_reference01_, 1e-12));
        }


        TEST_F(Fixture_FixedWheelsRootTibiaModel, ComputeCoMJacobian00_00)
        {
            Eigen::MatrixXd com_jacobian;

            model_.getCoMJacobian(com_jacobian);


            ASSERT_TRUE(com_jacobian.block(
                        0,
                        9,
                        com_jacobian.rows(),
                        com_jacobian.cols() - 9
                        ).isApprox(com_jacobian_reference00_.block(
                                0,
                                9,
                                com_jacobian.rows(),
                                com_jacobian.cols() - 9
                                ), 1e-12));
        }

/*
        TEST_F(Fixture_FixedWheelsRootTibiaModel, ComputeCoMJacobian00_01)
        {
            Eigen::MatrixXd base_com_jacobian;

            model_.getBaseCoMJacobian(base_com_jacobian);


            ASSERT_TRUE(base_com_jacobian.block(
                        0,
                        6,
                        base_com_jacobian.rows(),
                        base_com_jacobian.cols() - 6
                        ).isApprox(base_com_jacobian_reference00_.block(
                                0,
                                6,
                                base_com_jacobian.rows(),
                                base_com_jacobian.cols() - 6
                                ), 1e-12));

            ASSERT_TRUE(base_com_jacobian.block(
                        0,
                        0,
                        base_com_jacobian.rows(),
                        3
                        ).isApprox(base_com_jacobian_reference00_.block(
                                0,
                                0,
                                base_com_jacobian.rows(),
                                3
                                ), 1e-12));
        }
*/

        TEST_F(Fixture_FixedWheelsRootTibiaModel, ComputeCoMJacobian00_02)
        {
            Eigen::MatrixXd body_com_jacobian;

            model_.getBodyCoMJacobian(body_com_jacobian);


            ASSERT_TRUE(body_com_jacobian.block(
                        0,
                        9,
                        body_com_jacobian.rows(),
                        body_com_jacobian.cols() - 9
                        ).isApprox(body_com_jacobian_reference00_.block(
                                0,
                                9,
                                body_com_jacobian.rows(),
                                body_com_jacobian.cols() - 9
                                ), 1e-12));
        }


        TEST_F(Fixture_FixedWheelsRootTibiaModel, ComputeCoMJacobian01_00)
        {
            Eigen::MatrixXd com_jacobian;
            Eigen::MatrixXd com_jacobian_rbdl;

            model_.getCoMJacobian(com_jacobian);
            model_.rbdl_model_.getCoMJacobianRBDL(com_jacobian_rbdl);

            ASSERT_TRUE(com_jacobian.isApprox(com_jacobian_rbdl, 1e-12));
        }


        TEST_F(Fixture_FixedWheelsRootTibiaModel, ComputeTagOrientation00_00)
        {
            etools::Matrix3 orientation;

            orientation = model_.rbdl_model_.getTagOrientation(model_.getLinkTag("l_wrist"));

            ASSERT_TRUE(orientation.isApprox(wrist_orientation_reference00_, 1e-12));
        }


#undef MODEL_FEATURES
// ===========================================================================
// ===========================================================================
// ===========================================================================
#define MODEL_FEATURES humoto::pepper_ik::ModelFeatures::FIXED_WHEELS | humoto::pepper_ik::ModelFeatures::ROOT_PLANAR


        class Fixture_FixedWheelsRootTibiaPlanarModel : public ::testing::Test, public humoto::config::ConfigurableBase
        {
            #define HUMOTO_CONFIG_SECTION_ID "TestFixture"
            #define HUMOTO_CONFIG_ENTRIES \
                HUMOTO_CONFIG_COMPOUND_(com_reference00); \
                HUMOTO_CONFIG_COMPOUND_(com_reference02); \
                HUMOTO_CONFIG_COMPOUND_(com_jacobian_reference00); \
                HUMOTO_CONFIG_COMPOUND_(base_com_jacobian_reference00); \
                HUMOTO_CONFIG_COMPOUND_(body_com_jacobian_reference00); \
                HUMOTO_CONFIG_COMPOUND_(wrist_orientation_reference00);
            #include HUMOTO_CONFIG_DEFINE_ACCESSORS

            protected:
                etools::Vector3     com_reference00_;
                etools::Vector3     com_reference02_;

                etools::Matrix3     wrist_orientation_reference00_;

                Eigen::MatrixXd    com_jacobian_reference00_;
                Eigen::MatrixXd    base_com_jacobian_reference00_;
                Eigen::MatrixXd    body_com_jacobian_reference00_;

                humoto::pepper_ik::Model<MODEL_FEATURES>                   model_;
                humoto::pepper_ik::GeneralizedCoordinates<MODEL_FEATURES>              generalized_coordinates_;


            protected:
                void setDefaults() {}


                Fixture_FixedWheelsRootTibiaPlanarModel()
                {
                    model_.loadParameters(g_config_path + "pepper_fixedwheels_roottibia_planar.urdf");
                    bool crash_on_missing_entries = true;
                    readConfig<humoto::config::yaml::Reader>(g_ref_filename, crash_on_missing_entries);
                    generalized_coordinates_.readConfig<humoto::config::yaml::Reader>(g_config_path + "initial_state_pepper_ik_planar_default.yaml");
                    model_.updateState(generalized_coordinates_);

                    reorderJacobian(com_jacobian_reference00_);
                    reorderJacobian(base_com_jacobian_reference00_);
                    reorderJacobian(body_com_jacobian_reference00_);
                }


                void reorderJacobian(Eigen::MatrixXd & jacobian)
                {
                    humoto::pepper_ik::ModelDescription<MODEL_FEATURES>::reorderJointsColumns<MODEL_FEATURES, humoto::pepper_ik::ModelFeatures::FIXED_WHEELS|humoto::pepper_ik::ModelFeatures::ROOT_DEFAULT>(jacobian);
                }


                void changeState02()
                {
                    etools::Vector3  root_position;
                    root_position.x() = 3.0;
                    root_position.y() = 2.0;
                    root_position.z() = 0.0;
                    generalized_coordinates_.setRootPosition(root_position);

                    etools::Vector3  root_orient;
                    root_orient.x() = 0.0;
                    root_orient.y() = 0.0;
                    root_orient.z() = 3.0;
                    generalized_coordinates_.setRootOrientation(root_orient);

                    model_.updateState(generalized_coordinates_);
                }
        };


        TEST_F(Fixture_FixedWheelsRootTibiaPlanarModel, ComputeCoM00_00)
        {
            etools::Vector3 com = model_.getCoM();
            double mass                 = model_.getMass();

            etools::Vector3 com_rbdl;
            double mass_rbdl;

            mass_rbdl = model_.rbdl_model_.getCoMRBDL(com_rbdl);


            EXPECT_LE(std::abs(mass_rbdl - mass), 1e-12);
            ASSERT_TRUE(com.isApprox(com_rbdl, 1e-12));
        }


        TEST_F(Fixture_FixedWheelsRootTibiaPlanarModel, ComputeCoM00_01)
        {
            etools::Vector3 com = model_.getCoM();
            double mass                 = model_.getMass();

            etools::Vector3 com_bb;

            etools::Vector3 base_com = model_.getBaseCoM();
            etools::Vector3 body_com = model_.getBodyCoM();
            double base_mass    = model_.getBaseMass();
            double body_mass    = model_.getBodyMass();

            com_bb = (base_com*base_mass + body_com*body_mass)/(body_mass+base_mass);


            EXPECT_LE(std::abs(base_mass + body_mass - mass), 1e-12);
            ASSERT_TRUE(com.isApprox(com_bb, 1e-12));
        }


        TEST_F(Fixture_FixedWheelsRootTibiaPlanarModel, ComputeCoM00_02)
        {
            etools::Vector3 com = model_.getCoM();

            ASSERT_TRUE(com.isApprox(com_reference00_, 1e-12));
        }


        TEST_F(Fixture_FixedWheelsRootTibiaPlanarModel, ComputeCoM02_00)
        {
            changeState02();

            etools::Vector3 com = model_.getCoM();
            double mass                 = model_.getMass();

            etools::Vector3 com_rbdl;
            double mass_rbdl;

            mass_rbdl = model_.rbdl_model_.getCoMRBDL(com_rbdl);


            EXPECT_LE(std::abs(mass_rbdl - mass), 1e-12);
            ASSERT_TRUE(com.isApprox(com_rbdl, 1e-12));
        }


        TEST_F(Fixture_FixedWheelsRootTibiaPlanarModel, ComputeCoM02_01)
        {
            changeState02();

            etools::Vector3 com = model_.getCoM();
            double mass                 = model_.getMass();

            etools::Vector3 com_bb;

            etools::Vector3 base_com = model_.getBaseCoM();
            etools::Vector3 body_com = model_.getBodyCoM();
            double base_mass    = model_.getBaseMass();
            double body_mass    = model_.getBodyMass();

            com_bb = (base_com*base_mass + body_com*body_mass)/(body_mass+base_mass);


            EXPECT_LE(std::abs(base_mass + body_mass - mass), 1e-12);
            ASSERT_TRUE(com.isApprox(com_bb, 1e-12));
        }


        TEST_F(Fixture_FixedWheelsRootTibiaPlanarModel, ComputeCoM02_02)
        {
            changeState02();

            etools::Vector3 com;

            model_.rbdl_model_.getCoMRBDL(com);

            ASSERT_TRUE(com.isApprox(com_reference02_, 1e-12));
        }


        TEST_F(Fixture_FixedWheelsRootTibiaPlanarModel, ComputeCoMJacobian00_00)
        {
            Eigen::MatrixXd com_jacobian;

            model_.getCoMJacobian(com_jacobian);


            ASSERT_TRUE(com_jacobian.block(
                        0,
                        9,
                        com_jacobian.rows(),
                        com_jacobian.cols() - 9
                        ).isApprox(com_jacobian_reference00_.block(
                                0,
                                9,
                                com_jacobian.rows(),
                                com_jacobian.cols() - 9
                                ), 1e-12));
        }


        TEST_F(Fixture_FixedWheelsRootTibiaPlanarModel, ComputeCoMJacobian00_02)
        {
            Eigen::MatrixXd body_com_jacobian;

            model_.getBodyCoMJacobian(body_com_jacobian);


            ASSERT_TRUE(body_com_jacobian.block(
                        0,
                        9,
                        body_com_jacobian.rows(),
                        body_com_jacobian.cols() - 9
                        ).isApprox(body_com_jacobian_reference00_.block(
                                0,
                                9,
                                body_com_jacobian.rows(),
                                body_com_jacobian.cols() - 9
                                ), 1e-12));
        }


        TEST_F(Fixture_FixedWheelsRootTibiaPlanarModel, ComputeCoMJacobian01_00)
        {
            Eigen::MatrixXd com_jacobian;
            Eigen::MatrixXd com_jacobian_rbdl;

            model_.getCoMJacobian(com_jacobian);
            model_.rbdl_model_.getCoMJacobianRBDL(com_jacobian_rbdl);

            ASSERT_TRUE(com_jacobian.isApprox(com_jacobian_rbdl, 1e-12));
        }


        TEST_F(Fixture_FixedWheelsRootTibiaPlanarModel, ComputeTagOrientation00_00)
        {
            etools::Matrix3 orientation;

            orientation = model_.rbdl_model_.getTagOrientation(model_.getLinkTag("l_wrist"));

            ASSERT_TRUE(orientation.isApprox(wrist_orientation_reference00_, 1e-12));
        }
    }
}



HUMOTO_DEFINE_REGRESSION_TEST_MAIN()
