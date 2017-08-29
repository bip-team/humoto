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
    namespace pepper_ik
    {
        /**
         * @brief State of the model
         *
         * @tparam t_features features which identify model
         */
        template <int   t_features>
            class HUMOTO_LOCAL GeneralizedCoordinates : public humoto::ModelState, public humoto::config::ConfigurableBase
        {
            #define HUMOTO_CONFIG_SECTION_ID "GeneralizedCoordinates"
            #define HUMOTO_CONFIG_CONSTRUCTOR GeneralizedCoordinates
            #define HUMOTO_CONFIG_ENTRIES \
                HUMOTO_CONFIG_COMPOUND_(root_pose)\
                HUMOTO_CONFIG_COMPOUND_(joint_angles)
            #include HUMOTO_CONFIG_DEFINE_ACCESSORS


            protected:
                /**
                 * @brief Default configuration
                 */
                void setDefaults()
                {
                    ModelDescription<t_features>::getDefaultGeneralizedCoordinates(joint_angles_, root_pose_);
                }


                /**
                 * @brief Check length of the vector of joint angles after
                 * reading it from a configuration file.
                 */
                void finalize()
                {
                    HUMOTO_ASSERT(  ModelDescription<t_features>::JOINTS_DOF_NUMBER == joint_angles_.rows(),
                                    "Wrong size of the vector of joint angles.");
                }


            public:
                EIGENTOOLS_CONSTANT_SIZE_VECTOR(ModelDescription<t_features>::ROOT_DOF_NUMBER)      root_pose_;
                EIGENTOOLS_CONSTANT_SIZE_VECTOR(ModelDescription<t_features>::JOINTS_DOF_NUMBER)    joint_angles_;


            public:
                /**
                 * @brief Default constructor
                 */
                GeneralizedCoordinates()
                {
                    setDefaults();
                }


                /**
                 * @brief Set position of the root
                 *
                 * @param[in] position
                 */
                void setRootPosition(const etools::Vector3 & position)
                {
                    ModelDescription<t_features>::setRootPosition(root_pose_, position);
                }


                /**
                 * @brief Set orientation of the root
                 *
                 * @param[in] rpy
                 */
                void setRootOrientation(const etools::Vector3 & rpy)
                {
                    ModelDescription<t_features>::setRootOrientation(root_pose_, rpy);
                }


                /**
                 * @brief Get position of the root
                 *
                 * @param[out] position
                 */
                void getRootPosition(etools::Vector3 & position) const
                {
                    ModelDescription<t_features>::getRootPosition(position, root_pose_);
                }


                /**
                 * @brief Get orientation of the root
                 *
                 * @param[out] rpy
                 */
                void getRootOrientation(etools::Vector3 & rpy) const
                {
                    ModelDescription<t_features>::getRootOrientation(rpy, root_pose_);
                }



                /**
                 * @brief Returns generalized coordinates as a single vector
                 * [root_pose; joint_angles].
                 *
                 * @return generalized coordinates.
                 */
                EIGENTOOLS_CONSTANT_SIZE_VECTOR(ModelDescription<t_features>::DOF_NUMBER)
                    asVector() const
                {
                    Eigen::Matrix<double, ModelDescription<t_features>::DOF_NUMBER, 1> generalized_coordinates;
                    generalized_coordinates << root_pose_, joint_angles_;
                    return (generalized_coordinates);
                }


                /**
                 * @brief Log
                 *
                 * @param[in,out] logger logger
                 * @param[in] parent parent
                 * @param[in] name name
                 */
                void log(   humoto::Logger &logger HUMOTO_GLOBAL_LOGGER_IF_DEFINED,
                            const LogEntryName &parent = LogEntryName(),
                            const std::string &name = "model_state") const
                {
                    logger.log(LogEntryName(parent).add(name).add("root_pose"), root_pose_);
                    logger.log(LogEntryName(parent).add(name).add("joint_angles"), joint_angles_);
                }
        };
    }
}
