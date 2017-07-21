/**
    @file
    @author  Alexander Sherikov
    @author  Jan Michalczyk
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
         * @brief Stores robot command
         */
        class HUMOTO_LOCAL RobotCommand
        {
            public:
                Eigen::VectorXd     joint_angles_;
                etools::Vector3     wheel_velocities_;
                etools::Vector3     base_velocities_;
                double              time_interval_;


            public:
                /**
                 * @brief Log
                 *
                 * @param[in,out] logger logger
                 * @param[in] parent parent
                 * @param[in] name name
                 */
                void log(   humoto::Logger &logger HUMOTO_GLOBAL_LOGGER_IF_DEFINED,
                            const LogEntryName &parent = LogEntryName(),
                            const std::string &name = "robot_command") const
                {
                    LogEntryName subname = parent; subname.add(name);
                    logger.log(LogEntryName(subname).add("joint_angles"), joint_angles_);
                    logger.log(LogEntryName(subname).add("wheel_velocities"), wheel_velocities_);
                    logger.log(LogEntryName(subname).add("base_velocities"), base_velocities_);
                    logger.log(LogEntryName(subname).add("time_interval"), time_interval_);
                }
        };



        /**
         * @brief Model
         *
         * @tparam t_features these features identify model
         */
        template <int t_features>
            class HUMOTO_LOCAL Model :  public humoto::Model
        {
            private:
                class SavedState
                {
                    public:
                        bool                                        initialized_;

                        etools::Vector3                             base_position_;
                        etools::Matrix3                             base_orientation_;

                        humoto::pepper_ik::GeneralizedCoordinates<t_features>   generalized_coordinates_;

                    public:
                        SavedState()
                        {
                            initialized_ = false;
                        }
                };


                rbdl::TagPartialCoMPtr    base_com_tag_;
                rbdl::TagPartialCoMPtr    body_com_tag_;

                rbdl::TagLinkPtr      base_orientation_tag_;
                rbdl::TagLinkPtr      base_position_tag_;

                etools::Matrix3                 root_motion_to_wheels_;

                SavedState                      saved_state_;

                /// state of the model
                humoto::pepper_ik::GeneralizedCoordinates<t_features>   generalized_coordinates_;


            private:
                /**
                 * @brief Initialize tags
                 */
                void initialize()
                {
                    std::string                     base_orientation_string_id;
                    std::string                     base_position_string_id;


                    ModelDescription<t_features>::getBaseOrientationLinkStringId(base_orientation_string_id);
                    ModelDescription<t_features>::getBasePositionLinkStringId(base_position_string_id);


                    base_orientation_tag_   = rbdl_model_.getLinkTag(base_orientation_string_id);
                    base_position_tag_      = rbdl_model_.getLinkTag(base_position_string_id);


                    std::vector<std::string>        base_links_string_ids;
                    std::vector<std::string>        body_links_string_ids;

                    ModelDescription<t_features>::getBaseLinksStringIds(base_links_string_ids);
                    ModelDescription<t_features>::getBodyLinksStringIds(body_links_string_ids);

                    base_com_tag_ = rbdl_model_.getCoMTag(base_links_string_ids);
                    body_com_tag_ = rbdl_model_.getCoMTag(body_links_string_ids);
                }


            public:
                typedef EIGENTOOLS_CONSTANT_SIZE_VECTOR(ModelDescription<t_features>::JOINTS_DOF_NUMBER) JointAnglesVector;


            public:
                class Constraints
                {
                    public:
                        JointAnglesVector   joint_position_bounds_min_;
                        JointAnglesVector   joint_position_bounds_max_;
                        JointAnglesVector   joint_velocity_bounds_;
                        etools::Vector3     wheel_velocities_max_;
                        double              error_tol_;


                    public:
                        /**
                         * @brief Initialize bounds.
                         */
                        Constraints()
                        {
                            error_tol_ = 1e-7;

                            ModelDescription<t_features>::getJointPositionBounds(joint_position_bounds_min_,
                                                                                 joint_position_bounds_max_);

                            ModelDescription<t_features>::getJointVelocityBounds(joint_velocity_bounds_);

                            ModelDescription<t_features>::getMaxWheelVelocities(wheel_velocities_max_);
                        }


                        /**
                         * @brief Check joint angle position bounds
                         *
                         * @param[in] joint_angles
                         */
                        void checkJointPositions(const JointAnglesVector & joint_angles) const
                        {
                            for (EigenIndex i = 0; i < joint_angles.size(); ++i)
                            {
                                double value = joint_angles[i];

                                if ( boost::math::isnan(value) )
                                {
                                    std::stringstream msg;
                                    msg << std::setprecision(std::numeric_limits<double>::digits10);
                                    msg << "Joint angle '" << i
                                        << "' cannot be set to '" << value
                                        << "'.";
                                    HUMOTO_THROW_MSG(msg.str());
                                }


                                if (value < joint_position_bounds_min_[i] - error_tol_)
                                {
                                    std::stringstream msg;
                                    msg << std::setprecision(std::numeric_limits<double>::digits10);
                                    msg << "Lower bound of joint '" << i
                                        << "' is violated. Value = '" << value
                                        << "'. Bound = '" << joint_position_bounds_min_[i] << "'.";
                                    HUMOTO_THROW_MSG(msg.str());
                                }


                                if (value > joint_position_bounds_max_[i] + error_tol_)
                                {
                                    std::stringstream msg;
                                    msg << std::setprecision(std::numeric_limits<double>::digits10);
                                    msg << "Upper bound of joint '" << i
                                        << "' is violated. Value = '" << value
                                        << "'. Bound = '" << joint_position_bounds_max_[i] << "'.";
                                    HUMOTO_THROW_MSG(msg.str());
                                }
                            }
                        }


                        /**
                         * @brief Check joint velocity bounds
                         *
                         * @param[in] joint_angles_from    from this configuration
                         * @param[in] joint_angles_to      to this configuration
                         * @param[in] dt                within this time interval
                         */
                        void checkJointVelocities(  const JointAnglesVector & joint_angles_from,
                                                    const JointAnglesVector & joint_angles_to,
                                                    const double dt) const
                        {
                            for (EigenIndex i = 0; i < joint_angles_from.size(); ++i)
                            {
                                double joint_vel = std::abs(joint_angles_to[i] - joint_angles_from[i]) / dt;
                                if (joint_vel > joint_velocity_bounds_[i])
                                {
                                    std::stringstream msg;
                                    msg << std::setprecision(std::numeric_limits<double>::digits10);
                                    msg << "Velocity bound of joint '" << i
                                        << "' is violated. Value = '" << joint_vel
                                        << "'. Bound = '" << joint_velocity_bounds_[i] << "'.";
                                    HUMOTO_THROW_MSG(msg.str());
                                }
                            }
                        }


                        /**
                         * @brief Check velocities of the wheels
                         *
                         * @param[in] velocities
                         */
                        void checkWheelVelocities(const etools::Vector3 &velocities) const
                        {
                            for (EigenIndex i = 0; i < velocities.size(); ++i)
                            {
                                double value = velocities[i];

                                if ( boost::math::isnan(value) )
                                {
                                    std::stringstream msg;
                                    msg << std::setprecision(std::numeric_limits<double>::digits10);
                                    msg << "Velocity of wheel '" << i
                                        << "' cannot be set to '" << value
                                        << "'.";
                                    HUMOTO_THROW_MSG(msg.str());
                                }

                                if ( (value < -wheel_velocities_max_[i])
                                    || (value > wheel_velocities_max_[i]) )
                                {
                                    std::stringstream msg;
                                    msg << "Velocity of the wheel '" << i
                                        << "' is too high. Value = '" << value
                                        << "'. Bound = '" << wheel_velocities_max_[i]
                                        << "'.";
                                    HUMOTO_THROW_MSG(msg.str());
                                }
                            }
                        }
                };


                humoto::rbdl::Model                         rbdl_model_;

                Constraints                                 constraints_;



            public:
                /**
                 * @brief Initialize model based on an URDF file.
                 *
                 * @param[in] filename
                 */
                void loadParameters(const std::string & filename)
                {
                    humoto::rbdl::ModelParameters   rbdl_param;

                    if (ModelFeatures::ROOT_PLANAR & t_features)
                    {
                        rbdl_param.floating_base_ = false;
                    }
                    else
                    {
                        rbdl_param.floating_base_ = true;
                    }

                    rbdl_model_.load(rbdl_param, filename);

                    HUMOTO_ASSERT(  ModelDescription<t_features>::DOF_NUMBER == rbdl_model_.getDOFNumber(),
                                    "The loaded model has unexpected number of degrees of freedom.");

                    initialize();
                    root_motion_to_wheels_ = ModelDescription<t_features>::getRootMotionToWheelsTransform();

                    rbdl_model_.update(generalized_coordinates_.asVector());
                }


                /**
                 * @brief Make internal copy of the current state to avoid
                 * losing it while iteratively solving IK.
                 */
                void saveCurrentState()
                {
                    saved_state_.base_orientation_ = getBaseOrientation();
                    saved_state_.base_position_ = getBasePosition();
                    saved_state_.generalized_coordinates_ = generalized_coordinates_;
                    saved_state_.initialized_ = true;
                }


                /**
                 * @brief Get position of the base CoM.
                 *
                 * @return com_position
                 */
                etools::Vector3 getBaseCoM() const
                {
                    return (rbdl_model_.getTagPosition(base_com_tag_));
                }


                /**
                 * @brief Get base mass
                 *
                 * @return mass
                 */
                double getBaseMass() const
                {
                    return (rbdl_model_.getTagMass(base_com_tag_));
                }


                /**
                 * @brief Get position of the upper body CoM.
                 *
                 * @return com_position
                 */
                etools::Vector3 getBodyCoM() const
                {
                    return (rbdl_model_.getTagPosition(body_com_tag_));
                }


                /**
                 * @brief Get body mass
                 *
                 * @return mass
                 */
                double getBodyMass() const
                {
                    return (rbdl_model_.getTagMass(body_com_tag_));
                }


                /**
                 * @brief Get position of the robot's CoM.
                 *
                 * @return  com_position
                 */
                etools::Vector3 getCoM() const
                {
                    return (rbdl_model_.getTagPosition(rbdl::TagCoMPtr (new rbdl::TagCoM)));
                }


                /**
                 * @brief Get mass of the robot
                 *
                 * @return mass of the upper body
                 */
                double getMass() const
                {
                    return (rbdl_model_.getTagMass(rbdl::TagCoMPtr (new rbdl::TagCoM)));
                }


                /**
                 * @brief Get Jacobian of the base CoM.
                 *
                 * @param[out] jacobian
                 */
                void getBaseCoMJacobian(Eigen::MatrixXd &jacobian) const
                {
                    rbdl_model_.getTagJacobian(jacobian, base_com_tag_);
                }


                /**
                 * @brief Get Jacobian of the upper body CoM.
                 *
                 * @param[out] jacobian
                 */
                void getCoMJacobian(Eigen::MatrixXd &jacobian) const
                {
                    rbdl_model_.getTagJacobian(jacobian, rbdl::TagCoMPtr (new rbdl::TagCoM));
                }


                /**
                 * @brief Get Jacobian of the upper body CoM.
                 *
                 * @param[out] jacobian
                 */
                void getBodyCoMJacobian(Eigen::MatrixXd &jacobian) const
                {
                    rbdl_model_.getTagJacobian(jacobian, body_com_tag_);
                }


                /**
                 * @brief Get base orientation Jacobian.
                 *
                 * @param[out] jacobian
                 */
                void getBaseOrientationJacobian(Eigen::MatrixXd &jacobian) const
                {
                    rbdl_model_.getTagJacobian<rbdl::SpatialType::ROTATION>(jacobian, base_orientation_tag_);
                }


                /**
                 * @brief Get base orientation.
                 *
                 * @return rotation matrix
                 */
                etools::Matrix3 getBaseOrientation() const
                {
                    return (rbdl_model_.getTagOrientation(base_orientation_tag_));
                }


                /**
                 * @brief Return Yaw angle of base orientation
                 *
                 * @return Yaw angle
                 */
                double getBaseYaw() const
                {
                    return (convertMatrixToEulerAngles(getBaseOrientation(), rigidbody::EulerAngles::RPY).z());
                }


                /**
                 * @brief Get tag
                 *
                 * @param[in] id id (string)
                 *
                 * @return tag
                 */
                rbdl::TagLinkPtr getLinkTag(const std::string & id) const
                {
                    return (rbdl_model_.getLinkTag(id));
                }


                /**
                 * @brief Get tag orientation.
                 *
                 * @param[in] tag
                 *
                 * @return rotation matrix
                 */
                etools::Matrix3 getTagOrientation(const rbdl::TagLinkPtr  tag) const
                {
                    return (rbdl_model_.getTagOrientation(tag));
                }
                
                
                /**
                 * @brief Get tag position.
                 *
                 * @param[in] tag
                 *
                 * @return position vector
                 */
                etools::Vector3 getTagPosition(const rbdl::TagLinkPtr  tag) const
                {
                    return (rbdl_model_.getTagPosition(tag));
                }


                /**
                 * @brief Get tag orientation Jacobian
                 *
                 * @param[out] jacobian
                 * @param[in] tag
                 */
                void getTagOrientationJacobian( Eigen::MatrixXd &jacobian,
                                                const rbdl::TagLinkPtr  tag) const
                {
                    rbdl_model_.getTagJacobian<rbdl::SpatialType::ROTATION>(jacobian, tag);
                }
                
                
                /**
                 * @brief Get tag complete Jacobian
                 *
                 * @param[out] jacobian
                 * @param[in] tag
                 */
                void getTagCompleteJacobian( Eigen::MatrixXd &jacobian,
                                             const rbdl::TagLinkPtr  tag) const
                {
                    rbdl_model_.getTagJacobian<rbdl::SpatialType::COMPLETE>(jacobian, tag);
                }


                /**
                 * @brief Get base position.
                 *
                 * @return rotation matrix
                 */
                etools::Vector3 getBasePosition() const
                {
                    return (rbdl_model_.getTagPosition(base_position_tag_));
                }


                /**
                 * @brief Compute robot command
                 *
                 * @param[out] command
                 * @param[in] dt time interval between the new and the saved state
                 * @param[in] error_tol tolerance for checking satisfaction of the bounds
                 */
                void getRobotCommand(   RobotCommand & command,
                                        const double dt,
                                        const double error_tol = 1e-7) const
                {
                    if (! saved_state_.initialized_)
                    {
                        HUMOTO_THROW_MSG("This function requires the previous state to be saved.");
                    }

                    constraints_.checkJointPositions(generalized_coordinates_.joint_angles_);
                    constraints_.checkJointVelocities(  saved_state_.generalized_coordinates_.joint_angles_,
                                                        generalized_coordinates_.joint_angles_,
                                                        dt);


                    etools::Matrix3     base_orientation = getBaseOrientation();

                    etools::Vector3     base_translation_velocity = (getBasePosition() - saved_state_.base_position_)/dt;
                    etools::Vector3     base_rotation_velocity = rigidbody::getRotationErrorAngleAxis(
                                                                        saved_state_.base_orientation_,
                                                                        base_orientation)/dt;

                    command.joint_angles_ = generalized_coordinates_.joint_angles_;

                    command.base_velocities_ << base_translation_velocity.x(),
                                                base_translation_velocity.y(),
                                                base_rotation_velocity.z();

                    command.wheel_velocities_ = root_motion_to_wheels_*base_orientation.transpose()*command.base_velocities_;
                    constraints_.checkWheelVelocities(command.wheel_velocities_);

                    command.time_interval_ = dt;
                }


                /**
                 * @brief Correct state if the actual sampling interval is
                 * different from the expected.
                 *
                 * @param[in] command
                 * @param[in] time_interval
                 */
                void correct(   const RobotCommand  & command,
                                const double        time_interval)
                {
                    if (time_interval > 0.0)
                    {
                        if (t_features & ModelFeatures::ROOT_PLANAR)
                        {
                            // base pose
                            etools::Vector3 position;
                            etools::Vector3 rpy;

                            generalized_coordinates_.getRootPosition(position);
                            generalized_coordinates_.getRootOrientation(rpy);

                            position.x() += time_interval * command.base_velocities_.x();
                            position.y() += time_interval * command.base_velocities_.y();

                            rpy.z() += time_interval * command.base_velocities_(2);

                            generalized_coordinates_.setRootPosition(position);
                            generalized_coordinates_.setRootOrientation(rpy);

                            // joint angles
                            generalized_coordinates_.joint_angles_ +=
                                (command.joint_angles_ - generalized_coordinates_.joint_angles_)
                                * time_interval / command.time_interval_;
                        }
                        else
                        {
                            /**
                             * @todo Implement correction of the base position for
                             * the models where the root does not coincide with
                             * the base.
                             */
                            HUMOTO_THROW_MSG("Correction for this type of model is not implemented.");
                        }

                        // update model
                        rbdl_model_.update(generalized_coordinates_.asVector());
                    }
                }


                /**
                 * @brief Correct state if the actual sampling interval is
                 * different from the expected.
                 *
                 * @param[in] generalized_coordinates
                 * @param[in] command
                 * @param[in] time_interval
                 *
                 * @note This version of correct() function updates the state
                 * and then corrects it, in some cases this allows to avoid
                 * update of the kinematic model.
                 */
                void correct(   const humoto::pepper_ik::GeneralizedCoordinates<t_features> &generalized_coordinates,
                                const RobotCommand  & command,
                                const double        time_interval)
                {
                    if (time_interval > 0.0)
                    {
                        generalized_coordinates_ = generalized_coordinates;
                        correct(command, time_interval);
                    }
                    else
                    {
                        updateState(generalized_coordinates);
                    }
                }


                /**
                 * @brief Checks validity of the current state.
                 */
                void checkCurrentState() const
                {
                    constraints_.checkJointPositions(generalized_coordinates_.joint_angles_);
                }


                /**
                 * @brief Check validity of translation from one state to
                 * another in the given time interval. 'generalized_coordinates_to' is assumed to
                 * be model's current state.
                 *
                 * @param[in] generalized_coordinates_from
                 * @param[in] dt
                 * @param[in] error_tol
                 */
                void checkStateTransition(  const humoto::pepper_ik::GeneralizedCoordinates<t_features> & generalized_coordinates_from,
                                            const double dt,
                                            const double error_tol = 1e-7) const
                {
                    checkStateTransition(generalized_coordinates_from, generalized_coordinates_, dt, error_tol);
                }


                /**
                 * @brief Check validity of translation from one state to
                 * another in the given time interval.
                 *
                 * @param[in] generalized_coordinates_from
                 * @param[in] generalized_coordinates_to
                 * @param[in] dt
                 * @param[in] error_tol
                 */
                void checkStateTransition(  const humoto::pepper_ik::GeneralizedCoordinates<t_features> & generalized_coordinates_from,
                                            const humoto::pepper_ik::GeneralizedCoordinates<t_features> & generalized_coordinates_to,
                                            const double dt,
                                            const double error_tol = 1e-7) const
                {
                    constraints_.checkJointPositions(generalized_coordinates_to.joint_angles_);
                    constraints_.checkJointVelocities(  generalized_coordinates_from.joint_angles_,
                                                        generalized_coordinates_to.joint_angles_,
                                                        dt);

                    etools::Vector3 root_from;
                    etools::Vector3 root_to;

                    ModelDescription<t_features>::getRootPosition(root_from, generalized_coordinates_from.joint_angles_);
                    ModelDescription<t_features>::getRootPosition(root_to, generalized_coordinates_to.joint_angles_);

                    if (! root_from.isApprox(root_to, error_tol))
                    {
                        HUMOTO_THROW_MSG("Root translational motion is not allowed.");
                    }

                    generalized_coordinates_from.getRootOrientation(root_from);
                    generalized_coordinates_to.getRootOrientation(root_to);


                    etools::Vector3 rot_error = rigidbody::getRotationErrorAngleAxis(
                            convertEulerAnglesToMatrix(root_from, rigidbody::EulerAngles::RPY),
                            convertEulerAnglesToMatrix(root_to, rigidbody::EulerAngles::RPY));

                    if (! rot_error.isZero(error_tol))
                    {
                        HUMOTO_THROW_MSG("Root rotational motion is not allowed.");
                    }
                }



                /**
                 * @brief Update model state.
                 *
                 * @param[in] model_state model state.
                 */
                void    updateState(const humoto::ModelState &model_state)
                {
                    const humoto::pepper_ik::GeneralizedCoordinates<t_features> &generalized_coordinates =
                        dynamic_cast <const humoto::pepper_ik::GeneralizedCoordinates<t_features> &> (model_state);

                    generalized_coordinates_ = generalized_coordinates;

                    rbdl_model_.update(generalized_coordinates_.asVector());
                }


                /**
                 * @brief Return current model state.
                 *
                 * @return current state
                 */
                const GeneralizedCoordinates<t_features> & getState() const
                {
                    return(generalized_coordinates_);
                }


                /**
                 * @brief Computes errors with respect to the given motion parameters.
                 *
                 * @param[out] motion_param_errors
                 * @param[in] motion_param
                 *
                 * @return MotionParameters class, where all members represent
                 * errors instead of the actual values.
                 */
                void getStateError( MotionParameters       &motion_param_errors,
                                    const MotionParameters &motion_param) const
                {
                    motion_param_errors.base_com_position_    = motion_param.base_com_position_    - getBaseCoM();
                    motion_param_errors.body_com_position_    = motion_param.body_com_position_    - getBodyCoM();

                    motion_param_errors.base_orientation_rpy_ = rigidbody::getRotationErrorAngleAxis(
                            getBaseOrientation(),
                            convertEulerAnglesToMatrix( motion_param.base_orientation_rpy_,
                                                        rigidbody::EulerAngles::RPY));
                }



                /**
                 * @brief Returns pose of the root of the default robot model
                 * (root = torso)
                 *
                 * @return root pose
                 */
                rigidbody::RigidBodyPose getTorsoRootPose()
                {
                    std::string     id;
                    etools::Vector3 position;

                    ModelDescription<t_features>::getTorsoRootLocation(id, position);

                    return (rbdl_model_.getTagPose(rbdl_model_.getPointTag(id, position)));
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
                            const std::string &name = "model") const
                {
                    LogEntryName subname = parent; subname.add(name);
                    generalized_coordinates_.log(logger, subname, "state");

                    logger.log(LogEntryName(subname).add("root_motion_to_wheels"), root_motion_to_wheels_);
                }
        };
    }
}
