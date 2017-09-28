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
    namespace rbdl
    {
        /**
         * @brief Parameters of the model.
         */
        class HUMOTO_LOCAL ModelParameters
        {
            public:
                // RBDL parameter
                bool floating_base_;

            public:
                ModelParameters()
                {
                    floating_base_ = true;
                }
        };



        /**
         * @brief Wraps RBDL model and provides additional functionality.
         */
        class HUMOTO_LOCAL Model
        {
            public:
                ModelParameters             model_parameters_;
                RigidBodyDynamics::Model    rbdl_model_;
                double                      total_mass_;


            public:
                /**
                 * @brief Default constructor. It is necesary to load() model
                 * before using.
                 */
                Model()
                {
                    setDefaults();
                }


                /**
                 * @brief Construct using URDF model
                 *
                 * @param[in] urdf_filename
                 */
                explicit Model(const std::string & urdf_filename)
                {
                    load(urdf_filename);
                }


                /**
                 * @brief Construct using URDF model
                 *
                 * @param[in] model_parameters
                 * @param[in] urdf_filename
                 */
                Model(  const ModelParameters & model_parameters,
                        const std::string & urdf_filename)
                {
                    load(model_parameters, urdf_filename);
                }


                /**
                 * @brief Load URDF model.
                 *
                 * @param[in] model_parameters
                 * @param[in] urdf_filename
                 */
                void load(  const ModelParameters & model_parameters,
                            const std::string & urdf_filename)
                {
                    model_parameters_ = model_parameters;
                    load(urdf_filename);
                }


                /**
                 * @brief Load URDF model.
                 *
                 * @param[in] urdf_filename
                 */
                void load(const std::string & urdf_filename)
                {
                    setDefaults();

                    bool rbdl_status = RigidBodyDynamics::Addons::URDFReadFromFile(
                                            urdf_filename.c_str(),
                                            &rbdl_model_,
                                            model_parameters_.floating_base_,
                                            false);
                    if (false == rbdl_status)
                    {
                        HUMOTO_THROW_MSG(std::string("Failed to load URDF model: ") + urdf_filename);
                    }

                    total_mass_ = 0.0;
                    for (std::size_t i = 1; i < rbdl_model_.mBodies.size(); ++i)
                    {
                        total_mass_ += rbdl_model_.mBodies[i].mMass;
                    }

                    joint_data_.resize(rbdl_model_.mJoints.size());
                }


                /**
                 * @brief Update model
                 *
                 * @param[in] joint_angles new joint angles
                 */
                void update(const Eigen::VectorXd & joint_angles)
                {
                    RigidBodyDynamics::UpdateKinematicsCustom(rbdl_model_, &joint_angles, NULL, NULL);
                    for (std::size_t i = 0; i < joint_data_.size(); ++i)
                    {
                        joint_data_[i].compute(rbdl_model_, i);
                    }
                }


                /**
                 * @brief Return number of DoF
                 *
                 * @return number of DoF
                 */
                std::size_t getDOFNumber() const
                {
                    return(rbdl_model_.q_size);
                }



        // get*Tag
                /**
                 * @brief Tag for a link
                 *
                 * @param[in] link_string_id
                 *
                 * @return tag
                 */
                TagLinkPtr getLinkTag(const std::string &link_string_id) const
                {
                    boost::shared_ptr<TagLink>  ptr(new TagLink(getLinkId(link_string_id)));
                    return(ptr);
                }


                /**
                 * @brief Tag for a point on a link
                 *
                 * @param[in] link_string_id
                 * @param[in] position_local position of the point in the local frame
                 *
                 * @return tag
                 */
                TagPointPtr getPointTag(const std::string &link_string_id,
                                        const etools::Vector3 &position_local) const
                {
                    boost::shared_ptr<TagPoint> ptr(new TagPoint(   getLinkId(link_string_id),
                                                                    position_local));
                    return(ptr);
                }


                /**
                 * @brief Tag for the CoM of a link
                 *
                 * @param[in] link_string_id
                 *
                 * @return tag
                 */
                TagPointPtr getLinkCoMTag(const std::string &link_string_id) const
                {
                    LinkId id = getLinkId(link_string_id);
                    boost::shared_ptr<TagPoint> ptr(new TagPoint(   id,
                                                                    rbdl_model_.mBodies[id].mCenterOfMass));
                    return(ptr);
                }


                /**
                 * @brief CoM Tag for several bodies
                 *
                 * @param[in] link_string_ids
                 *
                 * @return tag
                 */
                TagPartialCoMPtr getCoMTag(const std::vector<std::string> &link_string_ids) const
                {
                    std::vector<LinkId>     link_ids;
                    double                  total_mass = 0.0;

                    link_ids.resize(link_string_ids.size());

                    for (std::size_t i = 0; i < link_ids.size(); ++i)
                    {
                        link_ids[i] = getLinkId(link_string_ids[i]);

                        total_mass += rbdl_model_.mBodies[link_ids[i]].mMass;
                    }

                    boost::shared_ptr<TagPartialCoM> ptr(new  TagPartialCoM(link_ids, total_mass));
                    return(ptr);
                }


                /**
                 * @brief CoM tag
                 *
                 * @return tag
                 */
                TagCoMPtr getCoMTag() const
                {
                    boost::shared_ptr<TagCoM>   ptr(new TagCoM());
                    return(ptr);
                }


        // getTagPosition
                /**
                 * @brief Get CoM of several bodies
                 *
                 * @param[in] tag           CoM tag
                 *
                 * @return CoM position
                 */
                etools::Vector3 getTagPosition(const TagPartialCoMPtr  tag) const
                {
                    etools::Vector3 com_position;

                    com_position.setZero(3);

                    for (std::size_t i = 0; i < tag->link_ids_.size(); ++i)
                    {
                        LinkId  id = tag->link_ids_[i];

                        com_position.noalias() +=
                            rbdl_model_.mBodies[id].mMass
                            * getPointTagPosition(id, rbdl_model_.mBodies[id].mCenterOfMass);

                        /*
                        com_position.noalias() +=
                            rbdl_model_.X_base[id].applyTranspose(rbdl_model_.I[id]).h;
                        */
                    }

                    com_position /= tag->mass_;
                    return (com_position);
                }


                /**
                 * @brief Get position of a tag
                 *
                 * @param[in] tag
                 *
                 * @return 3d position vector
                 */
                etools::Vector3 getTagPosition(const TagLinkPtr  tag) const
                {
                    return(getLinkTransform(tag->link_id_).r);
                }


                /**
                 * @brief Get position of a tag
                 *
                 * @param[in] tag
                 *
                 * @return 3d position vector
                 */
                etools::Vector3 getTagPosition( const TagPointPtr  tag) const
                {
                    return(getPointTagPosition(tag->link_id_, tag->local_position_));
                }



                /**
                 * @brief Get CoM of the robot
                 *
                 * @param[in] tag
                 *
                 * @return total mass
                 */
                etools::Vector3 getTagPosition(const TagCoMPtr  tag) const
                {
                    etools::Vector3 com_position;

                    com_position.setZero(3);

                    for (std::size_t i = 1; i < rbdl_model_.mBodies.size(); ++i)
                    {
                        com_position.noalias() +=
                            rbdl_model_.mBodies[i].mMass
                            * getPointTagPosition(i, rbdl_model_.mBodies[i].mCenterOfMass);
                    }

                    com_position /= total_mass_;

                    return (com_position);
                }


        // getTagOrientation

                /**
                 * @brief Get orientation of a tag
                 *
                 * @param[in] tag
                 *
                 * @return 3x3 matrix
                 */
                etools::Matrix3 getTagOrientation(const TagLinkPtr  tag) const
                {
                    return(getLinkTransform(tag->link_id_).E.transpose());
                }


                /**
                 * @brief Get orientation of a tag
                 *
                 * @param[in] tag
                 *
                 * @return 3x3 matrix
                 */
                etools::Matrix3 getTagOrientation(const TagPointPtr  tag) const
                {
                    return ( getTagOrientation(TagLinkPtr (new TagLink(tag->link_id_))) );
                }


        // getTagPose
                /**
                 * @brief Get tag pose
                 *
                 * @param[in] tag
                 *
                 * @return 3x3 matrix
                 */
                rigidbody::RigidBodyPose getTagPose(const TagLinkPtr  tag) const
                {
                    rigidbody::RigidBodyPose pose;

                    RigidBodyDynamics::Math::SpatialTransform   mBaseTransform = getLinkTransform(tag->link_id_);

                    pose.position_  = mBaseTransform.r;
                    pose.rpy_       = rigidbody::convertMatrixToEulerAngles(mBaseTransform.E.transpose(),
                                                                            rigidbody::EulerAngles::RPY);

                    return (pose);
                }


                /**
                 * @brief Get tag pose
                 *
                 * @param[in] tag
                 *
                 * @return 3x3 matrix
                 */
                rigidbody::RigidBodyPose getTagPose(const TagPointPtr tag) const
                {
                    rigidbody::RigidBodyPose pose;

                    RigidBodyDynamics::Math::SpatialTransform   mBaseTransform = getLinkTransform(tag->link_id_);

                    pose.position_  = mBaseTransform.E.transpose() * tag->local_position_ + mBaseTransform.r;
                    pose.rpy_       = rigidbody::convertMatrixToEulerAngles(mBaseTransform.E.transpose(),
                                                                            rigidbody::EulerAngles::RPY);

                    return (pose);
                }


        // getTagMass
                /**
                 * @brief Get mass
                 *
                 * @param[in] tag
                 *
                 * @return mass
                 */
                double getTagMass(const TagCoMPtr  tag) const
                {
                    return (total_mass_);
                }


                /**
                 * @brief Get mass
                 *
                 * @param[in] tag           CoM tag
                 *
                 * @return total mass of the bodies
                 */
                double getTagMass(const TagPartialCoMPtr  tag) const
                {
                    return (tag->mass_);
                }


                /**
                 * @brief Get mass
                 *
                 * @param[in] tag           CoM tag
                 *
                 * @return total mass of the bodies
                 */
                double getTagMass(const TagLinkPtr  tag) const
                {
                    return (rbdl_model_.mBodies[tag->link_id_].mMass);
                }


        // getTagJacobian
                /**
                 * @brief Get CoM jacbian for a set of bodies
                 *
                 * @param[out] jacobian
                 * @param[in] tag
                 */
                void getTagJacobian(Eigen::MatrixXd         &jacobian,
                                    const TagPartialCoMPtr  tag) const
                {
                    Eigen::VectorXd q;
                    Eigen::MatrixXd jacobian_i;


                    for (std::size_t i = 0; i < tag->link_ids_.size(); ++i)
                    {
                        LinkId id = tag->link_ids_[i];

                        getTagJacobian<SpatialType::TRANSLATION>(jacobian_i, id, rbdl_model_.mBodies[id].mCenterOfMass);

                        if (i == 0)
                        {
                            jacobian.noalias() = jacobian_i * rbdl_model_.mBodies[id].mMass;
                        }
                        else
                        {
                            jacobian.noalias() += jacobian_i * rbdl_model_.mBodies[id].mMass;
                        }
                    }

                    jacobian /= tag->mass_;
                }


                /**
                 * @brief Get Jacobian for a specific tag.
                 *
                 * @param[out] jacobian
                 * @param[in] tag
                 *
                 * @attention Based on CalcPointJacobian() function from RBDL.
                 */
                template<SpatialType::Type t_jacobian_type>
                    void getTagJacobian(    Eigen::MatrixXd     &jacobian,
                                            const TagLinkPtr    tag) const
                {
                    getTagJacobian<t_jacobian_type>(jacobian, tag->link_id_, etools::Vector3::Zero());
                }


                /**
                 * @brief Get Jacobian for a specific tag.
                 *
                 * @param[out] jacobian
                 * @param[in] tag
                 *
                 * @attention Based on CalcPointJacobian() function from RBDL.
                 */
                template<SpatialType::Type t_jacobian_type>
                    void getTagJacobian(    Eigen::MatrixXd    &jacobian,
                                            const TagPointPtr  tag) const
                {
                    getTagJacobian<t_jacobian_type>(jacobian, tag->link_id_, tag->local_position_);
                }



                /**
                 * @brief Compute CoM Jacobian of the robot
                 *
                 * @param[out] jacobian
                 * @param[in] tag
                 */
                void getTagJacobian(Eigen::MatrixXd  &jacobian,
                                    const TagCoMPtr  tag) const
                {
                    Eigen::MatrixXd jacobian_i;


                    for (std::size_t i = 1; i < rbdl_model_.mBodies.size(); ++i)
                    {
                        getTagJacobian<SpatialType::TRANSLATION>(jacobian_i, i, rbdl_model_.mBodies[i].mCenterOfMass);

                        if (i == 1)
                        {
                            jacobian.noalias() = jacobian_i*rbdl_model_.mBodies[i].mMass;
                        }
                        else
                        {
                            jacobian.noalias() += jacobian_i*rbdl_model_.mBodies[i].mMass;
                        }
                    }

                    jacobian /= total_mass_;
                }



#if defined(HUMOTO_BUILD_TESTS) || defined(HUMOTO_BUILD_REGRESSION_TESTS)
                /**
                 * @brief Compute CoM position using RBDL functions
                 *
                 * @param[out] com_position
                 *
                 * @return total mass
                 *
                 * @attention This method should be used in tests only.
                 */
                double getCoMRBDL(etools::Vector3     &com_position)
                {
                    double mass;
                    RigidBodyDynamics::Math::Vector3d com;
                    Eigen::VectorXd q, dq;

                    RigidBodyDynamics::Utils::CalcCenterOfMass(
                            rbdl_model_,
                            q,
                            dq,
                            mass,
                            com,
                            NULL,
                            NULL,
                            false);
                    com_position = com;

                    return (mass);
                }


                /**
                 * @brief Compute CoM Jacobian using RBDL functions
                 *
                 * @param[out] jacobian
                 *
                 * @attention This method should be used in tests only.
                 */
                void getCoMJacobianRBDL(Eigen::MatrixXd &jacobian)
                {
                    Eigen::VectorXd q;
                    Eigen::MatrixXd jacobian_i;


                    for (std::size_t i = 1; i < rbdl_model_.mBodies.size(); ++i)
                    {
                        jacobian_i.setZero(3, getDOFNumber());
                        RigidBodyDynamics::CalcPointJacobian (
                                rbdl_model_,
                                q,
                                i,
                                rbdl_model_.mBodies[i].mCenterOfMass,
                                jacobian_i,
                                false);

                        if (i == 1)
                        {
                            jacobian.noalias() = jacobian_i*rbdl_model_.mBodies[i].mMass;
                        }
                        else
                        {
                            jacobian.noalias() += jacobian_i*rbdl_model_.mBodies[i].mMass;
                        }
                    }

                    jacobian /= total_mass_;
                }


                /**
                 * @brief Compute CoM Jacobian for a set of bodies using RBDL
                 * functions
                 *
                 * @param[out]  jacobian
                 * @param[in]   ids
                 *
                 * @attention This method should be used in tests only.
                 */
                void getCoMJacobianRBDL(Eigen::MatrixXd     &jacobian,
                                        const IndexVector   &ids)
                {
                    double total_mass = 0;
                    Eigen::VectorXd q;
                    Eigen::MatrixXd jacobian_i;


                    for (EigenIndex i = 0; i < ids.size(); ++i)
                    {
                        std::size_t id = ids(i);
                        double mass = rbdl_model_.mBodies[id].mMass;
                        total_mass += mass;

                        jacobian_i.setZero(3, getDOFNumber());
                        RigidBodyDynamics::CalcPointJacobian (
                                rbdl_model_,
                                q,
                                id,
                                rbdl_model_.mBodies[id].mCenterOfMass,
                                jacobian_i,
                                false);

                        if (i == 0)
                        {
                            jacobian.noalias() = jacobian_i*mass;
                        }
                        else
                        {
                            jacobian.noalias() += jacobian_i*mass;
                        }
                    }

                    jacobian /= total_mass;
                }
#endif

            private:
                /**
                 * @brief Stores precomputed data for each joint.
                 */
                class HUMOTO_LOCAL JointData
                {
                    public:
                        Eigen::MatrixXd     transformed_joint_axis_;
                        EigenIndex          dof_number_;


                    public:
                        /**
                         * @brief Computes data for a given joint
                         *
                         * @param[in] rbdl_model
                         * @param[in] joint_id
                         */
                        void compute(   const RigidBodyDynamics::Model & rbdl_model,
                                        const unsigned int joint_id)
                        {
                            if(rbdl_model.mJoints[joint_id].mJointType != RigidBodyDynamics::JointTypeCustom)
                            {
                                dof_number_ = rbdl_model.mJoints[joint_id].mDoFCount;

                                switch(dof_number_)
                                {
                                    case 1:
                                        transformed_joint_axis_ = rbdl_model.X_base[joint_id].inverse().apply(rbdl_model.S[joint_id]);
                                        break;
                                    case 3:
                                        transformed_joint_axis_ = rbdl_model.X_base[joint_id].inverse().toMatrix()
                                                                    * rbdl_model.multdof3_S[joint_id];
                                        break;
                                    default:
                                        // Skip this : why?
                                        break;
                                }
                            }
                            else
                            {
                                unsigned int k = rbdl_model.mJoints[joint_id].custom_joint_index;

                                dof_number_ = rbdl_model.mCustomJoints[k]->mDoFCount;
                                transformed_joint_axis_ = rbdl_model.X_base[joint_id].inverse().toMatrix()
                                                            * rbdl_model.mCustomJoints[k]->S;
                            }
                        }
                };

            private:
                std::vector<JointData>  joint_data_;


            private:
                /**
                 * @brief Initialization
                 */
                void setDefaults()
                {
                    total_mass_ = 0.0;
                }


                /**
                 * @brief True if a body is fixed to its parent
                 *
                 * @param[in] link_id
                 *
                 * @return true/false
                 *
                 * @attention Based on IsFixedLinkId() function from RBDL.
                 */
                bool isLinkFixed (const std::size_t link_id) const
                {
                    if (    ( link_id >= rbdl_model_.fixed_body_discriminator )
                            && ( link_id < std::numeric_limits<unsigned int>::max() )
                            && ( link_id - rbdl_model_.fixed_body_discriminator < rbdl_model_.mFixedBodies.size() )  )
                    {
                        return (true);
                    }
                    else
                    {
                        return (false);
                    }
                }



                /**
                 * @brief Determines id of a body based on its string id.
                 *
                 * @param[in] string_id
                 *
                 * @return id
                 */
                std::size_t getLinkId(const std::string   &string_id) const
                {
                    return (rbdl_model_.GetBodyId(string_id.c_str()));
                }



                /**
                 * @brief Express local position in root frame
                 *
                 * @param[in] link_id
                 * @param[in] point_body_coordinates
                 *
                 * @return 3d vector
                 *
                 * @attention Based on CalcBodyToBaseCoordinates() function from RBDL.
                 */
                etools::Vector3     expressInRootFrame(
                        const std::size_t link_id,
                        const etools::Vector3 &point_body_coordinates) const
                {
                    if (isLinkFixed(link_id))
                    {
                        unsigned int flink_id = link_id - rbdl_model_.fixed_body_discriminator;
                        unsigned int parent_id = rbdl_model_.mFixedBodies[flink_id].mMovableParent;

                        etools::Matrix3 fixed_rotation = rbdl_model_.mFixedBodies[flink_id].mParentTransform.E.transpose();
                        etools::Vector3 fixed_position = rbdl_model_.mFixedBodies[flink_id].mParentTransform.r;

                        etools::Matrix3 parent_body_rotation = rbdl_model_.X_base[parent_id].E.transpose();
                        etools::Vector3 parent_body_position = rbdl_model_.X_base[parent_id].r;

                        return (parent_body_position
                                +
                                parent_body_rotation * (fixed_position + fixed_rotation * (point_body_coordinates)) );
                    }
                    else
                    {
                        etools::Matrix3 body_rotation = rbdl_model_.X_base[link_id].E.transpose();
                        etools::Vector3 body_position = rbdl_model_.X_base[link_id].r;

                        return (body_position + body_rotation * point_body_coordinates);
                    }
                }


                /**
                 * @brief Get position of a tag
                 *
                 * @param[in] id             parent link id
                 * @param[in] position_local position in the link frame
                 *
                 * @return 3d position vector
                 */
                etools::Vector3 getPointTagPosition(const std::size_t       id,
                                                    const etools::Vector3   &position_local) const
                {
                    RigidBodyDynamics::Math::SpatialTransform mBaseTransform = getLinkTransform(id);

                    return (mBaseTransform.E.transpose() * position_local + mBaseTransform.r);
                }



                /**
                 * @brief Get Jacobian for a specific tag.
                 *
                 * @param[out] jacobian
                 * @param[in] link_id
                 * @param[in] tag_position (local)
                 *
                 * @attention Based on CalcPointJacobian() function from RBDL.
                 */
                template<SpatialType::Type t_jacobian_type>
                    void getTagJacobian(    Eigen::MatrixXd         &jacobian,
                                            const LinkId            link_id,
                                            const etools::Vector3   tag_position) const
                {
                    unsigned int joint_id = link_id;

                    if (isLinkFixed(link_id))
                    {
                        unsigned int flink_id = link_id - rbdl_model_.fixed_body_discriminator;
                        joint_id = rbdl_model_.mFixedBodies[flink_id].mMovableParent;
                    }

                    jacobian.setZero(SpatialType::getNumberOfElements(t_jacobian_type), getDOFNumber());


                    SpatialTransformWithoutRotation trans(expressInRootFrame(link_id, tag_position));
                    while (joint_id != 0)
                    {
                        trans.applySelective<t_jacobian_type>(
                                jacobian.middleCols(rbdl_model_.mJoints[joint_id].q_index,
                                                    joint_data_[joint_id].dof_number_),
                                joint_data_[joint_id].transformed_joint_axis_);

                        joint_id = rbdl_model_.lambda[joint_id];
                    }
                }


                /**
                 * @brief Returns spatial transform for a link
                 *
                 * @param[in] id
                 *
                 * @return spatial transform
                 */
                RigidBodyDynamics::Math::SpatialTransform   getLinkTransform(const LinkId id) const
                {
                    if (isLinkFixed(id))
                    {
                        unsigned int flink_id = id - rbdl_model_.fixed_body_discriminator;

                        return(rbdl_model_.mFixedBodies[flink_id].mParentTransform
                                * rbdl_model_.X_base[rbdl_model_.mFixedBodies[flink_id].mMovableParent]);
                    }
                    else
                    {
                        return (rbdl_model_.X_base[id]);
                    }
                }
        };
    }
}
