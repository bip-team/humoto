/**
    @file
    @author Jan Michalczyk
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
         * @brief Tag complete velocity (translational and rotational)
         */
        template <int t_features>
            class HUMOTO_LOCAL TaskTagCompleteVelocity : public humoto::TaskAB
        {
            protected:
                double           k_complete_velocity_gain_;
                std::string      tag_string_id_;

                rbdl::TagLinkPtr tag_;


            protected:
                #define HUMOTO_CONFIG_ENTRIES \
                    HUMOTO_CONFIG_PARENT_CLASS(TaskAB); \
                    HUMOTO_CONFIG_SCALAR_(k_complete_velocity_gain); \
                    HUMOTO_CONFIG_SCALAR_(tag_string_id);
                #include HUMOTO_CONFIG_DEFINE_ACCESSORS


                virtual void setDefaults()
                {
                    TaskAB::setDefaults();
                    k_complete_velocity_gain_ = 0.0;
                }


                /**
                 * @brief Log task.
                 *
                 * @param[in] logger logger
                 * @param[in] parent parent
                 * @param[in] name name
                 */
                virtual void logTask(humoto::Logger &logger HUMOTO_GLOBAL_LOGGER_IF_DEFINED,
                                     const LogEntryName &parent = LogEntryName(),
                                     const std::string &name = "task") const
                {
                    LogEntryName subname = parent; subname.add(name);
                    TaskAB::logTask(logger, subname, "");
                    logger.log(LogEntryName(subname).add("k_complete_velocity_gain"), k_complete_velocity_gain_);
                    logger.log(LogEntryName(subname).add("tag_string_id"),            tag_string_id_);
                }


            public:
                TaskTagCompleteVelocity(const std::string& tag_string_id = "",
                                        const double       gain = 1.0,
                                        const double       k_complete_velocity_gain = 1.0)
                    : TaskAB(std::string("TaskTagCompleteVelocity_") + tag_string_id, gain)
                {
                    k_complete_velocity_gain_ = k_complete_velocity_gain;
                    tag_string_id_            = tag_string_id;
                }


                /// @copydoc humoto::TaskBase::form
                void form(const humoto::SolutionStructure &sol_structure,
                          const humoto::Model             &model_base,
                          const humoto::ControlProblem    &control_problem)
                {
                    const Model<t_features>& model =
                        dynamic_cast <const Model<t_features>& >(model_base);

                    const WholeBodyController<t_features>& wb_controller =
                        dynamic_cast <const WholeBodyController<t_features>& >(control_problem);

                    if(!tag_)
                    {
                        tag_ = model.getLinkTag(tag_string_id_);
                    }

                    Eigen::MatrixXd &A = getA();
                    Eigen::VectorXd &b = getB();

                    model.getTagCompleteJacobian(A, tag_);

                    b.resize(rbdl::SpatialType::getNumberOfElements(rbdl::SpatialType::COMPLETE));
                   
                    std::size_t linear_part  = rbdl::SpatialType::getNumberOfElements(rbdl::SpatialType::TRANSLATION);
                    std::size_t angular_part = rbdl::SpatialType::getNumberOfElements(rbdl::SpatialType::ROTATION);

                    etools::Vector6 tag_ref_velocity;
                    wb_controller.getTagRefVelocity(tag_ref_velocity, tag_string_id_);

                    b.head(angular_part) = model.getTagOrientation(tag_) *
                                            tag_ref_velocity.tail(angular_part);
                    
                    b.tail(linear_part)  = model.getTagPosition(tag_).cross(model.getTagOrientation(tag_)
                                            * tag_ref_velocity.tail(angular_part))
                                            + model.getTagOrientation(tag_) *
                                              tag_ref_velocity.head(linear_part);
                    
                    b.noalias() = k_complete_velocity_gain_ * b;

                    if(!isApproximatelyEqual(1.0, getGain()))
                    {
                        A*=getGain();
                        b*=getGain();
                    }
                }
        };
    }//pepper
}//humoto
