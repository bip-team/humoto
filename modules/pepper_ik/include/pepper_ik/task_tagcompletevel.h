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
            #define HUMOTO_CONFIG_ENTRIES \
                HUMOTO_CONFIG_PARENT_CLASS(TaskAB) \
                HUMOTO_CONFIG_SCALAR_(k_complete_velocity_gain) \
                HUMOTO_CONFIG_SCALAR_(tag_string_id)
            #include HUMOTO_CONFIG_DEFINE_ACCESSORS


            protected:
                double           k_complete_velocity_gain_;
                std::string      tag_string_id_;

                rbdl::TagLinkPtr tag_;


            protected:
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
                    
                    b.noalias() = k_complete_velocity_gain_ * 
                            wb_controller.getTagVelocityInGlobal(model, tag_string_id_,
                                      wb_controller.getTagVelocityInLocal(tag_string_id_, rbdl::SpatialType::COMPLETE),
                                        rbdl::SpatialType::COMPLETE);

                    if(!isApproximatelyEqual(1.0, getGain()))
                    {
                        A*=getGain();
                        b*=getGain();
                    }
                }
        };
    }//pepper
}//humoto
