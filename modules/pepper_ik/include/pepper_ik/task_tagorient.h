/**
    @file
    @author Alexander Sherikov
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
         * @brief Tag orientation
         */
        template <int t_features>
            class HUMOTO_LOCAL TaskTagOrientation : public humoto::TaskAB
        {
            #define HUMOTO_CONFIG_ENTRIES \
                HUMOTO_CONFIG_PARENT_CLASS(TaskAB) \
                HUMOTO_CONFIG_SCALAR_(k_orientation_gain) \
                HUMOTO_CONFIG_SCALAR_(tag_string_id) \
                HUMOTO_CONFIG_COMPOUND_(reference_rpy)
            #include HUMOTO_CONFIG_DEFINE_ACCESSORS


            protected:
                double              k_orientation_gain_;
                std::string         tag_string_id_;
                etools::Vector3     reference_rpy_;

                rbdl::TagLinkPtr    tag_;


            protected:
                virtual void setDefaults()
                {
                    TaskAB::setDefaults();
                    k_orientation_gain_ = 0.0;
                }


                /**
                 * @brief Log task.
                 *
                 * @param[in] logger logger
                 * @param[in] parent parent
                 * @param[in] name name
                 */
                virtual void    logTask(humoto::Logger &logger HUMOTO_GLOBAL_LOGGER_IF_DEFINED,
                                        const LogEntryName &parent = LogEntryName(),
                                        const std::string &name = "task") const
                {
                    LogEntryName subname = parent; subname.add(name);
                    TaskAB::logTask(logger, subname, "");
                    logger.log(LogEntryName(subname).add("k_orientation_gain"), k_orientation_gain_);
                    logger.log(LogEntryName(subname).add("tag_string_id"), tag_string_id_);
                    logger.log(LogEntryName(subname).add("reference_rpy"), reference_rpy_);
                }


            public:
                TaskTagOrientation( const std::string      &tag_string_id = "",
                                    const etools::Vector3  &reference_rpy = etools::Vector3::Zero(),
                                    const double            gain = 1.0,
                                    const double            k_orientation_gain = 1.0)
                    : TaskAB(std::string("TaskTagOrientation_") + tag_string_id, gain)
                {
                    k_orientation_gain_ = k_orientation_gain;
                    tag_string_id_ = tag_string_id;
                    reference_rpy_ = reference_rpy;
                }


                /// @copydoc humoto::TaskBase::form
                void form(  const humoto::SolutionStructure &sol_structure,
                            const humoto::Model &model_base,
                            const humoto::ControlProblem &control_problem)
                {
                    const Model<t_features>  &model   =
                        dynamic_cast <const Model<t_features> &>(model_base);

                    if(!tag_)
                    {
                        tag_ = model.getLinkTag(tag_string_id_);
                    }

                    Eigen::MatrixXd &A = getA();
                    Eigen::VectorXd &b = getB();


                    model.getTagOrientationJacobian(A, tag_);

                    b.noalias() = k_orientation_gain_
                                    * rigidbody::getRotationErrorAngleAxis(
                                            model.getTagOrientation(tag_),
                                            convertEulerAnglesToMatrix(
                                                reference_rpy_,
                                                rigidbody::EulerAngles::RPY));

                    if(! isApproximatelyEqual(1.0, getGain()))
                    {
                        A*=getGain();
                        b*=getGain();
                    }
                }
        };
    }//pepper
}//humoto
