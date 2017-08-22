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
         * @brief maintain reference joint angles
         */
        template <int t_features>
            class HUMOTO_LOCAL TaskJointsReference : public humoto::TaskGIB
        {
            #define HUMOTO_CONFIG_ENTRIES \
                HUMOTO_CONFIG_PARENT_CLASS(TaskGIB) \
                HUMOTO_CONFIG_SCALAR_(k_position_gain) \
                HUMOTO_CONFIG_COMPOUND_(joint_angles_reference)
            #include HUMOTO_CONFIG_DEFINE_ACCESSORS


            private:
                double                                                                              k_position_gain_;
                EIGENTOOLS_CONSTANT_SIZE_VECTOR(ModelDescription<t_features>::JOINTS_DOF_NUMBER)    joint_angles_reference_;


            protected:
                virtual void setDefaults()
                {
                    TaskGIB::setDefaults();
                    k_position_gain_ = 0.0;
                    ModelDescription<t_features>::getDefaultJointAngles(joint_angles_reference_);
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
                    TaskGIB::logTask(logger, subname, "");

                    logger.log(LogEntryName(subname).add("k_position_gain"), k_position_gain_);
                    logger.log(LogEntryName(subname).add("joint_angles_reference"), joint_angles_reference_);
                }


            public:
                TaskJointsReference(const double gain = 1.0,
                                    const double k_position_gain = 1.0) : TaskGIB("TaskJointsReference", gain)
                {
                    k_position_gain_ = k_position_gain;

                    ModelDescription<t_features>::getDefaultJointAngles(joint_angles_reference_);
                }


                /// @copydoc humoto::TaskBase::form
                void form(  const humoto::SolutionStructure &sol_structure,
                            const humoto::Model &model_base,
                            const humoto::ControlProblem &control_problem)
                {
                    const humoto::pepper_ik::Model<t_features>  &model   =
                        dynamic_cast <const humoto::pepper_ik::Model<t_features> &>(model_base);

                    Location loc_var = sol_structure.getSolutionPartLocation(pepper_ik::JOINTS_VARIABLES_ID);

                    humoto::IndexVector &I     = getIndices();
                    Eigen::VectorXd     &b     = getB();
                    Eigen::VectorXd     &gains = getIGains();

                    I.resize(loc_var.length_);
                    b.resize(loc_var.length_);
                    gains.setConstant(loc_var.length_, getGain());

                    for (std::size_t i = 0; i < loc_var.length_; ++i)
                    {
                        I[i]     = loc_var.offset_ + i;
                    }

                    b.noalias() = k_position_gain_*getGain()*(joint_angles_reference_ - model.getState().joint_angles_);
                }
        };
    } //pepper
} //humoto
