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
         * @brief body CoM tracking
         */
        template <int t_features>
            class HUMOTO_LOCAL TaskBodyCoMTracking: public humoto::TaskAB
        {
            #define HUMOTO_CONFIG_ENTRIES \
                HUMOTO_CONFIG_PARENT_CLASS(TaskAB) \
                HUMOTO_CONFIG_SCALAR_(k_position_gain) \
                HUMOTO_CONFIG_SCALAR_(axis_flag)
            #include HUMOTO_CONFIG_DEFINE_ACCESSORS


            private:
                double  k_position_gain_;
                int     axis_flag_;


            protected:
                virtual void setDefaults()
                {
                    TaskAB::setDefaults();
                    k_position_gain_ = 0.0;
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
                    logger.log(LogEntryName(subname).add("k_position_gain"), k_position_gain_);
                    logger.log(LogEntryName(subname).add("axis_flag"), axis_flag_);
                }


            public:
                TaskBodyCoMTracking(const double gain = 1.0,
                                    const double k_position_gain = 1.0,
                                    const int    axis_flag = AxisIndex::FLAG_X | AxisIndex::FLAG_Y | AxisIndex::FLAG_Z,
                                    const char * description = "TaskBodyCoMTracking")
                    : TaskAB(description, gain)
                {
                    k_position_gain_    = k_position_gain;
                    axis_flag_          = axis_flag;
                }


                /// @copydoc humoto::TaskBase::form
                void form(  const humoto::SolutionStructure &sol_structure,
                            const humoto::Model &model_base,
                            const humoto::ControlProblem &control_problem)
                {
                    const WholeBodyController<t_features>  &wbc   =
                        dynamic_cast <const humoto::pepper_ik::WholeBodyController<t_features> &>(control_problem);

                    const Model<t_features>  &model   =
                        dynamic_cast <const humoto::pepper_ik::Model<t_features> &>(model_base);

                    Eigen::MatrixXd &A = getA();
                    Eigen::VectorXd &b = getB();

                    etools::Vector3 com = model.getBodyCoM();

                    int all_flags_set = AxisIndex::FLAG_X | AxisIndex::FLAG_Y | AxisIndex::FLAG_Z;
                    if (all_flags_set == axis_flag_)
                    {
                        model.getBodyCoMJacobian(A);
                        b = k_position_gain_ * (wbc.motion_parameters_.body_com_position_ - com);
                    }
                    else
                    {
                        Eigen::MatrixXd A_tmp;
                        Eigen::VectorXd b_tmp;

                        EigenIndex  num = 0;


                        if (AxisIndex::FLAG_X & axis_flag_)
                        {
                            ++num;
                        }
                        if (AxisIndex::FLAG_Y & axis_flag_)
                        {
                            ++num;
                        }
                        if (AxisIndex::FLAG_Z & axis_flag_)
                        {
                            ++num;
                        }

                        if (    (num == 0)
                                || ((all_flags_set | axis_flag_) != all_flags_set)  )
                        {
                            HUMOTO_THROW_MSG(std::string("Wrong set of flags in task '") + getDescription() + "'.");
                        }


                        model.getBodyCoMJacobian(A_tmp);
                        b_tmp = k_position_gain_ * (wbc.motion_parameters_.body_com_position_ - com);


                        A.resize(num, A_tmp.cols());
                        b.resize(num);

                        num = 0;
                        if (AxisIndex::FLAG_X & axis_flag_)
                        {
                            A.row(num) = A_tmp.row(AxisIndex::X);
                            b(num) = b_tmp(AxisIndex::X);
                            ++num;
                        }
                        if (AxisIndex::FLAG_Y & axis_flag_)
                        {
                            A.row(num) = A_tmp.row(AxisIndex::Y);
                            b(num) = b_tmp(AxisIndex::Y);
                            ++num;
                        }
                        if (AxisIndex::FLAG_Z & axis_flag_)
                        {
                            A.row(num) = A_tmp.row(AxisIndex::Z);
                            b(num) = b_tmp(AxisIndex::Z);
                            ++num;
                        }
                    }

                    if(! isApproximatelyEqual(1.0, getGain()))
                    {
                        A*=getGain();
                        b*=getGain();
                    }
                }
        };
    }//pepper
}//humoto
