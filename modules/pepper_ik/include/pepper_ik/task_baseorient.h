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
         * @brief base orientation
         */
        class HUMOTO_LOCAL TaskBaseOrientationBase : public humoto::TaskAB
        {
            #define HUMOTO_CONFIG_ENTRIES \
                HUMOTO_CONFIG_PARENT_CLASS(TaskAB) \
                HUMOTO_CONFIG_SCALAR_(k_orientation_gain)
            #include HUMOTO_CONFIG_DEFINE_ACCESSORS


            protected:
                double  k_orientation_gain_;


            protected:
                TaskBaseOrientationBase(const std::string & id,
                                        const double        gain,
                                        const double        k_orientation_gain) : TaskAB(id, gain)
                {
                    k_orientation_gain_ = k_orientation_gain;
                }


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
                }
        };



        /**
         * @brief base orientation
         */
        template <int t_features>
            class HUMOTO_LOCAL TaskBaseOrientation : public TaskBaseOrientationBase
        {
            public:
                TaskBaseOrientation(const double gain = 1.0,
                                    const double k_orientation_gain = 1.0) : TaskBaseOrientationBase(   "TaskBaseOrientation",
                                                                                                        gain,
                                                                                                        k_orientation_gain) {}

                /// @copydoc humoto::TaskBase::form
                void form(  const humoto::SolutionStructure &sol_structure,
                            const humoto::Model &model_base,
                            const humoto::ControlProblem &control_problem)
                {
                    const WholeBodyController<t_features>  &wbc   =
                        dynamic_cast <const WholeBodyController<t_features> &>(control_problem);

                    const Model<t_features>  &model   =
                        dynamic_cast <const Model<t_features> &>(model_base);


                    Eigen::MatrixXd &A = getA();
                    Eigen::VectorXd &b = getB();

                    model.getBaseOrientationJacobian(A);

                    b.noalias() = k_orientation_gain_
                                    * rigidbody::getRotationErrorAngleAxis(
                                            model.getBaseOrientation(),
                                            convertEulerAnglesToMatrix(
                                                wbc.motion_parameters_.base_orientation_rpy_,
                                                rigidbody::EulerAngles::RPY));

                    if(! isApproximatelyEqual(1.0, getGain()))
                    {
                        A*=getGain();
                        b*=getGain();
                    }
                }
        };


        /**
         * @brief base orientation
         */
#ifdef HUMOTO_DOXYGEN_PROCESSING
        template<>
            class HUMOTO_LOCAL TaskBaseOrientation<FIXED_WHEELS | ROOT_PLANAR>
            : public TaskBaseOrientationBase
#else
        // Doxygen chokes on this for some reason
        template<>
            class HUMOTO_LOCAL TaskBaseOrientation<ModelFeatures::FIXED_WHEELS | ModelFeatures::ROOT_PLANAR>
            : public TaskBaseOrientationBase
#endif
        {
            public:
                TaskBaseOrientation(const double gain = 1.0,
                                    const double k_orientation_gain = 1.0) : TaskBaseOrientationBase(   "TaskBaseOrientation",
                                                                                                        gain,
                                                                                                        k_orientation_gain) {}

                /// @copydoc humoto::TaskBase::form
                void form(  const humoto::SolutionStructure &sol_structure,
                            const humoto::Model &model_base,
                            const humoto::ControlProblem &control_problem)
                {
                    const WholeBodyController<ModelFeatures::FIXED_WHEELS | ModelFeatures::ROOT_PLANAR>  &wbc   =
                        dynamic_cast <const WholeBodyController<ModelFeatures::FIXED_WHEELS | ModelFeatures::ROOT_PLANAR> &>(control_problem);

                    const Model<ModelFeatures::FIXED_WHEELS | ModelFeatures::ROOT_PLANAR>  &model   =
                        dynamic_cast <const Model<ModelFeatures::FIXED_WHEELS | ModelFeatures::ROOT_PLANAR> &>(model_base);


                    Eigen::MatrixXd &A = getA();
                    Eigen::VectorXd &b = getB();

                    Eigen::MatrixXd J;
                    model.getBaseOrientationJacobian(J);
                    A = J.bottomRows(1);

                    b.resize(1);
                    b(0) =  k_orientation_gain_
                                *
                                rigidbody::getRotationErrorAngleAxis(
                                        model.getBaseOrientation(),
                                        convertEulerAnglesToMatrix(
                                            wbc.motion_parameters_.base_orientation_rpy_,
                                            rigidbody::EulerAngles::RPY)).z();

                    if(! isApproximatelyEqual(1.0, getGain()))
                    {
                        A*=getGain();
                        b*=getGain();
                    }
                }
        };
    }//pepper
}//humoto
