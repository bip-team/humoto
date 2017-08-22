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
         * @brief base CoM tracking
         */
        class HUMOTO_LOCAL TaskBaseCoMTrackingBase : public humoto::TaskAB
        {
            #define HUMOTO_CONFIG_ENTRIES \
                HUMOTO_CONFIG_PARENT_CLASS(TaskAB) \
                HUMOTO_CONFIG_SCALAR_(k_position_gain)
            #include HUMOTO_CONFIG_DEFINE_ACCESSORS


            protected:
                double  k_position_gain_;


            protected:
                TaskBaseCoMTrackingBase(const std::string & id,
                                        const double        gain,
                                        const double        k_position_gain) : TaskAB(id, gain)
                {
                    k_position_gain_ = k_position_gain;
                }


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
                }
        };



        /**
         * @brief base CoM tracking
         */
        template <int t_features>
            class HUMOTO_LOCAL TaskBaseCoMTracking : public TaskBaseCoMTrackingBase
        {
            public:
                TaskBaseCoMTracking(const double gain = 1.0,
                                    const double k_position_gain = 1.0) : TaskBaseCoMTrackingBase(  "TaskBaseCoMTracking",
                                                                                                    gain,
                                                                                                    k_position_gain) {}

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


                    model.getBaseCoMJacobian(A);
                    b = k_position_gain_ * (wbc.motion_parameters_.base_com_position_ - model.getBaseCoM());

                    if(! isApproximatelyEqual(1.0, getGain()))
                    {
                        A*=getGain();
                        b*=getGain();
                    }
                }
        };



        /**
         * @brief base CoM tracking
         */
#ifdef HUMOTO_DOXYGEN_PROCESSING
        template <>
            class HUMOTO_LOCAL TaskBaseCoMTracking<FIXED_WHEELS | ROOT_PLANAR>
            : public TaskBaseCoMTrackingBase
#else
        // Doxygen chokes on this for some reason
        template <>
            class HUMOTO_LOCAL TaskBaseCoMTracking<ModelFeatures::FIXED_WHEELS | ModelFeatures::ROOT_PLANAR>
            : public TaskBaseCoMTrackingBase
#endif
        {
            public:
                TaskBaseCoMTracking(const double gain = 1.0,
                                    const double k_position_gain = 1.0) : TaskBaseCoMTrackingBase(  "TaskBaseCoMTracking",
                                                                                                    gain,
                                                                                                    k_position_gain) {}


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


                    model.getBaseCoMJacobian(A);
                    A.conservativeResize(2, A.cols());

                    etools::Vector3 com = model.getBaseCoM();
                    b = k_position_gain_ * (wbc.motion_parameters_.base_com_position_.segment(0,2) - com.segment(0,2));

                    if(! isApproximatelyEqual(1.0, getGain()))
                    {
                        A*=getGain();
                        b*=getGain();
                    }
                }
        };
    }//pepper
}//humoto
