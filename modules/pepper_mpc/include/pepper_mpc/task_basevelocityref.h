/**
    @file
    @author Jan Michalczyk
    @author Alexander Sherikov
    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

namespace humoto
{
    namespace pepper_mpc
    {
        /**
         * @brief Task for minimizing base velocity against base reference velocity
         */
        class HUMOTO_LOCAL TaskBaseVelocityReference: public humoto::TaskGIB
        {
            public:
                explicit TaskBaseVelocityReference(const double gain = 1.0) 
                    : TaskGIB("TaskBaseVelocityReference", gain)
                {
                }


                /// @copydoc humoto::TaskBase::form
                void form(  const humoto::SolutionStructure &sol_structure,
                            const humoto::Model &model_base,
                            const humoto::ControlProblem &control_problem)
                {
                    const humoto::pepper_mpc::MPCforMG &mpc   = dynamic_cast <const humoto::pepper_mpc::MPCforMG &> (control_problem);

                    Location loc_var = sol_structure.getSolutionPartLocation(BASE_VEL_VARIABLES_ID);

                    humoto::IndexVector &I     = getIndices();
                    Eigen::VectorXd     &b     = getB();
                    Eigen::VectorXd     &gains = getIGains();

                    I.resize(loc_var.length_);
                    b.resize(loc_var.length_);
                    gains.setConstant(loc_var.length_, getGain());

                    for (std::size_t i = 0; i < loc_var.length_/2; ++i)
                    {
                        I[i*2]     = loc_var.offset_ + i*2;
                        I[i*2+1]   = loc_var.offset_ + i*2 + 1;

                        b.segment(i*2, 2) = getGain()*mpc.preview_horizon_.getBaseReferenceVelocity(i);
                    }
                };
        };
    } //pepper
} //humoto
