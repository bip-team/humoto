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
         * @brief base and body jerk minimization task
         */
        class HUMOTO_LOCAL TaskCoPCentering: public humoto::TaskAB
        {
            public:
                explicit TaskCoPCentering(const double gain = 0.707106781186548) 
                    : TaskAB("TaskCoPCentering", gain)
                {
                }


                /// @copydoc humoto::TaskBase::form
                void form(  const humoto::SolutionStructure &sol_structure,
                            const humoto::Model &model_base,
                            const humoto::ControlProblem &control_problem)
                {
                    const humoto::pepper_mpc::MPCforMG  &mpc   = dynamic_cast <const humoto::pepper_mpc::MPCforMG&>(control_problem);

                    getA().noalias() =  getGain()*mpc.Ap_;
                    getB().noalias() = -getGain()*mpc.sp_;
                }
        };
    }//pepper
}//humoto
