/**
    @file
    @author  Alexander Sherikov
    @author  Jan Michalczyk
    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

namespace humoto
{
    namespace wpg04
    {
        /**
         * @brief [task_dz.m]
         */
        class HUMOTO_LOCAL TaskCoPVelocity : public humoto::TaskAB
        {
            #define HUMOTO_CONFIG_ENTRIES \
                HUMOTO_CONFIG_PARENT_CLASS(TaskAB)
            #include HUMOTO_CONFIG_DEFINE_ACCESSORS
                
                
            protected:
                void setDefaults()
                {
                    TaskAB::setDefaults();
                    setGain(0.223606797749979);
                }


                void finalize()
                {
                    TaskAB::finalize();
                }
            
            
            public:
                explicit TaskCoPVelocity (const double gain = 0.223606797749979) 
                    : TaskAB("TaskCoPVelocity", gain)
                {
                }


                /// @copydoc humoto::TaskBase::form
                void form(  const humoto::SolutionStructure &sol_structure,
                            const humoto::Model &model_base,
                            const humoto::ControlProblem &control_problem)
                {
                    const humoto::wpg04::MPCforWPG  &mpc = dynamic_cast <const humoto::wpg04::MPCforWPG &> (control_problem);


                    Eigen::MatrixXd &A  = getA();
                    Eigen::VectorXd &b = getB();

                    A.resize(mpc.Sdz_.rows(), sol_structure.getNumberOfVariables());


                    sol_structure.getMatrixPart(COP_VARIABLES_ID, A).noalias() =
                        getGain() * sol_structure.getMatrixPart(COP_VARIABLES_ID, mpc.Sdz_);

                    sol_structure.getMatrixPart(FOOTPOS_VARIABLES_ID, A).noalias() =
                        getGain() * sol_structure.getMatrixPart(FOOTPOS_VARIABLES_ID, mpc.Sdz_);


                    b.noalias() = -getGain() * mpc.sdz_;
                };
        };
    }
}
