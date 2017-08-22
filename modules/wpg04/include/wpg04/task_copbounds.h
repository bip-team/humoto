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
         * @brief [task_copbounds.m]
         */
        class HUMOTO_LOCAL TaskCoPBounds : public humoto::TaskILU
        {
            #define HUMOTO_CONFIG_ENTRIES \
                HUMOTO_CONFIG_PARENT_CLASS(TaskILU)
            #include HUMOTO_CONFIG_DEFINE_ACCESSORS
                
                
            protected:
                void setDefaults()
                {
                    TaskILU::setDefaults();
                }


            public:
                TaskCoPBounds() : TaskILU("TaskCoPBounds")
                {
                }


                /// @copydoc humoto::TaskBase::form
                void form(  const humoto::SolutionStructure &sol_structure,
                            const humoto::Model &model_base,
                            const humoto::ControlProblem &control_problem)
                {
                    const humoto::wpg04::MPCforWPG  &mpc = dynamic_cast <const humoto::wpg04::MPCforWPG &> (control_problem);

                    Location        loc_var = sol_structure.getSolutionPartLocation(COP_VARIABLES_ID);

                    humoto::IndexVector &I  = getIndices();
                    Eigen::VectorXd &lb = getLowerBounds();
                    Eigen::VectorXd &ub = getUpperBounds();


                    I.resize(loc_var.length_);
                    lb.resize(loc_var.length_);
                    ub.resize(loc_var.length_);


                    for (std::size_t i = 0; i < loc_var.length_/2; ++i)
                    {
                        I[i*2]     = loc_var.offset_ + i*2;
                        I[i*2+1]   = loc_var.offset_ + i*2 + 1;

                        lb.segment(i*2, 2) = mpc.preview_horizon_.getCoPBounds(i).col(0);
                        ub.segment(i*2, 2) = mpc.preview_horizon_.getCoPBounds(i).col(1);
                    }
                }


                /// @copydoc humoto::TaskBase::guessActiveSet
				void guessActiveSet(const humoto::SolutionStructure &sol_structure,
									const humoto::Model &model_base,
									const humoto::ControlProblem &control_problem)
                {
                    Location loc_var = sol_structure.getSolutionPartLocation(COP_VARIABLES_ID);


					if (getActualActiveSet().size() == 0)
                    {
                        getActiveSetGuess().initialize(loc_var.length_, ConstraintActivationType::INACTIVE);
                    }
                    else
                    {
                        HUMOTO_ASSERT(  (getActualActiveSet().size() == loc_var.length_),
                                        "The number of CoP variables is not supposed to change.");

                        getActiveSetGuess() = getActualActiveSet();

                        getActiveSetGuess().shift(2, ConstraintActivationType::INACTIVE);
                    }
                }
        };
    }
}
