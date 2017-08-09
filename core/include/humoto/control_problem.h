/**
    @file
    @author  Alexander Sherikov
    @author  Don Joven Agravante
    @copyright 2014-2017 INRIA, 2014-2015 CNRS. Licensed under the Apache
    License, Version 2.0. (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

namespace humoto
{
    /**
     * @brief Status of control problem
     */
    class ControlProblemStatus
    {
        public:
            enum Status
            {
                /// Not initialized
                UNDEFINED   = 0,
                /// Working
                OK          = 1,
                /// Control terminated
                STOPPED     = 2
            };
    };



    /**
     * @brief Abstract base class (for control problems)
     *
     * To be extended to represent each particular problem, i.e. walking pattern
     * generator or whole body controller.
     */
    class HUMOTO_LOCAL ControlProblem
    {
        protected:
            SolutionStructure   sol_structure_;


        protected:
            /**
             * @brief Protected destructor: prevent destruction of the child
             * classes through a base pointer.
             */
            ~ControlProblem() {}
            ControlProblem() {}


        public:
            /**
             * @brief Initialize structure of the given solution based on the
             * internally stored solution structure.
             *
             * @param[out] solution solution
             *
             * @attention This method is called automatically.
             */
            void initSolutionStructure(humoto::Solution &solution) const
            {
                HUMOTO_ASSERT(sol_structure_.isNonEmpty(),
                              "Structure of the solution in ControlProblem class is not properly initialized.");
                solution.initialize(sol_structure_);
            }



            /**
             * @brief Guess solution
             *
             * @param[out] solution_guess solution guess
             * @param[in] old_solution old solution
             *
             * @attention This method is called automatically. Can (and often
             * should) be redefined in derived classes.
             */
            virtual void guessSolution( Solution       &solution_guess,
                                        const Solution &old_solution) const
            {
                solution_guess.initialize(sol_structure_, old_solution);
            }


            virtual void log(   humoto::Logger &, const LogEntryName &, const std::string &) const = 0;
    };



    /**
     * @brief Abstract base class for Model Predictive Control problems.
     */
    class HUMOTO_LOCAL MPC : public humoto::ControlProblem
    {
        protected:
            /**
             * @brief Protected destructor: prevent destruction of the child
             * classes through a base pointer.
             */
            ~MPC() {}
            MPC() {}


            /**
             * @brief Condense output of the system.
             *
             * output = Ox*x0 + Ou*(u0,...,uN)
             *
             * @tparam t_num_vars               number of state variables
             * @tparam t_num_outputs            number of output variables
             * @tparam t_DMatrix                D matrix, should be an Eigen matrix or a scalar
             * @tparam t_EMatrix                E matrix, should be an Eigen matrix or a scalar
             *
             * @param[out] Ox       matrix Ox
             * @param[out] Ou       vector of Ou matrices
             * @param[in] D         vector of D matrices
             * @param[in] E         vector of E matrices
             * @param[in] Ux        matrix Ux
             * @param[in] Uu        vector of Uu matrices
             */
            template<int t_num_vars,
                     int t_num_outputs,
                     typename t_DMatrix,
                     typename t_EMatrix>
            void condenseOutput(etools::GenericBlockMatrix<t_num_outputs, t_num_vars>                               &Ox,
                                std::vector< etools::LeftLowerTriangularBlockMatrix<t_num_outputs,
                                                                            etools::MatrixBlockSizeType::DYNAMIC > >    &Ou,
                                const std::vector<t_DMatrix>    &D,
                                const std::vector<t_EMatrix>    &E,
                                const etools::GenericBlockMatrix<t_num_vars, t_num_vars>                                &Ux,
                                const std::vector< etools::LeftLowerTriangularBlockMatrix< t_num_vars,
                                                                                   etools::MatrixBlockSizeType::DYNAMIC > > &Uu)
            {
                std::ptrdiff_t N = Ux.getNumberOfBlocksVertical();

                HUMOTO_ASSERT(Ux.getNumberOfBlocksHorizontal() == 1,
                                "Wrong size of Ux matrix.")
                HUMOTO_ASSERT(D.size() == E.size(), "Mismatching number of D and E matrices.")
                HUMOTO_ASSERT(E.size() == static_cast<std::size_t>(N), "Wrong number of matrices.")


                std::ptrdiff_t  num_controls = 0;
                std::vector<std::ptrdiff_t> control_offsets;


                Ox.resize(N, 1);
                Ox(0, 0) = D[0];

                Ou.resize(Uu.size());
                control_offsets.resize(Uu.size());
                for (std::size_t i = 0; i < Uu.size(); ++i)
                {
                    control_offsets[i] = num_controls;
                    num_controls += Uu[i].getBlockColsNum();
                    Ou[i].setBlockSize(etools::MatrixBlockSizeType::UNDEFINED, Uu[i].getBlockColsNum());
                    Ou[i].setZero(N, N);
                }

                for (std::size_t i = 0; i < Uu.size(); ++i)
                {
                    Ou[i](N-1, N-1) = E[0].block( 0,
                                                  control_offsets[i],
                                                  t_num_outputs,
                                                  Uu[i].getBlockColsNum());
                }

                for (std::ptrdiff_t i = 1; i < N; ++i)
                {
                    Ox(i, 0).noalias() = D[i] * Ux(i-1, 0);

                    for (std::size_t j = 0; j < Uu.size(); ++j)
                    {
                        Ou[j].row(i, 0, i).noalias() = D[i] * Uu[j].row(i-1, 0, i);
                        Ou[j](i, i) = E[i].block( 0,
                                                  control_offsets[j],
                                                  t_num_outputs,
                                                  Uu[j].getBlockColsNum());
                    }
                }
            }



            /**
             * @brief Condense output of the system.
             *
             * output = Ox*x0 + Ou*(u0,...,uN)
             *
             * @tparam t_num_vars               number of state variables
             * @tparam t_num_controls           number of control variables
             * @tparam t_num_outputs            number of output variables
             * @tparam t_DMatrix                D matrix, should be an Eigen matrix or a scalar
             * @tparam t_EMatrix                E matrix, should be an Eigen matrix or a scalar
             *
             * @param[out] Ox       matrix Ox
             * @param[out] Ou       matrix Ou
             * @param[in] D         vector of D matrices
             * @param[in] E         vector of E matrices
             * @param[in] Ux        matrix Ux
             * @param[in] Uu        matrix Uu
             */
            template<int t_num_vars,
                     int t_num_controls,
                     int t_num_outputs,
                     typename t_DMatrix,
                     typename t_EMatrix>
            void condenseOutput(etools::GenericBlockMatrix<t_num_outputs, t_num_vars>                       &Ox,
                                etools::LeftLowerTriangularBlockMatrix<t_num_outputs, t_num_controls>       &Ou,
                                const std::vector<t_DMatrix>    &D,
                                const std::vector<t_EMatrix>    &E,
                                const etools::GenericBlockMatrix<t_num_vars, t_num_vars>                    &Ux,
                                const etools::LeftLowerTriangularBlockMatrix<t_num_vars, t_num_controls>    &Uu)
            {
                std::ptrdiff_t N = Ux.getNumberOfBlocksVertical();

                HUMOTO_ASSERT(Ux.getNumberOfBlocksHorizontal() == 1,
                                "Wrong size of Ux matrix.")
                HUMOTO_ASSERT( (Uu.getNumberOfBlocksHorizontal() == N)
                                && (Uu.getNumberOfBlocksVertical() == N),
                                "Mismatching size of input matrices.")
                HUMOTO_ASSERT(D.size() == E.size(), "Mismatching number of D and E matrices.")
                HUMOTO_ASSERT(E.size() == static_cast<std::size_t>(N), "Wrong number of matrices.")


                Ox.resize(N, 1);
                Ox(0, 0) = D[0];

                Ou.setZero(N, N);
                Ou(0, 0) = E[0];

                for (std::ptrdiff_t i = 1; i < N; ++i)
                {
                    Ox(i, 0).noalias() = D[i] * Ux(i-1, 0);

                    Ou.row(i, 0, i).noalias() = D[i] * Uu.row(i-1, 0, i);
                    Ou(i, i) = E[i];
                }
            }


            /**
             * @brief Create the condensed matrices (S,U) of a Model Predictive Control problem such that X = S*x0 + U*u
             *
             * @tparam t_num_vars       number of variables
             * @tparam t_num_controls   number of controls
             * @tparam t_AMatrix    A matrix, should be an Eigen matrix or a scalar
             * @tparam t_BMatrix    B matrix, should be an Eigen matrix
             *
             * @param[out] S
             * @param[out] U
             * @param[in] A
             * @param[in] B
             */
            template<int t_num_vars,
                     int t_num_controls,
                     typename t_AMatrix,
                     typename t_BMatrix,
                     etools::MatrixSparsityType::Type t_bmatrix_sparsity_type>
            void condense(  etools::GenericBlockMatrix<t_num_vars, t_num_vars>                       &S,
                            etools::BlockMatrix<t_num_vars, t_num_controls, t_bmatrix_sparsity_type> &U,
                            const std::vector<t_AMatrix> &A,
                            const std::vector<t_BMatrix> &B)
            {
                HUMOTO_ASSERT(A.size() == B.size(), "Mismatching number of A and B matrices.")
                HUMOTO_ASSERT(A.size() > 0, "Wrong number of matrices.")

                std::ptrdiff_t N = A.size();

                S.resize(N, 1);
                U.setZero(N, N);

                // form U (right to left)
                // S is partially formed in the process
                S(N-1, 0).setIdentity();
                // last column
                U(N-1, N-1) = B[N-1];
                // other columns
                for(std::ptrdiff_t k=N-2; k>=0; --k)
                {
                    //S.column(0, k+1, N-k-1) *= A[k+1];
                    //loop is faster for blocks with constant size
                    for (std::ptrdiff_t i = k+1; i < N; ++i)
                    {
                        S(i, 0) *= A[k+1];
                    }
                    S(k, 0).setIdentity();

                    U(k, k) = B[k];
                    //U.column(k, k+1, N-k-1).noalias() = S.column(0, k+1, N-k-1) * B[k];
                    //loop is faster for blocks with constant size
                    for (std::ptrdiff_t i = k+1; i < N; ++i)
                    {
                        U(i, k).noalias() = S(i, 0) * B[k];
                    }
                }


                // finalize formulation of S
                S.column(0) *= A[0];
            }


            /**
             * @brief Create the condensed matrices (S,U) of a Model Predictive Control problem such that X = S*x0 + U*u
             *
             * @tparam t_AMatrix    A matrix, should be an Eigen matrix or a scalar
             * @tparam t_BMatrix    B matrix, should be an Eigen matrix
             *
             * @param[out] S
             * @param[out] U
             * @param[in] A
             * @param[in] B
             */
            template<typename t_AMatrix, typename t_BMatrix>
            void condense(  Eigen::MatrixXd &S,
                            Eigen::MatrixXd &U,
                            const std::vector<t_AMatrix> &A,
                            const std::vector<t_BMatrix> &B)
            {
                HUMOTO_ASSERT(A.size() == B.size(), "Mismatching number of A and B matrices.")
                HUMOTO_ASSERT(A.size() > 0, "Wrong number of matrices.")
                HUMOTO_ASSERT((A[0].rows() == B[0].rows()) && (A[0].cols() == B[0].rows()), "Wrong size of matrices.");

                int N = A.size();
                int Ns = B[0].rows();
                int Nu = B[0].cols();


                S.resize(Ns * N, Ns);
                U.resize(Ns*N, Nu*N);

                // form U (right to left)
                // S is partially formed in the process
                U.setZero();
                S.block((N-1)*Ns, 0,        Ns, Ns).setIdentity();
                // last column
                U.block((N-1)*Ns, (N-1)*Nu, Ns, Nu) = B[N-1];
                // other columns
                for(int k=N-2; k>=0; --k)
                {
                    S.block((k+1)*Ns, 0, (N-k-1)*Ns, Ns) *= A[k+1];
                    S.block(k*Ns, 0, Ns, Ns).setIdentity();

                    U.block((k)*Ns, k*Nu, Ns, Nu) = B[k];
                    U.block((k+1)*Ns, k*Nu, (N-k-1)*Ns, Nu).noalias() = S.block((k+1)*Ns, 0, (N-k-1)*Ns, Ns) * B[k];
                }


                // finalize formulation of S
                S *= A[0];
            }


            /**
             * @brief Create the condensed matrices (S,U) of a Time Invariant (constant A,B)
             *        Model Predictive Control problem such that X = S*x0 + U*u
             *
             * @param[out] S
             * @param[out] U
             * @param[in] preview_horizon_len length of the preview horizon
             * @param[in] A
             * @param[in] B
             */
            void condenseTimeInvariant( Eigen::MatrixXd &S,
                                        Eigen::MatrixXd &U,
                                        const std::size_t preview_horizon_len,
                                        const Eigen::MatrixXd &A,
                                        const Eigen::MatrixXd &B)
            {
                // form S
                Eigen::MatrixXd M = A;
                int num_rows_A = A.rows();
                int num_cols_A = A.cols();
                S.resize(num_rows_A*preview_horizon_len, num_cols_A);
                S.block(0, 0 , num_rows_A, num_cols_A) = M;

                for(std::size_t i=1; i<preview_horizon_len; ++i)
                {
                    M = M * A;
                    S.block(i*num_rows_A, 0 , num_rows_A, num_cols_A) = M;
                }

                // form U
                int num_rows_B = B.rows();
                int num_cols_B = B.cols();
                Eigen::MatrixXd Ucol(num_rows_B*preview_horizon_len, num_cols_B);
                Ucol << B,
                        S.block(0, 0 , num_rows_B*(preview_horizon_len-1), num_cols_A) * B;
                U.resize(num_rows_B*preview_horizon_len, num_cols_B*preview_horizon_len);
                U.setZero();
                for(std::size_t i=0; i<preview_horizon_len; ++i)
                {
                    U.block(num_rows_B*i, num_cols_B*i, num_rows_B*(preview_horizon_len-i), num_cols_B) = Ucol.block(0, 0, num_rows_B*(preview_horizon_len-i), num_cols_B);
                }
            }
    };
}
