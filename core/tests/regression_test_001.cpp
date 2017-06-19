/**
    @file
    @author  Alexander Sherikov
    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

// common & abstract classes (must be first)
#include "humoto/humoto.h"
#include "humoto/rigid_body.h"

//testing
#include "gtest/gtest.h"


namespace humoto
{
    class BlockMatrixTests : public ::testing::Test
    {
        protected:
            void createMatrix00_00()
            {
                etools::GenericBlockMatrix<2,2> M;
            }

            void createMatrix00_01()
            {
                etools::GenericBlockMatrix<0,0> M;
            }

            void createMatrix00_02()
            {
                etools::GenericBlockMatrix< etools::MatrixBlockSizeType::DYNAMIC,
                                            etools::MatrixBlockSizeType::DYNAMIC> M;
            }

            void createMatrix00_03()
            {
                etools::GenericBlockMatrix< etools::MatrixBlockSizeType::DYNAMIC,
                                            etools::MatrixBlockSizeType::DYNAMIC> M(2,2);
            }

            void createMatrix00_04()
            {
                etools::GenericBlockMatrix< etools::MatrixBlockSizeType::DYNAMIC,
                                            1> M(2,2);
            }

            void createMatrix00_05()
            {
                etools::GenericBlockMatrix< etools::MatrixBlockSizeType::DYNAMIC,
                                            1> M(2,etools::MatrixBlockSizeType::UNDEFINED);
            }


            void manipulateDynamicMatrix00_00()
            {
                etools::GenericBlockMatrix< etools::MatrixBlockSizeType::DYNAMIC,
                                            etools::MatrixBlockSizeType::DYNAMIC> M(2,2);

                M.setZero(3,1);
            }

            void manipulateDynamicMatrix00_01()
            {
                etools::GenericBlockMatrix< etools::MatrixBlockSizeType::DYNAMIC,
                                            etools::MatrixBlockSizeType::DYNAMIC> M(2,2);

                M.setZero(3);

                M(2) = Eigen::MatrixXd::Random(2,2);
            }


            void manipulateDynamicMatrix01_00()
            {
                std::vector< etools::GenericBlockMatrix< etools::MatrixBlockSizeType::DYNAMIC,
                                                         etools::MatrixBlockSizeType::DYNAMIC> > vector_of_matrices;

                vector_of_matrices.resize(3);
            }


            void manipulateDynamicMatrix02_00()
            {
                etools::GenericBlockMatrix< etools::MatrixBlockSizeType::DYNAMIC,
                                            etools::MatrixBlockSizeType::DYNAMIC>  M;
                M.setBlockSize(3, 2);
            }

            void manipulateDynamicMatrix02_01()
            {
                etools::GenericBlockMatrix< etools::MatrixBlockSizeType::DYNAMIC,
                                            etools::MatrixBlockSizeType::DYNAMIC>  M;
                M.setBlockSize(3, 0);
            }

            void manipulateDynamicMatrix02_02()
            {
                etools::GenericBlockMatrix< etools::MatrixBlockSizeType::DYNAMIC,
                                            2>  M;
                M.setBlockSize(3, etools::MatrixBlockSizeType::UNDEFINED);
            }

            void manipulateDynamicMatrix01_01()
            {
                std::vector< etools::GenericBlockMatrix< etools::MatrixBlockSizeType::DYNAMIC,
                                                         etools::MatrixBlockSizeType::DYNAMIC> > vector_of_matrices;

                vector_of_matrices.resize(3);
                vector_of_matrices[1].setBlockSize(3, 1);;
            }
    };


    TEST_F(BlockMatrixTests, MatrixCreation)
    {
        ASSERT_NO_THROW(createMatrix00_00());
        ASSERT_THROW(createMatrix00_01(), std::runtime_error);
        ASSERT_NO_THROW(createMatrix00_02());
        ASSERT_NO_THROW(createMatrix00_03());
        ASSERT_THROW(createMatrix00_04(), std::runtime_error);
        ASSERT_NO_THROW(createMatrix00_05());
    }


    TEST_F(BlockMatrixTests, MatrixManipulation)
    {
        ASSERT_NO_THROW(manipulateDynamicMatrix00_00());
        ASSERT_NO_THROW(manipulateDynamicMatrix00_01());
        ASSERT_NO_THROW(manipulateDynamicMatrix02_00());
        ASSERT_THROW(manipulateDynamicMatrix02_01(), std::runtime_error);
        ASSERT_NO_THROW(manipulateDynamicMatrix02_02());
        ASSERT_NO_THROW(manipulateDynamicMatrix01_00());
        ASSERT_NO_THROW(manipulateDynamicMatrix01_01());
    }


    TEST(EulerAngles, Transformation)
    {
        etools::Vector3 euler_angles;

        euler_angles << 0.5, 1.5, 2.5;

        etools::Vector3 euler_angles1 = convertEulerAngles(euler_angles, rigidbody::EulerAngles::RPY, rigidbody::EulerAngles::RPY);
        etools::Vector3 euler_angles2 = convertEulerAngles(euler_angles, rigidbody::EulerAngles::YPR, rigidbody::EulerAngles::YPR);

        ASSERT_TRUE(euler_angles1.isApprox(euler_angles, 1e-12));
        ASSERT_TRUE(euler_angles2.isApprox(euler_angles, 1e-12));
    }



    TEST(ConcatenateMatrixTests, ConcatenateVertically)
    {
        // empty matrix
        Eigen::MatrixXd a;
        // full matrix
        Eigen::MatrixXd b;
        b.resize(2, 2);
        b << 0., 1., 1., 0.;
        // empty matrix
        Eigen::MatrixXd c;
        // full matrix
        Eigen::MatrixXd d;
        d.resize(2, 2);
        d << 0., 2., 2., 0.;

        // matrix to match against
        Eigen::MatrixXd match;
        match.resize(4, 2);
        match << 0., 1., 1., 0., 0., 2., 2., 0.;

        std::vector<Eigen::MatrixXd> matrices;
        matrices.push_back(a);
        matrices.push_back(b);
        matrices.push_back(c);
        matrices.push_back(d);

        Eigen::MatrixXd result;

        etools::concatenateMatricesVertically(result, matrices);
        ASSERT_TRUE(result.isApprox(match,  1e-8));
    }


    TEST(ConcatenateMatrixTests, ConcatenateHorizontallyTwo)
    {
        // empty matrix
        Eigen::MatrixXd a;
        // full matrix
        Eigen::MatrixXd b;
        b.resize(2, 2);
        b << 0., 1., 1., 0.;
        // full matrix
        Eigen::MatrixXd c;
        c.resize(2, 2);
        c << 0., 2., 2., 0.;

        // matrix to match against
        Eigen::MatrixXd match;
        match.resize(2, 4);
        match << 0., 1., 0., 2., 1., 0., 2., 0.;

        Eigen::MatrixXd result;

        etools::concatenateMatricesHorizontally(result, a, b);
        ASSERT_TRUE(result.isApprox(b,  1e-8));

        etools::concatenateMatricesHorizontally(result, b, c);
        ASSERT_TRUE(result.isApprox(match,  1e-8));
    }


    TEST(ConcatenateMatrixTests, ConcatenateHorizontallyThree)
    {
        // empty matrix
        Eigen::MatrixXd a;
        // full matrix
        Eigen::MatrixXd b;
        b.resize(2, 2);
        b << 0., 1., 1., 0.;
        // empty matrix
        Eigen::MatrixXd c;
        // full matrix
        Eigen::MatrixXd d;
        d.resize(2, 2);
        d << 0., 2., 2., 0.;
        // full matrix
        Eigen::MatrixXd e;
        e.resize(2, 2);
        e << 0., 3., 3., 0.;

        // matrix to match against
        Eigen::MatrixXd match;
        match.resize(2, 6);
        match << 0., 1., 0., 2., 0., 3., 1., 0., 2., 0., 3., 0.;

        Eigen::MatrixXd result;

        etools::concatenateMatricesHorizontally(result, a, b, c);
        ASSERT_TRUE(result.isApprox(b,  1e-8));

        etools::concatenateMatricesHorizontally(result, b, d, e);
        ASSERT_TRUE(result.isApprox(match,  1e-8));
    }



    class TripleIntegratorTest : public ::testing::Test
    {
        protected:
            TripleIntegratorTest() : T_(0.1), Ts_(0.01)
            {
                // generate random initial state and control input
                X0_   = Eigen::VectorXd::Random(3);
                dddx_ = Eigen::VectorXd::Random(1);


                // A matrix for simple integrator
                Eigen::MatrixXd A = humoto::rigidbody::TripleIntegrator::getAJerk<1>(T_);

                // B matrix for simple integrator
                Eigen::MatrixXd B = humoto::rigidbody::TripleIntegrator::getBJerk<1>(T_);

                X1_ = A * X0_ + B * dddx_;


                // A matrix for simple integrator
                A = humoto::rigidbody::TripleIntegrator::getAJerk<1>(Ts_);

                // B matrix for simple integrator
                B = humoto::rigidbody::TripleIntegrator::getBJerk<1>(Ts_);

                X1s_ = A * X0_ + B * dddx_;
            }


        protected:
            const double    T_;
            const double    Ts_;

            Eigen::VectorXd X0_;
            Eigen::VectorXd X1_;
            Eigen::VectorXd X1s_;
            Eigen::VectorXd dddx_;
    };


    /**
     * @brief Check consistency of TripleIntegrator class
     */
    TEST_F(TripleIntegratorTest, OutputMatchesReferenceAcceleration)
    {
        Eigen::MatrixXd A;
        Eigen::MatrixXd B;
        Eigen::MatrixXd D;
        Eigen::MatrixXd E;

        A = humoto::rigidbody::TripleIntegrator::getAAcc<1>(T_);
        B = humoto::rigidbody::TripleIntegrator::getBAcc<1>(T_);
        D = humoto::rigidbody::TripleIntegrator::getDAcc<1>(T_);
        E = humoto::rigidbody::TripleIntegrator::getEAcc<1>(T_);

        // last state and jerk to compare
        Eigen::VectorXd X;
        Eigen::VectorXd j;

        X = A * X0_ + B * X1_(2);
        j = D * X0_ + E * X1_(2);

        ASSERT_TRUE(dddx_.isApprox(j,  1e-8));
        ASSERT_TRUE(    X.isApprox(X1_, 1e-8));
    }


    /**
     * @brief Check consistency of TripleIntegrator class
     */
    TEST_F(TripleIntegratorTest, OutputMatchesReferenceVelocity)
    {
        Eigen::MatrixXd A;
        Eigen::MatrixXd B;
        Eigen::MatrixXd D;
        Eigen::MatrixXd E;

        A = humoto::rigidbody::TripleIntegrator::getAVel<1>(T_);
        B = humoto::rigidbody::TripleIntegrator::getBVel<1>(T_);
        D = humoto::rigidbody::TripleIntegrator::getDVel<1>(T_);
        E = humoto::rigidbody::TripleIntegrator::getEVel<1>(T_);

        // last state and jerk to compare
        Eigen::VectorXd X;
        Eigen::VectorXd j;

        X = A * X0_ + B * X1_(1);
        j = D * X0_ + E * X1_(1);

        ASSERT_TRUE(dddx_.isApprox(j,  1e-8));
        ASSERT_TRUE(    X.isApprox(X1_, 1e-8));
    }


    /**
     * @brief Check consistency of TripleIntegrator class
     */
    TEST_F(TripleIntegratorTest, OutputMatchesReferencePosition)
    {
        Eigen::MatrixXd A;
        Eigen::MatrixXd B;
        Eigen::MatrixXd D;
        Eigen::MatrixXd E;

        A = humoto::rigidbody::TripleIntegrator::getAPos<1>(T_);
        B = humoto::rigidbody::TripleIntegrator::getBPos<1>(T_);
        D = humoto::rigidbody::TripleIntegrator::getDPos<1>(T_);
        E = humoto::rigidbody::TripleIntegrator::getEPos<1>(T_);

        // last state and jerk to compare
        Eigen::VectorXd X;
        Eigen::VectorXd j;

        X = A * X0_ + B * X1_(0);
        j = D * X0_ + E * X1_(0);

        ASSERT_TRUE(dddx_.isApprox(j,  1e-8));
        ASSERT_TRUE(    X.isApprox(X1_, 1e-8));
    }


    /**
     * @brief Check consistency of TripleIntegrator class
     */
    TEST_F(TripleIntegratorTest, OutputMatchesReferenceAccelerationSubsampling)
    {
        Eigen::MatrixXd As = humoto::rigidbody::TripleIntegrator::getAsAcc<1>(T_, Ts_);
        Eigen::MatrixXd Bs = humoto::rigidbody::TripleIntegrator::getBsAcc<1>(T_, Ts_);

        // last state and jerk to compare
        Eigen::VectorXd X1s = As * X0_ + Bs * X1_(2);

        ASSERT_TRUE(    X1s.isApprox(X1s_, 1e-8));
    }


    /**
     * @brief Check consistency of TripleIntegrator class
     */
    TEST_F(TripleIntegratorTest, OutputMatchesReferenceVelocitySubsampling)
    {
        Eigen::MatrixXd As = humoto::rigidbody::TripleIntegrator::getAsVel<1>(T_, Ts_);
        Eigen::MatrixXd Bs = humoto::rigidbody::TripleIntegrator::getBsVel<1>(T_, Ts_);

        // last state and jerk to compare
        Eigen::VectorXd X1s = As * X0_ + Bs * X1_(1);

        ASSERT_TRUE(    X1s.isApprox(X1s_, 1e-8));
    }


    /**
     * @brief Check consistency of TripleIntegrator class
     */
    TEST_F(TripleIntegratorTest, OutputMatchesReferencePositionSubsampling)
    {
        Eigen::MatrixXd As = humoto::rigidbody::TripleIntegrator::getAsPos<1>(T_, Ts_);
        Eigen::MatrixXd Bs = humoto::rigidbody::TripleIntegrator::getBsPos<1>(T_, Ts_);

        // last state and jerk to compare
        Eigen::VectorXd X1s = As * X0_ + Bs * X1_(0);

        ASSERT_TRUE(    X1s.isApprox(X1s_, 1e-8));
    }
}


/**
 * @brief main
 *
 * @param[in] argc number of args
 * @param[in] argv args
 *
 * @return status
 */
int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
