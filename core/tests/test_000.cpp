/**
    @file
    @author  Alexander Sherikov
    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#define HUMOTO_GLOBAL_LOGGER_ENABLED

// Enable YAML configuration files (must be first)
#include "humoto/config_yaml.h"
// common & abstract classes
#include "humoto/humoto.h"

HUMOTO_INITIALIZE_GLOBAL_LOGGER(std::cout);
// Octave file can be used for output
//HUMOTO_INITIALIZE_GLOBAL_LOGGER(output.m);


int main()
{
    HUMOTO_LOG_RAW("============================================");
    HUMOTO_LOG_RAW("Constants");
    HUMOTO_LOG_RAW("============================================");

    HUMOTO_LOG_RAW("Gravitational acceleration:");
    HUMOTO_LOG("g", humoto::g_gravitational_acceleration);

    HUMOTO_LOG_RAW("Pi:");
    HUMOTO_LOG("pi", humoto::g_pi);

    HUMOTO_LOG_RAW("Infinity:");
    HUMOTO_LOG("infinity", humoto::g_infinity);
    HUMOTO_LOG_RAW("============================================\n\n");


    HUMOTO_LOG_RAW("============================================");
    HUMOTO_LOG_RAW("Transformations");
    HUMOTO_LOG_RAW("============================================");

    Eigen::Rotation2D<double> rot(1.1);
    etools::Vector2 tran(1.2, 1.4);
    Eigen::MatrixXd matrix = Eigen::MatrixXd::Random(2,4);


    HUMOTO_LOG("rotation_matrix", rot.matrix());
    HUMOTO_LOG("translation_vector", tran);
    HUMOTO_LOG("original_matrix", matrix);

    HUMOTO_LOG("transformed_matrix", etools::transform(matrix, rot.matrix(), tran));
    HUMOTO_LOG_RAW("============================================\n\n");


    HUMOTO_LOG_RAW("============================================");
    HUMOTO_LOG_RAW("Timer");
    HUMOTO_LOG_RAW("============================================");

    humoto::Timer timer; // timer.start() is not necessary here
    for (int k = 0; k < 1000; ++k)
    {
        etools::transform(matrix, rot.matrix(), tran);
    }
    timer.stop();
    HUMOTO_LOG_RAW(timer);

    timer.start(); // restart timer
    for (int k = 0; k < 1000; ++k)
    {
        etools::transform(matrix, rot.matrix(), tran);
    }
    timer.stop();
    HUMOTO_LOG("timer", timer.get()); // alternative output
    HUMOTO_LOG_RAW("============================================\n\n");


    HUMOTO_LOG_RAW("============================================");
    HUMOTO_LOG_RAW("Construct block diagonal matrix");
    HUMOTO_LOG_RAW("============================================");
    std::vector<Eigen::MatrixXd>  matrices;
    matrices.push_back(rot.matrix());
    matrices.push_back(rot.matrix());
    matrices.push_back(rot.matrix());

    HUMOTO_LOG("block_diagonal_matrix", etools::makeBlockDiagonal(matrices));
    HUMOTO_LOG_RAW("============================================\n\n");


    HUMOTO_LOG_RAW("============================================");
    HUMOTO_LOG_RAW("Log matrices as parts of a cell array");
    HUMOTO_LOG_RAW("============================================");
    HUMOTO_LOG(humoto::LogEntryName("Matrix").add(std::size_t(0)), rot.matrix());
    HUMOTO_LOG(humoto::LogEntryName("Matrix").add(1), matrix);
    HUMOTO_LOG(humoto::LogEntryName("Matrix").add(2), tran);
    HUMOTO_LOG_RAW("============================================\n\n");


    HUMOTO_LOG_RAW("============================================");
    HUMOTO_LOG_RAW("Convert degrees to radians");
    HUMOTO_LOG_RAW("============================================");
    std::cout << humoto::convertDegreesToRadians(42) << std::endl;
    HUMOTO_LOG_RAW("============================================\n\n");


    HUMOTO_LOG_RAW("============================================");
    HUMOTO_LOG_RAW("Throwing and handling exceptions");
    HUMOTO_LOG_RAW("============================================");
    try
    {
        HUMOTO_THROW_MSG("Exception message.");
    }
    catch (const std::exception &e)
    {
        HUMOTO_LOG_RAW(e.what());
    }
    HUMOTO_LOG_RAW("============================================\n\n");


    HUMOTO_LOG_RAW("============================================");
    HUMOTO_LOG_RAW("Logging of a task");
    HUMOTO_LOG_RAW("============================================");
    humoto::TaskZeroVariables      task_zero_vars;
    task_zero_vars.log();
    task_zero_vars.log(HUMOTO_GLOBAL_LOGGER, humoto::LogEntryName(), "other_task_name");
    HUMOTO_LOG_RAW("============================================\n\n");


    HUMOTO_LOG_RAW("============================================");
    HUMOTO_LOG_RAW("Logging / writing / reading of a rigid body state");
    HUMOTO_LOG_RAW("============================================");
    humoto::rigidbody::RigidBodyState body_state;
    body_state.log();
    body_state.writeConfig<humoto::config::yaml::Writer>("rigid_body_state.yaml", "SomeRigidBodyName");
    body_state.readConfig<humoto::config::yaml::Reader>("rigid_body_state.yaml", "SomeRigidBodyName", false);
    HUMOTO_LOG_RAW("============================================\n\n");



    HUMOTO_LOG_RAW("============================================");
    HUMOTO_LOG_RAW("Various versions of triple integrator");
    HUMOTO_LOG_RAW("============================================");
    // sampling time T
    const double T = 1.;

    // number of integrators
    const std::size_t N = 3;

    // acceleration controlled system
    HUMOTO_LOG("A matrix, control variable: acceleration", humoto::rigidbody::TripleIntegrator::getAAcc<N>(T));
    HUMOTO_LOG("B matrix, control variable: acceleration", humoto::rigidbody::TripleIntegrator::getBAcc<N>(T));
    HUMOTO_LOG("D matrix, control variable: acceleration", humoto::rigidbody::TripleIntegrator::getDAcc<N>(T));
    HUMOTO_LOG("E matrix, control variable: acceleration", humoto::rigidbody::TripleIntegrator::getEAcc<N>(T));

    // velocity controlled system
    HUMOTO_LOG("A matrix, control variable: velocity", humoto::rigidbody::TripleIntegrator::getAVel<N>(T));
    HUMOTO_LOG("B matrix, control variable: velocity", humoto::rigidbody::TripleIntegrator::getBVel<N>(T));
    HUMOTO_LOG("D matrix, control variable: velocity", humoto::rigidbody::TripleIntegrator::getDVel<N>(T));
    HUMOTO_LOG("E matrix, control variable: velocity", humoto::rigidbody::TripleIntegrator::getEVel<N>(T));

    // position controlled system
    HUMOTO_LOG("A matrix, control variable: position", humoto::rigidbody::TripleIntegrator::getAPos<N>(T));
    HUMOTO_LOG("B matrix, control variable: position", humoto::rigidbody::TripleIntegrator::getBPos<N>(T));
    HUMOTO_LOG("D matrix, control variable: position", humoto::rigidbody::TripleIntegrator::getDPos<N>(T));
    HUMOTO_LOG("E matrix, control variable: position", humoto::rigidbody::TripleIntegrator::getEPos<N>(T));
    HUMOTO_LOG_RAW("============================================");

    return (0);
}
