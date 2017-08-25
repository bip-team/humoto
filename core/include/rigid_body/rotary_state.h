/**
    @file
    @author  Alexander Sherikov
    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

namespace humoto
{
    namespace rigidbody
    {
        /**
         * @brief Types of Euler angles
         */
        class HUMOTO_LOCAL EulerAngles
        {
            public:
                enum Type
                {
                    UNDEFINED = 0,
                    XYZ = 1,
                    YPR = 1,
                    ZYX = 2,
                    RPY = 2
                };
        };


        /**
         * @brief Convert rotation matrix to Euler angles.
         *
         * @param[in] rotation_matrix
         * @param[in] euler_angles_type
         *
         * @return Euler angles
         */
        inline etools::Vector3 convertMatrixToEulerAngles( const etools::Matrix3 &rotation_matrix,
                                                           const EulerAngles::Type euler_angles_type)
        {
            switch (euler_angles_type)
            {
                case EulerAngles::YPR:
                    return (rotation_matrix.eulerAngles(0, 1, 2));
                    break;

                case EulerAngles::RPY:
                    return (rotation_matrix.eulerAngles(2, 1, 0).reverse());
                    break;

                default:
                    std::stringstream error_msg;
                    error_msg << "Euler angles of type '" << euler_angles_type << "' are not yet supported.";
                    HUMOTO_THROW_MSG(error_msg.str());
                    break;
            }
        }


        /**
         * @brief Convert rotation matrix to Euler angles.
         *
         * @param[in] euler_angles
         * @param[in] euler_angles_type
         *
         * @return Euler angles
         */
        inline etools::Matrix3 convertEulerAnglesToMatrix( const etools::Vector3 &euler_angles,
                                                           const EulerAngles::Type euler_angles_type)
        {
            switch (euler_angles_type)
            {
                case EulerAngles::RPY:
                    return(  ( Eigen::AngleAxisd(euler_angles(AngleIndex::YAW), Eigen::Vector3d::UnitZ())
                                * Eigen::AngleAxisd(euler_angles(AngleIndex::PITCH), Eigen::Vector3d::UnitY())
                                * Eigen::AngleAxisd(euler_angles(AngleIndex::ROLL), Eigen::Vector3d::UnitX())).matrix()  );
                    break;

                case EulerAngles::YPR:
                    return(  ( Eigen::AngleAxisd(euler_angles(AngleIndex::ROLL), Eigen::Vector3d::UnitX())
                                * Eigen::AngleAxisd(euler_angles(AngleIndex::PITCH), Eigen::Vector3d::UnitY())
                                * Eigen::AngleAxisd(euler_angles(AngleIndex::YAW), Eigen::Vector3d::UnitZ())).matrix()  );
                    break;

                default:
                    std::stringstream error_msg;
                    error_msg << "Euler angles of type '" << euler_angles_type << "' are not yet supported.";
                    HUMOTO_THROW_MSG(error_msg.str());
                    break;
            }
        }



        /**
         * @brief Convert Euler angles to a different set ofEuler angles
         *
         * @param[in] euler_angles
         * @param[in] euler_angles_type_from
         * @param[in] euler_angles_type_to
         *
         * @return Euler angles
         */
        inline etools::Vector3 convertEulerAngles( const etools::Vector3 &euler_angles,
                                                   const EulerAngles::Type euler_angles_type_from,
                                                   const EulerAngles::Type euler_angles_type_to)
        {
            etools::Matrix3 rotation_matrix = convertEulerAnglesToMatrix(euler_angles, euler_angles_type_from);
            return (convertMatrixToEulerAngles(rotation_matrix, euler_angles_type_to));
        }


        inline etools::Matrix3 getRotationError(   const etools::Matrix3 & current,
                                                   const etools::Matrix3 & reference)
        {
            return (reference * current.transpose());
        }


        inline etools::Vector3 getRotationErrorAngleAxis(  const etools::Matrix3 & current,
                                                           const etools::Matrix3 & reference)
        {
            return (0.5 * ( current.col(0).cross(reference.col(0))
                            +
                            current.col(1).cross(reference.col(1))
                            +
                            current.col(2).cross(reference.col(2)) )  );
        }


        /**
         * @brief Returns matrix M such that w = M * de.
         *
         * @param[in] euler_angles      current orientation (Euler angles)
         * @param[in] euler_angles_type euler angles type
         *
         * @return 3x3 matrix
         */
        inline etools::Matrix3  getEulerRatesToAngularVelocityTransform(
                const etools::Vector3   &euler_angles,
                const EulerAngles::Type euler_angles_type)
        {
            switch (euler_angles_type)
            {
                case EulerAngles::RPY:
                    return( ( etools::Matrix3()
                                <<  cos(euler_angles.y()) * cos(euler_angles.z()),  -sin(euler_angles.z()), 0.0,
                                    cos(euler_angles.y()) * sin(euler_angles.z()),  cos(euler_angles.z()),  0.0,
                                    -sin(euler_angles.y()),                         0.0,                    1.0 ).finished() );
                    break;

                default:
                    std::stringstream error_msg;
                    error_msg << "Euler angles of type '" << euler_angles_type << "' are not yet supported.";
                    HUMOTO_THROW_MSG(error_msg.str());
                    break;
            }
        }



        /**
         * @brief Integrate angular velocity to obtain orientation matrix.
         * Simple version.
         *
         * @param[in] orientation
         * @param[in] angular_velocity
         * @param[in] dt
         *
         * @return new orientation matrix
         */
        inline etools::Matrix3
            integrateAngularVelocity(
                const etools::Matrix3   & orientation,
                const etools::Vector3   & angular_velocity,
                const double            dt)
        {
            return(orientation + dt * orientation * etools::CrossProductMatrix(angular_velocity));
        }


        /**
         * @brief Integrate angular velocity to obtain orientation matrix.
         * Use Rodrigues formula.
         *
         * @param[in] orientation
         * @param[in] angular_velocity
         * @param[in] dt
         * @param[in] tolerance
         *
         * @return new orientation matrix
         */
        inline etools::Matrix3
            integrateAngularVelocityRodrigues(
                const etools::Matrix3   & orientation,
                const etools::Vector3   & angular_velocity,
                const double            dt,
                const double            tolerance = humoto::g_generic_tolerance)
        {
            double vel = angular_velocity.norm();

            if (std::abs(vel) < tolerance)
            {
                return(orientation);
            }
            else
            {
                etools::CrossProductMatrix tilde_axis(angular_velocity/vel);

                //R*(I + sin(v*dt)*tilde(ax) + (1-cos(v*dt))*tilde(ax)*tilde(ax))
                etools::Matrix3 orient_axis = orientation * tilde_axis;
                return (    orientation
                            + sin(vel*dt) * orient_axis
                            + (1 - cos(vel*dt)) * orient_axis * tilde_axis );
            }
        }


        /**
         * @brief Class that groups together parameters related to a robot foot
         */
        class HUMOTO_LOCAL RotaryState : public virtual humoto::config::ConfigurableBase
        {
            #define HUMOTO_CONFIG_SECTION_ID "RotaryState"
            #define HUMOTO_CONFIG_CONSTRUCTOR RotaryState
            #define HUMOTO_CONFIG_ENTRIES \
                    HUMOTO_CONFIG_COMPOUND_(rpy) \
                    HUMOTO_CONFIG_COMPOUND_(angular_velocity) \
                    HUMOTO_CONFIG_COMPOUND_(angular_acceleration)
            #include HUMOTO_CONFIG_DEFINE_ACCESSORS


            public:
                etools::Vector3 rpy_;
                etools::Vector3 angular_velocity_;
                etools::Vector3 angular_acceleration_;


            public:
                /**
                 * @brief Default constructor.
                 */
                RotaryState()
                {
                    setDefaults();
                }


                /**
                 * @brief Initialize state (everything is set to zeros).
                 */
                void setDefaults()
                {
                    rpy_.setZero();
                    angular_velocity_.setZero();
                    angular_acceleration_.setZero();
                }


                /**
                 * @brief Initialize state (everything is set to NaN).
                 */
                void unset()
                {
                    etools::unsetMatrix(rpy_);
                    etools::unsetMatrix(angular_velocity_);
                    etools::unsetMatrix(angular_acceleration_);
                }


                /**
                 * @brief Return orientation as a 3d matrix.
                 *
                 * @return 3d orientation matrix
                 */
                etools::Matrix3    getOrientationMatrix() const
                {
                    return ( convertEulerAnglesToMatrix(rpy_, EulerAngles::RPY));
                }


                /**
                 * @brief Log
                 *
                 * @param[in,out] logger logger
                 * @param[in] parent parent
                 * @param[in] name name
                 */
                void log(   humoto::Logger &logger HUMOTO_GLOBAL_LOGGER_IF_DEFINED,
                            const LogEntryName &parent = LogEntryName(),
                            const std::string &name = "rotary_state") const
                {
                    LogEntryName subname = parent; subname.add(name);

                    logger.log(LogEntryName(subname).add("rpy")             , rpy_);
                    logger.log(LogEntryName(subname).add("angular_velocity")    , angular_velocity_);
                    logger.log(LogEntryName(subname).add("angular_acceleration"), angular_acceleration_);
                }
        };
    }
}
