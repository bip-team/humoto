/**
    @file
    @author  Alexander Sherikov
    @author  Don Joven Agravante
    @author  Jan Michalczyk
    @copyright 2014-2017 INRIA, 2014-2015 CNRS. Licensed under the Apache
    License, Version 2.0. (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)
    @brief
*/

#pragma once

namespace humoto
{
    namespace walking
    {
        class FootBoundsType
        {
            public:
                enum Type
                {
                    UNDEFINED = 0,
                    WITH_RESPECT_TO_ADS = 1,
                    WITH_RESPECT_TO_SS = 2,
                    ALIGNED_TO_SS = 3
                };
        };


        /**
         * @brief Parameters
         *
         *
         *  '"""""""""""""'                 \
         *  |             |                  | max_feet_dist
         *  ._____________.                  |
         *                  } min_feet_dist /
         *         * <- foot position
         *  `^^^^^^' max_step_len
         *
         */
        class HUMOTO_LOCAL RobotFootParameters : public humoto::config::ConfigurableBase
        {
            #define HUMOTO_CONFIG_SECTION_ID "RobotFootParameters"
            #define HUMOTO_CONFIG_CONSTRUCTOR RobotFootParameters 
            #define HUMOTO_CONFIG_ENTRIES \
                HUMOTO_CONFIG_SCALAR_(max_step_len); \
                HUMOTO_CONFIG_SCALAR_(min_feet_dist); \
                HUMOTO_CONFIG_SCALAR_(max_feet_dist); \
                HUMOTO_CONFIG_SCALAR_(feet_dist_default); \
                HUMOTO_CONFIG_SCALAR_(foot_length); \
                HUMOTO_CONFIG_SCALAR_(foot_width);
            #include HUMOTO_CONFIG_DEFINE_ACCESSORS


            private:
                /**
                 * @brief These parameters are defined for left and right foot/side.
                 */
                class LeftRightParameters
                {
                    public:
                        /// @{
                        /**
                         * Automatically computed variables, which are used to
                         * determine positions of aligned double supports with respect
                         * to single supports and vice versa.
                         */
                        etools::Vector2 foot_position_in_ads_;
                        etools::Vector2 ads_position_from_ss_;
                        /// @}


                        /// @{
                        /**
                         * Bounds on positions of the feet under various conditions.
                         */
                        etools::Matrix2 foot_position_bounds_ss_;
                        etools::Matrix2 foot_position_fixed_ss_;
                        etools::Matrix2 foot_position_fixed_ds_;
                        /// @}


                    public:
                        /**
                         * @brief Compute position of a foot based on position
                         * and orientation of an aligned double support.
                         *
                         * @param[in] ds_R  2d orientation matrix
                         * @param[in] ds_position 2d position of the ADS
                         *
                         * @return 2d position of a foot
                         */
                        etools::Vector2 getFootPositionFromADS(const etools::Matrix2 &ds_R,
                                                                const etools::Vector2 &ds_position) const
                        {
                            return(etools::transform(foot_position_in_ads_, ds_R, ds_position));
                        }


                        /**
                         * @brief Compute position of an aligned double support
                         * based on position and orientation of a foot.
                         *
                         * @param[in] ss_R  2d orientation matrix
                         * @param[in] ss_position 2d position of the foot
                         *
                         * @return 2d position of an ADS
                         */
                        etools::Vector2 getADSPositionFromFoot(const etools::Matrix2 &ss_R,
                                                                const etools::Vector2 &ss_position) const
                        {
                            return(etools::transform(ads_position_from_ss_, ss_R, ss_position));
                        }


                        /**
                         * @brief Get bounds on the foot position
                         *
                         * @param[in] bounds_type type of the bounds
                         *
                         * @return 2d matix: [lb, ub]
                         */
                        etools::Matrix2 getFootPositionBounds(const FootBoundsType::Type bounds_type) const
                        {
                            switch (bounds_type)
                            {
                                case FootBoundsType::WITH_RESPECT_TO_ADS:
                                    return(foot_position_fixed_ds_);
                                    break;
                                case FootBoundsType::WITH_RESPECT_TO_SS:
                                    return(foot_position_bounds_ss_);
                                    break;
                                case FootBoundsType::ALIGNED_TO_SS:
                                    return(foot_position_fixed_ss_);
                                    break;
                                default:
                                    HUMOTO_THROW_MSG("Unsupported type of foot position bounds.");
                            }
                        }
                };


            private:
                /// @{
                /**
                 * Various parameters of the feet and steps
                 */
                double max_step_len_;
                double min_feet_dist_;
                double max_feet_dist_;
                double feet_dist_default_;
                double foot_length_;
                double foot_width_;
                /// @}


                /// @{
                /**
                 * CoP bounds in single supports and aligned double supports.
                 */
                etools::Matrix2 ss_cop_bounds_;
                etools::Matrix2 ads_cop_bounds_;
                /// @}


                /// parameters corresponding to the left and right feet
                LeftRightContainer<LeftRightParameters> feet_;



            protected:
                /**
                 * @brief Create some useful parameters derived from the primal robot parameters
                 */
                void finalize()
                {
                    double half_foot_length = foot_length_/2.;
                    double half_foot_width = foot_width_/2.;
                    double half_total_width = (foot_width_ + feet_dist_default_)/2.;
                    double half_feet_dist_default = feet_dist_default_/2.;

                    feet_.getLeft().foot_position_in_ads_ <<  0.,  half_feet_dist_default;
                    feet_.getRight().foot_position_in_ads_ << 0., -half_feet_dist_default;

                    feet_.getRight().ads_position_from_ss_ << 0.,  half_feet_dist_default;
                    feet_.getLeft().ads_position_from_ss_ << 0., -half_feet_dist_default;

                    ss_cop_bounds_ <<
                        -half_foot_length,  half_foot_length,
                        -half_foot_width,   half_foot_width;

                    ads_cop_bounds_ <<
                        -half_foot_length,  half_foot_length,
                        -half_total_width,  half_total_width;

                    // With respect to the center of prseceding SS
                    feet_.getLeft().foot_position_bounds_ss_ <<
                        -max_step_len_,     max_step_len_,
                         min_feet_dist_,    max_feet_dist_;

                    feet_.getRight().foot_position_bounds_ss_ <<
                        -max_step_len_,     max_step_len_,
                        -max_feet_dist_,   -min_feet_dist_;

                    // With respect to the center of preceding SS (fixed position)
                    feet_.getLeft().foot_position_fixed_ss_ <<
                        0.,                       0.,
                        half_feet_dist_default,   half_feet_dist_default;

                    feet_.getRight().foot_position_fixed_ss_ <<
                         0.,                        0.,
                        -half_feet_dist_default,   -half_feet_dist_default;

                    // With respect to the center of preceding SS (fixed_position)
                    feet_.getLeft().foot_position_fixed_ds_ <<
                        0.,                         0.,
                        half_feet_dist_default,     half_feet_dist_default;

                    feet_.getRight().foot_position_fixed_ds_ <<
                         0.,                         0.,
                        -half_feet_dist_default,    -half_feet_dist_default;
                }

            public:
                /**
                 * @brief Default constructor
                 */
                RobotFootParameters()
                {
                    setDefaults();
                }


                /**
                 * @brief Initialize to default values (HRP2)
                 */
                void setDefaults()
                {
                    max_step_len_ = 0.2;
                    min_feet_dist_ = 0.19; // In mpc-walkgen 0.19 is in the DS only, 0.2 afterwards
                    max_feet_dist_ = 0.3;
                    feet_dist_default_ = 0.19;
                    foot_length_ = 0.1372;
                    foot_width_ = 0.058;

                    finalize();
                }


                /**
                 * @brief Get position of a foot or aligned double support
                 * based on position of an aligned support or a foot.
                 *
                 * @param[in] left_or_right left or right foot
                 * @param[in] ref_R orientation of the reference foot / ADS
                 * @param[in] ref_position position of the reference foot / ADS
                 *
                 * @return 2d position.
                 */
                etools::Vector2 getFootPositionFromADS(const humoto::LeftOrRight::Type left_or_right,
                                                        const etools::Matrix2 &ref_R,
                                                        const etools::Vector2 &ref_position) const
                {
                    return(feet_[left_or_right].getFootPositionFromADS(ref_R, ref_position));
                }


                /// @copydoc getFootPositionFromADS
                etools::Vector2 getADSPositionFromFoot(const humoto::LeftOrRight::Type left_or_right,
                                                        const etools::Matrix2 &ref_R,
                                                        const etools::Vector2 &ref_position) const
                {
                    return(feet_[left_or_right].getADSPositionFromFoot(ref_R, ref_position));
                }



                /**
                 * @brief Get bounds on position of a foot.
                 *
                 * @param[in] left_or_right left or right foot
                 * @param[in] bounds_type type of the bounds
                 *
                 * @return 2x2 matrix: [lb, ub]
                 */
                etools::Matrix2 getFootBounds( const humoto::LeftOrRight::Type left_or_right,
                                                const FootBoundsType::Type bounds_type) const
                {
                    return(feet_[left_or_right].getFootPositionBounds(bounds_type));
                }



                /**
                 * @brief Return bounds on position of the CoP in ADS / SS
                 *
                 * @return 2x2 matrix: [lb, ub]
                 */
                etools::Matrix2 getADSCoPBounds() const
                {
                    return(ads_cop_bounds_);
                }


                /// @copydoc getADSCoPBounds
                etools::Matrix2 getSSCoPBounds() const
                {
                    return(ss_cop_bounds_);
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
                            const std::string &name = "robot_foot_parameters") const
                {
                    LogEntryName subname = parent; subname.add(name);

                    logger.log(LogEntryName(subname).add("max_step_len")     , max_step_len_     );
                    logger.log(LogEntryName(subname).add("min_feet_dist")    , min_feet_dist_    );
                    logger.log(LogEntryName(subname).add("max_feet_dist")    , max_feet_dist_    );
                    logger.log(LogEntryName(subname).add("feet_dist_default"), feet_dist_default_);
                    logger.log(LogEntryName(subname).add("foot_length")      , foot_length_      );
                    logger.log(LogEntryName(subname).add("foot_width")       , foot_width_       );
                }
        };
    }
}
