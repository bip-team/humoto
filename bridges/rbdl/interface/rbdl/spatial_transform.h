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
    namespace rbdl
    {
        /**
         * @brief This class collects enums and methods to facilitate partial
         * handling of spatial data, e.g., for discrimination of rotational,
         * translational, and complete Jacobians.
         */
        class HUMOTO_LOCAL SpatialType
        {
            public:
                /**
                 * @brief Type os spatial data.
                 */
                enum Type
                {
                    UNDEFINED = 0,
                    COMPLETE = 1,
                    TRANSLATION = 2,
                    ROTATION = 3
                };


                /**
                 * @brief Returns number of elements (dimensionality) for given
                 * spatial data type.
                 *
                 * @param[in] spatial_type
                 *
                 * @return 3 or 6
                 */
                static std::size_t  getNumberOfElements(const Type &spatial_type)
                {
                    switch(spatial_type)
                    {
                        case COMPLETE:
                            return(6);
                        case ROTATION:
                        case TRANSLATION:
                            return(3);
                        default:
                            HUMOTO_THROW_MSG("Unknown spatial type.");
                    }
                }
        };


        /**
         * @brief Efficient spatial transform with identity rotation matrix.
         */
        class HUMOTO_LOCAL SpatialTransformWithoutRotation
        {
            private:
                etools::Vector3     translation_;


            public:
                /**
                 * @brief Constructor
                 *
                 * @param[in] translation translation vector
                 */
                SpatialTransformWithoutRotation(const etools::Vector3 & translation) : translation_(translation)
                {
                }


                /**
                 * @brief Apply complete transformation
                 *
                 * @tparam t_Derived Eigen template parameter
                 *
                 * @param[out] result_block 6xN result matrix (@ref eigentools_casting_hack "const is dropped inside")
                 * @param[in] matrix        6xN matrix to transform
                 */
                template<class t_Derived>
                    void apply( Eigen::MatrixBase< t_Derived > const & result_block,
                                const Eigen::MatrixXd & matrix) const
                {
                    for (EigenIndex i = 0; i < result_block.cols(); ++i)
                    {
                        (const_cast< Eigen::MatrixBase<t_Derived>& >(result_block)).col(i)
                                            <<  matrix(0,i),
                                                matrix(1,i),
                                                matrix(2,i),
                                                translation_(2) * matrix(1,i) - translation_(1) * matrix(2,i) + matrix(3,i),
                                                translation_(0) * matrix(2,i) - translation_(2) * matrix(0,i) + matrix(4,i),
                                                translation_(1) * matrix(0,i) - translation_(0) * matrix(1,i) + matrix(5,i);
                    }
                }


                /**
                 * @brief Apply transformation and obtain only translational
                 * part of the result
                 *
                 * @tparam t_Derived Eigen template parameter
                 *
                 * @param[out] result_block 3xN result matrix (@ref eigentools_casting_hack "const is dropped inside")
                 * @param[in] matrix        6xN matrix to transform
                 */
                template<class t_Derived>
                    void applyGetTranslationPart(Eigen::MatrixBase< t_Derived > const & result_block,
                                                const Eigen::MatrixXd & matrix) const
                {
                    for (EigenIndex i = 0; i < result_block.cols(); ++i)
                    {
                        (const_cast< Eigen::MatrixBase<t_Derived>& >(result_block)).col(i)
                                            <<  translation_(2) * matrix(1,i) - translation_(1) * matrix(2,i) + matrix(3,i),
                                                translation_(0) * matrix(2,i) - translation_(2) * matrix(0,i) + matrix(4,i),
                                                translation_(1) * matrix(0,i) - translation_(0) * matrix(1,i) + matrix(5,i);
                    }
                }


                /**
                 * @brief Apply transformation and obtain only rotational part
                 * of the result
                 *
                 * @tparam t_Derived Eigen template parameter
                 *
                 * @param[out] result_block 3xN result matrix (@ref eigentools_casting_hack "const is dropped inside")
                 * @param[in] matrix        6xN matrix to transform
                 */
                template<class t_Derived>
                    void applyGetRotationPart(   Eigen::MatrixBase< t_Derived > const & result_block,
                                                const Eigen::MatrixXd & matrix) const
                {
                    const_cast< Eigen::MatrixBase<t_Derived>& >(result_block) = matrix.topRows(3);
                }


                template<SpatialType::Type t_type, class t_Derived>
                    void applySelective(Eigen::MatrixBase< t_Derived > const & result_block,
                                        const Eigen::MatrixXd & matrix) const
                {
                    switch (t_type)
                    {
                        case SpatialType::COMPLETE:
                            apply(result_block, matrix);
                            break;

                        case SpatialType::TRANSLATION:
                            applyGetTranslationPart(result_block, matrix);
                            break;

                        case SpatialType::ROTATION:
                            applyGetRotationPart(result_block, matrix);
                            break;

                        default:
                            HUMOTO_THROW_MSG("Unknown SpatialType.");
                    }
                }
        };
    }
}
