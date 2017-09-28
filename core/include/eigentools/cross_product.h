/**
    @file
    @author  Alexander Sherikov
    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once


namespace etools
{
    /**
     * @brief Skew-symmetric cross product matrix
     */
    class CrossProductMatrix
    {
        private:
            etools::Vector3  vector_;


        public:
            /**
             * @brief Default constructor
             *
             * @param[in] vector 3d vector
             */
            explicit CrossProductMatrix(const etools::Vector3 &vector) : vector_(vector)
            {
            }


            /**
             * @brief Static function for transformation of a vector to cross product matrix.
             *
             * @param[in] vector
             *
             * @return cross product matrix
             */
            static etools::Matrix3 eval(const etools::Vector3 &vector)
            {
                return (   (etools::Matrix3() <<    0.0,            -vector.z(),    vector.y(),
                                                    vector.z(),     0.0,            -vector.x(),
                                                    -vector.y(),    vector.x(),     0.0).finished() );
            }


            /**
             * @brief Evaluate this to matrix.
             *
             * @return cross product matrix.
             */
            etools::Matrix3 eval() const
            {
                return(eval(vector_));
            }


            /**
             * @brief matrix * this
             *
             * @tparam t_DerivedInput   Eigen parameter
             * @tparam t_DerivedOutput  Eigen parameter
             *
             * @param[out] result
             * @param[in] matrix
             */
            template<   class t_DerivedInput,
                        class t_DerivedOutput>
                void multiplyLeft(  Eigen::PlainObjectBase<t_DerivedOutput>     &result,
                                    const Eigen::MatrixBase<t_DerivedInput>      &matrix) const
            {
                result.noalias() = matrix * eval();
            }


            /**
             * @brief this * matrix
             *
             * @tparam t_DerivedInput   Eigen parameter
             * @tparam t_DerivedOutput  Eigen parameter
             *
             * @param[out] result
             * @param[in] matrix
             */
            template<   class t_DerivedInput,
                        class t_DerivedOutput>
                void multiplyRight( Eigen::PlainObjectBase<t_DerivedOutput>     &result,
                                    const Eigen::MatrixBase<t_DerivedInput>      &matrix) const
            {
                result.noalias() = vector_.cross(matrix);
            }
    };



    /**
     * @brief Multiplication operator
     *
     * @tparam t_Derived Eigen parameter
     *
     * @param[in] left
     * @param[in] right
     *
     * @return result of multiplication
     */
    template<class t_Derived>
        Eigen::Matrix<  etools::DefaultScalar,
                        Eigen::DenseBase<t_Derived>::RowsAtCompileTime,
                        3>
        EIGENTOOLS_VISIBILITY_ATTRIBUTE
            operator* ( const Eigen::MatrixBase<t_Derived> & left,
                        const CrossProductMatrix & right)
    {
        Eigen::Matrix<  etools::DefaultScalar,
                        Eigen::DenseBase<t_Derived>::RowsAtCompileTime,
                        3>   result;
        right.multiplyLeft(result, left);
        return (result);
    }



    /**
     * @brief Multiplication operator
     *
     * @tparam t_Derived Eigen parameter
     *
     * @param[in] left
     * @param[in] right
     *
     * @return result of multiplication
     */
    template<class t_Derived>
        Eigen::Matrix<  etools::DefaultScalar,
                        3,
                        Eigen::DenseBase<t_Derived>::ColsAtCompileTime>
        EIGENTOOLS_VISIBILITY_ATTRIBUTE
            operator* ( const CrossProductMatrix & left,
                        const Eigen::MatrixBase<t_Derived> & right)
    {
        Eigen::Matrix<  etools::DefaultScalar,
                        3,
                        Eigen::DenseBase<t_Derived>::ColsAtCompileTime>   result;
        left.multiplyRight(result, right);
        return (result);
    }
}
