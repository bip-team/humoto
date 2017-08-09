/**
    @file
    @author  Alexander Sherikov
    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

/**
 * @defgroup BlockMatrixOperators BlockMatrixOperators
 * @brief Arithmetic operators for various block matrices
 *
 * @ingroup EigenTools
 */

namespace etools
{
#define EIGENTOOLS_CODE_GENERATOR(ClassName, MatrixType) \
        template<   int t_block_rows_num, \
                    int t_block_cols_num, \
                    MatrixSparsityType::Type t_sparsity_type> \
            class EIGENTOOLS_VISIBILITY_ATTRIBUTE ClassName : \
                public BlockMatrixBase<MatrixType, t_block_rows_num, t_block_cols_num, t_sparsity_type> \
        {\
            public:\
                ClassName(  MatrixType matrix,\
                            const std::ptrdiff_t  block_rows_num = MatrixBlockSizeType::UNDEFINED,\
                            const std::ptrdiff_t  block_cols_num = MatrixBlockSizeType::UNDEFINED)\
                    : BlockMatrixBase<  MatrixType, \
                                        t_block_rows_num, \
                                        t_block_cols_num, \
                                        t_sparsity_type>(matrix, block_rows_num, block_cols_num) {} \
        };


    /**
     * @brief Provides block interface for arbitrary Matrix without copying it.
     *
     * @tparam t_block_rows_num number of rows in one block
     * @tparam t_block_cols_num number of columns in one block
     * @tparam t_sparsity_type  sparsity type
     */
    EIGENTOOLS_CODE_GENERATOR(ConstBlockMatrixInterface, const DefaultDynamicMatrix &)

    /// @copydoc ConstBlockMatrixInterface
    EIGENTOOLS_CODE_GENERATOR(BlockMatrixInterface, DefaultDynamicMatrix &)
#undef EIGENTOOLS_CODE_GENERATOR


    // ===========================================================================
    // ===========================================================================
    // ===========================================================================


#define EIGENTOOLS_CODE_GENERATOR(class_name, sparsity_type) \
        template<   int t_block_rows_num,\
                    int t_block_cols_num>\
            class EIGENTOOLS_VISIBILITY_ATTRIBUTE class_name \
                : public BlockKroneckerProductBase< const DefaultDynamicMatrix &, \
                                                    t_block_rows_num, \
                                                    t_block_cols_num, \
                                                    sparsity_type> \
        {\
            public:\
                class_name( const DefaultDynamicMatrix & matrix, \
                            const std::ptrdiff_t  identity_size = 1, \
                            const std::ptrdiff_t  block_rows_num = MatrixBlockSizeType::UNDEFINED, \
                            const std::ptrdiff_t  block_cols_num = MatrixBlockSizeType::UNDEFINED) \
                    : BlockKroneckerProductBase<const DefaultDynamicMatrix &, \
                                                t_block_rows_num, \
                                                t_block_cols_num, \
                                                sparsity_type>( matrix, \
                                                                identity_size, \
                                                                block_rows_num, \
                                                                block_cols_num) {};\
        };
    /**
     * @brief A shorthand class for a specific sparsity type.
     *
     * @tparam t_block_rows_num number of rows in one block
     * @tparam t_block_cols_num number of columns in one block
     */
    EIGENTOOLS_CODE_GENERATOR(GenericBlockKroneckerProduct, MatrixSparsityType::NONE)
    /// @copydoc GenericBlockMatrix
    EIGENTOOLS_CODE_GENERATOR(DiagonalBlockKroneckerProduct, MatrixSparsityType::DIAGONAL)
    /// @copydoc GenericBlockMatrix
    EIGENTOOLS_CODE_GENERATOR(LeftLowerTriangularBlockKroneckerProduct, MatrixSparsityType::LEFT_LOWER_TRIANGULAR)
#undef EIGENTOOLS_CODE_GENERATOR


    // ===========================================================================
    // ===========================================================================
    // ===========================================================================


#define EIGENTOOLS_PARENT_CLASS_SHORTHAND BlockMatrixBase<  const Eigen::Map<   const EIGENTOOLS_DYNAMIC_MATRIX(t_Scalar),\
                                                                                t_alignment, \
                                                                                t_Stride >, \
                                                            t_block_rows_num, t_block_cols_num, t_sparsity_type>
    /**
     * @brief Blocked access to an Eigen::Map. Quick hack -- use with care.
     *
     * @tparam t_block_rows_num number of rows in one block
     * @tparam t_block_cols_num number of columns in one block
     * @tparam t_sparsity_type  sparsity type
     */
    template<   typename t_Scalar,
                int t_alignment,
                class t_Stride,
                int t_block_rows_num,
                int t_block_cols_num,
                MatrixSparsityType::Type t_sparsity_type>
        class EIGENTOOLS_VISIBILITY_ATTRIBUTE BlockMatrixMap : public EIGENTOOLS_PARENT_CLASS_SHORTHAND
    {
        protected:
            using EIGENTOOLS_PARENT_CLASS_SHORTHAND::matrix_;
            using EIGENTOOLS_PARENT_CLASS_SHORTHAND::block_rows_num_;
            using EIGENTOOLS_PARENT_CLASS_SHORTHAND::block_cols_num_;


        public:
            typedef typename EIGENTOOLS_PARENT_CLASS_SHORTHAND::DecayedRawMatrix    DecayedRawMatrix;

            EIGENTOOLS_DEFINE_BLOCK_MATRIX_CONSTRUCTORS(BlockMatrixMap)



            /**
             * @brief Get block Kronecker product of this matrix
             *
             * @param[in] identity_size
             *
             * @return block Kronecker product
             */
            BlockKroneckerProductBase<
                                const Eigen::Map<   const EIGENTOOLS_DYNAMIC_MATRIX(t_Scalar),
                                                    t_alignment,
                                                    t_Stride >,
                                t_block_rows_num,
                                t_block_cols_num,
                                t_sparsity_type>
                getBlockKroneckerProduct (const std::ptrdiff_t       identity_size) const
            {
                return(BlockKroneckerProductBase<
                        const Eigen::Map<   const EIGENTOOLS_DYNAMIC_MATRIX(t_Scalar),
                                            t_alignment,
                                            t_Stride >,
                        t_block_rows_num,
                        t_block_cols_num,
                        t_sparsity_type> (  matrix_,
                                            identity_size,
                                            ((t_block_rows_num > 0) ? MatrixBlockSizeType::UNDEFINED : block_rows_num_),
                                            ((t_block_cols_num > 0) ? MatrixBlockSizeType::UNDEFINED : block_cols_num_)));
            }
    };
#undef EIGENTOOLS_PARENT_CLASS_SHORTHAND


    // ===========================================================================
    // ===========================================================================
    // ===========================================================================


#define EIGENTOOLS_PARENT_CLASS_SHORTHAND BlockMatrixBase<DefaultDynamicMatrix, t_block_rows_num, t_block_cols_num, t_sparsity_type>
    /**
     * @brief Block matrix, the raw matrix is stored inside (not a reference).
     *
     * @tparam t_block_rows_num number of rows in one block
     * @tparam t_block_cols_num number of columns in one block
     * @tparam t_sparsity_type  sparsity type
     */
    template<   int t_block_rows_num,
                int t_block_cols_num,
                MatrixSparsityType::Type t_sparsity_type>
    class EIGENTOOLS_VISIBILITY_ATTRIBUTE BlockMatrix : public EIGENTOOLS_PARENT_CLASS_SHORTHAND
    {
        protected:
            using EIGENTOOLS_PARENT_CLASS_SHORTHAND::matrix_;
            using EIGENTOOLS_PARENT_CLASS_SHORTHAND::block_rows_num_;
            using EIGENTOOLS_PARENT_CLASS_SHORTHAND::block_cols_num_;


        public:
            typedef typename EIGENTOOLS_PARENT_CLASS_SHORTHAND::DecayedRawMatrix    DecayedRawMatrix;

            EIGENTOOLS_DEFINE_BLOCK_MATRIX_CONSTRUCTORS(BlockMatrix)


            typedef BlockMatrixMap< DefaultScalar,
                                    Eigen::Unaligned,
                                    Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>,
                                    1,
                                    t_block_cols_num,
                                    t_sparsity_type>     SelectionMap;


            SelectionMap    selectRowInBlocks(const std::ptrdiff_t row_in_a_block)
            {
                return (SelectionMap(etools::selectRows(matrix_,
                                                        EIGENTOOLS_BLOCKMATRIX_BLOCK_ROWS_NUM,
                                                        row_in_a_block),
                                     MatrixBlockSizeType::UNDEFINED,
                                     ((t_block_rows_num > 0) ? MatrixBlockSizeType::UNDEFINED : block_rows_num_)));
            }



            /**
             * @brief Get block Kronecker product of this matrix
             *
             * @param[in] identity_size
             *
             * @return block Kronecker product
             */
            BlockKroneckerProductBase<
                                const DecayedRawMatrix &,
                                t_block_rows_num,
                                t_block_cols_num,
                                t_sparsity_type>
                getBlockKroneckerProduct (const std::ptrdiff_t       identity_size) const
            {
                return(BlockKroneckerProductBase<
                        const DecayedRawMatrix &,
                        t_block_rows_num,
                        t_block_cols_num,
                        t_sparsity_type> (  matrix_,
                                            identity_size,
                                            ((t_block_rows_num > 0) ? MatrixBlockSizeType::UNDEFINED : block_rows_num_),
                                            ((t_block_cols_num > 0) ? MatrixBlockSizeType::UNDEFINED : block_cols_num_)));
            }
    };
#undef EIGENTOOLS_PARENT_CLASS_SHORTHAND


    // ===========================================================================
    // ===========================================================================
    // ===========================================================================

#define EIGENTOOLS_CODE_GENERATOR(class_name, sparsity_type) \
        template<   int t_block_rows_num,\
                    int t_block_cols_num>\
        class EIGENTOOLS_VISIBILITY_ATTRIBUTE class_name : public BlockMatrix<t_block_rows_num, t_block_cols_num, sparsity_type> \
        {\
            public:\
                class_name( const std::ptrdiff_t  block_rows_num = MatrixBlockSizeType::UNDEFINED, \
                            const std::ptrdiff_t  block_cols_num = MatrixBlockSizeType::UNDEFINED) \
                    : BlockMatrix<t_block_rows_num, t_block_cols_num, sparsity_type>(block_rows_num, block_cols_num) {};\
                class_name( const DefaultDynamicMatrix & matrix, \
                            const std::ptrdiff_t  block_rows_num = MatrixBlockSizeType::UNDEFINED, \
                            const std::ptrdiff_t  block_cols_num = MatrixBlockSizeType::UNDEFINED) \
                    : BlockMatrix<t_block_rows_num, t_block_cols_num, sparsity_type>(matrix, block_rows_num, block_cols_num) {};\
                class_name( DefaultDynamicMatrix & matrix, \
                            const std::ptrdiff_t  block_rows_num = MatrixBlockSizeType::UNDEFINED, \
                            const std::ptrdiff_t  block_cols_num = MatrixBlockSizeType::UNDEFINED) \
                    : BlockMatrix<t_block_rows_num, t_block_cols_num, sparsity_type>(matrix, block_rows_num, block_cols_num) {};\
        };
    /**
     * @brief A shorthand class for a specific sparsity type.
     *
     * @tparam t_block_rows_num number of rows in one block
     * @tparam t_block_cols_num number of columns in one block
     */
    EIGENTOOLS_CODE_GENERATOR(GenericBlockMatrix, MatrixSparsityType::NONE)
    /// @copydoc GenericBlockMatrix
    EIGENTOOLS_CODE_GENERATOR(DiagonalBlockMatrix, MatrixSparsityType::DIAGONAL)
    /// @copydoc GenericBlockMatrix
    EIGENTOOLS_CODE_GENERATOR(LeftLowerTriangularBlockMatrix, MatrixSparsityType::LEFT_LOWER_TRIANGULAR)
#undef EIGENTOOLS_CODE_GENERATOR

} // etools
