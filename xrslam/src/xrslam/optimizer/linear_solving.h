#ifndef __OPTIMIZER_LINEAR_SOLVING_H__
#define __OPTIMIZER_LINEAR_SOLVING_H__

#include <xrslam/common.h>
#include <ceres/block_random_access_diagonal_matrix.h>
#include <ceres/block_random_access_sparse_matrix.h>
#include <ceres/conjugate_gradients_solver.h>
#include <ceres/compressed_row_sparse_matrix.h>
#include <ceres/sparse_cholesky.h>
#include <ceres/triplet_sparse_matrix.h>
#include <xrslam/optimizer/linear_base.h>


namespace ceres::internal{

    class BlockRandomAccessSparseMatrixAdapter : public LinearOperator
    {
    public:
        explicit BlockRandomAccessSparseMatrixAdapter(const BlockRandomAccessSparseMatrix &m) : m_(m) {}

        virtual ~BlockRandomAccessSparseMatrixAdapter() {}

        // y = y + Ax;
        void RightMultiply(const double *x, double *y) const final { m_.SymmetricRightMultiply(x, y); }

        // y = y + A'x;
        void LeftMultiply(const double *x, double *y) const final { m_.SymmetricRightMultiply(x, y); }

        int num_rows() const final { return m_.num_rows(); }
        int num_cols() const final { return m_.num_rows(); }

    private:
        const BlockRandomAccessSparseMatrix &m_;
    };

    class BlockRandomAccessDiagonalMatrixAdapter : public LinearOperator{

    public:
        explicit BlockRandomAccessDiagonalMatrixAdapter(const BlockRandomAccessDiagonalMatrix &m) : m_(m) {}

        virtual ~BlockRandomAccessDiagonalMatrixAdapter() {}

        // y = y + Ax;
        void RightMultiply(const double *x, double *y) const final { m_.RightMultiply(x, y); }

        // y = y + A'x;
        void LeftMultiply(const double *x, double *y) const final { m_.RightMultiply(x, y); }

        int num_rows() const final { return m_.num_rows(); }
        int num_cols() const final { return m_.num_rows(); }

    private:
        const BlockRandomAccessDiagonalMatrix &m_;
    };

    inline void ConvertToBRSM(const xrslam::SparseBlockStorage &lhs, BlockRandomAccessSparseMatrix *sc){
        const int num_blocks = lhs.size();
        for (int id_row_block = 0; id_row_block < num_blocks; ++id_row_block)
        {
            for (auto &[id_col_block, block_matrix] : lhs.at(id_row_block))
            {
                int row, col, rs, cs;
                CellInfo *cell_info = sc->GetCell(id_row_block, id_col_block, &row, &col, &rs, &cs);
                xrslam::map<xrslam::matrix6> cell(cell_info->values);
                cell = block_matrix.transpose();
            }
        }
    }

    inline CompressedRowSparseMatrix *ConvertToCRSM(const int num_block, const xrslam::SparseBlockStorage &A){

        const int num_rows = 6 * num_block, num_cols = 6 * num_block;
        int num_nozero_block_ = 0;
        for (int id_row_block = 0; id_row_block < num_block; ++id_row_block){
            num_nozero_block_ += A[id_row_block].size();
        }
        CompressedRowSparseMatrix *output = new CompressedRowSparseMatrix(num_rows, num_cols, num_nozero_block_ * 36);
        int *output_rows = output->mutable_rows();
        int *output_cols = output->mutable_cols();
        double *output_values = output->mutable_values();

        int id = 0;
        output_rows[0] = 0;
        for (int id_row_block = 0; id_row_block < num_block; ++id_row_block)
        {
            for (int r = 0; r < 6; ++r)
            {
                const int row_id = 6 * id_row_block + r;
                for (auto &[id_col_block, block_matrix] : A[id_row_block]){
                    for (int c = 0; c < 6; ++c)
                    {
                        const int col_id = 6 * id_col_block + c;
                        output_cols[id] = col_id;
                        output_values[id] = block_matrix(r, c);
                        id++;
                    }
                }
                output_rows[row_id + 1] = output_rows[row_id] + A[id_row_block].size() * 6;
            }
        }

        return output;
    }

} // namespace ceres::internal


#endif