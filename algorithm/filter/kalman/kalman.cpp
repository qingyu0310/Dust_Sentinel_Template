/**
 * @file kalman.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2026-06-01
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "kalman.hpp"

#include <math.h>
#include <string.h>

namespace alg::filter {

namespace {

constexpr uint8_t kMaxMatrixDim = KalmanFilter::kMaxStateSize;
constexpr float   kPivotEpsilon = 1e-8f;

} // namespace

void MatrixInit(Matrix *mat, uint8_t rows, uint8_t cols, float *data)
{
    mat->numRows = rows;
    mat->numCols = cols;
    mat->pData   = data;
}

MatrixStatus MatrixAdd(const Matrix *lhs, const Matrix *rhs, Matrix *out)
{
    if (lhs->numRows != rhs->numRows || lhs->numCols != rhs->numCols ||
        out->numRows != lhs->numRows || out->numCols != lhs->numCols) {
        return MatrixStatus::SizeMismatch;
    }

    const uint16_t count = lhs->numRows * lhs->numCols;
    for (uint16_t i = 0; i < count; i++) {
        out->pData[i] = lhs->pData[i] + rhs->pData[i];
    }

    return MatrixStatus::Ok;
}

MatrixStatus MatrixSubtract(const Matrix *lhs, const Matrix *rhs, Matrix *out)
{
    if (lhs->numRows != rhs->numRows || lhs->numCols != rhs->numCols ||
        out->numRows != lhs->numRows || out->numCols != lhs->numCols) {
        return MatrixStatus::SizeMismatch;
    }

    const uint16_t count = lhs->numRows * lhs->numCols;
    for (uint16_t i = 0; i < count; i++) {
        out->pData[i] = lhs->pData[i] - rhs->pData[i];
    }

    return MatrixStatus::Ok;
}

MatrixStatus MatrixMultiply(const Matrix *lhs, const Matrix *rhs, Matrix *out)
{
    if (lhs->numCols != rhs->numRows ||
        out->numRows != lhs->numRows || out->numCols != rhs->numCols) {
        return MatrixStatus::SizeMismatch;
    }

    for (uint8_t row = 0; row < out->numRows; row++) {
        for (uint8_t col = 0; col < out->numCols; col++) {
            float sum = 0.0f;
            for (uint8_t k = 0; k < lhs->numCols; k++) {
                sum += lhs->pData[row * lhs->numCols + k] *
                       rhs->pData[k * rhs->numCols + col];
            }
            out->pData[row * out->numCols + col] = sum;
        }
    }

    return MatrixStatus::Ok;
}

MatrixStatus MatrixTranspose(const Matrix *src, Matrix *dst)
{
    if (dst->numRows != src->numCols || dst->numCols != src->numRows) {
        return MatrixStatus::SizeMismatch;
    }

    for (uint8_t row = 0; row < src->numRows; row++) {
        for (uint8_t col = 0; col < src->numCols; col++) {
            dst->pData[col * dst->numCols + row] = src->pData[row * src->numCols + col];
        }
    }

    return MatrixStatus::Ok;
}

MatrixStatus MatrixInverse(const Matrix *src, Matrix *dst)
{
    if (src->numRows != src->numCols || dst->numRows != src->numRows || dst->numCols != src->numCols) {
        return MatrixStatus::SizeMismatch;
    }

    const uint8_t n = src->numRows;
    if (n == 0 || n > kMaxMatrixDim) {
        return MatrixStatus::SizeMismatch;
    }

    float aug[kMaxMatrixDim][kMaxMatrixDim * 2] {};

    /* 使用 Gauss-Jordan 消元求逆：[A | I] -> [I | A^-1]。 */
    for (uint8_t row = 0; row < n; row++) {
        for (uint8_t col = 0; col < n; col++) {
            aug[row][col] = src->pData[row * src->numCols + col];
        }
        aug[row][n + row] = 1.0f;
    }

    for (uint8_t pivot = 0; pivot < n; pivot++) {
        uint8_t pivot_row = pivot;
        float   pivot_abs = fabsf(aug[pivot_row][pivot]);

        /* 选取当前列绝对值最大的主元，减小小主元带来的数值误差。 */
        for (uint8_t row = pivot + 1; row < n; row++) {
            const float cur_abs = fabsf(aug[row][pivot]);
            if (cur_abs > pivot_abs) {
                pivot_abs = cur_abs;
                pivot_row = row;
            }
        }

        if (pivot_abs < kPivotEpsilon) {
            return MatrixStatus::Singular;
        }

        if (pivot_row != pivot) {
            for (uint8_t col = 0; col < 2 * n; col++) {
                const float tmp = aug[pivot][col];
                aug[pivot][col] = aug[pivot_row][col];
                aug[pivot_row][col] = tmp;
            }
        }

        const float scale = aug[pivot][pivot];
        for (uint8_t col = 0; col < 2 * n; col++) {
            aug[pivot][col] /= scale;
        }

        for (uint8_t row = 0; row < n; row++) {
            if (row == pivot) {
                continue;
            }

            const float factor = aug[row][pivot];
            for (uint8_t col = 0; col < 2 * n; col++) {
                aug[row][col] -= factor * aug[pivot][col];
            }
        }
    }

    for (uint8_t row = 0; row < n; row++) {
        for (uint8_t col = 0; col < n; col++) {
            dst->pData[row * dst->numCols + col] = aug[row][n + col];
        }
    }

    return MatrixStatus::Ok;
}

KalmanFilter::KalmanFilter()
{
    BindStorage();
}

bool KalmanFilter::Init(uint8_t xhat_size, uint8_t u_size, uint8_t z_size)
{
    if (xhat_size == 0 || xhat_size > kMaxStateSize ||
        u_size > kMaxControlSize || z_size == 0 || z_size > kMaxMeasureSize) {
        return false;
    }

    xhatSize = xhat_size;
    uSize    = u_size;
    zSize    = z_size;

    UseAutoAdjustment   = false;
    MeasurementValidNum = 0;
    SkipEq1 = false;
    SkipEq2 = false;
    SkipEq3 = false;
    SkipEq4 = false;
    SkipEq5 = false;
    MatStatus = MatrixStatus::Ok;

    User_Func0_f = nullptr;
    User_Func1_f = nullptr;
    User_Func2_f = nullptr;
    User_Func3_f = nullptr;
    User_Func4_f = nullptr;
    User_Func5_f = nullptr;
    User_Func6_f = nullptr;

    ResetStorage();
    ResetMatrices();
    return true;
}

void KalmanFilter::Measure()
{
    if (UseAutoAdjustment) {
        AdjustMeasurementMatrices();
    } else {
        memcpy(z_data, MeasuredVector, sizeof(float) * zSize);
        memset(MeasuredVector, 0, sizeof(float) * zSize);
    }

    if (uSize > 0) {
        memcpy(u_data, ControlVector, sizeof(float) * uSize);
    }
}

void KalmanFilter::UpdateXhatMinus()
{
    if (SkipEq1) {
        return;
    }

    /* Eq1: x(k|k-1) = F * x(k-1|k-1) + B * u。 */
    if (uSize > 0) {
        temp_vector.numRows  = xhatSize;
        temp_vector.numCols  = 1;
        temp_vector1.numRows = xhatSize;
        temp_vector1.numCols = 1;

        MatStatus = MatrixMultiply(&F, &xhat, &temp_vector);
        MatStatus = MatrixMultiply(&B, &u, &temp_vector1);
        MatStatus = MatrixAdd(&temp_vector, &temp_vector1, &xhatminus);
    } else {
        MatStatus = MatrixMultiply(&F, &xhat, &xhatminus);
    }
}

void KalmanFilter::UpdatePminus()
{
    if (SkipEq2) {
        return;
    }

    /* Eq2: P(k|k-1) = F * P(k-1|k-1) * F^T + Q。 */
    MatStatus = MatrixTranspose(&F, &FT);
    MatStatus = MatrixMultiply(&F, &P, &Pminus);
    temp_matrix.numRows = Pminus.numRows;
    temp_matrix.numCols = FT.numCols;
    MatStatus = MatrixMultiply(&Pminus, &FT, &temp_matrix);
    MatStatus = MatrixAdd(&temp_matrix, &Q, &Pminus);
}

void KalmanFilter::SetGain()
{
    if (SkipEq3) {
        return;
    }

    /* Eq3: K = Pminus * H^T * (H * Pminus * H^T + R)^-1。 */
    MatStatus = MatrixTranspose(&H, &HT);
    temp_matrix.numRows = H.numRows;
    temp_matrix.numCols = Pminus.numCols;
    MatStatus = MatrixMultiply(&H, &Pminus, &temp_matrix);

    temp_matrix1.numRows = temp_matrix.numRows;
    temp_matrix1.numCols = HT.numCols;
    MatStatus = MatrixMultiply(&temp_matrix, &HT, &temp_matrix1);

    S.numRows = R.numRows;
    S.numCols = R.numCols;
    MatStatus = MatrixAdd(&temp_matrix1, &R, &S);
    MatStatus = MatrixInverse(&S, &temp_matrix1);

    temp_matrix.numRows = Pminus.numRows;
    temp_matrix.numCols = HT.numCols;
    MatStatus = MatrixMultiply(&Pminus, &HT, &temp_matrix);
    MatStatus = MatrixMultiply(&temp_matrix, &temp_matrix1, &K);
}

void KalmanFilter::UpdateXhat()
{
    if (SkipEq4) {
        return;
    }

    /* Eq4: xhat = xhatminus + K * (z - H * xhatminus)。 */
    temp_vector.numRows = H.numRows;
    temp_vector.numCols = 1;
    MatStatus = MatrixMultiply(&H, &xhatminus, &temp_vector);

    temp_vector1.numRows = z.numRows;
    temp_vector1.numCols = 1;
    MatStatus = MatrixSubtract(&z, &temp_vector, &temp_vector1);

    temp_vector.numRows = K.numRows;
    temp_vector.numCols = 1;
    MatStatus = MatrixMultiply(&K, &temp_vector1, &temp_vector);
    MatStatus = MatrixAdd(&xhatminus, &temp_vector, &xhat);
}

void KalmanFilter::UpdateCovariance()
{
    if (SkipEq5) {
        return;
    }

    /* Eq5: P = Pminus - K * H * Pminus。 */
    temp_matrix.numRows  = K.numRows;
    temp_matrix.numCols  = H.numCols;
    temp_matrix1.numRows = temp_matrix.numRows;
    temp_matrix1.numCols = Pminus.numCols;

    MatStatus = MatrixMultiply(&K, &H, &temp_matrix);
    MatStatus = MatrixMultiply(&temp_matrix, &Pminus, &temp_matrix1);
    MatStatus = MatrixSubtract(&Pminus, &temp_matrix1, &P);
}

float *KalmanFilter::Update()
{
    /*
     * 标准更新顺序：
     * 1. 整理观测/控制输入。
     * 2. 做先验预测。
     * 3. 根据有效观测修正后验。
     * Hook 插在关键步骤之间，给 EKF 或业务层改写矩阵的机会。
     */
    Measure();
    if (User_Func0_f != nullptr) {
        User_Func0_f(this);
    }

    UpdateXhatMinus();
    if (User_Func1_f != nullptr) {
        User_Func1_f(this);
    }

    UpdatePminus();
    if (User_Func2_f != nullptr) {
        User_Func2_f(this);
    }

    if (MeasurementValidNum != 0 || !UseAutoAdjustment) {
        SetGain();
        if (User_Func3_f != nullptr) {
            User_Func3_f(this);
        }

        UpdateXhat();
        if (User_Func4_f != nullptr) {
            User_Func4_f(this);
        }

        UpdateCovariance();
    } else {
        /* 自动观测模式下没有有效观测时，只保留预测值，不做后验修正。 */
        memcpy(xhat_data, xhatminus_data, sizeof(float) * xhatSize);
        memcpy(P_data, Pminus_data, sizeof(float) * xhatSize * xhatSize);
    }

    if (User_Func5_f != nullptr) {
        User_Func5_f(this);
    }

    for (uint8_t i = 0; i < xhatSize; i++) {
        const uint16_t idx = i * xhatSize + i;
        if (P_data[idx] < StateMinVariance[i]) {
            /* 协方差下限避免状态被“过度自信”锁死，后续观测仍能拉动估计。 */
            P_data[idx] = StateMinVariance[i];
        }
    }

    memcpy(FilteredValue, xhat_data, sizeof(float) * xhatSize);

    if (User_Func6_f != nullptr) {
        User_Func6_f(this);
    }

    return FilteredValue;
}

void KalmanFilter::BindStorage()
{
    /* 所有对外指针都绑定到对象内部固定数组，避免堆分配和生命周期问题。 */
    MeasurementMap        = measurement_map_storage_;
    MeasurementDegree     = measurement_degree_storage_;
    MatR_DiagonalElements = mat_r_diagonal_storage_;
    StateMinVariance      = state_min_variance_storage_;
    FilteredValue         = filtered_value_storage_;
    MeasuredVector        = measured_vector_storage_;
    ControlVector         = control_vector_storage_;

    xhat_data         = xhat_data_storage_;
    xhatminus_data    = xhatminus_data_storage_;
    u_data            = u_data_storage_;
    z_data            = z_data_storage_;
    P_data            = p_data_storage_;
    Pminus_data       = pminus_data_storage_;
    F_data            = f_data_storage_;
    FT_data           = ft_data_storage_;
    B_data            = b_data_storage_;
    H_data            = h_data_storage_;
    HT_data           = ht_data_storage_;
    Q_data            = q_data_storage_;
    R_data            = r_data_storage_;
    K_data            = k_data_storage_;
    S_data            = s_data_storage_;
    temp_matrix_data  = temp_matrix_data_storage_;
    temp_matrix_data1 = temp_matrix1_data_storage_;
    temp_vector_data  = temp_vector_data_storage_;
    temp_vector_data1 = temp_vector1_data_storage_;
}

void KalmanFilter::ResetStorage()
{
    memset(measurement_map_storage_, 0, sizeof(measurement_map_storage_));
    memset(measurement_degree_storage_, 0, sizeof(measurement_degree_storage_));
    memset(mat_r_diagonal_storage_, 0, sizeof(mat_r_diagonal_storage_));
    memset(state_min_variance_storage_, 0, sizeof(state_min_variance_storage_));
    memset(temp_index_storage_, 0, sizeof(temp_index_storage_));

    memset(filtered_value_storage_, 0, sizeof(filtered_value_storage_));
    memset(measured_vector_storage_, 0, sizeof(measured_vector_storage_));
    memset(control_vector_storage_, 0, sizeof(control_vector_storage_));

    memset(xhat_data_storage_, 0, sizeof(xhat_data_storage_));
    memset(xhatminus_data_storage_, 0, sizeof(xhatminus_data_storage_));
    memset(u_data_storage_, 0, sizeof(u_data_storage_));
    memset(z_data_storage_, 0, sizeof(z_data_storage_));
    memset(p_data_storage_, 0, sizeof(p_data_storage_));
    memset(pminus_data_storage_, 0, sizeof(pminus_data_storage_));
    memset(f_data_storage_, 0, sizeof(f_data_storage_));
    memset(ft_data_storage_, 0, sizeof(ft_data_storage_));
    memset(b_data_storage_, 0, sizeof(b_data_storage_));
    memset(h_data_storage_, 0, sizeof(h_data_storage_));
    memset(ht_data_storage_, 0, sizeof(ht_data_storage_));
    memset(q_data_storage_, 0, sizeof(q_data_storage_));
    memset(r_data_storage_, 0, sizeof(r_data_storage_));
    memset(k_data_storage_, 0, sizeof(k_data_storage_));
    memset(s_data_storage_, 0, sizeof(s_data_storage_));
    memset(temp_matrix_data_storage_, 0, sizeof(temp_matrix_data_storage_));
    memset(temp_matrix1_data_storage_, 0, sizeof(temp_matrix1_data_storage_));
    memset(temp_vector_data_storage_, 0, sizeof(temp_vector_data_storage_));
    memset(temp_vector1_data_storage_, 0, sizeof(temp_vector1_data_storage_));
}

void KalmanFilter::ResetMatrices()
{
    /* Matrix 只是视图，Init 后把每块工作区解释成对应行列数。 */
    MatrixInit(&xhat, xhatSize, 1, xhat_data);
    MatrixInit(&xhatminus, xhatSize, 1, xhatminus_data);
    MatrixInit(&u, uSize, 1, u_data);
    MatrixInit(&z, zSize, 1, z_data);
    MatrixInit(&P, xhatSize, xhatSize, P_data);
    MatrixInit(&Pminus, xhatSize, xhatSize, Pminus_data);
    MatrixInit(&F, xhatSize, xhatSize, F_data);
    MatrixInit(&FT, xhatSize, xhatSize, FT_data);
    MatrixInit(&B, xhatSize, uSize, B_data);
    MatrixInit(&H, zSize, xhatSize, H_data);
    MatrixInit(&HT, xhatSize, zSize, HT_data);
    MatrixInit(&Q, xhatSize, xhatSize, Q_data);
    MatrixInit(&R, zSize, zSize, R_data);
    MatrixInit(&K, xhatSize, zSize, K_data);
    MatrixInit(&S, xhatSize, xhatSize, S_data);
    MatrixInit(&temp_matrix, xhatSize, xhatSize, temp_matrix_data);
    MatrixInit(&temp_matrix1, xhatSize, xhatSize, temp_matrix_data1);
    MatrixInit(&temp_vector, xhatSize, 1, temp_vector_data);
    MatrixInit(&temp_vector1, xhatSize, 1, temp_vector_data1);
}

void KalmanFilter::AdjustMeasurementMatrices()
{
    MeasurementValidNum = 0;

    memcpy(z_data, MeasuredVector, sizeof(float) * zSize);
    memset(MeasuredVector, 0, sizeof(float) * zSize);

    memset(R_data, 0, sizeof(float) * zSize * zSize);
    memset(H_data, 0, sizeof(float) * xhatSize * zSize);

    /*
     * 自动观测模式：MeasuredVector 中值为 0 的通道视为无效观测。
     * 有效观测会被压缩到 z 的前 MeasurementValidNum 项，同时重建对应 H/R。
     */
    for (uint8_t i = 0; i < zSize; i++) {
        if (z_data[i] != 0.0f) {
            z_data[MeasurementValidNum] = z_data[i];
            temp_index_storage_[MeasurementValidNum] = i;
            H_data[xhatSize * MeasurementValidNum + MeasurementMap[i] - 1] = MeasurementDegree[i];
            MeasurementValidNum++;
        }
    }

    for (uint8_t i = 0; i < MeasurementValidNum; i++) {
        R_data[i * MeasurementValidNum + i] = MatR_DiagonalElements[temp_index_storage_[i]];
    }

    H.numRows  = MeasurementValidNum;
    H.numCols  = xhatSize;
    HT.numRows = xhatSize;
    HT.numCols = MeasurementValidNum;
    R.numRows  = MeasurementValidNum;
    R.numCols  = MeasurementValidNum;
    K.numRows  = xhatSize;
    K.numCols  = MeasurementValidNum;
    z.numRows  = MeasurementValidNum;
    z.numCols  = 1;
}

} // namespace alg::filter
