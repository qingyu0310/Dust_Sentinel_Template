/**
 * @file kalman.hpp
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2026-06-01
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include <stdint.h>

namespace alg::filter {

/**
 * @brief 矩阵运算返回状态。
 *
 * 这里保持非常小的状态集合，是为了让滤波器在裸机/RTOS 环境里也容易处理：
 * - Ok: 运算成功。
 * - SizeMismatch: 行列数不满足矩阵公式。
 * - Singular: 矩阵不可逆，通常说明观测协方差退化或数据异常。
 */
enum class MatrixStatus : int8_t
{
    Ok = 0,
    SizeMismatch = -1,
    Singular = -2,
};

/**
 * @brief 轻量矩阵视图。
 *
 * Matrix 不持有内存，只记录行列数和外部数据指针。这样 KalmanFilter 可以把所有
 * 工作区都放在对象内部的固定数组里，避免运行时动态分配，也方便在 Zephyr 线程里使用。
 */
struct Matrix
{
    uint8_t numRows = 0;
    uint8_t numCols = 0;
    float  *pData   = nullptr;
};

/* 基础矩阵运算。所有矩阵都使用行主序存储。 */
void         MatrixInit(Matrix *mat, uint8_t rows, uint8_t cols, float *data);
MatrixStatus MatrixAdd(const Matrix *lhs, const Matrix *rhs, Matrix *out);
MatrixStatus MatrixSubtract(const Matrix *lhs, const Matrix *rhs, Matrix *out);
MatrixStatus MatrixMultiply(const Matrix *lhs, const Matrix *rhs, Matrix *out);
MatrixStatus MatrixTranspose(const Matrix *src, Matrix *dst);
MatrixStatus MatrixInverse(const Matrix *src, Matrix *dst);

/**
 * @brief 通用线性 Kalman 滤波器。
 *
 * 本类只负责标准 Kalman 五步公式和矩阵缓存管理，不绑定具体传感器。姿态 EKF 这类非线性算法
 * 可以通过 Hook 在每一步之间改写 F/H/Q/R/K 等矩阵，从而复用同一个滤波核心。
 *
 * 状态向量:
 * - xhat: 当前后验估计。
 * - xhatminus: 当前先验预测。
 *
 * 常用矩阵:
 * - F: 状态转移矩阵。
 * - B: 控制输入矩阵，uSize 为 0 时不参与计算。
 * - H: 观测矩阵。
 * - Q/R: 过程噪声和观测噪声。
 * - P/Pminus: 后验/先验协方差。
 * - K: Kalman 增益。
 */
class KalmanFilter final
{
public:
    /* Hook 会在 Update() 的固定步骤间被调用，用于 EKF 线性化、自适应增益等扩展。 */
    using Hook = void (*)(KalmanFilter *kf);

    /* 固定上限用于避免堆内存。需要更大维度时优先评估 RAM 占用再扩大这些常量。 */
    static constexpr uint8_t kMaxStateSize   = 16;
    static constexpr uint8_t kMaxControlSize = 8;
    static constexpr uint8_t kMaxMeasureSize = 8;

    KalmanFilter();

    bool   Init(uint8_t xhat_size, uint8_t u_size, uint8_t z_size);
    void   Measure();
    void   UpdateXhatMinus();
    void   UpdatePminus();
    void   SetGain();
    void   UpdateXhat();
    void   UpdateCovariance();
    float *Update();

    void  SetUserData(void *user_data) { user_data_ = user_data; }
    void *UserData() const { return user_data_; }

    /* 对外暴露这些指针是为了兼容旧工程的调参/Hook 写法，实际内存仍由对象内部数组持有。 */
    float   *FilteredValue         = nullptr;
    float   *MeasuredVector        = nullptr;
    float   *ControlVector         = nullptr;
    uint8_t  xhatSize              = 0;
    uint8_t  uSize                 = 0;
    uint8_t  zSize                 = 0;
    bool     UseAutoAdjustment     = false;
    uint8_t  MeasurementValidNum   = 0;
    uint8_t *MeasurementMap        = nullptr;
    float   *MeasurementDegree     = nullptr;
    float   *MatR_DiagonalElements = nullptr;
    float   *StateMinVariance      = nullptr;

    /*
     * 跳过标志对应标准 Kalman 公式的几个步骤。
     * EKF 在某些阶段会自己实现部分公式，因此保留这些开关。
     */
    bool SkipEq1 = false;
    bool SkipEq2 = false;
    bool SkipEq3 = false;
    bool SkipEq4 = false;
    bool SkipEq5 = false;

    /* 矩阵视图区。上层可直接填 F/H/Q/R/P 等矩阵数据。 */
    Matrix xhat {};
    Matrix xhatminus {};
    Matrix u {};
    Matrix z {};
    Matrix P {};
    Matrix Pminus {};
    Matrix F {};
    Matrix FT {};
    Matrix B {};
    Matrix H {};
    Matrix HT {};
    Matrix Q {};
    Matrix R {};
    Matrix K {};
    Matrix S {};
    Matrix temp_matrix {};
    Matrix temp_matrix1 {};
    Matrix temp_vector {};
    Matrix temp_vector1 {};

    MatrixStatus MatStatus = MatrixStatus::Ok;

    /* Update() 各阶段的扩展点，按编号依次执行。 */
    Hook User_Func0_f = nullptr;
    Hook User_Func1_f = nullptr;
    Hook User_Func2_f = nullptr;
    Hook User_Func3_f = nullptr;
    Hook User_Func4_f = nullptr;
    Hook User_Func5_f = nullptr;
    Hook User_Func6_f = nullptr;

    float *xhat_data         = nullptr;
    float *xhatminus_data    = nullptr;
    float *u_data            = nullptr;
    float *z_data            = nullptr;
    float *P_data            = nullptr;
    float *Pminus_data       = nullptr;
    float *F_data            = nullptr;
    float *FT_data           = nullptr;
    float *B_data            = nullptr;
    float *H_data            = nullptr;
    float *HT_data           = nullptr;
    float *Q_data            = nullptr;
    float *R_data            = nullptr;
    float *K_data            = nullptr;
    float *S_data            = nullptr;
    float *temp_matrix_data  = nullptr;
    float *temp_matrix_data1 = nullptr;
    float *temp_vector_data  = nullptr;
    float *temp_vector_data1 = nullptr;

private:
    static constexpr uint16_t kMaxStateSquare   = kMaxStateSize * kMaxStateSize;
    static constexpr uint16_t kMaxStateControl  = kMaxStateSize * kMaxControlSize;
    static constexpr uint16_t kMaxStateMeasure  = kMaxStateSize * kMaxMeasureSize;
    static constexpr uint16_t kMaxMeasureState  = kMaxMeasureSize * kMaxStateSize;
    static constexpr uint16_t kMaxMeasureSquare = kMaxMeasureSize * kMaxMeasureSize;
    static constexpr uint8_t  kMaxVectorSize    = kMaxStateSize;

    void BindStorage();
    void ResetStorage();
    void ResetMatrices();
    void AdjustMeasurementMatrices();

    void *user_data_ = nullptr;

    uint8_t measurement_map_storage_[kMaxMeasureSize] {};
    float   measurement_degree_storage_[kMaxMeasureSize] {};
    float   mat_r_diagonal_storage_[kMaxMeasureSize] {};
    float   state_min_variance_storage_[kMaxStateSize] {};
    uint8_t temp_index_storage_[kMaxMeasureSize] {};

    float filtered_value_storage_[kMaxStateSize] {};
    float measured_vector_storage_[kMaxMeasureSize] {};
    float control_vector_storage_[kMaxControlSize] {};

    float xhat_data_storage_[kMaxStateSize] {};
    float xhatminus_data_storage_[kMaxStateSize] {};
    float u_data_storage_[kMaxControlSize] {};
    float z_data_storage_[kMaxMeasureSize] {};
    float p_data_storage_[kMaxStateSquare] {};
    float pminus_data_storage_[kMaxStateSquare] {};
    float f_data_storage_[kMaxStateSquare] {};
    float ft_data_storage_[kMaxStateSquare] {};
    float b_data_storage_[kMaxStateControl] {};
    float h_data_storage_[kMaxMeasureState] {};
    float ht_data_storage_[kMaxStateMeasure] {};
    float q_data_storage_[kMaxStateSquare] {};
    float r_data_storage_[kMaxMeasureSquare] {};
    float k_data_storage_[kMaxStateMeasure] {};
    float s_data_storage_[kMaxStateSquare] {};
    float temp_matrix_data_storage_[kMaxStateSquare] {};
    float temp_matrix1_data_storage_[kMaxStateSquare] {};
    float temp_vector_data_storage_[kMaxVectorSize] {};
    float temp_vector1_data_storage_[kMaxVectorSize] {};
};

} // namespace alg::filter
