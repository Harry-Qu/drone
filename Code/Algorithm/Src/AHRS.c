/** 
 *  @brief	    姿态计算相关算法
 *  @details    提供了MadgWick和Mahony两种姿态解算算法
 *              提供了改进版MadgWick算法
 *              提供了四元数转换角度的方法
 *  @author     Harry-Qu
 *  @date       2022/10/26
 *  @version    1.1
 *  @par        日志
 *              1.0     |       实现姿态解算基本功能
 *              1.1     |       新增四元数初始化功能
*/

#include "AHRS.h"
#include "sdk_time.h"
#include "sdk_math.h"

#define twoKpDef    (2.0f * 5.0f)    // 2 * proportional gain
#define twoKiDef    (2.0f * 1.0f)    // 2 * integral gain

float twoKp = twoKpDef;                                            // 2 * proportional gain (Kp)
float twoKi = twoKiDef;                                            // 2 * integral gain (Ki)
float stepSize;

float stepMax = 0.4f, stepMin = 0.066f;
float betaMax = 0.5f, betaK = 0.05f;


/**
 * 根据步长动态求得beta值，最终公式为 β * ▽S(Q₀) / |▽S(Q₀)|
 * @param step 步长
 * @return β
 */
static float AHRS_dynamicBeta(float step) {
    if (step > stepMax) {
        return betaMax + (step - stepMax) * betaK;
    }
    return step / stepMax * betaMax;
}

/**
 * 根据步长动态求得K值，最终公式为 k * ▽S(Q₀)
 * @param step 步长
 * @return k
 */
static float AHRS_MadgWick_DynamicK(float step) {
    if (step < 0.5f) {
        return 0.8f;
    }
    return 0.15f + 0.5f * step;
}

void AHRS_MadgWick(quat_t *q, vector3f_t a, vector3f_t g, vector3f_t m) {
    uint32_t tickStart = sdk_time_GetMicroSecondsTick(); //用于计算算法耗时
    static uint32_t lastMadgwickTick = 0;

    float q0 = q->a, q1 = q->b, q2 = q->c, q3 = q->d;

    float beta = MADGWICK_BETA;
    float detaT = AHRS_DELTA_TIME;


    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0, _2q1, _2q2, _2q3;
    float _2q0q1, _2q0q2, _2q0q3, _2q1q1, _2q1q2, _2q1q3, _2q2q2, _2q2q3, _2q3q3;
    float bx, bz;

    float f[6];
    float Q_E2S[3][3], Q_S2E[3][3];

    // ½Q*Wₛ
    qDot1 = 0.5f * (-q1 * g.x - q2 * g.y - q3 * g.z);
    qDot2 = 0.5f * (q0 * g.x + q2 * g.z - q3 * g.y);
    qDot3 = 0.5f * (q0 * g.y - q1 * g.z + q3 * g.x);
    qDot4 = 0.5f * (q0 * g.z + q1 * g.y - q2 * g.x);

    if (!((a.x == 0.0f) && (a.y == 0.0f) && (a.z == 0.0f))) {

        recipNorm = invSqrt(a.x * a.x + a.y * a.y + a.z * a.z);
        a.x *= recipNorm;
        a.y *= recipNorm;
        a.z *= recipNorm;

        recipNorm = invSqrt(m.x * m.x + m.y * m.y + m.z * m.z);
        m.x *= recipNorm;
        m.y *= recipNorm;
        m.z *= recipNorm;

        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;

        _2q0q1 = _2q0 * q1;
        _2q0q2 = _2q0 * q2;
        _2q0q3 = _2q0 * q3;
        _2q1q1 = _2q1 * q1;
        _2q1q2 = _2q1 * q2;
        _2q1q3 = _2q1 * q3;
        _2q2q2 = _2q2 * q2;
        _2q2q3 = _2q2 * q3;
        _2q3q3 = _2q3 * q3;

        Q_E2S[0][0] = Q_S2E[0][0] = 1 - _2q2q2 - _2q3q3;
        Q_E2S[0][1] = Q_S2E[1][0] = _2q0q3 + _2q1q2;
        Q_E2S[0][2] = Q_S2E[2][0] = _2q1q3 - _2q0q2;
        Q_E2S[1][0] = Q_S2E[0][1] = -_2q0q3 + _2q1q2;
        Q_E2S[1][1] = Q_S2E[1][1] = 1 - _2q1q1 - _2q3q3;
        Q_E2S[1][2] = Q_S2E[2][1] = _2q0q1 + _2q2q3;
        Q_E2S[2][0] = Q_S2E[0][2] = _2q1q3 + _2q0q2;
        Q_E2S[2][1] = Q_S2E[1][2] = -_2q0q1 + _2q2q3;
        Q_E2S[2][2] = Q_S2E[2][2] = 1 - _2q1q1 - _2q2q2;

        // 磁偏校正
        hx = Q_S2E[0][0] * m.x + Q_S2E[0][1] * m.y + Q_S2E[0][2] * m.z;
        hy = Q_S2E[1][0] * m.x + Q_S2E[1][1] * m.y + Q_S2E[1][2] * m.z;
        bx = sqrtf(hx * hx + hy * hy);
        bz = Q_S2E[2][0] * m.x + Q_S2E[2][1] * m.y + Q_S2E[2][2] * m.z;


        //cal F
        f[0] = Q_E2S[0][2] - a.x;
        f[1] = Q_E2S[1][2] - a.y;
        f[2] = Q_E2S[2][2] - a.z;
        f[3] = Q_E2S[0][0] * bx + Q_E2S[0][2] * bz - m.x;
        f[4] = Q_E2S[1][0] * bx + Q_E2S[1][2] * bz - m.y;
        f[5] = Q_E2S[2][0] * bx + Q_E2S[2][2] * bz - m.z;

//        printf("%.2f %.2f %.2f %.2f %.2f %.2f\n", f[0], f[1], f[2], f[3], f[4], f[5]);

        //cal ▽S(Q₀) = JᵀF
        s0 = -q2 * f[0] + q1 * f[1] - q2 * bz * f[3] + (-q3 * bx + q1 * bz) * f[4] + q2 * bx * f[5];
        s1 = q3 * f[0] + q0 * f[1] - _2q1 * f[2] + (q3 * bx - _2q1 * bz) * f[5];
        s2 = -q0 * f[0] + q3 * f[1] - _2q2 * f[2] + (-_2q2 * bx - q0 * bz) * f[3] +
             (q1 * bx + q3 * bz) * f[4] + (q0 * bx - _2q2 * bz) * f[5];
        s3 = q1 * f[0] + q2 * f[1] + (-_2q3 * bx + q1 * bz) * f[3] + (-q0 * bx + q2 * bz) * f[4] + q1 * bx * f[5];

        //cal ▽S(Q₀) / |▽S(Q₀)|
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;
//        printf("stepSize:%.2f\n", stepSize);

        // β * ▽S(Q₀) / |▽S(Q₀)|
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    if (lastMadgwickTick) {
        detaT = (float) (HAL_GetTick() - lastMadgwickTick) / 1000.0f;
//        printf("%.2f\n", detaT);
    }
    lastMadgwickTick = HAL_GetTick();

    // Q=Q₀+(½Q * Wₛ - β * ▽S(Q₀) / |▽S(Q₀)|)Δt
    q0 += qDot1 * detaT;
    q1 += qDot2 * detaT;
    q2 += qDot3 * detaT;
    q3 += qDot4 * detaT;


    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    q->a = q0;
    q->b = q1;
    q->c = q2;
    q->d = q3;

    uint32_t tickEnd = sdk_time_GetMicroSecondsTick(); //用于计算算法耗时
//    printf("usedTime:%lu us\n", tickEnd - tickStart);
}

void AHRS_MadgWickTest(quat_t *q, vector3f_t a, vector3f_t g, vector3f_t m) {
//    uint32_t tickStart = sdk_time_GetMicroSecondsTick();
    static uint32_t lastMadgwickTick = 0;

    float q0 = q->a, q1 = q->b, q2 = q->c, q3 = q->d;

    float k = MADGWICK_BETA;
    float detaT = AHRS_DELTA_TIME;


    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0, _2q1, _2q2, _2q3;
    float _2q0q1, _2q0q2, _2q0q3, _2q1q1, _2q1q2, _2q1q3, _2q2q2, _2q2q3, _2q3q3;
    float bx, bz;

    float f[6];
    float Q_E2S[3][3], Q_S2E[3][3];

    // ½Q*Wₛ
    qDot1 = 0.5f * (-q1 * g.x - q2 * g.y - q3 * g.z);
    qDot2 = 0.5f * (q0 * g.x + q2 * g.z - q3 * g.y);
    qDot3 = 0.5f * (q0 * g.y - q1 * g.z + q3 * g.x);
    qDot4 = 0.5f * (q0 * g.z + q1 * g.y - q2 * g.x);

    if (!((a.x == 0.0f) && (a.y == 0.0f) && (a.z == 0.0f))) {

        recipNorm = invSqrt(a.x * a.x + a.y * a.y + a.z * a.z);
        a.x *= recipNorm;
        a.y *= recipNorm;
        a.z *= recipNorm;

        recipNorm = invSqrt(m.x * m.x + m.y * m.y + m.z * m.z);
        m.x *= recipNorm;
        m.y *= recipNorm;
        m.z *= recipNorm;

        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;

        _2q0q1 = _2q0 * q1;
        _2q0q2 = _2q0 * q2;
        _2q0q3 = _2q0 * q3;
        _2q1q1 = _2q1 * q1;
        _2q1q2 = _2q1 * q2;
        _2q1q3 = _2q1 * q3;
        _2q2q2 = _2q2 * q2;
        _2q2q3 = _2q2 * q3;
        _2q3q3 = _2q3 * q3;

        Q_E2S[0][0] = Q_S2E[0][0] = 1 - _2q2q2 - _2q3q3;
        Q_E2S[0][1] = Q_S2E[1][0] = _2q0q3 + _2q1q2;
        Q_E2S[0][2] = Q_S2E[2][0] = _2q1q3 - _2q0q2;
        Q_E2S[1][0] = Q_S2E[0][1] = -_2q0q3 + _2q1q2;
        Q_E2S[1][1] = Q_S2E[1][1] = 1 - _2q1q1 - _2q3q3;
        Q_E2S[1][2] = Q_S2E[2][1] = _2q0q1 + _2q2q3;
        Q_E2S[2][0] = Q_S2E[0][2] = _2q1q3 + _2q0q2;
        Q_E2S[2][1] = Q_S2E[1][2] = -_2q0q1 + _2q2q3;
        Q_E2S[2][2] = Q_S2E[2][2] = 1 - _2q1q1 - _2q2q2;

        // 磁偏校正
        hx = Q_S2E[0][0] * m.x + Q_S2E[0][1] * m.y + Q_S2E[0][2] * m.z;
        hy = Q_S2E[1][0] * m.x + Q_S2E[1][1] * m.y + Q_S2E[1][2] * m.z;
        bx = sqrtf(hx * hx + hy * hy);
        bz = Q_S2E[2][0] * m.x + Q_S2E[2][1] * m.y + Q_S2E[2][2] * m.z;


        //cal F
        f[0] = Q_E2S[0][2] - a.x;
        f[1] = Q_E2S[1][2] - a.y;
        f[2] = Q_E2S[2][2] - a.z;
        f[3] = Q_E2S[0][0] * bx + Q_E2S[0][2] * bz - m.x;
        f[4] = Q_E2S[1][0] * bx + Q_E2S[1][2] * bz - m.y;
        f[5] = Q_E2S[2][0] * bx + Q_E2S[2][2] * bz - m.z;

//        printf("%.2f %.2f %.2f %.2f %.2f %.2f\n", f[0], f[1], f[2], f[3], f[4], f[5]);

        //cal ▽S(Q₀) = JᵀF
        s0 = -q2 * f[0] + q1 * f[1] - q2 * bz * f[3] + (-q3 * bx + q1 * bz) * f[4] + q2 * bx * f[5];
        s1 = q3 * f[0] + q0 * f[1] - _2q1 * f[2] + (q3 * bx - _2q1 * bz) * f[5];
        s2 = -q0 * f[0] + q3 * f[1] - _2q2 * f[2] + (-_2q2 * bx - q0 * bz) * f[3] +
             (q1 * bx + q3 * bz) * f[4] + (q0 * bx - _2q2 * bz) * f[5];
        s3 = q1 * f[0] + q2 * f[1] + (-_2q3 * bx + q1 * bz) * f[3] + (-q0 * bx + q2 * bz) * f[4] + q1 * bx * f[5];

//        //cal ▽S(Q₀) / |▽S(Q₀)|
//        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
//        s0 *= recipNorm;
//        s1 *= recipNorm;
//        s2 *= recipNorm;
//        s3 *= recipNorm;
        stepSize = sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
//        printf("stepSize:%.2f\n", stepSize);

        // β * ▽S(Q₀) / |▽S(Q₀)|
        k = AHRS_MadgWick_DynamicK(stepSize);
        qDot1 -= k * s0;
        qDot2 -= k * s1;
        qDot3 -= k * s2;
        qDot4 -= k * s3;
    }

    if (lastMadgwickTick) {
        detaT = (float) (HAL_GetTick() - lastMadgwickTick) / 1000.0f;
//        printf("%.2f\n", detaT);
    }
    lastMadgwickTick = HAL_GetTick();

    // Q=Q₀+(½Q * Wₛ - β * ▽S(Q₀) / |▽S(Q₀)|)Δt
    q0 += qDot1 * detaT;
    q1 += qDot2 * detaT;
    q2 += qDot3 * detaT;
    q3 += qDot4 * detaT;


    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    q->a = q0;
    q->b = q1;
    q->c = q2;
    q->d = q3;

//    uint32_t tickEnd = sdk_time_GetMicroSecondsTick();
//    printf("usedTime:%lu us\n", tickEnd - tickStart);
}

void AHRS_Mahony(quat_t *q, vector3f_t a, vector3f_t g, vector3f_t m) {
    float q0 = q->a, q1 = q->b, q2 = q->c, q3 = q->d;

    static float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;

    float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float hx, hy, bx, bz;
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    float detaT = AHRS_DELTA_TIME;

    static uint32_t lastTick = 0;

    if (lastTick) {
        detaT = (float) (HAL_GetTick() - lastTick) / 1000.0f;
    }
    lastTick = HAL_GetTick();

    if (!((a.x == 0.0f) && (a.y == 0.0f) && (a.z == 0.0f))) {

        recipNorm = invSqrt(a.x * a.x + a.y * a.y + a.z * a.z);
        a.x *= recipNorm;
        a.y *= recipNorm;
        a.z *= recipNorm;

        recipNorm = invSqrt(m.x * m.x + m.y * m.y + m.z * m.z);
        m.x *= recipNorm;
        m.y *= recipNorm;
        m.z *= recipNorm;

        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

        hx = 2.0f * (m.x * (0.5f - q2q2 - q3q3) + m.y * (q1q2 - q0q3) + m.z * (q1q3 + q0q2));
        hy = 2.0f * (m.x * (q1q2 + q0q3) + m.y * (0.5f - q1q1 - q3q3) + m.z * (q2q3 - q0q1));
        bx = sqrtf(hx * hx + hy * hy);
        bz = 2.0f * (m.x * (q1q3 - q0q2) + m.y * (q2q3 + q0q1) + m.z * (0.5f - q1q1 - q2q2));

        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

        halfex = (a.y * halfvz - a.z * halfvy) + (m.y * halfwz - m.z * halfwy);
        halfey = (a.z * halfvx - a.x * halfvz) + (m.z * halfwx - m.x * halfwz);
        halfez = (a.x * halfvy - a.y * halfvx) + (m.x * halfwy - m.y * halfwx);

        if (twoKi > 0.0f) {
            integralFBx += twoKi * halfex * detaT;
            integralFBy += twoKi * halfey * detaT;
            integralFBz += twoKi * halfez * detaT;
            g.x += integralFBx;
            g.y += integralFBy;
            g.z += integralFBz;
        } else {
            integralFBx = 0.0f;
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        g.x += twoKp * halfex;
        g.y += twoKp * halfey;
        g.z += twoKp * halfez;
    }

    g.x *= (0.5f * detaT);
    g.y *= (0.5f * detaT);
    g.z *= (0.5f * detaT);
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * g.x - qc * g.y - q3 * g.z);
    q1 += (qa * g.x + qc * g.z - q3 * g.y);
    q2 += (qa * g.y - qb * g.z + q3 * g.x);
    q3 += (qa * g.z + qb * g.y - qc * g.x);

    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    q->a = q0;
    q->b = q1;
    q->c = q2;
    q->d = q3;
}

void AHRS_ConvertQuatToDegree(quat_t *q, vector3f_t *angle) {

    float tmp = 2.0f * q->b * q->d - 2.0f * q->a * q->c;
    LIMIT(tmp, -1.0f, 1.0f);
    angle->y = asinf(tmp) * RAD_TO_DEGREE;
    angle->x =
            -atan2f(2.0f * q->a * q->b + 2.0f * q->c * q->d, 2.0f * SQR(q->a) - 1 + 2.0f * SQR(q->d)) * RAD_TO_DEGREE;
    angle->z =
            -atan2f(2.0f * q->a * q->d + 2.0f * q->b * q->c, 2.0f * SQR(q->a) - 1 + 2.0f * SQR(q->b)) * RAD_TO_DEGREE;
}

void AHRS_InitQuat_MAG(quat_t *q, vector3f_t m) {
    float roll = 0, pitch = 0, yaw = 0;

    yaw = atan2f(m.y, m.x);

    yaw = -yaw;

    q->a = cosf(yaw / 2);
    q->b = 0;
    q->c = 0;
    q->d = sinf(yaw / 2);
}

void AHRS_InitQuat_IMUAndAXIS(quat_t *q, vector3f_t a, vector3f_t m) {

    float cosRoll, cosPitch, cosYaw;
    float sinRoll, sinPitch, sinYaw;

    float roll = 0, pitch = 0, yaw = 0;

    roll = atanf(a.y / sqrtf(SQR(a.x) + SQR(a.z)));
    pitch = atanf(a.x / sqrtf(SQR(a.y) + SQR(a.z)));
    yaw = -atan2f(m.y, m.x);

    cosRoll = cosf(roll / 2);
    cosPitch = cosf(pitch / 2);
    cosYaw = cosf(yaw / 2);
    sinRoll = sinf(roll / 2);
    sinPitch = sinf(pitch / 2);
    sinYaw = sinf(yaw / 2);


    q->a = cosRoll * cosPitch * cosYaw - sinRoll * sinPitch * sinYaw;
    q->b = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
    q->c = cosRoll * sinPitch * cosYaw - sinRoll * cosPitch * sinYaw;
    q->d = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;
}