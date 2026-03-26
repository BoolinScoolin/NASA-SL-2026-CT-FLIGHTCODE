#include "KalmanFilter.h"

KalmanFilter::KalmanFilter()
: R(20.0f),
  last_micros(0),
  is_initialized(false)
{
    // Initial state
    x[0] = 0.0f; // altitude
    x[1] = 0.0f; // velocity

    // Initial covariance
    mat3_identity(P);

    // Process noise covariance

    mat3_zero(Q);

    // (Q off-diagonals are 0 by default)
}

void KalmanFilter::initialize(float initial_alt) {
    x[0] = initial_alt;
    x[1] = 0.0f;
    last_micros = micros();
    is_initialized = true;
}

void KalmanFilter::predict(uint32_t current_micros) {

    /*
    sigma_a = 1e-1;  % IMU Process Noise
    sigma_b = 1.0;   % Accel-Bias Driving Process Noise

    Qk =   [sigma_a^2*deltat^4/4 sigma_a^2*deltat^3/2 0;
        sigma_a^2*deltat^3/2 sigma_a^2*deltat^2   0;
        0                    0                    sigma_b^2*deltat];
    */

    const float sigma_a = 1.0e-1f;
    const float sigma_b = 1.0f;

    if (last_predict_micros == 0)
    {
        last_predict_micros = current_micros;
        return;
    }

    float dt = (current_micros - last_predict_micros) * 1.0e-6f;
    last_predict_micros = current_micros;

    if (dt <= 0.0f)
    {
        return;
    }

    const float dt2 = dt * dt;
    const float dt3 = dt2 * dt;
    const float dt4 = dt2 * dt2;

    float F[3][3] = {
        {1.0f, dt,   -0.5f * dt2},
        {0.0f, 1.0f, -dt        },
        {0.0f, 0.0f, 1.0f       }
    };

    float Qk[3][3] = {
        {sigma_a * sigma_a * dt4 * 0.25f, sigma_a * sigma_a * dt3 * 0.5f, 0.0f},
        {sigma_a * sigma_a * dt3 * 0.5f,  sigma_a * sigma_a * dt2,        0.0f},
        {0.0f,                            0.0f,                           sigma_b * sigma_b * dt}
    };

    float Ft[3][3];
    float FP[3][3];
    float FPFt[3][3];
    float Pnew[3][3];

    transpose_3x3(Ft, F);
    matrix_multiplication_3x3(FP, F, P);
    matrix_multiplication_3x3(FPFt, FP, Ft);
    matrix_addition_3x3(Pnew, FPFt, Qk);
    symmetrize_3x3(Pnew);
    copy_3x3(P, Pnew);
}


void KalmanFilter::update(float z) {
    /*
    function [x_out, Pk_out] = baro_update_kalman(z_meas, Pk_in, x_in)
    sigma_baro = sqrt(0.03);
    R = sigma_baro.^2;
    
    % extract pseudostate (vertical channel and bias)
    px_in = [x_in(3) x_in(6) x_in(7)]';
   
    H = [-1 0 0];
    y = z_meas - H*px_in;   
    S = H*Pk_in*H' + R;
    K = (Pk_in*H')/S;
    
    % State update
    dpx_out = K*y;
    dx_out = [0 0 dpx_out(1) 0 0 dpx_out(2) dpx_out(3)]';
    x_out = x_in + dx_out;
    
    
    % Covariance update (Joseph form)
    I = eye(size(Pk_in));
    Pk_out = (I - K*H)*Pk_in*(I - K*H)' + K*R*K';
    Pk_out = 0.5*(Pk_out + Pk_out');  % Enforce symmetry
    end
    */

    // ------------------------------------------------------------
    // sigma_baro = sqrt(0.03);
    // R = sigma_baro^2;
    // ------------------------------------------------------------
    const float sigma_baro = sqrtf(0.03f);
    const float R = sigma_baro * sigma_baro;

    // ------------------------------------------------------------
    // extract pseudostate
    // px_in = [p v b]'
    // ------------------------------------------------------------
    float px_in[3];
    copy_3(px_in, x);

    // ------------------------------------------------------------
    // H = [-1 0 0]
    // ------------------------------------------------------------
    float H[3] = {-1.0f, 0.0f, 0.0f};

    // ------------------------------------------------------------
    // y = z_meas - H*px_in
    // ------------------------------------------------------------
    const float Hx = dot_product_3(H, px_in);
    const float y  = z - Hx;

    // ------------------------------------------------------------
    // S = H*P*H' + R
    //
    // compute PHt = P * H'
    // then S = H * PHt + R
    // ------------------------------------------------------------
    float PHt[3];
    matrix_vector_multiplication_3x3(PHt, P, H);

    const float S = dot_product_3(H, PHt) + R;

    if (S <= 1.0e-9f)
    {
        return;
    }

    // ------------------------------------------------------------
    // K = (P*H') / S
    // ------------------------------------------------------------
    float K[3];
    scalar_multiplication_3(K, PHt, 1.0f / S);

    // ------------------------------------------------------------
    // State update
    // dpx_out = K*y
    // x_out = x_in + dpx_out
    // ------------------------------------------------------------
    float dpx_out[3];
    scalar_multiplication_3(dpx_out, K, y);
    Serial.print(x[0]);
    Serial.print("  ");
    vector_addition_3(x, px_in, dpx_out);
    Serial.println(x[0]);

    // ------------------------------------------------------------
    // Covariance update (Joseph form)
    //
    // Pk_out = (I - K*H) * P * (I - K*H)' + K*R*K'
    // ------------------------------------------------------------
    float I[3][3];
    float KH[3][3];
    float A[3][3];
    float At[3][3];
    float AP[3][3];
    float APAt[3][3];
    float KKt[3][3];
    float KRKt[3][3];
    float Pnew[3][3];

    identity_3x3(I);
    outer_product_3x1(KH, K, H);
    matrix_subtraction_3x3(A, I, KH);

    transpose_3x3(At, A);
    matrix_multiplication_3x3(AP, A, P);
    matrix_multiplication_3x3(APAt, AP, At);

    outer_product_3x1(KKt, K, K);
    scalar_multiplication_3x3(KRKt, KKt, R);

    matrix_addition_3x3(Pnew, APAt, KRKt);
    symmetrize_3x3(Pnew);
    copy_3x3(P, Pnew);

}

/*
void KalmanFilter::update(float z, uint32_t current_micros) {
    if (!is_initialized) {
        initialize(z);
        return;
    }

    // --- Dynamic dt Calculation with Safety ---
    float dt = (current_micros - last_micros) * 1.0e-6f;
    last_micros = current_micros;

    // Guard against first-loop spikes or zero-time errors
    if (dt <= 0.0f || dt > 0.5f) {
        dt = 0.031f;
    }

    // ============================================================
    // 1) PREDICT
    // Model: x' = A x,  A = [1 dt; 0 1]
    // ============================================================

    // Predicted state xp = A*x
    const float xp0 = x[0] + dt * x[1];
    const float xp1 = x[1];

    // Predicted covariance: Pp = A*P*A' + Q
    // With A = [1 dt; 0 1], expand in closed-form for speed.
    const float P00 = P[0][0], P01 = P[0][1];
    const float P10 = P[1][0], P11 = P[1][1];

    const float dtP11 = dt * P11;
    const float dt2P11 = dt * dtP11;

    float Pp00 = P00 + dt * (P10 + P01) + dt2P11 + Q[0][0];
    float Pp01 = P01 + dtP11 + Q[0][1];
    float Pp10 = P10 + dtP11 + Q[1][0];
    float Pp11 = P11 + Q[1][1];

    // ============================================================
    // 2) UPDATE
    // H = [1 0]  (measure altitude only)
    // S = H*Pp*H' + R = Pp00 + R
    // K = Pp*H'/S = [Pp00; Pp10] / S
    // ============================================================

    const float S = Pp00 + R;

    // Safety: avoid divide-by-zero / weird negatives (shouldn't happen, but embedded life)
    if (S <= 1.0e-9f) {
        // If S is degenerate, just accept the prediction and keep covariance as-is
        x[0] = xp0;
        x[1] = xp1;
        P[0][0] = Pp00; P[0][1] = Pp01;
        P[1][0] = Pp10; P[1][1] = Pp11;
        return;
    }

    const float invS = 1.0f / S;
    const float K0 = Pp00 * invS;
    const float K1 = Pp10 * invS;

    const float innovation = z - xp0;

    // Updated state
    x[0] = xp0 + K0 * innovation;
    x[1] = xp1 + K1 * innovation;

    // Updated covariance: P = Pp - K*(H*Pp)
    // H*Pp = [Pp00 Pp01]
    float newP00 = Pp00 - K0 * Pp00;
    float newP01 = Pp01 - K0 * Pp01;
    float newP10 = Pp10 - K1 * Pp00;
    float newP11 = Pp11 - K1 * Pp01;

    // Optional: enforce symmetry (helps with float drift)
    const float sym01 = 0.5f * (newP01 + newP10);
    newP01 = sym01;
    newP10 = sym01;

    P[0][0] = newP00; P[0][1] = newP01;
    P[1][0] = newP10; P[1][1] = newP11;
}
*/
