#include "KalmanFilter.h"

KalmanFilter::KalmanFilter()
{
    // Initial covariance
    mat3_zero(P);
}


void KalmanFilter::predict(uint32_t imu_dt_us) {

    /*
    sigma_a = 1e-1;  % IMU Process Noise
    sigma_b = 1.0;   % Accel-Bias Driving Process Noise

    Qk =   [sigma_a^2*deltat^4/4 sigma_a^2*deltat^3/2 0;
        sigma_a^2*deltat^3/2 sigma_a^2*deltat^2   0;
        0                    0                    sigma_b^2*deltat];
    */

    const float sigma_a = 1.0e1*0.2f;
    const float sigma_b = 1.0e-5*0.05f;

    float dt = imu_dt_us * 1.0e-6f;

    if (dt <= 0.0f || dt > 0.2)  // ideally, dt = 0.01 for 100 Hz IMU rate
    {
        Serial.print("Clamped. dt = ");
        Serial.println(dt);
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


void KalmanFilter::update(float z, INS_State& ins) {
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
    const float sigma_baro = 2.0e1*0.3;
    const float R = sigma_baro * sigma_baro;

    // ------------------------------------------------------------
    // extract pseudostate
    // px_in = [p v b]'
    // ------------------------------------------------------------
    float px_in[3];
    px_in[0] = ins.p3_n_m;
    px_in[1] = ins.v3_n_mps;
    px_in[2] = ins.b3_a_mps2;

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
    ins.p3_n_m += dpx_out[0];
    ins.v3_n_mps += dpx_out[1];
    ins.b3_a_mps2 += dpx_out[2];
    // Serial.print(dpx_out[0],5);
    // Serial.print("  ");
    // Serial.print(dpx_out[1],5);
    // Serial.print("  ");
    // Serial.println(dpx_out[2],5);

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

void KalmanFilter::printP() {
    Serial.print("P = [");
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            Serial.print(P[i][j], 6);
            if (j < 2) Serial.print(", ");
        }
        if (i < 2) Serial.print("; ");
    }
    Serial.println("];");
}

void KalmanFilter::reset() {
    Serial.print("\nRESET\n");
    mat3_zero(P);
}