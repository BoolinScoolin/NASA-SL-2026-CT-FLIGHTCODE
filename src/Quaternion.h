#pragma once
#include <cmath>

struct Quaternion {
    float q0, q1, q2, q3;
};

static inline Quaternion quat_identity() {
    return {1.0f, 0.0f, 0.0f, 0.0f};
}

static inline Quaternion quat_conj(Quaternion q) {
    return {q.q0, -q.q1, -q.q2, -q.q3};
}

static inline float quat_norm2(Quaternion q) {
    return q.q0*q.q0 + q.q1*q.q1 + q.q2*q.q2 + q.q3*q.q3;
}

static inline Quaternion quat_normalize(Quaternion q) {
    float n2 = quat_norm2(q);
    if (n2 <= 0.0f) return quat_identity();
    float inv = 1.0f / std::sqrt(n2);
    return {q.q0*inv, q.q1*inv, q.q2*inv, q.q3*inv};
}

static inline Quaternion quat_mult(Quaternion a, Quaternion b) {
    return {
        a.q0*b.q0 - a.q1*b.q1 - a.q2*b.q2 - a.q3*b.q3,
        a.q0*b.q1 + a.q1*b.q0 + a.q2*b.q3 - a.q3*b.q2,
        a.q0*b.q2 - a.q1*b.q3 + a.q2*b.q0 + a.q3*b.q1,
        a.q0*b.q3 + a.q1*b.q2 - a.q2*b.q1 + a.q3*b.q0
    };
}

static inline void quat_transform(const Quaternion& q,
                                  float v1, float& w1,
                                  float v2, float& w2,
                                  float v3, float& w3) 
{
    Quaternion v_quat = {0.0f, v1, v2, v3};
    Quaternion qstar = quat_conj(q);
    Quaternion w_quat = quat_mult(q,v_quat);
    w_quat = quat_mult(w_quat,qstar);
    w1 = w_quat.q1;
    w2 = w_quat.q2;
    w3 = w_quat.q3;
}

static inline Quaternion eul2quat(float phi_rad, float theta_rad, float psi_rad) {

    float cphio2 = std::cos(phi_rad * 0.5f);
    float sphio2 = std::sin(phi_rad * 0.5f);
    float cthetao2 = std::cos(theta_rad * 0.5f);
    float sthetao2 = std::sin(theta_rad * 0.5f);
    float cpsio2 = std::cos(psi_rad * 0.5f);
    float spsio2 = std::sin(psi_rad * 0.5f);

    Quaternion q;

    q.q0 = cpsio2 * cthetao2 * cphio2 +
           spsio2 * sthetao2 * sphio2;
    q.q1 = cpsio2 * cthetao2 * sphio2 -
           spsio2 * sthetao2 * cphio2;
    q.q2 = cpsio2 * sthetao2 * cphio2 +
           spsio2 * cthetao2 * sphio2;
    q.q3 = spsio2 * cthetao2 * cphio2 -
           cpsio2 * sthetao2 * sphio2;

    return q;
}

static inline void quat2eul(const Quaternion& q,
                            float& phi_rad,
                            float& theta_rad,
                            float& psi_rad) 
{

    float q0 = q.q0;
    float q1 = q.q1;
    float q2 = q.q2;
    float q3 = q.q3;

    phi_rad = std::atan2(2.0f*(q2*q3 + q0*q1),(q0*q0 - q1*q1 - q2*q2 + q3*q3));
    theta_rad = std::asin(-2.0f*(q1*q3 - q0*q2));
    psi_rad = std::atan2(2.0f*(q1*q2 + q0*q3),(q0*q0 + q1*q1 - q2*q2 - q3*q3));
    
}