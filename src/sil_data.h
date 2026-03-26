#pragma once
#include <cstdint>

// Put this in the header file!
constexpr uint32_t SIL_N = 850;

extern const float sil_time_s[SIL_N];
extern const float sil_accel_x_mps2[SIL_N];
extern const float sil_accel_y_mps2[SIL_N];
extern const float sil_accel_z_mps2[SIL_N];
extern const float sil_gyro_x_rps[SIL_N];
extern const float sil_gyro_y_rps[SIL_N];
extern const float sil_gyro_z_rps[SIL_N];
extern const float sil_alt_z_m[SIL_N];
extern const float sil_euler_1_rad[SIL_N];
extern const float sil_euler_2_rad[SIL_N];
extern const float sil_euler_3_rad[SIL_N];
extern const float sil_true_z_m[SIL_N];
