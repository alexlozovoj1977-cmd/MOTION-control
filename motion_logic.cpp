#include "motion_logic.h"
#include <math.h>

MotionController::MotionController() {
    initSinLUT();
    K_gain = 1280;   // ~5.0 (у форматі Q8)
    B_damping = 25;  // Коефіцієнт демпфування
}

void MotionController::initSinLUT() {
    // Ініціалізація синусоїди. 
    // 16384.0 - це амплітуда для майбутнього зсуву на 14 біт (>> 14)
    for (int i = 0; i < 1024; i++) {
        sinLUT[i] = (int16_t)(16384.0 * sin(2.0 * M_PI * i / 1024.0));
    }
}

// --- helper: circular shortest difference ---
// Використовуємо властивості доповнювального коду для блискавичного переходу через нуль
inline int16_t circularDiff(uint16_t a, uint16_t b) {
    int16_t diff = (int16_t)(a - b);
    return diff;
}

void MotionController::update(
    const uint16_t* target_phi,
    const uint16_t* current_phi,
    int16_t* output_torque
) {
    for (int i = 0; i < N_DOF; i++) {

        // 1. Shortest circular phase error (Геодезика на колі)
        int16_t d_phi = circularDiff(target_phi[i], current_phi[i]);

        // 2. Map to LUT index (10-bit)
        // Приведення до uint16_t і зсув на 6 біт перетворює діапазон 65536 у 1024
        uint16_t idx = ((uint16_t)d_phi) >> 6;  
        idx &= 0x03FF; // Safety mask (захист від виходу за межі пам'яті)

        int32_t sin_val = sinLUT[idx];

        // 3. Energy shaping (Обчислення градієнта потенціалу)
        int32_t u_effort = (sin_val * K_gain) >> 14;

        // 4. Optional damping (finite difference)
        // Закоментовано, але готово до використання у робототехніці
        // int16_t vel = current_phi[i] - prev_phi[i];
        // u_effort -= (vel * B_damping) >> 8;

        // 5. Saturation (Критичний захист апаратної частини від перевантаження)
        if (u_effort > 32767) u_effort = 32767;
        if (u_effort < -32768) u_effort = -32768;

        output_torque[i] = (int16_t)u_effort;
    }
}

void MotionController::setGains(int16_t k_gain, int16_t damping_b) {
    K_gain = k_gain;
    B_damping = damping_b;
}
