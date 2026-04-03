#include "motion_logic.h"
#include <math.h>

MotionController::MotionController() {
    initSinLUT();
    K_gain = 1280;  // Дефолтний Gain (приблизно 5.0 у Q8)
    B_damping = 25; // Дефолтне демпфування
}

void MotionController::initSinLUT() {
    for (int i = 0; i < 1024; i++) {
        // Заповнюємо таблицю синуса в діапазоні int16
        // 16384 - це амплітуда (Q14), щоб уникнути переповнення при множенні
        sinLUT[i] = (int16_t)(16384.0 * sin(i * 2.0 * M_PI / 1024.0));
    }
}

void MotionController::update(const uint16_t* target_phi, const uint16_t* current_phi, int16_t* output_torque) {
    for (int i = 0; i < N_DOF; i++) {
        // 1. Обчислюємо фазову помилку
        // uint16_t автоматично виконує операцію за модулем 2^16 (S1 manifold)
        uint16_t d_phi = target_phi[i] - current_phi[i];

        // 2. Отримуємо значення синуса з LUT (зсув на 6 біт для переходу 16->10 біт)
        int32_t sin_val = sinLUT[d_phi >> 6];

        // 3. Розрахунок зусилля (Energy Shaping)
        // u = -K * sin(d_phi)
        int32_t u_effort = (sin_val * K_gain) >> 14; 

        // 4. Додавання демпфування (опціонально, якщо є оцінка швидкості)
        // Тут може бути різниця між поточним і попереднім кроком
        
        output_torque[i] = (int16_t)u_effort;
    }
}

void MotionController::setGains(int16_t k_gain, int16_t damping_b) {
    K_gain = k_gain;
    B_damping = damping_b;
}