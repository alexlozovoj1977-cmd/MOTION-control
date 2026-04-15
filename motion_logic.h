#ifndef MOTION_LOGIC_H
#define MOTION_LOGIC_H

#include <stdint.h>

// Визначаємо кількість ступенів свободи (або каналів ЕЕГ)
#define N_DOF 16 

class MotionController {
public:
    MotionController();
    
    // Головний метод оновлення стану
    void update(const uint16_t* target_phi, const uint16_t* current_phi, int16_t* output_torque);
    
    // Динамічна зміна коефіцієнтів
    void setGains(int16_t k_gain, int16_t damping_b);

private:
    int16_t K_gain;
    int16_t B_damping;
    
    // LUT на 1024 елементи для економії пам'яті
    int16_t sinLUT[1024];

    void initSinLUT();
};

#endif // MOTION_LOGIC_H
