#ifndef MOTION_LOGIC_H
#define MOTION_LOGIC_H

#include <stdint.h>

// Кількість ступенів свободи (моторів/каналів)
#define N_DOF 4 

class MotionController {
public:
    MotionController();

    /**
     * @brief Основний цикл оновлення (викликати в ISR або Task)
     * @param target_phi Масив цільових фаз (0..65535)
     * @param current_phi Масив поточних фаз від сенсорів
     * @param output_torque Масив для запису розрахованих моментів
     */
    void update(const uint16_t* target_phi, const uint16_t* current_phi, int16_t* output_torque);

    // Налаштування Gain (Q8 формат)
    void setGains(int16_t k_gain, int16_t damping_b);

private:
    int16_t sinLUT[1024];      // 10-bit Sine Look-Up Table
    int16_t coupling_matrix[N_DOF][N_DOF];
    int16_t K_gain;            // Коефіцієнт жорсткості
    int16_t B_damping;         // Коефіцієнт демпфування
    
    void initSinLUT();
};

#endif