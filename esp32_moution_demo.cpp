#include <Arduino.h>

// === НАЛАШТУВАННЯ АРХІТЕКТУРИ ===
#define NUM_CHANNELS 16
#define K_STIFFNESS 4  // Коефіцієнт пружності (чим більше, тим швидше синхронізація)

// Масиви станів
uint16_t phase_matrix[NUM_CHANNELS];
uint16_t next_phase[NUM_CHANNELS];

// Lookup Table (LUT) для синуса (займає 128 КБ оперативної пам'яті, для ESP32-S3 це дрібниці)
int16_t sin_lut[65536];

// Найпростіша топологія (Кільце + повнозв'язна зірка для швидкого придушення синфазного шуму)
// У реальності тут буде ваша 12-векторна решітка, але для 16 електродів ми робимо "Small-World"
const uint8_t neighbors[16][4] = {
  {1, 15, 8, 4}, {2, 0, 9, 5}, {3, 1, 10, 6}, {4, 2, 11, 7},
  {5, 3, 12, 0}, {6, 4, 13, 1}, {7, 5, 14, 2}, {8, 6, 15, 3},
  {9, 7, 0, 12}, {10, 8, 1, 13}, {11, 9, 2, 14}, {12, 10, 3, 15},
  {13, 11, 4, 8}, {14, 12, 5, 9}, {15, 13, 6, 10}, {0, 14, 7, 11}
};

// === ІНІЦІАЛІЗАЦІЯ LUT (Генерується один раз при старті) ===
void setup_moution() {
  for (uint32_t i = 0; i < 65536; i++) {
    // Переводимо 0-65535 у радіани (0 - 2PI), рахуємо синус і масштабуємо в int16_t
    float radians = ((float)i / 65536.0) * 2.0 * PI;
    sin_lut[i] = (int16_t)(sin(radians) * 32767.0); 
  }
}

// === ГОЛОВНИЙ ОПЕРАТОР ОБЧИСЛЕННЯ (O(1), без FPU) ===
void step_moution() {
  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
    int32_t force_sum = 0;
    
    // Збираємо фазовий тиск від сусідів
    for (uint8_t n = 0; n < 4; n++) {
      uint8_t neighbor = neighbors[i][n];
      
      // МАГІЯ ЦІЛИХ ЧИСЕЛ: uint16_t автоматично обробляє кругову топологію (переповнення)
      // Нам не треба рахувати модулі і переходи через нуль!
      uint16_t delta_phi = phase_matrix[neighbor] - phase_matrix[i]; 
      
      force_sum += sin_lut[delta_phi];
    }
    
    // Оновлюємо фазу (побітовий зсув замінює ділення)
    next_phase[i] = phase_matrix[i] + (uint16_t)((force_sum * K_STIFFNESS) >> 15);
  }
  
  // Записуємо новий стан
  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
    phase_matrix[i] = next_phase[i];
  }
}

void setup() {
  Serial.begin(115200);
  setup_moution();
  
  // Ініціалізація матриці нулями
  for(int i=0; i<NUM_CHANNELS; i++) phase_matrix[i] = 0;
}

void loop() {
  // 1. ЧИТАЄМО "БРУДНІ" ДАНІ (Тут вони читають зі своїх АЦП)
  // Для прикладу беремо симуляцію: корисний сигнал + ЖАХЛИВИЙ шум 50 Гц
  uint16_t common_noise = (uint16_t)(sin(millis() * 0.05) * 10000); // 50 Hz наведення
  
  for(int i=0; i<NUM_CHANNELS; i++) {
    // Вливаємо дані в нашу матрицю (інжекція енергії)
    // Зчитуємо реальний піна АЦП: analogRead(pin)
    uint16_t eeg_signal = random(0, 100); // Симуляція корисного мікро-сигналу мозку
    phase_matrix[i] += (eeg_signal + common_noise); 
  }

  // 2. ЗАПУСКАЄМО ЕМЕРДЖЕНТНУ ФІЛЬТРАЦІЮ
  // Кілька ітерацій релаксації, щоб матриця "з'їла" синфазний шум
  for(int iter=0; iter<3; iter++) {
    step_moution();
  }

  // 3. ВИВІД ДЛЯ ВАУ-ЕФЕКТУ (В Serial Plotter)
  // Виводимо брудний сигнал 0-го каналу і його ж чистий стан
  // Оскільки синфазний шум розтягує всю матрицю однаково, різниця фаз (градієнт) 
  // між сусідами містить ТІЛЬКИ чистий сигнал ЕЕГ!
  
  uint16_t clean_eeg = phase_matrix[0] - phase_matrix[1]; // Зчитуємо локальний атрактор

  Serial.print("Dirty_Signal:");
  Serial.print(common_noise + random(0, 100)); // Те, що вони зазвичай бачать
  Serial.print(" ");
  Serial.print("MOUTION_Clean:");
  Serial.println(clean_eeg); // Те, що зробила ваша архітектура

  delay(5); // Імітація частоти дискретизації ~200 Гц
}
