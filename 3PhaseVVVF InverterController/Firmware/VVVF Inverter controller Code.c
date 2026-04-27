#include "main.h"
#include <math.h>

/* ---------------- CONFIG ---------------- */
#define TABLE_SIZE        256
#define PWM_FREQUENCY     20000.0f
#define MAX_FREQ          50.0f
#define MIN_FREQ          1.0f
#define RAMP_RATE         0.02f

/* ---------------- GLOBALS ---------------- */
extern TIM_HandleTypeDef htim1;
extern ADC_HandleTypeDef hadc1;

float sineTable[TABLE_SIZE];

volatile float phase = 0.0f;
volatile float currentFreq = 2.0f;
volatile float targetFreq = 2.0f;
float modulation = 0.1f;

uint16_t pwm_period;

/* Control flags */
uint8_t inverterEnabled = 0;
uint8_t faultDetected = 0;

/* ---------------- FUNCTION PROTOTYPES ---------------- */
void generateSineTable(void);
void updateVFControl(void);
void readSpeedInput(void);
void inverterStart(void);
void inverterStop(void);

/* ---------------- SINE TABLE ---------------- */
void generateSineTable() {
    for (int i = 0; i < TABLE_SIZE; i++) {
        sineTable[i] = (sinf(2 * M_PI * i / TABLE_SIZE) + 1.0f) / 2.0f;
    }
}

/* ---------------- ADC SPEED CONTROL ---------------- */
void readSpeedInput() {
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);

    uint16_t adcVal = HAL_ADC_GetValue(&hadc1);

    // Map ADC (0–4095) → frequency
    targetFreq = MIN_FREQ + ((float)adcVal / 4095.0f) * MAX_FREQ;
}

/* ---------------- V/F CONTROL ---------------- */
void updateVFControl() {
    // Smooth ramp
    if (currentFreq < targetFreq)
        currentFreq += RAMP_RATE;
    else if (currentFreq > targetFreq)
        currentFreq -= RAMP_RATE;

    // V/f ratio
    modulation = currentFreq / MAX_FREQ;

    if (modulation > 1.0f) modulation = 1.0f;
    if (modulation < 0.05f) modulation = 0.05f;
}

/* ---------------- INVERTER CONTROL ---------------- */
void inverterStart() {
    inverterEnabled = 1;

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

    HAL_TIM_Base_Start_IT(&htim1);
}

void inverterStop() {
    inverterEnabled = 0;

    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);

    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
}

/* ---------------- PWM UPDATE ISR ---------------- */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

    if (htim->Instance == TIM1 && inverterEnabled && !faultDetected) {

        // Update phase
        phase += currentFreq * TABLE_SIZE / PWM_FREQUENCY;
        if (phase >= TABLE_SIZE) phase -= TABLE_SIZE;

        int indexU = (int)phase;
        int indexV = (indexU + 85) % TABLE_SIZE;
        int indexW = (indexU + 170) % TABLE_SIZE;

        uint16_t dutyU = sineTable[indexU] * modulation * pwm_period;
        uint16_t dutyV = sineTable[indexV] * modulation * pwm_period;
        uint16_t dutyW = sineTable[indexW] * modulation * pwm_period;

        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, dutyU);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, dutyV);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, dutyW);
    }
}

/* ---------------- FAULT HANDLING ---------------- */
void checkFaultInput() {
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_SET) {
        faultDetected = 1;
        inverterStop();
    }
}

/* ---------------- MAIN ---------------- */
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_TIM1_Init();
    MX_ADC1_Init();

    generateSineTable();

    pwm_period = __HAL_TIM_GET_AUTORELOAD(&htim1);

    inverterStart();

    while (1)
    {
        readSpeedInput();     // Potentiometer
        updateVFControl();    // V/f logic
        checkFaultInput();    // Fault pin

        HAL_Delay(1);
    }
}