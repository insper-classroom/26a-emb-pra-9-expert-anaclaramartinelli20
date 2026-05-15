#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#include "pico/stdlib.h"
#include <stdio.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/uart.h"
#include "mpu6050.h"

#include "Fusion.h"
#define SAMPLE_PERIOD (0.01f)
#define UART_ID uart0

const int MPU_ADDRESS = 0x68;
const int I2C_SDA_GPIO = 4;
const int I2C_SCL_GPIO = 5;
const int LED_PIN_R = 16;
const int LED_PIN_G = 15;
const int LED_PIN_B = 14;

#define TASK_PIN_MPU      18
#define TASK_PIN_FUSION   19
#define TASK_PIN_UART     20
#define TASK_PIN_PWM      21

#define ENABLE_STACK_MONITOR 0

#define CORE_0 (1 << 0)
#define CORE_1 (1 << 1)

#define PRIO_MPU      (tskIDLE_PRIORITY + 3)
#define PRIO_FUSION   (tskIDLE_PRIORITY + 2)
#define PRIO_UART     (tskIDLE_PRIORITY + 1)
#define PRIO_PWM      (tskIDLE_PRIORITY + 1)

#define STACK_MPU       256
#define STACK_FUSION    512
#define STACK_UART      192
#define STACK_PWM       160
#define STACK_MONITOR   1024

#define CLICK_DEBOUNCE_MS  100

QueueHandle_t xQueueMPU;
QueueHandle_t xQueuePos;
QueueHandle_t xQueueColor;
SemaphoreHandle_t xSemaphoreBtn;

TaskHandle_t xHandleMpu;
TaskHandle_t xHandleFusion;
TaskHandle_t xHandleUart;
TaskHandle_t xHandlePwm;

typedef struct adc {
    int axis;
    int val;
} adc_t;

typedef struct data {
    FusionVector gyroscope;
    FusionVector accelerometer;
} data_t;

typedef struct rgb {
    int r;
    int g;
    int b;
} rgb_t;

static void uart_clear_rx_fifo(uart_inst_t *uart) {
    while (uart_is_readable(uart)) {
        (void)uart_getc(uart);
    }
}

static int angle_to_255(float angle_deg) {
    if (angle_deg > 180.0f) {
        angle_deg = 180.0f;
    } else if (angle_deg < -180.0f) {
        angle_deg = -180.0f;
    }
    return (int)(angle_deg * (255.0f / 180.0f));
}

static int tilt_to_255(float tilt_deg) {
    if (tilt_deg < 0.0f) {
        tilt_deg = 0.0f;
    } else if (tilt_deg > 90.0f) {
        tilt_deg = 90.0f;
    }
    return (int)(tilt_deg * (255.0f / 90.0f));
}

static void mpu6050_init() {
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO);
    gpio_pull_up(I2C_SCL_GPIO);

    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c_default, MPU_ADDRESS, buf, 2, false);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    uint8_t buffer[14];

    uint8_t val = 0x3B;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 14, false);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    *temp = buffer[6] << 8 | buffer[7];

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[8 + i * 2] << 8 | buffer[8 + (i * 2) + 1]);
    }
}

#if ENABLE_STACK_MONITOR
void stack_monitor_task(void *p) {
    static TaskStatus_t tasks[16];
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        UBaseType_t n = uxTaskGetSystemState(tasks, 16, NULL);
        printf("+------------------+-------+\n");
        printf("| %-16s | %5s |\n", "task", "free");
        printf("+------------------+-------+\n");
        for (UBaseType_t i = 0; i < n; i++) {
            printf("| %-16s | %5u |\n",
                   tasks[i].pcTaskName,
                   (unsigned)tasks[i].usStackHighWaterMark);
        }
        printf("+------------------+-------+\n");
        printf("| heap livre min   | %5u |\n",
               (unsigned)xPortGetMinimumEverFreeHeapSize());
        printf("+------------------+-------+\n\n");
    }
}
#endif

void mpu6050_task(void *p) {
    gpio_init(TASK_PIN_MPU);
    gpio_set_dir(TASK_PIN_MPU, GPIO_OUT);

    mpu6050_init();
    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);

    while (true) {
        gpio_put(TASK_PIN_MPU, 1);

        int16_t acceleration[3], gyro[3], temp;
        mpu6050_read_raw(acceleration, gyro, &temp);

        FusionVector gyroscope = {
            .axis.x = gyro[0] / 131.0f,
            .axis.y = gyro[1] / 131.0f,
            .axis.z = gyro[2] / 131.0f,
        };

        FusionVector accelerometer = {
            .axis.x = acceleration[0] / 16384.0f,
            .axis.y = acceleration[1] / 16384.0f,
            .axis.z = acceleration[2] / 16384.0f,
        };

        data_t sensor_data;
        sensor_data.gyroscope = gyroscope;
        sensor_data.accelerometer = accelerometer;

        xQueueSend(xQueueMPU, &sensor_data, 0);   

        gpio_put(TASK_PIN_MPU, 0);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void fusion_task(void *p) {
    gpio_init(TASK_PIN_FUSION);
    gpio_set_dir(TASK_PIN_FUSION, GPIO_OUT);

    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);
    data_t sensor_data;
    adc_t adc_x, adc_y;
    rgb_t rgb_data;
    adc_x.axis = 0;
    adc_y.axis = 1;
    int dead_zone = 10;
    float mouse_speed = 0.4;
    int send_counter = 0;

    TickType_t last_click_tick = 0;

    while (true) {
        if (xQueueReceive(xQueueMPU, &sensor_data, portMAX_DELAY)) {
            gpio_put(TASK_PIN_FUSION, 1);

            FusionVector gyroscope = sensor_data.gyroscope;
            FusionVector accelerometer = sensor_data.accelerometer;

            FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);

            const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

            // Debounce por timestamp em vez de vTaskDelay
            if (accelerometer.axis.y > .8f) {
                TickType_t now = xTaskGetTickCount();
                if ((now - last_click_tick) >= pdMS_TO_TICKS(CLICK_DEBOUNCE_MS)) {
                    xSemaphoreGive(xSemaphoreBtn);
                    last_click_tick = now;
                }
            }

            adc_x.val = -angle_to_255(euler.angle.yaw) * mouse_speed;
            adc_y.val = -angle_to_255(euler.angle.roll) * mouse_speed;

            rgb_data.r = tilt_to_255(euler.angle.pitch);
            rgb_data.g = tilt_to_255(-euler.angle.pitch);
            const float roll_tilt = euler.angle.roll < 0.0f ? -euler.angle.roll : euler.angle.roll;
            rgb_data.b = tilt_to_255(roll_tilt);
            xQueueOverwrite(xQueueColor, &rgb_data);

            send_counter = (send_counter + 1) % 5;
            if (send_counter == 0) {
                if (adc_x.val > dead_zone || adc_x.val < -dead_zone) {
                    xQueueSend(xQueuePos, &adc_x, 0);
                }
                if (adc_y.val > dead_zone || adc_y.val < -dead_zone) {
                    xQueueSend(xQueuePos, &adc_y, 0);
                }
            }

            gpio_put(TASK_PIN_FUSION, 0);
        }
    }
}

void uart_task(void *p) {
    gpio_init(TASK_PIN_UART);
    gpio_set_dir(TASK_PIN_UART, GPIO_OUT);

    adc_t adc_data;
    while (true) {
        if (xQueueReceive(xQueuePos, &adc_data, pdMS_TO_TICKS(100))) {
            gpio_put(TASK_PIN_UART, 1);
            uart_putc(UART_ID, adc_data.axis);
            uart_putc(UART_ID, adc_data.val);
            uart_putc(UART_ID, adc_data.val >> 8);
            uart_putc(UART_ID, -1);
            gpio_put(TASK_PIN_UART, 0);
        }

        if (xSemaphoreTake(xSemaphoreBtn, 0) == pdTRUE) {
            gpio_put(TASK_PIN_UART, 1);
            uart_putc(UART_ID, 2);
            uart_putc(UART_ID, 1);
            uart_putc(UART_ID, 0);
            uart_putc(UART_ID, -1);
            gpio_put(TASK_PIN_UART, 0);

            vTaskDelay(pdMS_TO_TICKS(50));

            gpio_put(TASK_PIN_UART, 1);
            uart_putc(UART_ID, 2);
            uart_putc(UART_ID, 0);
            uart_putc(UART_ID, 0);
            uart_putc(UART_ID, -1);
            gpio_put(TASK_PIN_UART, 0);
        }
    }
}

void pwm_task(void *p) {
    gpio_set_function(LED_PIN_R, GPIO_FUNC_PWM);
    const uint slice_num_r = pwm_gpio_to_slice_num(LED_PIN_R);
    const uint chan_r = pwm_gpio_to_channel(LED_PIN_R);

    gpio_set_function(LED_PIN_G, GPIO_FUNC_PWM);
    const uint slice_num_g = pwm_gpio_to_slice_num(LED_PIN_G);
    const uint chan_g = pwm_gpio_to_channel(LED_PIN_G);

    gpio_set_function(LED_PIN_B, GPIO_FUNC_PWM);
    const uint slice_num_b = pwm_gpio_to_slice_num(LED_PIN_B);
    const uint chan_b = pwm_gpio_to_channel(LED_PIN_B);

    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 4.0f);
    pwm_config_set_wrap(&config, 255);

    pwm_init(slice_num_r, &config, true);
    if (slice_num_g != slice_num_r) {
        pwm_init(slice_num_g, &config, true);
    }
    if (slice_num_b != slice_num_r && slice_num_b != slice_num_g) {
        pwm_init(slice_num_b, &config, true);
    }

    pwm_set_chan_level(slice_num_r, chan_r, 0);
    pwm_set_chan_level(slice_num_g, chan_g, 0);
    pwm_set_chan_level(slice_num_b, chan_b, 0);

    gpio_init(TASK_PIN_PWM);
    gpio_set_dir(TASK_PIN_PWM, GPIO_OUT);

    rgb_t rgb_data = {0};

    while (true) {
        if (xQueueReceive(xQueueColor, &rgb_data, portMAX_DELAY)) {
            gpio_put(TASK_PIN_PWM, 1);
            pwm_set_chan_level(slice_num_r, chan_r, rgb_data.r);
            pwm_set_chan_level(slice_num_g, chan_g, rgb_data.g);
            pwm_set_chan_level(slice_num_b, chan_b, rgb_data.b);
            gpio_put(TASK_PIN_PWM, 0);
        }
    }
}

int main() {
    stdio_init_all();
    uart_clear_rx_fifo(UART_ID);
    gpio_init(LED_PIN_R);

    xQueueMPU = xQueueCreate(64, sizeof(data_t));
    xQueuePos = xQueueCreate(64, sizeof(adc_t));
    xQueueColor = xQueueCreate(1, sizeof(rgb_t));
    xSemaphoreBtn = xSemaphoreCreateBinary();

    xTaskCreate(mpu6050_task, "mpu",    STACK_MPU,    NULL, PRIO_MPU,    &xHandleMpu);
    xTaskCreate(fusion_task,  "fusion", STACK_FUSION, NULL, PRIO_FUSION, &xHandleFusion);
    xTaskCreate(uart_task,    "uart",   STACK_UART,   NULL, PRIO_UART,   &xHandleUart);
    xTaskCreate(pwm_task,     "pwm",    STACK_PWM,    NULL, PRIO_PWM,    &xHandlePwm);

#if ENABLE_STACK_MONITOR
    xTaskCreate(stack_monitor_task, "monitor", STACK_MONITOR, NULL, tskIDLE_PRIORITY + 1, NULL);
#endif

    // Core 0: mpu  + uart
    // Core 1: fusion + pwm
    vTaskCoreAffinitySet(xHandleMpu,    CORE_0);
    vTaskCoreAffinitySet(xHandleUart,   CORE_0);
    vTaskCoreAffinitySet(xHandleFusion, CORE_1);
    vTaskCoreAffinitySet(xHandlePwm,    CORE_1);

    vTaskStartScheduler();

    while (true)
        ;
}