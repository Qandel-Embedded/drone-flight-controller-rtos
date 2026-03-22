/*
 * FreeRTOS Main Application for Drone Flight Controller
 */
#include "FreeRTOS.h"
#include "task.h"
#include "ahrs_madgwick.h"
#include "pid_cascade.h"
#include "sbus_parser.h"

void vTaskFlightControl(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1); // 1000Hz loop

    while(1) {
        // 1. Read IMU
        float gx = read_gyro_x(), gy = read_gyro_y(), gz = read_gyro_z();
        float ax = read_acc_x(),  ay = read_acc_y(),  az = read_acc_z();
        
        // 2. Update AHRS
        MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
        float roll, pitch, yaw;
        get_euler_angles(&roll, &pitch, &yaw);

        // 3. Get Receiver Inputs
        sbus_data_t rx = get_latest_sbus();

        // 4. Compute PID & Mixer
        float motors[4];
        compute_attitude_control(rx.roll_sp, rx.pitch_sp, rx.yaw_sp, 
                                 roll, pitch, gx, gy, gz, 0.001f, motors);

        // 5. Output to ESCs (DShot600/PWM)
        set_motor_pwms(motors);

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

int main(void) {
    system_clock_init();
    imu_spi_init();
    motor_pwm_init();
    sbus_uart_init();

    xTaskCreate(vTaskFlightControl, "FlightCtrl", 1024, NULL, configMAX_PRIORITIES-1, NULL);
    vTaskStartScheduler();

    while(1);
    return 0;
}
