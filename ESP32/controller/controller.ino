#include <ESP32Servo.h>
#include <Arduino.h>
#include <Wire.h>
#include <vl53l4cd_class.h>
#include <SimpleKalmanFilter.h>
#include <math.h>

#define DEV_I2C Wire
#define SerialPort Serial

VL53L4CD sensor(&DEV_I2C, A1);
Servo servo;


// 20Hz control loop
constexpr double dt = 0.05;
constexpr uint32_t CONTROL_PERIOD_US = (uint32_t)(dt * 1e6);

uint32_t last_control_us = 0;

// servo stuff
double servo_center = 95;
double servo_range  = 32.0;

// PID gains
double kp = 0.15;
double ki =  0.0;     // integral has been weird :(
double kd = 0.17;

// Constant controller reference
double setpoint = 130.0;

double integral = 0.0; // integral
double prev_position = 130.0;

// Output of PID
double output = 0.0;

// Sensor state data
int raw_pos = 0;
double filtered_pos = 0.0;

// Kalman filter stuff
float kfq = 0.35;
SimpleKalmanFilter kf(2, 2, kfq);

// Serial parser for PID tuning through serial
#define BUF_SIZE 64
char serialBuf[BUF_SIZE];
uint8_t bufIdx = 0;
void parseSerial() {
    while (SerialPort.available())
    {
        char c = SerialPort.read();

        if (c == '\n' || c == '\r')\
        {
            if (bufIdx == 0) return;

            serialBuf[bufIdx] = '\0';
            bufIdx = 0;

            double val;
            char key[16];

            if (strncmp(serialBuf, "print", 5) == 0)
            {
                SerialPort.printf(
                    "kp=%.4f ki=%.4f kd=%.4f sp=%.2f range=%.2f\n",
                    kp, ki, kd, setpoint, servo_range
                );

                return;
            }
            if (sscanf(serialBuf, "%15[^= ] = %lf", key, &val) == 2 ||
                sscanf(serialBuf, "%15[^=]=%lf", key, &val) == 2)
            {
                if (strcmp(key, "kp") == 0)
                {
                    kp = val;
                    SerialPort.printf("kp = %.4f\n", kp);
                }
                else if (strcmp(key, "ki") == 0)
                {
                    ki = val;
                    SerialPort.printf("ki = %.4f\n", ki);
                }
                else if (strcmp(key, "kd") == 0)
                {
                    kd = val;
                    SerialPort.printf("kd = %.4f\n", kd);
                }
                else if (strcmp(key, "range") == 0)
                {
                    servo_range = val;
                    SerialPort.printf("range = %.4f\n", servo_range);
                }
                else if (strcmp(key, "kfq") == 0)
                {
                    kfq = val;
                    SimpleKalmanFilter kf(2, 2, kfq);
                    SerialPort.printf("kfq = %.4f\n", kfq);
                }
                else if (strcmp(key, "center") == 0)
                {
                    servo_center = val;
                    SerialPort.printf("servo_center = %.4f\n", servo_center);
                }
                else if (strcmp(key, "sp") == 0)
                {
                    setpoint = val;
                    integral = 0.0;

                    SerialPort.printf(
                        "sp = %.2f (integral reset)\n",
                        setpoint
                    );
                }
                else {
                    SerialPort.printf("Unknown key: %s\n", key);
                }
            }
        }
        else {
            if (bufIdx < BUF_SIZE - 1)
            {
                serialBuf[bufIdx++] = c;
            }
        }
    }
}

// Sensor update
void updateSensor() {
    uint8_t ready = 0;

    sensor.VL53L4CD_CheckForDataReady(&ready);

    if (!ready) return;

    VL53L4CD_Result_t results;

    sensor.VL53L4CD_ClearInterrupt();
    sensor.VL53L4CD_GetResult(&results);

    if (results.range_status != 0) return;

    raw_pos = results.distance_mm;

    filtered_pos = kf.updateEstimate(raw_pos);
}

// Controller function
void runController() {
    // error calculation
    double e = filtered_pos - setpoint;

    // derivative (velocity) calc
    double v = (filtered_pos - prev_position) / dt;

    prev_position = filtered_pos;

    // integral calc
    double i = integral + (e * dt);

    // unsaturated controller output
    double unsat = (kp * e) + (ki * i) + (kd * v);

    // saturated output
    output = constrain(unsat,-servo_range,servo_range);

    // Anti-windup
    bool helps_unsaturate =
        ((unsat > servo_range)  && (e < 0)) ||
        ((unsat < -servo_range) && (e > 0));
    if ((fabs(unsat) < servo_range) || helps_unsaturate)
    {
        integral = i;
    }

    // integral clamp
    integral = constrain(integral, -500, 500);

    // Debug telemetry
    static int decimate = 0;
    if (++decimate >= 5)
    {
        decimate = 0;
        SerialPort.print(setpoint);
        SerialPort.print(",");
        // SerialPort.print(raw_pos);
        // SerialPort.print(",");
        SerialPort.print(filtered_pos);
        SerialPort.print(",");
        SerialPort.print((kp * e));
        SerialPort.print(",");
        SerialPort.print((ki * i));
        SerialPort.print(",");
        SerialPort.print((kd * v));
        SerialPort.print(",");
        SerialPort.println(output);
    }
}

// Setup
void setup()
{
    SerialPort.begin(115200);

    SerialPort.println("Ball-Beam Controller Ready");

    DEV_I2C.begin();

    sensor.begin();

    sensor.VL53L4CD_Off();
    sensor.InitSensor();

    // 50 ms timing budget
    sensor.VL53L4CD_SetRangeTiming(50, 0);

    sensor.VL53L4CD_StartRanging();

    servo.attach(9);

    servo.write(servo_center);

    filtered_pos = setpoint;
    prev_position = setpoint;

    last_control_us = micros();

    delay(1000);
}

// Main Loop
void loop()
{
    parseSerial();

    // sensor update
    updateSensor();

    // controller timing
    uint32_t now = micros();
    if (now - last_control_us >= CONTROL_PERIOD_US)
    {
        last_control_us += CONTROL_PERIOD_US;

        runController();
        servo.write(servo_center + output);
    }
}