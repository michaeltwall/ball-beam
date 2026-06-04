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
constexpr double dt = 0.1;
constexpr uint32_t CONTROL_PERIOD_US = (uint32_t)(dt * 1e6);

uint32_t last_control_us = 0;

// servo stuff
double servo_center = 93;
double servo_range  = 25.0;

// PID gains
double kp = 0.09;
double ki =  0.05;     // integral has been weird :(
double kd = 0.12;

// Constant controller reference
double setpoint = 120.0;

double integral = 0.0; // integral
double prev_position = 130.0;

float cv_pos = 0.0;
bool send_filtered = false;
bool control_use_cv = true;

// Output of PID
double output = 0.0;

// Sensor state data
int raw_pos = 0;
double filtered_pos = 0.0;

// Kalman filter stuff
float kfq = 0.38;
SimpleKalmanFilter kf(2, 2, kfq);

// Serial parser for PID tuning through serial
String input_buffer = "";
unsigned long last_receive_time = 0;
const unsigned long TIMEOUT_MS = 500;
bool cv_receiving = false;
void parseSerial() {
    while (Serial.available() > 0) {
        char c = Serial.read();
        
        if (c == '\n') {
        processMessage(input_buffer);
        input_buffer = "";
        } else {
        input_buffer += c;
        }
    }
}

void processMessage(String msg) {
    msg.trim(); // strip \r and whitespace
    cv_pos = msg.toFloat();
    last_receive_time = millis();

    int pipe_index = msg.indexOf('|');
  
    if (pipe_index != -1) {
        float cv_pos = msg.substring(0, pipe_index).toFloat();
        String command = msg.substring(pipe_index + 1);

        if (command.length() > 0) {
            handleCommand(command);
        }
    } else {
    // no delimiter, plain value
    float cv_pos = msg.toFloat();
    }
}

void handleCommand(String cmd) {
    if (cmd == "switch_send") {
        send_filtered = !send_filtered;
    } else if (cmd == "switch_control") {
        control_use_cv = !control_use_cv;
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
    static int decimate = 0;
    if (++decimate >= 1)
    {
        decimate = 0;
        // SerialPort.print(filtered_pos);
        // SerialPort.print(',');
        SerialPort.println(send_filtered ? filtered_pos : raw_pos);
    }
}

// Controller function
void runController() {
    // error calculation
    double p = (cv_receiving && control_use_cv) ? cv_pos : filtered_pos;
    
    double e = p - setpoint;

    // derivative (velocity) calc
    double v = (p - prev_position) / dt;

    prev_position = p;

    // integral calc
    double i = integral + (e * dt);

    // unsaturated controller output
    double unsat = (kp * e) + (ki * i) + (kd * v);

    // saturated output
    output = constrain(unsat,-servo_range,servo_range);

    // Anti-windup
    if (ki != 0){
        if ((fabs(unsat) < servo_range) || ((unsat > servo_range)  && (e < 0)) ||
            ((unsat < -servo_range) && (e > 0)))
        {
            integral = i;
        }
        // integral clamp
        float limit = abs(15/ki);
        integral = constrain(integral, -limit, limit);
    }

    // Debug telemetry
        // SerialPort.print(setpoint);
        // SerialPort.print(",");
        // SerialPort.print(raw_pos);
        // SerialPort.print(",");
        // SerialPort.print(filtered_pos);
        // SerialPort.print(",");
        // SerialPort.print((kp * e));
        // SerialPort.print(",");
        // SerialPort.print((ki * i));
        // SerialPort.print(",");
        // SerialPort.print((kd * v));
        // SerialPort.print(",");
        // SerialPort.println(output);
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
    sensor.VL53L4CD_SetRangeTiming(CONTROL_PERIOD_US*1000, 0);

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
    cv_receiving = (millis() - last_receive_time) < TIMEOUT_MS;

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