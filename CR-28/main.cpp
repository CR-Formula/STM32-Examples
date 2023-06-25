#include "mbed.h"

typedef struct data_struct {
  float RPM;        // RPM
  float TPS;        // TPS
  float FOT;        // Fuel Open Time
  float IA;         // Ignition Angle
  float Lam;        // Lambda
  float AirT;       // Air Temp
  float CoolT;      // Coolant Temp
  float Lat;        // Latitude
  float Lng;        // Longitude
  float Speed;      // GPS Speed
  float OilP;       // Oil Pressure
  float FuelP;      // Fuel Pressure
  float FLTemp;     // Front Left Brake Temp
  float FRTemp;     // Front Right Brake Temp
  float RLTemp;     // Rear Left Brake Temp
  float RRTemp;     // Rear Right Brake Temp
  float FRPot;      // Front Right Suspension Damper
  float FLPot;      // Front Left Suspension Damper
  float RRPot;      // Rear Right Suspension Damper
  float RLPot;      // Rear Left Suspension Damper
  float BrakeFront; // Front Brake Pressure
  float BrakeRear;  // Rear Brake Pressure
  float BrakeBias;  // Brake Bias
  float AccX;       // Accelerometer X Axis
  float AccY;       // Accelerometer Y Axis
  float AccZ;       // Accelerometer Z Axis
  float GyrX;       // Gyroscope X Axis
  float GyrY;       // Gyroscope Y Axis
  float GyrZ;       // Gyroscope Z Axis
  float MagX;       // Magnetometer X Axis
  float MagY;       // Magnetometer Y Axis
  float MagZ;       // Magnetometer Z Axis
} data_struct;
data_struct telemetry;

int main()
{
    DigitalOut led(LED1); // TX activity LED
    BufferedSerial serial(PA_15, PF_6, 115200); // Serial Port 7
    BufferedSerial debug(USBTX, USBRX, 115200); // Debug Port
    char val[32] = "Test\n";

    while (true) {
        serial.write(&telemetry.RPM, sizeof(telemetry.RPM));
        // debug.write(val, sizeof(val));
        led = !led;
        ThisThread::sleep_for(500); // delay function
    }
}

