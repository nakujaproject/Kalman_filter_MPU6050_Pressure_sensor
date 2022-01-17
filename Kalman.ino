#include <BasicLinearAlgebra.h>

using namespace BLA;

Adafruit_BMP085 bmp;
Adafruit_MPU6050 mpu;

float altitude, velocity, acceleration, ax, ay, az, kalmanAltitude;
float liftoffAltitude, apogeeAltitude;
float s, v, a;

float q = 0.0001;

float T = 0.1;

// The system dynamics
BLA::Matrix<3, 3> A = {1.0, 0.1, 0.005,
                       0, 1.0, 0.1,
                       0, 0, 1.0};

// Relationship between measurement and states
BLA::Matrix<2, 3> H = {1.0, 0, 0,
                       0, 0, 1.0};

// Initial posteriori estimate error covariance
BLA::Matrix<3, 3> P = {1, 0, 0,
                       0, 1, 0,
                       0, 0, 1};

// Measurement error covariance
BLA::Matrix<2, 2> R = {0.25, 0,
                       0, 0.75};

// Process noise covariance
BLA::Matrix<3, 3> Q = {q, q, q,
                       q, q, q,
                       q, q, q};

// Identity Matrix
BLA::Matrix<3, 3> I = {1, 0, 0,
                       0, 1, 0,
                       0, 0, 1};

BLA::Matrix<3, 1> x_hat = {1500.0,
                           0.0,
                           0.0};

BLA::Matrix<2, 1> Y = {0.0,
                       0.0};

//function to initialize sensors and the sd card module
void init_components()
{

    Serial.println("BMP180 test!");
    if (!bmp.begin())
    {
        Serial.println("Could not find a valid BMP085 sensor, check wiring!");
        while (1)
        {
            delay(SHORT_DELAY);
        }
    }
    Serial.println("BMP180 Found!");

    Serial.println("MPU6050 test!");
    if (!mpu.begin())
    {
        Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
        while (1)
        {
            delay(SHORT_DELAY);
        }
    }
    Serial.println("MPU6050 Found!");

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
 

    Serial.println("Servo Initialization done!");    
}

void get_readings()
{
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    altitude = bmp.readAltitude(SEA_LEVEL_PRESSURE);
    ax = a.acceleration.x + 0.35;
    ay = a.acceleration.y -0.3;
    az = a.acceleration.z -10.31;
}

void kalmanUpdate()
{
    //Measurement matrix
    BLA::Matrix<2, 1> Z = {altitude,
                           az};
    //Predicted state estimate
    BLA::Matrix<3, 1> x_hat_minus = A * x_hat;
    
    //Predicted estimate covariance
    BLA::Matrix<3, 3> P_minus = A * P * (~A) + Q;
    
    //Kalman gain
    BLA::Matrix<3, 2> K = P_minus * (~H) * ((H * P_minus * (~H) + R)).Inverse();
    
    //Measurement residual
    Y = Z - (H * x_hat_minus);
    
    //Updated state estimate
    x_hat = x_hat_minus + K * Y;
    
    //Updated estimate covariance
    P = (I - K * H) * P_minus;
    

    s = x_hat(0);
    v = x_hat(1);
    a = x_hat(2);
    
}