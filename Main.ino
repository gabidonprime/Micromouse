#include <Arduino.h>
#include <math.h>

#include <Wire.h>
#include <VL6180X.h>
#include <VL53L0X.h>
#include "Graph.hpp"
#include "ascii2graph.hpp"
#include "shortest_path.hpp"

#include "Encoder.hpp"
#include "Motor.hpp"
#include "PIDController.hpp"
#include "MovingAverageFilter.hpp"
#include "lidars.hpp"
#include "IMU.hpp"

////////////////////////////////////////////////////////////////////////////////
// Motors 
mtrn3100::Motor r_motor(2, 4, 3);
mtrn3100::Motor l_motor(7, 5, 6);

////////////////////////////////////////////////////////////////////////////////
// Lidars
const int left_address = 0x20;
const int right_address = 0x22;
const int front_address = 0x27;

const int left_enablePin = 32;
const int right_enablePin = 36;
const int front_enablePin = 35;

shortLidar left_lidar(left_address, left_enablePin);
shortLidar right_lidar(right_address, right_enablePin);
longLidar front_lidar(front_address, front_enablePin);

////////////////////////////////////////////////////////////////////////////////
// Blueotooth Globals
mtrn3100::Tuple<int, int> start(0, 0);
mtrn3100::Tuple<int, int> end(0, 0);
int initialHeading = 0; 

////////////////////////////////////////////////////////////////////////////////
// Encoders
void readLeftEncoder();
void readRightEncoder();
mtrn3100::Encoder r_encoder(18, 22, readRightEncoder);
mtrn3100::Encoder l_encoder(19, 23, readLeftEncoder);
void readLeftEncoder() { l_encoder.readEncoder(); }
void readRightEncoder() { r_encoder.readEncoder(); }

////////////////////////////////////////////////////////////////////////////////
// PIDs
constexpr float forward_align_windup = 1;
mtrn3100::PIDController forward_l_pid(10, 5, 0.25, 0, 81, forward_align_windup);
mtrn3100::PIDController forward_r_pid(10, 5, 0.25, 0, 70, forward_align_windup);
mtrn3100::PIDController forward_yaw_pid(3, 0.5, 0.25, 0, 60, 1);

////////////////////////////////////////////////////////////////////////////////
// PIDs
constexpr float forward2_align_windup = 0.8;
mtrn3100::PIDController forward2_l_pid(10, 8, 0.2, 0, 160, forward2_align_windup);
mtrn3100::PIDController forward2_r_pid(10, 8, 0.2, 0, 100, forward2_align_windup);
mtrn3100::PIDController forward2_yaw_pid(5, 2, 0.5, 0, 130, forward2_align_windup);

////////////////////////////////////////////////////////////////////////////////
// Wall Approach
constexpr float wall_approach_target = 120;
constexpr float align_windup = 0.1;
mtrn3100::PIDController forward_align_pid_l(0.32, 38 / align_windup, 0, 0, 80, align_windup);
mtrn3100::PIDController forward_align_pid_r(0.27, 30 / align_windup, 0, 0, 70, align_windup);

constexpr float turn_windup = 1;
mtrn3100::PIDController turn_l_pid(45, 9.5, 0.75, 0, 70, turn_windup);
mtrn3100::PIDController turn_r_pid(45, 9.5, 0.75, 0, 60, turn_windup);
mtrn3100::PIDController turn_yaw_pid(3, 1.5, 0, 0, 40, 1);

////////////////////////////////////////////////////////////////////////////////
// Wall correction 
constexpr float guardrail_size = 80;                       
constexpr float wall_correct_target = 87;                  
mtrn3100::PIDController wall_correct_pid(15, 4, 0, 0, 40); 
bool wall_correct_pid_cleaned = true;


// MAFS
mtrn3100::MovingAverageFilter<float, 20> l_maf;
mtrn3100::MovingAverageFilter<float, 20> r_maf;

mtrn3100::MovingAverageFilter<float, 100> l2_maf;
mtrn3100::MovingAverageFilter<float, 100> r2_maf;

////////////////////////////////////////////////////////////////////////////////
// IMU
constexpr bool use_IMU = true;
mtrn3100::IMU imu;


// Drive parameters.
constexpr uint8_t pwm = 80;
const double wheelRadius = 0.023; // m
const double wheelBase = 0.13;  // m

const float turnArcLength_left = ( (M_PI/2)*(wheelBase/2) )/wheelRadius + 0.02;
const float turnArcLength_right = ( (M_PI/2)*(wheelBase/2) )/wheelRadius + 0.02 ;
const float forwardArcLength = (0.25 + 0.0205)/wheelRadius;
const float turnArcLength_left_180 = ( (M_PI)*(wheelBase/2) )/wheelRadius;

// Global variables.
float leftMotorsignal;
float rightMotorsignal;

float l_target = 0;
float r_target = 0;

int forward_count = 0;

// Drive plan.
constexpr size_t drivePlanLength = 10;
char drivePlan[10] = "FLFRRFRFLL";
size_t drivePlanIndex = 0;

// Setting up pose and cardinal directions
enum CardinalDirections {
    NORTH = 0,
    EAST = 1,
    SOUTH = 2,
    WEST = 3,
};

// Robot pose using maze cell coordinates and heading.
int row = 0;
int col = 0;
int head = SOUTH;


// Wall-E setup and run
void setup() {
    Serial.begin(115200);
    Serial3.begin(9600);

    Serial.println("STARTING SETUP");
    delay(100);
    Wire.begin();  // This must be called before IMU::begin().
    Serial.println("WIRE INITIALIZED");

    Serial.println("Initialising left lidar...");
    left_lidar.initialise();
    Serial.println("Initialising right lidar...");
    right_lidar.initialise();
    Serial.println("Initialising front lidar...");
    front_lidar.initialise();
    Serial.println("Lidars ready!");

    if (use_IMU) {
        Serial.println("INITIALIZING IMU");
        imu.begin();
        Serial.println("IMU BEGAN");
        imu.calibrateAcceleration();
        Serial.println("IMU CALIBRATED ACCELERATION");
        while (imu.dataReady())
            ;
        imu.reset();
        Serial.println("IMU INITIALIZED");
        imu.stabaliseYaw();
    }

    forward_l_pid.zeroEncoderAndSetTarget(l_encoder.position, l_target);
    forward_r_pid.zeroEncoderAndSetTarget(r_encoder.position, r_target);

    turn_l_pid.zeroEncoderAndSetTarget(l_encoder.position, l_target);
    turn_r_pid.zeroEncoderAndSetTarget(r_encoder.position, r_target);

    wall_correct_pid.zeroEncoderAndSetTarget(0, wall_correct_target);
    Serial.println("PIDS INITIALIZED");
}

// Main loop, calcuates bluetooth drive plan fed in from CV
// and follows it
void loop() {
   if (Serial3.available()) {
        String str = Serial3.readStringUntil('\0');
        Serial.print("Received: ");
        Serial.println(str);
        if (str.charAt(0) == 'S') {
            start = mtrn3100::Tuple<int, int>(str.charAt(1) - '0', str.charAt(2) - '0');
            initialHeading = str.charAt(3) - '0';
            Serial.print("Start: ");
            Serial.print(mtrn3100::get<0>(start));
            Serial.print(", ");
            Serial.println(mtrn3100::get<1>(start));
        } else if (str.charAt(0) == 'E') {
            end = mtrn3100::Tuple<int, int>(str.charAt(1) - '0', str.charAt(2) - '0');
            Serial.print("End: ");
            Serial.print(mtrn3100::get<0>(end));
            Serial.print(", ");
            Serial.println(mtrn3100::get<1>(end));
        } else {
            // Serial3.println(result);
            // Serial.println(result);
            String maze = "";
            char *token = strtok((char *)str.c_str(), "n");
            while (token != NULL) {
                maze += token;
                maze += "\n";
                Serial.println(token);
                token = strtok(NULL, "n");
            }

            mtrn3100::Graph<int, int> mazeGraph = mtrn3100::ascii2graph(maze.c_str());
            String drivePlan = solve_maze(mazeGraph, start, end, initialHeading);
            follow_drive_plan(drivePlan);
        }
    }
    
}

// Follows a drive plan with wall alignment
// and yaw correction after yaw
void follow_drive_plan(String plan) {
    for (int i = 0; i < plan.length(); i++) {
        switch (plan.charAt(i)) {
            case 'F':
                moveForward();

                for (int i = 0; i < 5; i++) { // Read the Front Lidar for 5 samples
                    front_lidar.sample();
                }
                if (front_lidar.value() < 100) { // If the front lidar detects a wall, approach it
                    approach_wall();
                }

                break;
            case 'L':
                turnLeft();
                imu.stabaliseYaw2();
                break;
            case 'R':
                turnRight();
                imu.stabaliseYaw2();
                break;
            case 'B':
                turn180();
                imu.stabaliseYaw2();
                break;
            default:
                break;
        }
    }
}

// For advanced use
// Estabishes Lidar readings as a guardrail 
// and calibrates
void guardrails_testing() {
    wall_correct_pid.reset();
    wall_correct_pid_cleaned = true;

    const float bench_arc_length = 1.2/wheelRadius;

    float wall_correction_force = 0;
    while (l_encoder.position < bench_arc_length && r_encoder.position < bench_arc_length) {
        wall_correction_force = compute_wall_correction();

        Serial.print("Left Lidar: ");
        Serial.print(left_lidar.value());
        Serial.print("; Right Lidar: ");
        Serial.println(right_lidar.value());

    }

    l_motor.setPWM(0);
    r_motor.setPWM(0);
}

// Standard Helper function
void moveForward() {
    Serial.println("MOVING FORWARD");
    l_target = forwardArcLength + 0.5;
    r_target = forwardArcLength;

    forward_l_pid.reset();
    forward_r_pid.reset();
    wall_correct_pid.reset();
    wall_correct_pid_cleaned = true;
    
    if (use_IMU) {
        forward_yaw_pid.reset();
        while (true) {
            if (imu.dataReady()) {
                Serial.println("IMU DATA READY");
                imu.read();
                if (!isnan(imu.yaw())) {break;}
            }
        }
        Serial.println(imu.yaw());
        forward_yaw_pid.zeroEncoderAndSetTarget(imu.yaw(), 0);
    }

    forward_l_pid.zeroEncoderAndSetTarget(l_encoder.position, l_target);
    forward_r_pid.zeroEncoderAndSetTarget(r_encoder.position, r_target);

    l_maf.clear();
    r_maf.clear();
    for (int i = 0; i < 10; i++) {
        l_maf.sample(1);
        r_maf.sample(1);
    }

    l2_maf.clear();
    r2_maf.clear();


    float wall_correction_force = 0;
    float heading_correction = 0;

    while (abs(r2_maf.difference()) > 0.5|| abs(l2_maf.difference()) > 0.5
            || abs(leftMotorsignal) >= 70 || abs(rightMotorsignal) >= 70) {

        // wall_correction_force = compute_wall_correction(); //Comment out if you don't want guardrails
        if (use_IMU && imu.dataReady()) {
            imu.read();
        }
        if (use_IMU && !isnan(imu.yaw())) {
            heading_correction = forward_yaw_pid.compute(imu.yaw());
        }

        leftMotorsignal = forward_l_pid.compute(l_encoder.position);
        rightMotorsignal = forward_r_pid.compute(r_encoder.position);

        l_motor.setPWM(leftMotorsignal - heading_correction + wall_correction_force); 
        r_motor.setPWM(rightMotorsignal + heading_correction - wall_correction_force);

        // l_maf.sample(abs(forward_l_pid.derivative));
        // r_maf.sample(abs(forward_r_pid.derivative));
        l2_maf.sample(leftMotorsignal - heading_correction + wall_correction_force);
        r2_maf.sample(rightMotorsignal + heading_correction - wall_correction_force);
    }
    Serial.println("DONE MOVING FORWARD");
    l_motor.setPWM(0);
    r_motor.setPWM(0);
}



/**
 * moveForward2 helper function
 * advanced movement
*/
void moveForward2() {
    Serial.println("MOVING FORWARD 2");
    l_target = 1.9 * (forwardArcLength + 0.3);
    r_target = 1.9 * (forwardArcLength);

    forward2_l_pid.reset();
    forward2_r_pid.reset();
    wall_correct_pid.reset();
    wall_correct_pid_cleaned = true;
    
    if (use_IMU) {
        forward2_yaw_pid.reset();
        while (true) {
            if (imu.dataReady()) {
                Serial.println("IMU DATA READY");
                imu.read();
                if (!isnan(imu.yaw())) {break;}
            }
        }
        Serial.println(imu.yaw());
        forward2_yaw_pid.zeroEncoderAndSetTarget(imu.yaw(), 0);
    }

    forward2_l_pid.zeroEncoderAndSetTarget(l_encoder.position, l_target);
    forward2_r_pid.zeroEncoderAndSetTarget(r_encoder.position, r_target);

    l_maf.clear();
    r_maf.clear();
    for (int i = 0; i < 10; i++) {
        l_maf.sample(1);
        r_maf.sample(1);
    }

    l2_maf.clear();
    r2_maf.clear();


    float wall_correction_force = 0;
    float heading_correction = 0;

    while (abs(r2_maf.difference()) > 0.5|| abs(l2_maf.difference()) > 0.5
            || abs(leftMotorsignal) >= 70 || abs(rightMotorsignal) >= 70) {
        //wall_correction_force = compute_wall_correction(); //Comment out if you don't want guardrails
        if (use_IMU && imu.dataReady()) {
            imu.read();
        }
        if (use_IMU && !isnan(imu.yaw())) {
            heading_correction = forward2_yaw_pid.compute(imu.yaw());
        } 

        leftMotorsignal = forward2_l_pid.compute(l_encoder.position);
        rightMotorsignal = forward2_r_pid.compute(r_encoder.position);

        l_motor.setPWM(leftMotorsignal - heading_correction);    //  + wall_correction_force
        r_motor.setPWM(rightMotorsignal + heading_correction); // - wall_correction_force

        // l_maf.sample(abs(forward_l_pid.derivative));
        // r_maf.sample(abs(forward_r_pid.derivative));
        l2_maf.sample(leftMotorsignal - heading_correction);
        r2_maf.sample(rightMotorsignal + heading_correction);
    }
    Serial.println("DONE MOVING FORWARD");
    l_motor.setPWM(0);
    r_motor.setPWM(0);
}

/**
 * turnLeft helper function
 * takes into account
*/
void turnLeft() {
    Serial.println("MOVING LEFT");
    l_target = -turnArcLength_left;
    r_target = turnArcLength_left;

    turn_l_pid.reset();
    turn_r_pid.reset();
    wall_correct_pid.reset();
    wall_correct_pid_cleaned = true;
    
    if (use_IMU) {
        imu.reset();
        turn_yaw_pid.reset();
        while (true) {
            if (imu.dataReady()) {
                imu.read();
                if (imu.yaw() != NAN) {break;}
            }
        }
        Serial.println("IMU READY");
        imu.read();
        turn_yaw_pid.zeroEncoderAndSetTarget(imu.yaw(), 90);
    }

    turn_l_pid.zeroEncoderAndSetTarget(l_encoder.position, l_target);
    turn_r_pid.zeroEncoderAndSetTarget(r_encoder.position, r_target);

    l_maf.clear();
    r_maf.clear();
    for (int i = 0; i < 10; i++) {
        l_maf.sample(1);
        r_maf.sample(1);
    }

    l2_maf.clear();
    r2_maf.clear();


    float wall_correction_force = 0;
    float heading_correction = 0;

    while (l_maf.average() > 0.025 || r_maf.average() > 0.025) {

        leftMotorsignal = turn_l_pid.compute(l_encoder.position);
        rightMotorsignal =  turn_r_pid.compute(r_encoder.position);

        l_motor.setPWM(leftMotorsignal - heading_correction);    //  + wall_correction_force
        r_motor.setPWM(rightMotorsignal + heading_correction); // - wall_correction_force

        l_maf.sample(abs(turn_l_pid.derivative));
        r_maf.sample(abs(turn_r_pid.derivative));
        // l2_maf.sample(leftMotorsignal - heading_correction);
        // r2_maf.sample(rightMotorsignal + heading_correction);
    }
    l_motor.setPWM(0);
    r_motor.setPWM(0);
}

/**
 * For advanced turning
 */
void turn180() {
    Serial.println("TURNING 180");
    l_target = -turnArcLength_left_180 - 0.1;
    r_target = turnArcLength_left_180 + 0.1;

    turn_l_pid.reset();
    turn_r_pid.reset();
    wall_correct_pid.reset();
    wall_correct_pid_cleaned = true;
    
    if (use_IMU) {
        imu.reset();
        forward_yaw_pid.reset();
        while (true) {
            if (imu.dataReady()) {
                imu.read();
                if (imu.yaw() != NAN) {break;}
            }
        }
        Serial.println("IMU READY");
        imu.read();
        forward_yaw_pid.zeroEncoderAndSetTarget(imu.yaw(), 0);
    }

    turn_l_pid.zeroEncoderAndSetTarget(l_encoder.position, l_target);
    turn_r_pid.zeroEncoderAndSetTarget(r_encoder.position, r_target);

    l_maf.clear();
    r_maf.clear();
    for (int i = 0; i < 10; i++) {
        l_maf.sample(1);
        r_maf.sample(1);
    }

    l2_maf.clear();
    r2_maf.clear();


    float wall_correction_force = 0;
    float heading_correction = 0;
    // {
    while (l_maf.average() > 0.025 || r_maf.average() > 0.025) {
        leftMotorsignal = turn_l_pid.compute(l_encoder.position);
        rightMotorsignal =  turn_r_pid.compute(r_encoder.position);

        l_motor.setPWM(leftMotorsignal - heading_correction);    //  + wall_correction_force
        r_motor.setPWM(rightMotorsignal + heading_correction); // - wall_correction_force

        l_maf.sample(abs(turn_l_pid.derivative));
        r_maf.sample(abs(turn_r_pid.derivative));
        // l2_maf.sample(leftMotorsignal);
        // r2_maf.sample(rightMotorsignal);
    }
    //Serial.println("DONE MOVING FORWARD");
    l_motor.setPWM(0);
    r_motor.setPWM(0);
}


// Standard Helper function for turningRight
void turnRight() {
    l_target = turnArcLength_right;
    r_target = -turnArcLength_right;

    turn_l_pid.reset();
    turn_r_pid.reset();
    wall_correct_pid.reset();
    wall_correct_pid_cleaned = true;

    turn_l_pid.zeroEncoderAndSetTarget(l_encoder.position, l_target);
    turn_r_pid.zeroEncoderAndSetTarget(r_encoder.position, r_target);

    l_maf.clear();
    r_maf.clear();
    for (int i = 0; i < 10; i++) {
        l_maf.sample(1);
        r_maf.sample(1);
    }

    l2_maf.clear();
    r2_maf.clear();


    float wall_correction_force = 0;
    float heading_correction = 0;

    while (l_maf.average() > 0.025 || r_maf.average() > 0.025) {
        leftMotorsignal = turn_l_pid.compute(l_encoder.position);
        rightMotorsignal =  turn_r_pid.compute(r_encoder.position);

        l_motor.setPWM(leftMotorsignal);    //  + wall_correction_force
        r_motor.setPWM(rightMotorsignal); // - wall_correction_force

        l_maf.sample(abs(turn_l_pid.derivative));
        r_maf.sample(abs(turn_r_pid.derivative));
        // l2_maf.sample(leftMotorsignal);
        // r2_maf.sample(rightMotorsignal);
    }

    l_motor.setPWM(0);
    r_motor.setPWM(0);
}


/**
 * Compute the wall correction force value based on the readings from the left and right lidars.
 * Positive is defined as turning CW, negative is CCW.
 * 
 * @return The wall correction force value, which is the output of a PID controller that adjusts the
 *         steering angle based on the distance to the nearest wall.
 */
float compute_wall_correction() {
    left_lidar.sample();
    right_lidar.sample();
    float leftRange = left_lidar.value();
    float rightRange = right_lidar.value();
    float correction_force = 0;

    if (leftRange < guardrail_size) {
        correction_force = wall_correct_pid.compute(leftRange);
        wall_correct_pid_cleaned = false;
        Serial.println("Left Guardrail Active");
    } 
    else if (rightRange < guardrail_size) {
        correction_force = -wall_correct_pid.compute(rightRange);
        wall_correct_pid_cleaned = false;
        Serial.println("Right Guardrail Active");
    }
    else {
        if (!wall_correct_pid_cleaned) {
            wall_correct_pid.reset();
            wall_correct_pid_cleaned = true;
            Serial.println("No Guardrails, resetting PID");
        }
    }

    return correction_force;
}

/**
 * Approach a wall using PID control to align the robot with the wall at
 * the specified distance.
 *
 * This function uses PID control to adjust the motor signals and align the robot with the wall.
 * It samples the front lidar to get the distance to the wall, and uses two PID controllers to
 * adjust the motor signals for the left and right motors. The function continues to adjust the
 * motor signals until the robot is aligned with the wall, as determined by the moving average
 * filter of the motor signals.
 */
void approach_wall() {
    Serial.println("APPROACHING WALL");
    forward_align_pid_l.zeroEncoderAndSetTarget(0, wall_approach_target); // Tested value 97
    forward_align_pid_r.zeroEncoderAndSetTarget(0, wall_approach_target); // Tested value 97

    r2_maf.clear();
    l2_maf.clear();

    float l_motor_signal = 0;
    float r_motor_signal = 0;
    float lidarRange = 0;

    while (abs(r2_maf.difference()) > 0.1 || abs(l2_maf.difference()) > 0.1
            || abs(l_motor_signal) == 70 || abs(r_motor_signal) == 70) {
        front_lidar.sample();
        lidarRange = front_lidar.value();
        l_motor_signal= -forward_align_pid_l.compute(lidarRange);
        r_motor_signal= -forward_align_pid_r.compute(lidarRange);


        l_motor.setPWM(l_motor_signal);
        r_motor.setPWM(r_motor_signal);

        l2_maf.sample(l_motor_signal);
        r2_maf.sample(r_motor_signal);
    }

    l_motor.setPWM(0);
    r_motor.setPWM(0);

}
