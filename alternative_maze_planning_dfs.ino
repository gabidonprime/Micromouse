#include <Arduino.h>
#include <math.h>
#include "Stack.hpp"
#include "Tuple.hpp"
#include "Graph.hpp"
#include "Encoder.hpp"
#include "Encoder.hpp"
#include "Motor.hpp"
#include "PIDController.hpp"
#include "MovingAverageFilter.hpp"
#include "IMU.hpp"
#include "graph2ascii.hpp"
#include "ascii2graph.hpp"

// Assumptions:
// 1. Starting at cell 1 facing south
// 2. Cells are ordered as in previous milestons

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
mtrn3100::PIDController forward_l_pid(10, 5, 0.25, 0, 82, forward_align_windup);
mtrn3100::PIDController forward_r_pid(10, 5, 0.25, 0, 70, forward_align_windup);
mtrn3100::PIDController forward_yaw_pid(3, 0.5, 0.25, 0, 60);

////////////////////////////////////////////////////////////////////////////////
// Wall Approach
constexpr float wall_approach_target = 120;
constexpr float align_windup = 0.1;
mtrn3100::PIDController forward_align_pid_l(0.32, 38 / align_windup, 0, 0, 80, align_windup);
mtrn3100::PIDController forward_align_pid_r(0.27, 30 / align_windup, 0, 0, 70, align_windup);

constexpr float turn_windup = 1;
mtrn3100::PIDController turn_l_pid(20, 20, 0.1, 0, 72, turn_windup);
mtrn3100::PIDController turn_r_pid(20, 20, 0.1, 0, 70, turn_windup);

////////////////////////////////////////////////////////////////////////////////
// Wall correction 
constexpr float guardrail_size = 70;                        //TODO: Tune & Calibtrate this value
constexpr float wall_correct_target = 87;                   //TODO: Tune & Calibtrate this value
mtrn3100::PIDController wall_correct_pid(10, 0, 0, 0, 40);  //TODO: Tune & Calibtrate this value
bool wall_correct_pid_cleaned = true;


// MAFS
mtrn3100::MovingAverageFilter<float, 20> l_maf;
mtrn3100::MovingAverageFilter<float, 20> r_maf;

mtrn3100::MovingAverageFilter<float, 100> l2_maf;
mtrn3100::MovingAverageFilter<float, 100> r2_maf;

////////////////////////////////////////////////////////////////////////////////
// IMU
constexpr bool use_IMU = false;
mtrn3100::IMU imu;

// Drive parameters.
constexpr uint8_t pwm = 80;
const double wheelRadius = 0.023; // m
const double wheelBase = 0.13;  // m

const float turnArcLength_left = ( (M_PI/2)*(wheelBase/2) )/wheelRadius - 0.2;
const float turnArcLength_right = ( (M_PI/2)*(wheelBase/2) )/wheelRadius - 0.2;
const float forwardArcLength = (0.25 + 0.0205)/wheelRadius;

// Global variables.
float leftMotorsignal;
float rightMotorsignal;
float l_target = 0;
float r_target = 0;
int forward_count = 0;

// Setting up pose and cardinal directions
enum CardinalDirections {
    NORTH = 0,
    EAST = 1,
    SOUTH = 2,
    WEST = 3,
};
int row = 0;
int col = 0;
int head = SOUTH;

// Initialise maze with nodes.
mtrn3100::Graph<int, bool> maze(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 
21, 22, 23, 24, 25, 26, 27, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45);

// Global variables
int cellsConnected = 0;
int frontConnected = false;
int leftConnected = false;
int rightConnected = false;
int currentCell = 0;
int unvisitedCells[45];
Stack<int> junctions;

// Mains
void setup() {
    Serial.begin(115200);
    delay(100);

    Wire.begin(); 
    initialiseLidars();

    if (use_IMU) {
        imu.begin();
        imu.calibrateAcceleration();
        while (imu.dataReady())
            ;
        imu.reset();
        imu.stabaliseYaw();
    }

    forward_l_pid.zeroEncoderAndSetTarget(l_encoder.position, l_target);
    forward_r_pid.zeroEncoderAndSetTarget(r_encoder.position, r_target);
    turn_l_pid.zeroEncoderAndSetTarget(l_encoder.position, l_target);
    turn_r_pid.zeroEncoderAndSetTarget(r_encoder.position, r_target);
    wall_correct_pid.zeroEncoderAndSetTarget(0, wall_correct_target);

    // Initialise unvisited cells array
    for (int i = 0; i < 45, i++) {
        unvisitedCells[i] = i + 1;
    }
}

void loop() {
    // Add data of current cell to graph
    mapCell();

    // If multiple cells connected, explore junction
    if (cellsConnected > 1) {
        junctions.push(currentCell);
        exploreJunction();
    } else { // Otherwise, continue
        wallFollowStep();
    }

    // If all cells visited and junctions stack is empty
    if (sizeof(unvisitedCells) == 0) {
        // Send maze data to bot
        sendMazeData();

        // Finish
        while(1) {
            // Do nothing.
        }
    }
}

// Helper functions
void exploreJunction() {
    if (rightConnected) {
        turnRight();
        moveForward();
        mapCell();
            while (isInArray(currentCell, unvisitedCells)) { // Continue until reached a visited cell
                if (cellsConnected > 1) {
                    junctions.push(currentCell);
                    // Nested explore junctions allows for backtracking
                    exploreJunction();
                } else {
                    wallFollowStep();
                    mapCell();
                }
                wallFollowStep();
                mapCell();
            }
            goToCell(junctions.top(), row, col);
        }
        if (frontConnected) {
            moveForward();
            mapCell();
            while (isInArray(currentCell, unvisitedCells)) {
                if (cellsConnected > 1) {
                    junctions.push(currentCell);
                    exploreJunction();
                } else {
                    wallFollowStep();
                    mapCell();
                }
            }
            goToCell(junctions.top(), row, col);
        } 
        if (leftConnected) {
            turnLeft();
            moveForward();
            mapCell();
            while (isInArray(currentCell, unvisitedCells)) {
                if (cellsConnected > 1) {
                    junctions.push(currentCell);
                    exploreJunction();
                } else {
                    wallFollowStep();
                    mapCell();
                }
            }
            goToCell(junctions.top(), row, col);
        }

        // Once junction is explored, pop  
        junctions.pop();
        if (junctions.isEmpty()) {
            return;
        } else {
            goToCell(junctions.top(), row, col);
        }
}

void mapCell() {
    // Get lidar readings
    leftLidar.getReading();
    rightLidar.getReading();
    frontLidar.getReading();

    // Detect walls
    if (leftLidar.getDistance() < threshold) {
        leftConnected = false;
    } 

    if (rightLidar.getDistance() < threshold) {
        rightConnected = false;
    } 

    if (frontLidar.getDistance() < threshold) {
        frontConnected = false;
    }

    // Determine which cells are connected by index
    currentCell = pos2index(row, col);

    if (head == NORTH) {
        int frontCell = (frontConnected == true) ? currentCell - 8 : -1;
        int leftCell = (leftConnected == true) ? currentCell - 1 : -1;
        int rightCell = (rightConnected == true) ? currentCell + 1 : -1;
    } else if (head == EAST) {
        int frontCell = (frontConnected == true) ? currentCell + 1 : -1;
        int leftCell = (leftConnected == true) ? currentCell - 8 : -1;
        int rightCell = (rightConnected == true) ? currentCell + 8: -1;
    } else if (head == SOUTH) {
        int frontCell = (frontConnected == true) ? currentCell + 8 : -1; 
        int leftCell = (leftConnected == true) ? currentCell + 1 : -1;
        int rightCell = (rightConnected == true) ? currentCell - 1 : -1;
    } else if (head == WEST) {
        int frontCell = (frontConnected == true) ? currentCell - 1: -1;
        int leftCell = (leftConnected == true) ? currentCell + 8: -1;
        int rightCell = (rightConnected == true) ? currentCell - 8: -1;
    }

    // Remove current cell from list of unvisited cells
    removeElementFromArray(unvisitedCells, currentCell);

    // Add data from cell to graph
    if (frontConnected) {
        insertEdge(maze, currentCell, frontCell);
        cellsConnected++;
    }
    if (leftConnected) {
        insertEdge(maze, currentCell, leftCell);
        cellsConnected++;
    }
    if (rightConnected) {
        insertEdge(maze, currentCell, rightCell);
        cellsConnected++;
    }
}

void wallFollowStep() {
    if (rightConnected){
        turnRight();
        moveForward();
    } else if (frontConnected) {
        moveForward();
    } else {
        turnLeft();
    }
}

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

        // l_motor.setPWM(90 + wall_correction_force);
        // r_motor.setPWM(80 - wall_correction_force);
        l_motor.setPWM(wall_correction_force);
        r_motor.setPWM(-wall_correction_force);

    }

    l_motor.setPWM(0);
    r_motor.setPWM(0);
}

void moveForward() {
    Serial.println("MOVING FORWARD");
    l_target = forwardArcLength + 0.5;
    r_target = forwardArcLength;

    forward_l_pid.reset();
    forward_r_pid.reset();
    forward_yaw_pid.reset();
    wall_correct_pid.reset();
    wall_correct_pid_cleaned = true;
    
    if (use_IMU) {
        while (true) {
            if (imu.dataReady()) {
                imu.read();
                if (imu.yaw() != NAN) {break;}
            }
        }
    }
    Serial.println("IMU READY");
    imu.read();

    forward_l_pid.zeroEncoderAndSetTarget(l_encoder.position, l_target);
    forward_r_pid.zeroEncoderAndSetTarget(r_encoder.position, r_target);
    forward_yaw_pid.zeroEncoderAndSetTarget(imu.yaw(), 0);

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
    // while (l_maf.average() > 0.025 || r_maf.average() > 0.025) {
    while (abs(r2_maf.difference()) > 0.5|| abs(l2_maf.difference()) > 0.5
            || abs(leftMotorsignal) >= 70 || abs(rightMotorsignal) >= 70) {
        //wall_correction_force = compute_wall_correction(); //Comment out if you don't want guardrails
        if (use_IMU && imu.dataReady()) {
            imu.read();
        }
        if (use_IMU && imu.yaw() != NAN) {
            heading_correction = forward_yaw_pid.compute(imu.yaw());
        }

        leftMotorsignal = forward_l_pid.compute(l_encoder.position);
        rightMotorsignal = forward_r_pid.compute(r_encoder.position);

        l_motor.setPWM(leftMotorsignal - heading_correction);    //  + wall_correction_force
        r_motor.setPWM(rightMotorsignal + heading_correction); // - wall_correction_force
        //l_motor.setPWM(-heading_correction);    //  + wall_correction_force
        //r_motor.setPWM(heading_correction); // - wall_correction_force

        // l_maf.sample(abs(forward_l_pid.derivative));
        // r_maf.sample(abs(forward_r_pid.derivative));
        l2_maf.sample(leftMotorsignal - heading_correction);
        r2_maf.sample(rightMotorsignal + heading_correction);
    }
    Serial.println("DONE MOVING FORWARD");
    l_motor.setPWM(0);
    r_motor.setPWM(0);

    // Update position
    switch(head) {
        case NORTH:
            row--;
            break;
        case EAST:
            col++;
            break;
        case SOUTH:
            row++;
            break;
        case WEST:
            col--;
            break;
    }
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

void turnLeft() {
    l_target = -turnArcLength_left - 0.15;
    r_target = turnArcLength_left + 0.15;

    turn_l_pid.reset();
    turn_r_pid.reset();
    //forward_yaw_pid.reset();
    wall_correct_pid.reset();
    wall_correct_pid_cleaned = true;
    
    if (use_IMU) {
        while (true) {
            if (imu.dataReady()) {
                imu.read();
                if (imu.yaw() != NAN) {break;}
            }
        }
    Serial.println("IMU READY");
    imu.read();
    }

    turn_l_pid.zeroEncoderAndSetTarget(l_encoder.position, l_target);
    turn_r_pid.zeroEncoderAndSetTarget(r_encoder.position, r_target);
    //forward_yaw_pid.zeroEncoderAndSetTarget(imu.yaw(), 0);

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
    // while (abs(r2_maf.difference()) > 0.5|| abs(l2_maf.difference()) > 0.5
    //         || abs(leftMotorsignal) >= 40 || abs(rightMotorsignal) >= 40) {
        //wall_correction_force = compute_wall_correction(); //Comment out if you don't want guardrails
        if (use_IMU && imu.dataReady()) {
            imu.read();
        }
        if (use_IMU && imu.yaw() != NAN) {
            heading_correction = forward_yaw_pid.compute(imu.yaw());
        }

        leftMotorsignal = turn_l_pid.compute(l_encoder.position);
        rightMotorsignal =  turn_r_pid.compute(r_encoder.position);

        l_motor.setPWM(leftMotorsignal - heading_correction);    //  + wall_correction_force
        r_motor.setPWM(rightMotorsignal + heading_correction); // - wall_correction_force
        //l_motor.setPWM(-heading_correction);    //  + wall_correction_force
        //r_motor.setPWM(heading_correction); // - wall_correction_force
        Serial.print("Left Motor Signal: ");
        Serial.print(leftMotorsignal);
        Serial.print("; Right Motor Signal: ");
        Serial.println(rightMotorsignal);
        l_maf.sample(abs(turn_l_pid.derivative));
        r_maf.sample(abs(turn_r_pid.derivative));
        // l2_maf.sample(leftMotorsignal);
        // r2_maf.sample(rightMotorsignal);
    }
    l_motor.setPWM(0);
    r_motor.setPWM(0);

    // Update heading
    head -= 1;
    if (head < 0) {
        head = 3;
    }
}

void turnRight() {
    Serial.println("TURNING RIGHT");
    l_target = turnArcLength_right;
    r_target = -turnArcLength_right;
    turn_l_pid.reset();
    turn_r_pid.reset();
    turn_yaw_pid.reset();
    while (!imu.dataReady()) {}
    imu.read();
    float startingYaw = imu.yaw();
    turn_l_pid.zeroEncoderAndSetTarget(l_encoder.position, l_target);
    turn_r_pid.zeroEncoderAndSetTarget(r_encoder.position, r_target);
    turn_yaw_pid.zeroEncoderAndSetTarget(startingYaw, 90); //FIXME: Add current yaw
    l_maf.clear();
    r_maf.clear();
    for (int i = 0; i < 10; i++) {
        l_maf.sample(1);
        r_maf.sample(1);
    }
    float yawMotorsignal = 0;
    while (l_maf.average() > 0.05 && r_maf.average() > 0.05) {
        if (imu.dataReady()) {
            imu.read();
        }
        leftMotorsignal = turn_l_pid.compute(l_encoder.position);
        rightMotorsignal = turn_r_pid.compute(r_encoder.position);
        yawMotorsignal = turn_yaw_pid.compute(imu.yaw()); 

        l_motor.setPWM(leftMotorsignal - yawMotorsignal);
        r_motor.setPWM(rightMotorsignal + yawMotorsignal);

        l_maf.sample(abs(turn_l_pid.derivative));
        r_maf.sample(abs(turn_r_pid.derivative));

        // Print Motor Signals
        Serial.print("Yaw Motor Signal: ");
        Serial.print(yawMotorsignal);
        Serial.print("; Current Yaw: ");
        Serial.print(imu.yaw());
        Serial.print("; Target Yaw: ");
        Serial.print(startingYaw);
        Serial.print("; Left Motor Signal: ");
        Serial.print(leftMotorsignal);
        Serial.print("; Right Motor Signal: ");
        Serial.println(rightMotorsignal);
    }
    l_motor.setPWM(0);
    r_motor.setPWM(0);

    // Update heading
    head += 1;
    if (head > 3) {
        head = 0;
    }
}

// Initialise lidars
void initialiseLidars() {
    leftLidar.initialise();
    rightLidar.initialise();
    frontLidar.initialise();
}

// Convert node positions to node indexes
int pos2index(int* row, int* col) {
    return (*row * 9 + *col + 1);
}

int* index2pos(int index) {
    int rowCol[2];
    rowCol[0] = floor((index - 1) / 5);
    rowCol[1] = (index - 1) % 9;
    return rowCol;
}

// Remove element from an array
void removeElementFromArray(const int* originalArray, int elementToRemove) {
    int size = sizeof(originalArray);
    int newArray[size];
    int newArrayIndex = 0;

    for (int i = 0; i < size; ++i) {
        if (originalArray[i] != elementToRemove) {
            newArray[newArrayIndex] = originalArray[i];
            ++newArrayIndex;
        }
    }

    while (newArrayIndex < size) {
        newArray[newArrayIndex] = 0;
        ++newArrayIndex;
    }

    originalArray = newArray;
}

bool isInArray(int number, int arr[]) {
    for (int i = 0; i < sizeof(arr); ++i) {
        if (arr[i] == number) {
            return true;  // Number is found in the array
        }
    }
    return false;  // Number is not found in the array
}

// Insert edges into maze
void insertEdge(mtrn3100::Graph<int, int>& maze, int cell1, int cell2) {
    maze.insert_edge(cell1, cell2, 0);
    maze.insert_edge(cell2, cell1, 0);
}

void goToCell() {
    // Harry to complete
}

void sendMazeData() {
    // Harry to complete
}
