#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/asset.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "lemlib/pose.hpp"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include <string>

pros::ADIDigitalOut intake_pistons('H'); // the pistons that hold the intake up by default to get to sub-6
bool intake_up = true;

pros::ADIDigitalOut wings('E'); // the pistons that push out the wings
bool wings_out = false;

pros::ADIDigitalOut PTO('G'); // the pistons that complete the gear system for the hang (PTO)
bool PTO_On = false;

pros::ADIDigitalOut Hang('F'); // the piston that triggers the hang mech (three sheets of plexi)
bool Hang_Activation = false;

pros::MotorGroup intake({11, -20});

pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::MotorGroup leftMotors({
    -4,
    -5,
    -6,
});
pros::MotorGroup rightMotors({1, 2, 3});
pros::Imu imu(10);


lemlib::Drivetrain drivetrain(
    &leftMotors, // left motor group
    &rightMotors, // right motor group
    12.6, // 10 inch track width
    3.25, // using new 2.75" omnis
    450, // drivetrain rpm is 360
    2 // chase power is 2. If we had traction wheels, it would have been 8
);
lemlib::ControllerSettings linearController(
    10, // proportional gain (kP)
    10, // integral gain (kI)
    3, // derivative gain (kD)
    3, // anti windup
    1, // small error range, in inches
    100, // small error range timeout, in milliseconds
    3, // large error range, in inches
    500, // large error range timeout, in milliseconds
    20 // maximum acceleration (slew)
);
lemlib::ControllerSettings angularController(
    2, // proportional gain (kP)
    1, // integral gain (kI)
    10, // derivative gain (kD)
    3, // anti windup
    1, // small error range, in degrees
    100, // small error range timeout, in milliseconds
    3, // large error range, in degrees
    500, // large error range timeout, in milliseconds
    0 // maximum acceleration (slew)
);
lemlib::OdomSensors sensors(
    nullptr, // vertical tracking wheel 1, set to null
    nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
    nullptr, // horizontal tracking wheel 1
    nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
    &imu // inertial sensor
);
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors
);



void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        lemlib::Pose pose(0, 0, 0);
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });

    //pros::Motor motor1 = pros::Motor(1);
    //pros::Motor motor2 = pros::Motor(2);
   // pros::Motor motor3 = pros::Motor(3);
   // pros::Motor motor4 = pros::Motor(4);
   // pros::Motor motor5 = pros::Motor(5);
   // pros::Motor motor6 = pros::Motor(6);

    intake_pistons.set_value(intake_up);
    wings.set_value(wings_out);
    PTO.set_value(PTO_On);
    Hang.set_value(Hang_Activation);
}


void disabled() {}

void competition_initialize() {}

void autonomous() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    // close WP
     chassis.moveToPose(1, -2, 90, 4000, {.maxSpeed = 127, .minSpeed = 80}, false);
     wings.set_value(1);
    chassis.moveToPose(0, 0, 90, 4000, {.maxSpeed = 127, .minSpeed = 80}, false);
    chassis.moveToPose(-20, 2, 0, 4000, {.maxSpeed = 127, .minSpeed = 80}, false);





    //* far WP
   // chassis.moveToPose(0, 8, 0, 4000, {.maxSpeed = 127, .minSpeed = 80}, false);
    //wings.set_value(1);
    //chassis.moveToPose(-20, 23, -75, 4000, {.maxSpeed = 127, .minSpeed = 80}, false);
    //intake.move(127);
    //chassis.moveToPose(-31, 23, -75, 4000, {.maxSpeed = 127, .minSpeed = 80}, false);
    //wings.set_value(0);
    //chassis.moveToPose(-10, 23, 45, 1000, {.forwards = false, .maxSpeed = 127, .minSpeed = 80}, false);
    //chassis.turnTo(-20, -60, 1000, false, 127, false);
    //chassis.moveToPose(-20, -24, 0, 4000, {.forwards = false, .maxSpeed = 127, .minSpeed = 80}, false);
    //chassis.moveToPose(-20, -24, 90, 4000, {.forwards = false, .maxSpeed = 127, .minSpeed = 80}, false);
    //chassis.setPose(lemlib::Pose(0,0,-90));
    //chassis.turnTo(2.5, 10, 4000);
    //chassis.moveToPose(9, 17.2, 20, 4000, {.forwards = true, .maxSpeed = 127, .minSpeed = 80}, false); 
     //wings.set_value(1);
 
    intake.move(0);
//close elims
    


//farside elims
 


}

void opcontrol() {
       // controller
    // loop to continuously update motors
    while (true) {
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);


    double temp1 = pros::c::motor_get_temperature(1);
   double temp2 = pros::c::motor_get_temperature(2);
    double temp3 = pros::c::motor_get_temperature(3);
    double temp4 = pros::c::motor_get_temperature(4);
    double temp5 = pros::c::motor_get_temperature(5);
    double temp6 = pros::c::motor_get_temperature(6);
// get temp of each motor
    controller.clear();
    controller.set_text(1, 1, std::to_string(temp1));
    controller.set_text(2, 1, std::to_string(temp2));
    controller.set_text(3, 1, std::to_string(temp3));
    controller.set_text(4, 1, std::to_string(temp4));
    controller.set_text(5, 1, std::to_string(temp5));
    controller.set_text(6, 1, std::to_string(temp6));
// print motor temps onto the controller screen


        drivetrain.leftMotors->move(leftY + rightX);
        drivetrain.rightMotors->move(leftY - rightX);
//moves drive based on joystick location
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            intake_up = !intake_up;
            intake_pistons.set_value(intake_up);
        }
//sets the intake down upon press via pneumatics
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            wings_out = !wings_out;
            wings.set_value(wings_out);
        }
//wing activation via pneumatics
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            PTO_On = !PTO_On;
            PTO.set_value(PTO_On);
        }
//PTO activation (completing gear system for hang) via pneumatics
if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            Hang_Activation = !Hang_Activation;
            Hang.set_value(Hang_Activation);
        }
//triggers hang to go up via pneumatic



        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            intake.move(127);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intake.move(-127);
        } else {
            intake.move(0);
        }
//Makes the intake intake when pressing l2 and outake when pressing R2, and sets it to stationary when neither is being pressed

        // delay to save resources
        pros::delay(10);
    }
}

