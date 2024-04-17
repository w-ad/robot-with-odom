#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/asset.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "lemlib/pose.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/motors.hpp"

pros::ADIDigitalOut intake_pistons('H');
bool intake_up = true;

pros::ADIDigitalOut wings('E');
bool wings_out = false;

pros::ADIDigitalOut PTO('G');
bool PTO_On = false;



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
    0, // integral gain (kI)
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
    0, // integral gain (kI)
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

    intake_pistons.set_value(intake_up);
    wings.set_value(wings_out);
    PTO.set_value(PTO_On);
    
}


void disabled() {}

void competition_initialize() {}

void autonomous() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    // close WP
    chassis.moveToPose(12, 56, 0, 4000, {.maxSpeed = 127, .minSpeed = 127}, false);
    chassis.moveToPose(12, 24, -135, 4000, {.forwards = false, .maxSpeed = 100, .minSpeed = 80}, false);
    chassis.moveToPose(-12, 12, 135, 4000, {.maxSpeed = 127, .minSpeed = 100}, false);
    chassis.turnTo(0, 0, 500, true, 127, false);
    chassis.moveToPose(0, 0, 45, 5000, {.maxSpeed = 127, .minSpeed = 100}, false);
    chassis.turnTo(chassis.getPose().x+24, chassis.getPose().y+24, 500, true, 127, false);
    chassis.moveToPose(32, 0, 90, 4000, {.maxSpeed = 80, .minSpeed = 40}, false);
}

void opcontrol() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    // controller
    // loop to continuously update motors
    while (true) {
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        drivetrain.leftMotors->move(leftY + rightX);
        drivetrain.rightMotors->move(leftY - rightX);

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            intake_up = !intake_up;
            intake_pistons.set_value(intake_up);
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            wings_out = !wings_out;
            wings.set_value(wings_out);
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            PTO_On = !PTO_On;
            PTO.set_value(PTO_On);
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intake.move(127);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            intake.move(-127);
        } else {
            intake.move(0);
        }

        // delay to save resources
        pros::delay(10);
    }
}

