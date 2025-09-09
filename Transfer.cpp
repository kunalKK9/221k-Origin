#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"

// currently playing silksong so zero coding will get done 

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({1, 2, -3}, pros::MotorGearset::blue); // left motor group - ports 1 2 3
pros::MotorGroup rightMotors({-4, -5, 6}, pros::MotorGearset::blue); // right motor group - ports 4 5 6

// optical sensor for the hood
pros::Optical hoodSensor(19); // optical sensor on port 19
pros::Optical hoodSensor2(16); // optical sensor on port 16, used for colour detection

// optical sensor for alliance colour detection
pros::Optical allianceSensor(17); // optical sensor on port 17
pros::Optical allianceSensor2(15); // optical sensor on port 15, used for colour detection

// Scraper piston
pros::adi::DigitalOut scraper('A');
bool scrapertoggle = false;

// Hood piston
pros::adi::DigitalOut hood('B');
bool hoodtoggle = false;

// Inertial Sensor on port 10
pros::Imu imu(10);

//rubber band intake motor
pros::Motor intake(18, pros::MotorGearset::green);

//recycler mech (big maybe if were gonna use it)
pros::Motor allianceHoard(13, pros::MotorGearset::green);


pros::Motor topMotor(14, pros::MotorGearset::green); // mid motor switching 

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed (will change later in testing)
pros::Rotation horizontalEnc(20);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed (will change later in testing)
pros::Rotation verticalEncL(-11);
//vertical tracking wheel encoder. rotation sensor, port 12, not reversed (will change later in testing, see config, determining reversal)
pros::Rotation verticalEndR(-12);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2, -5.75); //subject to change
// vertical tracking wheel Right. 2.75" diameter, 2.5" offset TBD, left of the robot (negative)
lemlib::TrackingWheel verticalL(&verticalEncL, lemlib::Omniwheel::NEW_2, -2.5); //subject to change
// vertical tracking wheel Left. 2.75" diameter, 2.5" offset TBD, right of the robot (positive)
lemlib::TrackingWheel verticalR(&verticalEndR, lemlib::Omniwheel::NEW_2, 2.5); //subject to change

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              10, // 10 inch track width
                              lemlib::Omniwheel::NEW_4, // using new 4" omnis
                              360, // drivetrain rpm is 360
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            3, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             10, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(&verticalL, // vertical tracking wheel 1,
                            &verticalR, // vertical tracking wheel 2, 
                            &horizontal, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);



// Runs intake and hoard logic based on mode and button input cuz why not
void niggaballs360(bool clearMode, bool intakeForward, bool intakeReverse) {
    allianceSensor.set_led_pwm(100);

    double ALhue1 = allianceSensor.get_hue();
    int ALprox1 = allianceSensor.get_proximity();

    // Use the sensor with the stronger signal (scrap this too lazy to delete)
    int maxProx = ALprox1;

    if (intakeForward) {
        intake.move(-127);

        if (clearMode) {
            hoodtoggle = true; // dumping mode (open piston)
            allianceHoard.move(-127);
        } else {
            if (maxProx > 120) {
                if ((ALhue1 >= 0 && ALhue1 <= 30) || (ALhue1 >= 330 && ALhue1 <= 360)) {
                    hoodtoggle = false; // allow red niggaballs
                } else if (ALhue1 >= 210 && ALhue1 <= 250) {
                    hoodtoggle = true; // eject blue niggaballs
                } else {
                    hoodtoggle = false; // fuck knows what it detected
                }
            } else {
                hoodtoggle = false; // nun nigga
            }
        }

    } else if (intakeReverse) {
        intake.move(127); //eject balls already in the tube if there is
        allianceHoard.move(0);

        if (clearMode) {
            allianceHoard.move(127); //eject balls from alliance hoard

        } 
    } else {
        intake.move(0);
        allianceHoard.move(0);
    }
}


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

    //turn all motors to hold and brake mode
    intake.set_brake_mode(pros::MotorBrake::brake); // set brake mode 
    allianceHoard.set_brake_mode(pros::MotorBrake::brake); // set brake mode 
    topMotor.set_brake_mode(pros::MotorBrake::brake);
    // Set DT motors to brake mode
    leftMotors.set_brake_mode(pros::MotorBrake::brake);
    rightMotors.set_brake_mode(pros::MotorBrake::brake);
    
    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            pros::lcd::print(3, "221K Origin");
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {
    //initialize ball colour sorter



    // intake balls in hoarding mode
    niggaballs360(false, true, false);  // hoard logic
    pros::delay(1000);

    // dump alliance balls
    niggaballs360(true, true, false);   // eject mode
    pros::delay(1000);

    // stop
    niggaballs360(false, false, false);

    // Move to x: 20 and y: 15, and face heading 90. Timeout set to 4000 ms
    chassis.moveToPose(20, 15, 90, 4000);
    // Move to x: 0 and y: 0 and face heading 270, going backwards. Timeout set to 4000ms
    chassis.moveToPose(0, 0, 270, 4000, {.forwards = false});
    // cancel the movement after it has traveled 10 inches
    chassis.waitUntil(10);
    chassis.cancelMotion();
    // Turn to face the point x:45, y:-45. Timeout set to 1000
    // dont turn faster than 60 (out of a maximum of 127)
    chassis.turnToPoint(45, -45, 1000, {.maxSpeed = 60});
    // Turn to face a direction of 90º. Timeout set to 1000
    // will always be faster than 100 (out of a maximum of 127)
    // also force it to turn clockwise, the long way around
    chassis.turnToHeading(90, 1000, {.direction = AngularDirection::CW_CLOCKWISE, .minSpeed = 100});
    // Follow the path in path.txt. Lookahead at 15, Timeout set to 4000
    // following the path with the back of the robot (forwards = false)
    // see line 116 to see how to define a path
    chassis.follow(example_txt, 15, 4000, false);
    // wait until the chassis has traveled 10 inches. Otherwise the code directly after
    // the movement will run immediately
    // Unless its another movement, in which case it will wait
    chassis.waitUntil(10);
    pros::lcd::print(4, "Traveled 10 inches during pure pursuit!");
    // wait until the movement is done
    chassis.waitUntilDone();
    pros::lcd::print(4, "pure pursuit finished!");
}

/**
 * Runs in driver control
 */
void opcontrol() {
    // Set motors to brake mode
    leftMotors.set_brake_mode(pros::MotorBrake::brake);
    rightMotors.set_brake_mode(pros::MotorBrake::brake);
    //toggle for clearing the balls
    bool clearMode = false;



    // Track direction toggle state
    static bool midMotorReversed = false;

    
    // controller
    // loop to continuously update motors
    while (true) {
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
        chassis.arcade(leftY, rightX);
       
        // Toggle mode
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
            clearMode = !clearMode;
        }

        controller.print(0, 0, "Mode: %s", clearMode ? "CLEAR" : "HOARD");

        // Use niggaballs function for eahch button
        niggaballs360(
            clearMode,
            controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1), // forward
            controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)  // reverse
        );

        //toggle scraper on and off
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
            scrapertoggle = !scrapertoggle;
            scraper.set_value(scrapertoggle);
        }

        //toggle direction of hood failsafe
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            hood.set_value(false);
        }

        // Toggle motor direction when A is pressed
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            midMotorReversed = !midMotorReversed;
        }

        // Control midMotor when R1 is pressed
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            int power = midMotorReversed ? 127 : -127;
            topMotor.move(power);
        } else {
            topMotor.move(0);
        }





        //delay to save resources ig
        pros::delay(10);


    }


}
