package org.firstinspires.ftc.teamcode;

public class Argo_Configuration {

        // Motor ports - Drivetrain - REV Hub
        public static final int FRONT_LEFT_MOTOR_PORT = 2;
        public static final int BACK_LEFT_MOTOR_PORT = 1;
        public static final int FRONT_RIGHT_MOTOR_PORT = 0;
        public static final int BACK_RIGHT_MOTOR_PORT = 3;

        // Motor port - Intake
        public static final int INTAKE_MOTOR_PORT = 2;

        // Motor Names - Drivetrain
        public static final String FRONT_LEFT_MOTOR = "leftFront";
        public static final String BACK_LEFT_MOTOR = "leftRear";
        public static final String FRONT_RIGHT_MOTOR = "rightFront";
        public static final String BACK_RIGHT_MOTOR = "rightRear";

        // Motor Name - Intake Slider
        public static final String SLIDER_MOTOR = "sliderMotor";

        //slider down limit switch
        public static final String SLIDER_LOWER_SWITCH = "sliderLowerSwitch";

        // Servo position - Claw grab
        public static final double CLAW_GRAB_MIN = 0.15;
        public static final double CLAW_GRAB_MAX = 0.35;

        //Claw positions

        public static final int CLAW_OPEN = 0;
        public static final int CLAW_CLOSE = 1;

        //Claw Spin positions

        public static final int CLAW_SPIN_LEFT = 0;
        public static final int CLAW_SPIN_RIGHT = 1;

        // Servo position - Claw Spin
        public static final double CLAW_SPIN_MIN = 1;
        public static final double CLAW_SPIN_MAX = 0.5;

        // Servo position - Arm
        public static final double ARM_MIN = 0.29;
        public static final double ARM_MAX = 0.65;
        public static final double ARM_MID_POINT = (ARM_MIN + ARM_MAX)/2;
        public static final double ARM_LARGE_INCREMENT = 0.01;
        public static final double ARM_SMALL_INCREMENT = 0.005;

        // Servo Motor Names
        public static final String CLAW_INTAKE = "servoGrab";//port #2
        public static final String CLAW_SPIN = "servoSpin";//port #1
        public static final String CLAW_UP_DOWN = "servoArm";//port #0

        // Motor speeds (Power settings)
        public static final double MAX_MOTOR_SPEED = 0.8;
        public static final double MIN_MOTOR_SPEED = 0.2;

        // Slider Motor speeds (Power settings)
        public static final double SLIDER_SPEED = 0.8;
        public static final double SLIDER_SPEED_HANG = 0.8;

        // Robot Movement Speeds (Drive system)
        public static final double DRIVE_SPEED = 0.5; // 50% speed
        public static final double TURN_SPEED = 0.3; // 30% turning speed

        //Slider Height
        public static final int SLIDE_MAX_HEIGHT = 5500;
        public static final int SLIDE_MIN_HEIGHT = 0;

        //Slider Height for hanging specimen
        public static final int SLIDE_MAX_HEIGHT_HANG = 2500;
        public static final int SLIDE_MIN_HEIGHT_HANG = 0;



        // Slider direction
        public static final int MOVE_DOWN = 0;
        public static final int MOVE_UP = 1;

        // Autonomous Mode Timeouts (in seconds)
        public static final double AUTONOMOUS_TIME_LIMIT = 30.0;

        //Ticks per inch from encoder
        static final double COUNTS_PER_INCH = 1440;

        // Motor Names - Drivetrain
        public static final String T_SENSOR = "tSensor";

}
