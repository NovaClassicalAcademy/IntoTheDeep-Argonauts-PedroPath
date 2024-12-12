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
        public static final String SLIDER_MOTOR = "SliderMotor";

        // Servo position - Claw grab
        public static final double CLAW_GRAB_MIN = 0.17;
        public static final double CLAW_GRAB_MAX = 0.35;

        // Servo Motor Names
        public static final String CLAW_INTAKE = "servoGrab";
        public static final String CLAW_SPIN = "ServoSpin";
        public static final String CLAW_UP_DOWN = "servoMain";

        // Motor speeds (Power settings)
        public static final double MAX_MOTOR_SPEED = 1.0;
        public static final double MIN_MOTOR_SPEED = 0.2;

        // Slider Motor speeds (Power settings)
        public static final double MAX_SLIDER_SPEED = 0.5;
        public static final double MIN_SLIDER_SPEED = 0.2;


        // Robot Movement Speeds (Drive system)
        public static final double DRIVE_SPEED = 0.5; // 50% speed
        public static final double TURN_SPEED = 0.3; // 30% turning speed

        //Slider Height
        public static final int SLIDE_MAX_HEIGHT = 5800;
        public static final int SLIDE_MIN_HEIGHT = 0;

        // Autonomous Mode Timeouts (in seconds)
        public static final double AUTONOMOUS_TIME_LIMIT = 30.0;
}
