package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.ARM_MAX;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.ARM_MIN;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.CLAW_GRAB_MIN;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.CLAW_INTAKE;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.CLAW_PIVOT;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.CLAW_SPIN;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.CLAW_SPIN_LEFT;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.CLAW_SPIN_MIN;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.CLAW_SPIN_RIGHT;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.CLAW_UP_DOWN_LEFT;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.CLAW_UP_DOWN_RIGHT;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.FRONT_LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.FRONT_RIGHT_MOTOR;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.BACK_LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.BACK_RIGHT_MOTOR;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.GEAR_RATIO;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.LINKAGE_MOTOR;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.MOTOR_STOP;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.SLIDER_MOTOR_LEFT;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.SLIDER_MOTOR_RIGHT;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.SLIDER_SPEED;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.MIN_MOTOR_SPEED;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.MAX_MOTOR_SPEED;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.MOVE_DOWN;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.MOVE_UP;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.SLIDER_MOTOR_LEFT;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.SLIDER_MOTOR_RIGHT;

import static org.firstinspires.ftc.teamcode.Argo_Configuration.SLIDE_MAX_HEIGHT;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.SLIDE_MIN_HEIGHT;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.T_SENSOR;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.SLIDER_LOWER_SWITCH;
import static java.lang.Thread.sleep;
import  org.firstinspires.ftc.teamcode.Argo_Movements.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsTouchSensor;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

//*TELEMETRY- Set power so that it will move but do sleep 2 sec so the servo moves*//

@TeleOp(name="Robot Centric Drive", group="Teleop")
//@Disabled
public class DriveRobot extends OpMode
{
    // Declare OpMode members.
    private DcMotor frontLeftMotor,backLeftMotor,frontRightMotor,backRightMotor,sliderMotorLeft,sliderMotorRight,linkageMotor;
    private Argo_Movements Argo_Robot_Move;
    private Claw_Movements Claw_Move;
    private Servo clawGrab;
    private Servo clawSpin;
    private Servo clawArm;
    private Servo pivotArmLeft;
    private Servo pivotArmRight;

    //TouchSensor tSensor;
    //private ModernRoboticsTouchSensor sliderLowerSwitch;

    //This is the Gyro (actually the Inertial Measurement Unit)
    IMU imu;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        // Drivetrain
        frontLeftMotor = hardwareMap.dcMotor.get(FRONT_LEFT_MOTOR);//Hub - Port #2
        backLeftMotor = hardwareMap.dcMotor.get(BACK_LEFT_MOTOR);//Hub - Port # 1
        frontRightMotor = hardwareMap.dcMotor.get(FRONT_RIGHT_MOTOR);//Hub - Port #0
        backRightMotor = hardwareMap.dcMotor.get(BACK_RIGHT_MOTOR);//Hub - Port #3

        frontLeftMotor.resetDeviceConfigurationForOpMode();
        backLeftMotor.resetDeviceConfigurationForOpMode();
        frontRightMotor.resetDeviceConfigurationForOpMode();
        backRightMotor.resetDeviceConfigurationForOpMode();

        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        Argo_Robot_Move = new Argo_Movements(frontLeftMotor,backLeftMotor,frontRightMotor,backRightMotor);

        //Slider motors
        sliderMotorLeft = hardwareMap.dcMotor.get(SLIDER_MOTOR_LEFT);//EHub- Port #0
        sliderMotorRight = hardwareMap.dcMotor.get(SLIDER_MOTOR_RIGHT);//EHub- Port #1
        sliderMotorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        sliderMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);
        sliderMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sliderMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sliderMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sliderMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define the IMU (gyro sensor)
        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
        //Field-centric initialization - end
        imu.resetYaw();  //reset the gyro

        //sliderLowerSwitch = hardwareMap.get(ModernRoboticsTouchSensor.class,SLIDER_LOWER_SWITCH);

        //Linkage
        linkageMotor= hardwareMap.dcMotor.get(LINKAGE_MOTOR);//EHub- Port #2
        //linkageMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        linkageMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linkageMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Claw
        clawGrab = hardwareMap.servo.get(CLAW_INTAKE);
        clawSpin = hardwareMap.servo.get(CLAW_SPIN);
        clawArm = hardwareMap.servo.get(CLAW_PIVOT);
        pivotArmLeft = hardwareMap.servo.get(CLAW_UP_DOWN_LEFT);
        pivotArmRight = hardwareMap.servo.get(CLAW_UP_DOWN_RIGHT);

       Claw_Move = new Claw_Movements(clawGrab,sliderMotorLeft,sliderMotorRight, clawSpin, clawArm,pivotArmLeft,pivotArmRight,telemetry);

        pivotArmLeft.setPosition(ARM_MIN);
        //pivotArmRight.setPosition(ARM_MIN);
       // clawGrab.setPosition(CLAW_GRAB_MIN);//initialize to avoid it spinning in some strange direction the first time it is used.
        clawSpin.setPosition(CLAW_SPIN_MIN);
        try {
            sleep(1000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        //tSensor = hardwareMap.touchSensor.get(T_SENSOR);//E Hub - Port #0
/*
        sliderMotor.setPower(-SLIDER_SPEED);
        if (sliderLowerSwitch.isPressed())
        {
            sliderMotor.setPower(0);
        }
*/
        telemetry.addData("Slider Position", sliderMotorLeft.getCurrentPosition());
        telemetry.update();

    }
    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {

        double robotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        //*************************
        //* Robot-centric driving *
        //*************************

        // Rotate the movement direction counter to the bot's rotation
        double driveY = -gamepad1.left_stick_y; // Forward/Backward
        double driveX = gamepad1.left_stick_x;  // Left/Right (Strafe)
        double turn = gamepad1.right_stick_x;   // Rotation

        double radians = Math.toRadians(robotHeading);

        // Calculate the robot-centric drive values
        double tempX = driveX * Math.cos(radians) - driveY * Math.sin(radians);
        double tempY = driveX * Math.sin(radians) + driveY * Math.cos(radians);

        double frontLeftPower = tempY + tempX + turn;
        double frontRightPower = tempY - tempX - turn;
        double backLeftPower = tempY - tempX + turn;
        double backRightPower = tempY + tempX - turn;

        double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));
/*
        //read the slider switch and show on the screen
        if(sliderLowerSwitch.isPressed()){
            sliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sliderMotor.setPower(0);
            telemetry.addLine("Slider Lower Limit Reached!");

        }
        else {
            telemetry.addLine("Slider is raised to position "+ sliderMotor.getCurrentPosition());
        }

*/
        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }
        //Run robot at reduced speeds when the trigger is pressed.
        if((gamepad1.left_trigger > 0.1) || (gamepad1.right_trigger > 0.1))
        {
            frontLeftPower *= MIN_MOTOR_SPEED;
            frontRightPower *= MIN_MOTOR_SPEED;
            backLeftPower  *= MIN_MOTOR_SPEED;
            backRightPower  *= MIN_MOTOR_SPEED;
        } else {
            frontLeftPower *= MAX_MOTOR_SPEED;
            frontRightPower *= MAX_MOTOR_SPEED;
            backLeftPower *= MAX_MOTOR_SPEED;
            backRightPower *= MAX_MOTOR_SPEED;
        }
        frontLeftPower *= GEAR_RATIO;
        frontRightPower *= GEAR_RATIO;
        // Run motor and regular speed
            Argo_Robot_Move.moveRobot(frontLeftPower,frontRightPower,backLeftPower,backRightPower);

        // Update telemetry
        telemetry.addData("Heading", robotHeading);
        telemetry.addData("Front Left Power", frontLeftPower);
        telemetry.addData("Front Right Power", frontRightPower);
        telemetry.addData("Back Left Power", backLeftPower);
        telemetry.addData("Back Right Power", backRightPower);
        telemetry.addData("Slider Position", sliderMotorLeft.getCurrentPosition());
        telemetry.addData("Wrist", clawSpin.getPosition());

        telemetry.addData("Front Left Direction", frontLeftMotor.getDirection());
        telemetry.addData("Front Right Direction", frontRightMotor.getDirection());
        telemetry.addData("Back Left Direction", backLeftMotor.getDirection());
        telemetry.addData("Back Right Direction", backRightMotor.getDirection());

        telemetry.update();

       //Claw operations - Open and Close

        if (gamepad2.right_trigger>0.1){ // Close claw
            Claw_Move.claw_Grabber(clawGrab,CLAW_CLOSE);
        } else if (gamepad2.left_trigger >0.1) { //Open claw
            Claw_Move.claw_Grabber(clawGrab, CLAW_OPEN);
            telemetry.addData("claw Grab position", clawGrab.getPosition());
            telemetry.update();
        }

        // Wrist Spin // Gamepad 2 - x,b

        if (gamepad2.x){ // Spin left
            Claw_Move.claw_Rotate(clawSpin,CLAW_SPIN_LEFT);
        } else if (gamepad2.b) { //spin right
            Claw_Move.claw_Rotate(clawSpin, CLAW_SPIN_RIGHT);
        }

        //Sliders - flat
        if (gamepad2.right_bumper)
        {
            linkageMotor.setTargetPosition(1100);
            linkageMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            linkageMotor.setPower(SLIDER_SPEED);
            try {
                sleep(2000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            linkageMotor.setPower(MOTOR_STOP);

        } else if (gamepad2.left_bumper){
            linkageMotor.setTargetPosition(0);
            linkageMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            linkageMotor.setPower(-SLIDER_SPEED);
            try {
                sleep(2000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            linkageMotor.setPower(MOTOR_STOP);
        }
//move arm up/down
        if (gamepad2.dpad_up){
            telemetry.addData("Arm position 1", pivotArmLeft.getPosition());
            telemetry.update();
            Claw_Move.claw_moveArm(pivotArmLeft,MOVE_UP);
            //Claw_Move.claw_moveArm(pivotArmRight,MOVE_UP);
        } else if (gamepad2.dpad_down) {
            telemetry.addData("Arm position 2", pivotArmLeft.getPosition());
            telemetry.update();
            Claw_Move.claw_moveArm(pivotArmLeft, MOVE_DOWN);
            //Claw_Move.claw_moveArm(pivotArmRight, MOVE_DOWN);
        }

        telemetry.addData("Arm current position ", pivotArmLeft.getPosition());
        telemetry.update();

        //move pivot up/down
        if (gamepad1.y){
            telemetry.addData("Arm position 1", clawArm.getPosition());
            telemetry.update();
            Claw_Move.claw_movePivot(clawArm,MOVE_UP);

        } else if (gamepad1.a) {
            telemetry.addData("Arm position 2", clawArm.getPosition());
            telemetry.update();
            Claw_Move.claw_movePivot(clawArm, MOVE_DOWN);
            //Claw_Move.claw_moveArm(pivotArmRight, MOVE_DOWN);
        }

        telemetry.addData("Arm current position ", clawArm.getPosition());
        telemetry.update();


    // Move sliders up and down
        if (gamepad1.dpad_up){
           Claw_Move.sliderMoveToPosition(sliderMotorLeft,sliderMotorRight,MOVE_UP);
            }

        if (gamepad1.dpad_down){
            Claw_Move.sliderMoveToPosition(sliderMotorLeft,sliderMotorRight,MOVE_DOWN);
            }
         else {
            Claw_Move.Slider_stop(sliderMotorLeft,sliderMotorRight);
        }
        /*

        // Move sliders up and down to hang the specimen
        if (gamepad2.y){
            //Claw_Move.sliderHangSpecimen(sliderMotorLeft,MOVE_UP,tSensor);
            Claw_Move.sliderHangSpecimen(sliderMotorLeft,MOVE_UP);
        }
        if (gamepad2.a){
            //Claw_Move.sliderHangSpecimen(sliderMotorLeft,MOVE_DOWN,tSensor);
            Claw_Move.sliderHangSpecimen(sliderMotorLeft,MOVE_DOWN);
        }
        else {
            //Claw_Move.Slider_stop(sliderMotorLeft);
        }

    */
    }
     /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        Argo_Robot_Move.stopMotors();
    }

}
