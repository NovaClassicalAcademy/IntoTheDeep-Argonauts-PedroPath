package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.Argo_Configuration.CLAW_UP_DOWN_LEFT;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.CLAW_UP_DOWN_RIGHT;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.FRONT_LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.FRONT_RIGHT_MOTOR;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.BACK_LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.BACK_RIGHT_MOTOR;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.MAX_MOTOR_SPEED;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.MIN_MOTOR_SPEED;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.SLIDER_MOTOR_LEFT;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.SLIDER_MOTOR_RIGHT;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

//*TELEMETRY- Set power so that it will move but do sleep 2 sec so the servo moves*//

@TeleOp(name="Robot Test Drivetrain", group="Teleop")
//@Disabled
public class TestNewRobot extends OpMode
{
    // Declare OpMode members.
    private DcMotor frontLeftMotor,backLeftMotor,frontRightMotor,backRightMotor,sliderMotorLeft,sliderMotorRight,linkageMotor;
    private Argo_Movements Argo_Robot_Move;
    private Servo pivotArmLeft;
    private Servo pivotArmRight;
    //This is the Gyro (actually the Inertial Measurement Unit)
    IMU imu;
    //private Servo servo1;
   // private Servo servo2;




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
        pivotArmLeft = hardwareMap.servo.get(CLAW_UP_DOWN_LEFT);//Hub - Port #3
        pivotArmRight = hardwareMap.servo.get(CLAW_UP_DOWN_RIGHT);//Hub - Port #4

        sliderMotorLeft = hardwareMap.dcMotor.get(SLIDER_MOTOR_LEFT);//EHub- Port #0
        sliderMotorRight = hardwareMap.dcMotor.get(SLIDER_MOTOR_RIGHT);//EHub- Port #1

        linkageMotor= hardwareMap.dcMotor.get(SLIDER_MOTOR_RIGHT);//EHub- Port #2


        /*
        double position = 0.5; // Example position between 0 and 1
        servo1.setPosition(position);
        servo2.setPosition(position);
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        // You can also dynamically change the positions over time
        // for example, moving the servos from 0 to 1:
        for (double p = 0; p <= position; p += 0.01) {
            servo1.setPosition(p);
            servo2.setPosition(p);
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
        */

        frontLeftMotor.resetDeviceConfigurationForOpMode();
        backLeftMotor.resetDeviceConfigurationForOpMode();
        frontRightMotor.resetDeviceConfigurationForOpMode();
        backRightMotor.resetDeviceConfigurationForOpMode();

        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        Argo_Robot_Move = new Argo_Movements(frontLeftMotor,backLeftMotor,frontRightMotor,backRightMotor);

        // Define the IMU (gyro sensor)
        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        //Field-centric initialization - end
        imu.resetYaw();  //reset the gyro



        double mpower = 0;
        mpower = 0.2;
        sliderMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);
        sliderMotorRight.setPower(mpower);
        sliderMotorLeft.setPower(mpower);
        try {
            Thread.sleep(5000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        sliderMotorRight.setPower(0);
        sliderMotorLeft.setPower(0);

        mpower = -0.2;
        sliderMotorRight.setPower(mpower);
        sliderMotorLeft.setPower(mpower);
        try {
            Thread.sleep(5000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        sliderMotorRight.setPower(0);
        sliderMotorLeft.setPower(0);







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
        // Set motor powers
        Argo_Robot_Move.moveRobot(frontLeftPower,frontRightPower,backLeftPower,backRightPower);

        double mpower = 0;
        mpower = 0.5;
        sliderMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);
        sliderMotorRight.setPower(mpower);
        sliderMotorLeft.setPower(mpower);
        try {
            Thread.sleep(3000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        sliderMotorRight.setPower(0);
        sliderMotorLeft.setPower(0);

        mpower = -0.5;
        sliderMotorRight.setPower(mpower);
        sliderMotorLeft.setPower(mpower);
        try {
            Thread.sleep(3000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        sliderMotorRight.setPower(0);
        sliderMotorLeft.setPower(0);


        // Update telemetry
        telemetry.addData("Heading", robotHeading);
        telemetry.addData("Front Left Power", frontLeftPower);
        telemetry.addData("Front Right Power", frontRightPower);
        telemetry.addData("Back Left Power", backLeftPower);
        telemetry.addData("Back Right Power", backRightPower);
/*
        telemetry.addData("Front Left Direction", frontLeftMotor.getDirection());
        telemetry.addData("Front Right Direction", frontRightMotor.getDirection());
        telemetry.addData("Back Left Direction", backLeftMotor.getDirection());
        telemetry.addData("Back Right Direction", backRightMotor.getDirection());
*/
        telemetry.addData("Slider Motor Left Direction", sliderMotorLeft.getDirection());
        telemetry.addData("Slider Motor Right Direction", sliderMotorRight.getDirection());

        telemetry.addData("Linkage Motor Direction", linkageMotor.getDirection());

       telemetry.addData("Claw Arm Left position", pivotArmLeft.getPosition());
       telemetry.addData("Claw Arm Right position", pivotArmRight.getPosition());


        telemetry.update();

       }
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        Argo_Robot_Move.stopMotors();
    }

}
