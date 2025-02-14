package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import static com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.ARM_LARGE_INCREMENT;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.ARM_MAX;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.ARM_MID_POINT;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.ARM_MIN;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.ARM_SMALL_INCREMENT;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.CLAW_GRAB_MAX;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.CLAW_GRAB_MIN;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.CLAW_SPIN_LEFT;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.CLAW_SPIN_MAX;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.CLAW_SPIN_MIN;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.CLAW_SPIN_RIGHT;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.FRONT_LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.MOTOR_STOP;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.MOVE_DOWN;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.MOVE_UP;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.SLIDER_SPEED;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.SLIDER_SPEED_HANG;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.SLIDE_HANG_TIMEOUT;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.SLIDE_MAX_HEIGHT;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.SLIDE_MAX_HEIGHT_HANG;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.SLIDE_MIN_HEIGHT;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.SLIDE_MIN_HEIGHT_HANG;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.SLIDE_TIMEOUT;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.T_SENSOR;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Claw_Movements {
    private Servo clawGrab;
    private Servo clawSpin;
    private Servo clawArm;
    private Servo pivotArmLeft;
    private Servo pivotArmRight;
    private DcMotor sliderMotorLeft;
    private DcMotor sliderMotorRight;
    //private TouchSensor tSensor;
    private Telemetry telemetry;
    private DcMotorEx coolMotor;
    ElapsedTime runtime = new ElapsedTime();  // Timer for timeout checking
    public Claw_Movements(Servo ClawGrabber, DcMotor sliderMotorLeft,DcMotor sliderMotorRight,Servo clawSpin, Servo clawArm,Servo pivotArmLeft, Servo pivotArmRight, Telemetry telemetry) {
        this.clawGrab = ClawGrabber;
        this.sliderMotorLeft = sliderMotorLeft;
        this.sliderMotorRight = sliderMotorRight;
        this.clawSpin = clawSpin;
        this.clawArm = clawArm;
        this.pivotArmLeft = pivotArmLeft;
        this.pivotArmRight = pivotArmRight;
        //this.tSensor = tSensor;
        this.telemetry = telemetry;
    }
    public void claw_Grabber(Servo clawGrab, int clawDirection) {
        //Claw Open = 0
        if (clawDirection == CLAW_OPEN) {
            clawGrab.setPosition(CLAW_GRAB_MIN);
            //Claw closed = 1
        } else if (clawDirection == CLAW_CLOSE) {
            clawGrab.setPosition(CLAW_GRAB_MAX);
        }
    }
    public void claw_moveArm(Servo pivotArmLeft, int clawDirection) {
        double newPos;
        double currPos;
        currPos = pivotArmLeft.getPosition();

        if (clawDirection == MOVE_UP) {
            if (currPos <=ARM_MID_POINT){
                newPos = currPos - ARM_LARGE_INCREMENT; // 0.01
            }
            else
            {
                newPos = currPos - ARM_SMALL_INCREMENT; // 0.01
            }
            if (newPos >ARM_MIN){
                pivotArmLeft.setPosition(newPos);
            }
        } else if (clawDirection == MOVE_DOWN) {
            if (currPos <=ARM_MID_POINT){
                newPos = currPos + ARM_LARGE_INCREMENT;
            }
            else
            {
                newPos = currPos + ARM_SMALL_INCREMENT;
            }

            if (newPos < ARM_MAX) {
                pivotArmLeft.setPosition(newPos);
            }
        }
        telemetry.addData("Arm position 3 ", pivotArmLeft.getPosition());
        telemetry.update();
    }

    public void claw_movePivot(Servo pivotServo, int pivotDirection) {
        double newPos;
        double currPos;
        currPos = pivotArmLeft.getPosition();

        if (pivotDirection == MOVE_UP) {
            if (currPos <=ARM_MID_POINT){
                newPos = currPos - ARM_LARGE_INCREMENT; // 0.01
            }
            else
            {
                newPos = currPos - ARM_SMALL_INCREMENT; // 0.01
            }
            if (newPos >ARM_MIN){
                pivotArmLeft.setPosition(newPos);
            }
        } else if (pivotDirection == MOVE_DOWN) {
            if (currPos <=ARM_MID_POINT){
                newPos = currPos + ARM_LARGE_INCREMENT;
            }
            else
            {
                newPos = currPos + ARM_SMALL_INCREMENT;
            }

            if (newPos < ARM_MAX) {
                pivotServo.setPosition(newPos);
            }
        }
        telemetry.addData("Arm position 3 ", pivotServo.getPosition());
        telemetry.update();
    }


    public void claw_Rotate(Servo clawSpin, int clawDirection) {
        double newPos;
        double currPos;
        currPos =clawSpin.getPosition();
        //Claw spin left
        if (clawDirection == CLAW_SPIN_LEFT) {
                newPos = currPos +0.01;
            if (newPos < CLAW_SPIN_MIN) {
                clawSpin.setPosition(newPos);
            }
            //Claw spin right
        } else if (clawDirection == CLAW_SPIN_RIGHT) {
            newPos = currPos -0.01;
            if (newPos > CLAW_SPIN_MAX) {
                clawSpin.setPosition(newPos);
            }
        }
    }
    //public void sliderMoveToPosition(DcMotor sliderMotor, int sliderDirection, TouchSensor tSensor) {
    public void sliderMoveToPosition(DcMotor sliderMotorLeft,DcMotor sliderMotorRight, int sliderDirection) {
        double motorSpeed = 0;
        int currPos;
        int newPos;

        currPos = sliderMotorLeft.getCurrentPosition();
        // sliderDirection = 0 - Down; 1 - Up
        if (sliderDirection == MOVE_DOWN) {
            // Set the target position for the motor (encoder position) - LOWEST POINT
            sliderMotorLeft.setTargetPosition(SLIDE_MIN_HEIGHT);
            sliderMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sliderMotorRight.setTargetPosition(SLIDE_MIN_HEIGHT);
            sliderMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motorSpeed = -SLIDER_SPEED;
            // Set the motor power to move towards the target
            sliderMotorLeft.setPower(motorSpeed);
            sliderMotorRight.setPower(motorSpeed);

        } else if (sliderDirection == MOVE_UP) {
            // Set the target position for the motor (encoder position) - HIGHEST POINT
            sliderMotorLeft.setTargetPosition(SLIDE_MAX_HEIGHT);
            sliderMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sliderMotorRight.setTargetPosition(SLIDE_MAX_HEIGHT);
            sliderMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorSpeed = SLIDER_SPEED;
            sliderMotorLeft.setPower(motorSpeed);
            sliderMotorRight.setPower(motorSpeed);
        }
        telemetry.addData("Slider LEFT position before while loop ", sliderMotorLeft.getCurrentPosition());
        telemetry.addData("Slider RIGHT position before while loop ", sliderMotorRight.getCurrentPosition());
        telemetry.update();

        runtime.reset();
        while (sliderMotorLeft.isBusy() || sliderMotorRight.isBusy()) {
            // This loop will wait until the motor reaches the target position
            // You can also add other logic here, like displaying telemetry data
            // telemetry.addData("Slider", "Moving to target...");
            //  telemetry.update();

            if (runtime.milliseconds() >= SLIDE_TIMEOUT) {
                telemetry.addData("Timeout", "Motor did not reach the target position within the timeout period.");
                telemetry.update();
                sliderMotorLeft.setPower(MOTOR_STOP);  // Stop the motor if timeout occurs
                sliderMotorRight.setPower(MOTOR_STOP);
                break;  // Exit the loop if timeout

           /* if (tSensor.isPressed() && sliderDirection == MOVE_DOWN) {
                telemetry.addData("Switch 2 ", "Pressed");
                telemetry.addData("Slider position before reset", sliderMotor.getCurrentPosition());
                telemetry.update();
                //Slider_stop(sliderMotor);

                sliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                sliderMotor.setTargetPosition(SLIDE_MIN_HEIGHT);
                sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sliderMotor.setPower(motorSpeed);

                */

            }
        }
    }
    //public void sliderHangSpecimen(DcMotor sliderMotor, int sliderDirection, TouchSensor tSensor) {
    public void sliderHangSpecimen(DcMotor sliderMotorLeft, DcMotor sliderMotorRight, int sliderDirection) {
        double motorSpeed = 0;
        int currPos;
        int newPos;

        currPos = sliderMotorLeft.getCurrentPosition();
        // sliderDirection = 0 - Down; 1 - Up
        if (sliderDirection == MOVE_DOWN) {

            newPos = currPos - 100;
            if (newPos >= SLIDE_MIN_HEIGHT_HANG) {
                // Set the target position for the motor (encoder position) - LOWEST POINT
                sliderMotorLeft.setTargetPosition(newPos);
                sliderMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sliderMotorRight.setTargetPosition(newPos);
                sliderMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorSpeed = -SLIDER_SPEED_HANG;
                // Set the motor power to move towards the target
                sliderMotorLeft.setPower(motorSpeed);
                sliderMotorRight.setPower(motorSpeed);
            }
        } else if (sliderDirection == MOVE_UP) {
            // Set the target position for the motor (encoder position) - HIGHEST POINT
            sliderMotorLeft.setTargetPosition(SLIDE_MAX_HEIGHT_HANG);
            sliderMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sliderMotorRight.setTargetPosition(SLIDE_MAX_HEIGHT_HANG);
            sliderMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorSpeed = SLIDER_SPEED_HANG;
            sliderMotorLeft.setPower(motorSpeed);
            sliderMotorRight.setPower(motorSpeed);
        }
        telemetry.addData("Slider LEFT position before while loop ", sliderMotorLeft.getCurrentPosition());
        telemetry.addData("Slider RIGHT position before while loop ", sliderMotorLeft.getCurrentPosition());
        telemetry.update();

        runtime.reset();

        while (sliderMotorLeft.isBusy() || sliderMotorRight.isBusy() ) {
            // This loop will wait until the motor reaches the target position
            // You can also add other logic here, like displaying telemetry data
            // telemetry.addData("Slider", "Moving to target...");
            //  telemetry.update();

            if (runtime.milliseconds() >= SLIDE_HANG_TIMEOUT) {
                telemetry.addData("Timeout", "Motor did not reach the target position within the timeout period.");
                telemetry.update();
                sliderMotorLeft.setPower(MOTOR_STOP);  // Stop the motor if timeout occurs
                sliderMotorRight.setPower(MOTOR_STOP);
                break;  // Exit the loop if timeout
            }
        }
    }
    // Method to stop the motor (just in case)
    public void Slider_stop(DcMotor sliderMotorLeft,DcMotor sliderMotorRight) {
        sliderMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sliderMotorLeft.setPower(MOTOR_STOP);
        sliderMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sliderMotorRight.setPower(MOTOR_STOP);
    }
    // Method to check if the slider motor has reached the target position
    public boolean isAtTarget() {
        return (!sliderMotorLeft.isBusy() || !sliderMotorRight.isBusy()); // Return true when motor is not busy (target reached)
    }
}