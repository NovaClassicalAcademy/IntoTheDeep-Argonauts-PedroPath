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
    private DcMotor sliderMotor;
    private TouchSensor tSensor;
    private Telemetry telemetry;
    private DcMotorEx coolMotor;
    ElapsedTime runtime = new ElapsedTime();  // Timer for timeout checking
    public Claw_Movements(Servo ClawGrabber, DcMotor sliderMotor,Servo clawSpin, Servo clawArm,TouchSensor tSensor,Telemetry telemetry) {
        this.clawGrab = ClawGrabber;
        this.sliderMotor = sliderMotor;
        this.clawSpin = clawSpin;
        this.clawArm = clawArm;
        this.tSensor = tSensor;
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
    public void claw_moveArm(Servo clawArm, int clawDirection) {
        double newPos;
        double currPos;
        currPos = clawArm.getPosition();

        if (clawDirection == MOVE_UP) {
            if (currPos <=ARM_MID_POINT){
                newPos = currPos - ARM_LARGE_INCREMENT; // 0.01
            }
            else
            {
                newPos = currPos - ARM_SMALL_INCREMENT; // 0.01
            }
            if (newPos >ARM_MIN){
                clawArm.setPosition(newPos);
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
                clawArm.setPosition(newPos);
            }
        }
        telemetry.addData("Arm position 3 ", clawArm.getPosition());
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
    public void sliderMoveToPosition(DcMotor sliderMotor, int sliderDirection, TouchSensor tSensor) {
        double motorSpeed = 0;
        int currPos;
        int newPos;

        currPos = sliderMotor.getCurrentPosition();
        // sliderDirection = 0 - Down; 1 - Up
        if (sliderDirection == MOVE_DOWN) {
            // Set the target position for the motor (encoder position) - LOWEST POINT
            sliderMotor.setTargetPosition(SLIDE_MIN_HEIGHT);
            sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorSpeed = -SLIDER_SPEED;
            // Set the motor power to move towards the target
            sliderMotor.setPower(motorSpeed);

        } else if (sliderDirection == MOVE_UP) {
            // Set the target position for the motor (encoder position) - HIGHEST POINT
            sliderMotor.setTargetPosition(SLIDE_MAX_HEIGHT);
            sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorSpeed = SLIDER_SPEED;
            sliderMotor.setPower(motorSpeed);
        }
        telemetry.addData("Slider position before while loop ", sliderMotor.getCurrentPosition());
        telemetry.update();

        runtime.reset();
        while (sliderMotor.isBusy()) {
            // This loop will wait until the motor reaches the target position
            // You can also add other logic here, like displaying telemetry data
            // telemetry.addData("Slider", "Moving to target...");
            //  telemetry.update();

            if (runtime.milliseconds() >= SLIDE_TIMEOUT) {
                telemetry.addData("Timeout", "Motor did not reach the target position within the timeout period.");
                telemetry.update();
                sliderMotor.setPower(0);  // Stop the motor if timeout occurs
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
    public void sliderHangSpecimen(DcMotor sliderMotor, int sliderDirection, TouchSensor tSensor) {
        double motorSpeed = 0;
        int currPos;
        int newPos;

        currPos = sliderMotor.getCurrentPosition();
        // sliderDirection = 0 - Down; 1 - Up
        if (sliderDirection == MOVE_DOWN) {

            newPos = currPos - 100;
            if (newPos >= SLIDE_MIN_HEIGHT_HANG) {
                // Set the target position for the motor (encoder position) - LOWEST POINT
                sliderMotor.setTargetPosition(newPos);
                sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorSpeed = -SLIDER_SPEED_HANG;
                // Set the motor power to move towards the target
                sliderMotor.setPower(motorSpeed);
            }


        } else if (sliderDirection == MOVE_UP) {
            // Set the target position for the motor (encoder position) - HIGHEST POINT
            sliderMotor.setTargetPosition(SLIDE_MAX_HEIGHT_HANG);
            sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorSpeed = SLIDER_SPEED_HANG;
            sliderMotor.setPower(motorSpeed);
        }
        telemetry.addData("Slider position before while loop ", sliderMotor.getCurrentPosition());
        telemetry.update();

        runtime.reset();

        while (sliderMotor.isBusy()) {
            // This loop will wait until the motor reaches the target position
            // You can also add other logic here, like displaying telemetry data
            // telemetry.addData("Slider", "Moving to target...");
            //  telemetry.update();

            if (runtime.milliseconds() >= SLIDE_HANG_TIMEOUT) {
                telemetry.addData("Timeout", "Motor did not reach the target position within the timeout period.");
                telemetry.update();
                sliderMotor.setPower(0);  // Stop the motor if timeout occurs
                break;  // Exit the loop if timeout
            }


        /*
        while (sliderMotor.isBusy()) {
            // This loop will wait until the motor reaches the target position
            // You can also add other logic here, like displaying telemetry data
            // telemetry.addData("Slider", "Moving to target...");
            //  telemetry.update();
            if (tSensor.isPressed() && sliderDirection == MOVE_DOWN) {
                telemetry.addData("Switch 2 ", "Pressed");
                telemetry.addData("Slider position before reset", sliderMotor.getCurrentPosition());
                telemetry.update();
                //Slider_stop(sliderMotor);

                sliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                sliderMotor.setTargetPosition(SLIDE_MIN_HEIGHT);
                sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sliderMotor.setPower(motorSpeed);


                telemetry.addData("Slider position after reset ", sliderMotor.getCurrentPosition());
                telemetry.update();
            }
        }
        */
            }
    }
    // Method to stop the motor (just in case)
    public void Slider_stop(DcMotor sliderMotor) {
        sliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sliderMotor.setPower(0);
    }
    // Method to check if the slider motor has reached the target position
    public boolean isAtTarget() {
        return !sliderMotor.isBusy(); // Return true when motor is not busy (target reached)
    }
}