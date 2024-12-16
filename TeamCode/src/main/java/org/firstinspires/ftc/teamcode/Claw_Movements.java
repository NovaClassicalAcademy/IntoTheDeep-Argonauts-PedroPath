package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import static com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.ARM_MAX;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.ARM_MIN;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.CLAW_GRAB_MAX;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.CLAW_GRAB_MIN;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.CLAW_SPIN_LEFT;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.CLAW_SPIN_MAX;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.CLAW_SPIN_MIN;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.CLAW_SPIN_RIGHT;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.MOVE_DOWN;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.MOVE_UP;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.SLIDER_SPEED;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.SLIDE_MAX_HEIGHT;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.SLIDE_MIN_HEIGHT;

public class Claw_Movements {
    private Servo clawGrab;
    private Servo clawSpin;
    private Servo clawArm;
    private DcMotor sliderMotor;


    public Claw_Movements(Servo ClawGrabber, DcMotor sliderMotor,Servo clawSpin, Servo clawArm) {
        this.clawGrab = ClawGrabber;
        this.sliderMotor = sliderMotor;
        this.clawSpin = clawSpin;
        this.clawArm = clawArm;
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
             newPos = currPos - 0.01;
            if (newPos >ARM_MIN){
                clawArm.setPosition(newPos);
            }
        } else if (clawDirection == MOVE_DOWN) {
            newPos = currPos + 0.01;
            if (newPos <ARM_MAX){
                clawArm.setPosition(newPos);
            }
        }
    }
    public void claw_Rotate(Servo clawSpin, int clawDirection) {
        double newPos;
        double currPos;
        currPos =clawSpin.getPosition();
        //Claw spin left
        if (clawDirection == CLAW_SPIN_LEFT) {
                newPos = currPos -0.01;
            if (newPos > CLAW_SPIN_MIN) {
                clawSpin.setPosition(newPos);
            }
            //Claw spin right
        } else if (clawDirection == CLAW_SPIN_RIGHT) {
            newPos = currPos +0.01;
            if (newPos < CLAW_SPIN_MAX) {
                clawSpin.setPosition(newPos);
            }
        }
    }

    public void sliderMoveToPosition(DcMotor sliderMotor, int sliderDirection) {
        double motorSpeed = 0;
        // sliderDirection = 0 - Down; 1 - Up

        if (sliderDirection == MOVE_DOWN) {
            // Set the target position for the motor (encoder position) - LOWEST POINT
            sliderMotor.setTargetPosition(SLIDE_MIN_HEIGHT);
            motorSpeed = -SLIDER_SPEED;
        } else if (sliderDirection == MOVE_UP) {
            // Set the target position for the motor (encoder position) - HIGHEST POINT
            sliderMotor.setTargetPosition(SLIDE_MAX_HEIGHT);
            motorSpeed = SLIDER_SPEED;
        }
        // Set the motor mode to run to the target position
        sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set the motor power to move towards the target
        sliderMotor.setPower(motorSpeed); // Full power, adjust as needed

        // Wait until the motor reaches the target position
        while (sliderMotor.isBusy()) {
            // This loop will wait until the motor reaches the target position
            // You can also add other logic here, like displaying telemetry data
            telemetry.addData("Slider", "Moving to target...");
            telemetry.update();
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