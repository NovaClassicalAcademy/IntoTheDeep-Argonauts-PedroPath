package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import static com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.CLAW_GRAB_MAX;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.CLAW_GRAB_MIN;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.MAX_SLIDER_SPEED;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.SLIDE_MAX_HEIGHT;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.SLIDE_MIN_HEIGHT;

public class Claw_Movements {
    private Servo clawGrab;
    private Servo clawSpin;
    private Servo clawHand;
    private DcMotor sliderMotor;


    public Claw_Movements(Servo ClawGrabber, DcMotor sliderMotor) {
        this.clawGrab = ClawGrabber;
        this.sliderMotor = sliderMotor;
    }


    public void claw_Grabber(Servo clawGrab, int clawDirection) {
        //Claw Open = 0
        if (clawDirection == 0) {
            clawGrab.setPosition(CLAW_GRAB_MIN);
            //Claw closed = 1
        } else if (clawDirection == 1) {
            clawGrab.setPosition(CLAW_GRAB_MAX);
        }
    }

    public void sliderMoveToPosition(DcMotor sliderMotor, int sliderDirection) {
        // sliderDirection = 0 - Down; 1 - Up
        sliderMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // Reset encoders and set to RUN_USING_ENCODER mode
        sliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (sliderDirection == 0) {
            // Set the target position for the motor (encoder position) - LOWEST POINT
            sliderMotor.setTargetPosition(SLIDE_MIN_HEIGHT);
        } else if (sliderDirection == 1) {
            // Set the target position for the motor (encoder position) - HIGHEST POINT
            sliderMotor.setTargetPosition(SLIDE_MAX_HEIGHT);
        }
        // Set the motor mode to run to the target position
        sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set the motor power to move towards the target
        sliderMotor.setPower(MAX_SLIDER_SPEED); // Full power, adjust as needed
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