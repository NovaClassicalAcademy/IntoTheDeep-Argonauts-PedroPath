package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
public class Argo_Movements {
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;

    // Constructor to initialize motors
    public Argo_Movements(DcMotor frontLeftMotor,DcMotor backLeftMotor,DcMotor frontRightMotor,DcMotor backRightMotor) {
        this.frontLeftMotor = frontLeftMotor;
        this.frontRightMotor = frontRightMotor;
        this.backLeftMotor = backLeftMotor;
        this.backRightMotor = backRightMotor;
    }

    //Robot centric Move
    public void moveRobot(double frontLeftPower,double frontRightPower, double backLeftPower, double backRightPower) {

        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
    }

    // Move forward
    public void moveForward(double speed) {

        frontLeftMotor.setPower(speed);
        frontRightMotor.setPower(speed);
        backLeftMotor.setPower(speed);
        backRightMotor.setPower(speed);
    }

    // Move backward
    public void moveBackward(double speed) {

        frontLeftMotor.setPower(-speed);
        frontRightMotor.setPower(-speed);
        backLeftMotor.setPower(-speed);
        backRightMotor.setPower(-speed);
    }

    // Turn left
    public void turnLeft(double speed) {
        frontLeftMotor.setPower(-speed);
        backLeftMotor.setPower(-speed);
        frontRightMotor.setPower(speed);
        backRightMotor.setPower(speed);
    }

    // Turn right
    public void turnRight(double speed) {
        frontLeftMotor.setPower(speed);
        backLeftMotor.setPower(speed);
        frontRightMotor.setPower(-speed);
        backRightMotor.setPower(-speed);
    }

    // Stop all motors
    public void stopMotors() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }




}
