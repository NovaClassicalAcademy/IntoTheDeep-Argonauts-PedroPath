package org.firstinspires.ftc.teamcode;// Importing necessary libraries
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDMotor {

    private DcMotor motor;  // The motor we're controlling
    private double kP = 2.5; // Proportional constant
    private double kI = 0.8;  // Integral constant
    private double kD = 1.2;  // Derivative constant

    private double targetPosition = 0.0; // Target position for the motor
    private double lastError = 0.0;
    private double integral = 0.0;
    private ElapsedTime runtime = new ElapsedTime();

    // Constructor to initialize the motor and PID constants
    public PIDMotor(HardwareMap hardwareMap, String motorName) {
        motor = hardwareMap.get(DcMotor.class, motorName);
    }

    // Method to set target position for the motor
    public void setTargetPosition(double position) {
        targetPosition = position;
        motor.setTargetPosition((int) targetPosition);
    }
    public void resetMotor(DcMotor motor ) {

        motor.resetDeviceConfigurationForOpMode();
    }
    // Method to update the motor speed based on the PID loop
    public void updateMotor() {
        double currentPosition = motor.getCurrentPosition();
        double error = targetPosition - currentPosition;

        // Proportional term
        double P = kP * error;

        // Integral term
        integral += error * runtime.seconds();
        double I = kI * integral;

        // Derivative term
        double D = kD * (error - lastError) / runtime.seconds();

        // Combine the terms
        double output = P + I + D;

        // Set the motor power
        motor.setPower(output);

        // Store the current error for the next loop
        lastError = error;
        runtime.reset(); // Reset the timer for next loop
    }

    // Optional: Method to stop the motor
    public void stopMotor() {
        motor.setPower(0);
    }


}
