package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.Argo_Configuration.BACK_LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.BACK_RIGHT_MOTOR;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.FRONT_LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.FRONT_RIGHT_MOTOR;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.MAX_MOTOR_SPEED;
import static org.firstinspires.ftc.teamcode.Argo_Configuration.MIN_MOTOR_SPEED;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

//*TELEMETRY- Set power so that it will move but do sleep 2 sec so the servo moves*//

@TeleOp(name="Robot Test PID", group="Teleop")
//@Disabled
public class TestPID extends OpMode
{
    // Declare OpMode members.
    private DcMotor sliderMotor;
    private PIDMotor pid;
    private String motorname;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        //sliderMotor = hardwareMap.get(DcMotor.class, motorName);
        motorname ="sliderMotor";
        pid =new PIDMotor(hardwareMap,motorname);
       }
    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {

       pid.setTargetPosition(500);
       pid.updateMotor();

       }
    /*
     * Code to run ONCE after the driver hits STOP
     */


}
