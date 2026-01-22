package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="ManualDrive", group="Linear OpMode")
public class ManualDrive extends LinearOpMode {

    int rumbleThreshold = 2000; // minimum rmp needed for rumble to start
    // Initialize motor variables
    Servo ballControl;
    DcMotor leftDrive;
    DcMotor rightDrive;
    DcMotorEx flywheelRight;
    DcMotorEx flywheelLeft;
    DcMotor elastiekWiel;
    DcMotor lift;
    double rpm;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Assign hardware map to variables
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        flywheelRight = hardwareMap.get(DcMotorEx.class, "flywheel_right");
        flywheelLeft = hardwareMap.get(DcMotorEx.class, "flywheel_left");
        elastiekWiel = hardwareMap.get(DcMotor.class, "elastiek_wiel");
        lift = hardwareMap.get(DcMotor.class, "lift");
        
        //ballControl = hardwareMap.get(Servo.class, "ball_control");

        
        // Set the motor directions
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        flywheelLeft.setDirection(DcMotor.Direction.REVERSE);
        flywheelRight.setDirection(DcMotor.Direction.FORWARD);
        elastiekWiel.setDirection(DcMotor.Direction.FORWARD);
        lift.setDirection(DcMotor.Direction.REVERSE);
        
        // Initialize variables
        double Power;
        double Direction;
        boolean SlowMode = false;
        double PowerL = 0;
        double PowerR = 0;
        
        // Wait for the game to start (driver presses START)
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            rpm = (flywheelLeft.getVelocity() + flywheelRight.getVelocity())/2;
            if (rpm < -rumbleThreshold){
                gamepad1.rumble(1000);
                
            } else {
                gamepad1.stopRumble();
            }
            
            Power = -gamepad1.left_stick_y; // Y axis on sticks are inverted
            Direction = gamepad1.right_stick_x;
            
            // SlowMode Control
            if (gamepad1.right_trigger > 0.25) {
                SlowMode = true;
            } else {
                SlowMode = false; 
            }
            
            // Set motor power
            PowerL = Power;
            PowerR = Power;

            // Apply turning direction
            if (Direction > 0) {
                if (Power > 0) {
                    PowerL = PowerL - Direction;
                } else if (Power < 0) {
                    PowerL = PowerL + Direction;   
                }
            } else if (Direction < 0) {
                if (Power > 0) {
                    PowerR = PowerR + Direction;
                } else if (Power < 0) {
                    PowerR = PowerR - Direction;
                }
            }
            
            // Makes the bot able to turn while standing still
            if (Power == 0 && Direction != 0){
                PowerL = Direction * 0.66 * -1;
                PowerR = Direction * 0.66;
            }

            // Apply SlowMode multiplier
            if (SlowMode){
                    PowerL = PowerL * 0.5;
                    PowerR = PowerR * 0.5;
            }

            // Send calculated power to wheels
            leftDrive.setPower(PowerL);
            rightDrive.setPower(PowerR);
            
            //Flywheel Control
            if (gamepad1.left_trigger > 0.5){
                flywheelLeft.setPower(1);
                flywheelRight.setPower(1);
            } else {
                flywheelLeft.setPower(0);
                flywheelRight.setPower(0);
            }

            //if (gamepad1.b) {
            //    ballControl.setPosition(0.3);
            //} else {
            //    ballControl.setPosition(0.10);
            //}
            
            if (gamepad1.a) {
                elastiekWiel.setPower(1);
            } else {
                elastiekWiel.setPower(0);
            }
            
            if (gamepad1.b) {
                lift.setPower(1);
            } else {
                lift.setPower(0);
            }
            
            // Debug
            telemetry.addData("PowerL", PowerL);
            telemetry.addData("PowerR", PowerR);
            //telemetry.addData("Servo Position", ballControl.getPosition());
            telemetry.addData("rpmL", flywheelLeft.getVelocity());
            telemetry.addData("rpmR", flywheelRight.getVelocity());
            //telemetry.addData("LY", gamepad1.left_stick_y);
            //telemetry.addData("LX", gamepad1.left_stick_x);
            //telemetry.addData("RY", gamepad1.right_stick_y);
            //telemetry.addData("RX", gamepad1.right_stick_x);
        
            telemetry.update();
        }
    }
}
