package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="FullAuto", group="Linear OpMode")
public class FullAuto extends LinearOpMode {
    // Initialize motor variables
    ElapsedTime runtime = new ElapsedTime();
    
    DcMotor leftDrive;
    DcMotor rightDrive;
    DcMotor flywheelRight;
    DcMotor flywheelLeft;

    @Override
    public void runOpMode() {
        
        //Assign hardware map to variables
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        
        // Set the motor directions
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        if (opModeIsActive()) {
            telemetry.addData("Status", "Running");
                telemetry.update();

            runtime.reset();

            // run until the end of the match (driver presses STOP)
            while (opModeIsActive() && runtime.seconds() <= 1) {
                leftDrive.setPower(0.25);
                rightDrive.setPower(0.25);
            }

            while (opModeIsActive() && runtime.seconds() > 1) {
                leftDrive.setPower(0);
                rightDrive.setPower(0);
            }
        }
    }
}
