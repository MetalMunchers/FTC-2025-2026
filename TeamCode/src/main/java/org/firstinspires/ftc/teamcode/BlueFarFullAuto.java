package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.cos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name="BlueFar", group="Linear OpMode")
public class BlueFarFullAuto extends LinearOpMode {
    // Variables that need to be adjusted based on testing
    float minimumMoveSpeed = 0; // To make sure the bot doesn't get stuck while providing too little power when slowing down
    int primaryDistance = 0; // Distance to travel to get the bot to the shooting point in cm
    double primaryTimeSeconds = 0; // Time in seconds that the bot needs to take to get to the shooting point
    double rotateDeg = 0; // Degrees that the bot needs to rotate to aim at the goal
    int secondaryDistance = 0; // Distance the bot needs to travel to get to a "safe" position in cm
    double secondaryTimeSeconds = 0; // How long the bot needs to take to get to that "safe position"
    float wheelDiameter = 0; // Wheel diamater to calculate how many ticks are needed to move the bot (in cm)
    double degDeadzone = 0; // Rotate the bot to within x degrees
    double turningSpeed = 0; // Speed at which the robot will rotate (0-1)
    int ballDistance = 0; // Distance the lift and or the rubberband wheel need to turn to advance it one space in ticks of the corehex motor (4 ticks per rotation)


    // Reset and make able to read time
    ElapsedTime runtime = new ElapsedTime();

    // Initialize motor variables
    DcMotorEx leftDrive;
    DcMotorEx rightDrive;
    DcMotorEx flywheelRight;
    DcMotorEx flywheelLeft;
    DcMotorEx lift;
    DcMotorEx elastiekWiel;

    IMU IMU;


    // Move bot to position forwards or backwards function

    public void moveTo(int distance, double seconds){
        // Set primary position target
        leftDrive.setTargetPosition( (int) (-28*(distance/(wheelDiameter/2)*2*PI)) );
        rightDrive.setTargetPosition( (int) (-28*(distance/(wheelDiameter/2)*2*PI)) );

        // Make motors move so bot gets to primary position
        while ((leftDrive.getCurrentPosition() <= leftDrive.getTargetPosition()) || (rightDrive.getCurrentPosition() <= rightDrive.getTargetPosition())) {

            // Make the motors ramp up and slow down to get to the primary position
            seconds = (1.0f / 3.0f) * seconds;
            if (seconds <= runtime.seconds()) {

                // Use cosine to ramp up motors to point 1 of track to primairy position
                leftDrive.setVelocity((int) (-2800 * (-cos(0.5 * (seconds / runtime.seconds() * PI)) + 1)));
                rightDrive.setVelocity((int) (-2800 * (-cos(0.5 * (seconds / runtime.seconds() * PI)) + 1)));

            } else if ( seconds > runtime.seconds() && (2 * seconds) <= runtime.seconds()) {

                // Make motors move to point 2 of track to primary position at a constant speed
                leftDrive.setVelocity(-2800);
                rightDrive.setVelocity(-2800);

            } else if ( (2 * seconds) > runtime.seconds() && (3* seconds) <= runtime.seconds()) {

                // Use cosine to ramp up motors to point 3 of track to primairy position
                leftDrive.setVelocity((int) (-2800 * (-cos(0.5 * (seconds / runtime.seconds() * PI - (0.5 * PI))) + 1)));
                rightDrive.setVelocity((int) (-2800 * (-cos(0.5 * (seconds / runtime.seconds() * PI - (0.5 * PI))) + 1)));

                // Make sure the bot doesn't get stuck from giving too little power to the motors
                if (((-cos(0.5 * (seconds / runtime.seconds() * PI - (0.5 * PI))) + 1)) < minimumMoveSpeed) {
                    leftDrive.setVelocity(-2800 * minimumMoveSpeed);
                    rightDrive.setVelocity(-2800 * minimumMoveSpeed);
                }
            }

            // make sure the bot exits the loop no mather what it's position is at
            if (runtime.seconds() > 3.0f * primaryTimeSeconds ) break;
        }
    }
    // Rotate bot function
    public void rotateToRelativeYaw(double rotateDeg)
    {
        // Setup IMU for use
        IMU.resetYaw(); // Try to minimize Yaw drift

        YawPitchRollAngles YPR = IMU.getRobotYawPitchRollAngles();

        // Set the target Yaw
        double targetYaw = (YPR.getYaw() + rotateDeg);
        double currentYaw = YPR.getYaw();

        // Rotate the bot to within degDeadzone deg of the target rotation
        while (abs(currentYaw - targetYaw) >= degDeadzone) {
            currentYaw = YPR.getYaw(); // Update the currentYaw

            if (currentYaw > YPR.getYaw()) {
                leftDrive.setVelocity(-2800.0d * turningSpeed);
                rightDrive.setVelocity(2800.0d * turningSpeed);
            } else {
                leftDrive.setVelocity(600.0d * turningSpeed);
                rightDrive.setVelocity(-600.0d * turningSpeed);
            }
        }
        leftDrive.setVelocity(0);
        rightDrive.setVelocity(0);
    }


    @Override
    public void runOpMode() {
        // Make use of the IMU for rotational feedback
        IMU = hardwareMap.get(IMU.class, "IMU");

        //Assign hardware map to variables
        leftDrive  = hardwareMap.get(DcMotorEx.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotorEx.class, "right_drive");
        flywheelRight = hardwareMap.get(DcMotorEx.class, "flywheel_right");
        flywheelLeft = hardwareMap.get(DcMotorEx.class, "flywheel_left");
        elastiekWiel = hardwareMap.get(DcMotorEx.class, "elastiek_wiel");
        lift = hardwareMap.get(DcMotorEx.class, "lift");

        // Set the motor directions
        leftDrive.setDirection(DcMotorEx.Direction.REVERSE);
        rightDrive.setDirection(DcMotorEx.Direction.FORWARD);
        flywheelLeft.setDirection(DcMotorEx.Direction.REVERSE);
        flywheelRight.setDirection(DcMotorEx.Direction.FORWARD);
        elastiekWiel.setDirection(DcMotorEx.Direction.FORWARD);
        lift.setDirection(DcMotorEx.Direction.REVERSE);

        // Reset all motor encoders
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set all correct modes for motors
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        telemetry.addData("Status", "Initialized succesfully");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Reset IMU Yaw measurement
        IMU.resetYaw();

        if (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();
            runtime.reset();

            // Old FullAuto Code

            // run until the end of the match (driver presses STOP)
            //while (opModeIsActive() && runtime.seconds() <= 1) {
            //    leftDrive.setPower(0.25);
            //    rightDrive.setPower(0.25);
            //}
            //
            //while (opModeIsActive() && runtime.seconds() > 1) {
            //    leftDrive.setPower(0);
            //    rightDrive.setPower(0);
            //}

            //--------------------------------------------------------------------------------------
            // New FullAuto code
            moveTo(primaryDistance, primaryTimeSeconds);
            //--------------------------------------------------------------------------------------
            // Make bot rotate by rotateDeg degrees clockwise
            rotateToRelativeYaw(rotateDeg);
            //--------------------------------------------------------------------------------------
            // SHOOTING TIME!!!

            flywheelLeft.setVelocity(-2800);
            flywheelRight.setVelocity(-2800);
            double time = runtime.seconds();
            double failsafeTime = 8.0d;
            while (!((flywheelLeft.getVelocity() + flywheelRight.getVelocity())/2 < -2500)) {
                sleep(10);
                if (failsafeTime + time - runtime.seconds() < 0) break;
            }

            for (int i = 0; i < 2; i++) {
                time = runtime.seconds();
                while (!((flywheelLeft.getVelocity() + flywheelRight.getVelocity())/2 < -2500)) {
                    sleep(10);
                    if (3.0d + time - runtime.seconds() < 0) break;
                }
                lift.setTargetPosition(ballDistance);
                elastiekWiel.setTargetPosition(ballDistance);
                lift.setVelocity(500);
                elastiekWiel.setVelocity(500);
                ballDistance = ballDistance + ballDistance;
            }

            //--------------------------------------------------------------------------------------
            //return bot to safe spot
            rotateToRelativeYaw(-rotateDeg);
            moveTo(secondaryDistance, secondaryTimeSeconds);
        }
    }
}
