package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;
import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="RedFarFullAuto", group="Linear OpMode")
public class RedFarFullAuto extends LinearOpMode {
    // Variables that need to be adjusted based on testing
    float minimumMoveSpeed = 600; // To make sure the bot doesn't get stuck while providing too little power when slowing down
    int primaryDistance = 150; // Distance to travel to get the bot to the shooting point in cm
    double primaryTimeSeconds = 6; // Time in seconds that the bot needs to take to get to the shooting point
    double rotateDeg = 45; // Degrees that the bot needs to rotate to aim at the goal
    int secondaryDistance = 0; // Distance the bot needs to travel to get to a "safe" position in cm
    double secondaryTimeSeconds = 0; // How long the bot needs to take to get to that "safe position"
    float wheelDiameter = 63.5f; // Wheel diamater to calculate how many ticks are needed to move the bot (in cm)h
    double degDeadzone = 5; // Rotate the bot to within x degrees
    double turningSpeed = 0.25; // Speed at which the robot will rotate (0-1)
    int ballDistance = 20; // Distance the lift and or the rubberband wheel need to turn to advance it one space in ticks of the corehex motor (4 ticks per rotation)


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

    public void moveTo(int distanceCm, double seconds) {

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int ticks = (int)(15 * 28 * (distanceCm / (wheelDiameter * PI)));

        leftDrive.setTargetPosition(-ticks);
        rightDrive.setTargetPosition(-ticks);

        runtime.reset();
        seconds /= 3.0;

        double maxSpeed = abs(ticks) / (2.0 * seconds);

        while (opModeIsActive() &&
                (leftDrive.isBusy() || rightDrive.isBusy())) {

            double t = runtime.seconds();
            double velocity;

            if (t < seconds) {
                velocity = (t / seconds) * (maxSpeed - minimumMoveSpeed) + minimumMoveSpeed;
            } else if (t < 2 * seconds) {
                velocity = maxSpeed;
            } else {
                velocity = ((3 * seconds - t) / seconds) * (maxSpeed - minimumMoveSpeed) + minimumMoveSpeed;
            }

            velocity = Math.max(velocity, minimumMoveSpeed);

            leftDrive.setVelocity(velocity);
            rightDrive.setVelocity(velocity);

            if (t > 9 * seconds) break; // failsafe
        }

        leftDrive.setVelocity(0);
        rightDrive.setVelocity(0);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Rotate bot function
    public void rotateToRelativeYaw(double rotateDeg) {

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        IMU.resetYaw();

        double startYaw = IMU.getRobotYawPitchRollAngles().getYaw();
        double targetYaw = startYaw + rotateDeg;

        while (opModeIsActive()) {

            double currentYaw = IMU.getRobotYawPitchRollAngles().getYaw();
            double error = targetYaw - currentYaw;

            if (abs(error) <= degDeadzone) break;

            double speed = 800 * turningSpeed;

            if (error > 0) {
                leftDrive.setVelocity(-speed);
                rightDrive.setVelocity(speed);
            } else {
                leftDrive.setVelocity(speed);
                rightDrive.setVelocity(-speed);
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
            //rotateToRelativeYaw(rotateDeg);
            //--------------------------------------------------------------------------------------
            // SHOOTING TIME!!!

            flywheelLeft.setVelocity(-2800);
            flywheelRight.setVelocity(-2800);
            double time = runtime.seconds();
            double failsafeTime = 8.0d;
            while (!((flywheelLeft.getVelocity() + flywheelRight.getVelocity())/2 < -2600)) {
                sleep(10);
                if (failsafeTime + time - runtime.seconds() < 0) break;
            }

            for (int i = 0; i < 2; i++) {
                time = runtime.seconds();
                while (!((flywheelLeft.getVelocity() + flywheelRight.getVelocity())/2 < -2600)) {
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
