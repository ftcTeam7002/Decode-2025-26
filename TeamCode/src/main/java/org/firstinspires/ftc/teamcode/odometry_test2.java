package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

    @Autonomous(name = "Odo test2", group = "Robot")
    public class odometry_test2 extends LinearOpMode {

        private IMU imu;

        private DcMotor frontLeftDrive;
        private DcMotor frontRightDrive;
        private DcMotor backLeftDrive;
        private DcMotor backRightDrive;

        public DcMotor odometerLeft;
        public DcMotor odometerAux;

        private DcMotorEx launcherLeft;
        private Servo kicker;

        static final double FORWARD_SPEED = 0.6;

        // Your odometry constant
        public static final double TICKS_PER_INCH = 1058.34;

        @Override
        public void runOpMode() {

            imu = hardwareMap.get(IMU.class, "imu");
            RevHubOrientationOnRobot orientationOnRobot =
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                            RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                    );
            imu.initialize(new IMU.Parameters(orientationOnRobot));

            frontLeftDrive  = hardwareMap.get(DcMotor.class, "FL");
            frontRightDrive = hardwareMap.get(DcMotor.class, "FR");
            backLeftDrive   = hardwareMap.get(DcMotor.class, "BL");
            backRightDrive  = hardwareMap.get(DcMotor.class, "BR");

            odometerLeft = backLeftDrive;
            odometerAux  = frontRightDrive;

            frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            telemetry.addLine("Initialized — waiting for start");
            telemetry.update();

            waitForStart();

            imu.resetYaw();

            // -------------------------
            // MOVE #1 — 2 inches
            // -------------------------
            driveToTicks((int)(2 * TICKS_PER_INCH));

            // -------------------------
            // MOVE #2 — 4 inches
            // -------------------------
            driveToTicks((int)(4 * TICKS_PER_INCH));

            telemetry.addLine("COMPLETE");
            telemetry.update();
            sleep(1000);
        }

        // ==========================================================
        // Drives forward until odometry reaches target ticks
        // ==========================================================
        private void driveToTicks(int targetTicks) {

            // Reset odometry encoders
            odometerLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            odometerAux.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            odometerLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            odometerAux.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // drive
            frontLeftDrive.setPower(FORWARD_SPEED);
            frontRightDrive.setPower(-FORWARD_SPEED);
            backLeftDrive.setPower(FORWARD_SPEED);
            backRightDrive.setPower(-FORWARD_SPEED);





            while (opModeIsActive()
                    && Math.abs(odometerLeft.getCurrentPosition()) < targetTicks
                    && Math.abs(odometerAux.getCurrentPosition()) < targetTicks) {

                YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
                double yaw = angles.getYaw(AngleUnit.DEGREES);

                telemetry.addData("Yaw", yaw);
                telemetry.addData("Left Odo", odometerLeft.getCurrentPosition());
                telemetry.addData("Aux Odo", odometerAux.getCurrentPosition());
                telemetry.addData("Target", targetTicks);
                telemetry.update();
            }

            // Stop motors
            frontLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backLeftDrive.setPower(0);
            backRightDrive.setPower(0);

            sleep(200); // settle time
        }
    }
