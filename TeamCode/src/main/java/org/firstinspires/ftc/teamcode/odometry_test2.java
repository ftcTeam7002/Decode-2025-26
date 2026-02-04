package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Odo test2 CLEAN", group = "Robot")
public class odometry_test2 extends LinearOpMode {

    // Drive
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    public DcMotor odometerRight;
    public DcMotor odometerAux;
    // Mechanisms
    private DcMotorEx intakeWheels;
    private DcMotorEx launcherLeft;
    private Servo kicker;

    // Sensors
    private IMU imu;

    // Constants
    static final double FORWARD_SPEED = 0.8;
    public static final double TICKS_PER_INCH = 1058.34;

    @Override
    public void runOpMode() {

        // -------------------- Hardware Map --------------------
        imu = hardwareMap.get(IMU.class, "imu");

        frontLeftDrive  = hardwareMap.get(DcMotor.class, "FL");
        frontRightDrive = hardwareMap.get(DcMotor.class, "FR");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "BL");
        backRightDrive  = hardwareMap.get(DcMotor.class, "BR");

        intakeWheels = hardwareMap.get(DcMotorEx.class, "IW");
        launcherLeft = hardwareMap.get(DcMotorEx.class, "LL");
        kicker = hardwareMap.get(Servo.class, "kicker");


        odometerRight = frontRightDrive;
        frontRightDrive.getCurrentPosition();
        frontRightDrive.getCurrentPosition();
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        odometerAux = frontLeftDrive;
        frontLeftDrive.getCurrentPosition();
        frontLeftDrive.getCurrentPosition();
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // -------------------- Motor Direction --------------------
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        intakeWheels.setDirection(DcMotor.Direction.REVERSE);

        // -------------------- Encoder Setup --------------------
        resetDriveEncoders();

        telemetry.addLine("Initialized — waiting for start");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        // -------------------- AUTO SEQUENCE --------------------
        driveForwardEncoder((int)(5 * TICKS_PER_INCH), FORWARD_SPEED);
        launch();

        telemetry.addLine("AUTO COMPLETE");
        telemetry.update();
        sleep(1000);
    }

    // ==========================================================
    // ===================== DRIVE METHOD =======================
    // ==========================================================
    private void driveForwardEncoder(int ticks, double power) {

        resetDriveEncoders();

        frontLeftDrive.setTargetPosition(ticks);
        frontRightDrive.setTargetPosition(ticks);
        backLeftDrive.setTargetPosition(ticks);
        backRightDrive.setTargetPosition(ticks);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(power);
        backLeftDrive.setPower(power);
        backRightDrive.setPower(power);

        while (opModeIsActive()
                && frontLeftDrive.isBusy()
                && frontRightDrive.isBusy()) {

            telemetry.addData("FL", frontLeftDrive.getCurrentPosition());
            telemetry.addData("FR", frontRightDrive.getCurrentPosition());
            telemetry.update();
        }

        stopDrive();

        // Return to normal mode
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // ==========================================================
    // ===================== LAUNCH METHOD ======================
    // ==========================================================
    private void launch() {

        telemetry.addLine("LAUNCHING");
        telemetry.update();

        launcherLeft.setVelocity(2500);
        sleep(1500); // spin-up

        kicker.setPosition(0.2);
        sleep(300);

        intakeWheels.setVelocity(1500);
        sleep(400);

        intakeWheels.setVelocity(0);
        sleep(300);

        launcherLeft.setVelocity(0);
    }

    // ==========================================================
    // ===================== HELPERS ============================
    // ==========================================================
    private void resetDriveEncoders() {
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void stopDrive() {
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }
}