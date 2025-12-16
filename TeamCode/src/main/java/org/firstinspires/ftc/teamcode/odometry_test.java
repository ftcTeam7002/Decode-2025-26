package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Odo test", group = "Robot")

public class odometry_test extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotorEx launcherLeft = null;
    // Encoder ||||||||||||||||||||||||||||||||||||||||||||||||||||||\\
    public DcMotor odometerLeft;
    public DcMotor odometerRight;
    public DcMotor odometerAux;
    // Kicker||||||||||||||||||||||||||||||||||||||||||||||||||||||||\\
    private Servo kicker = null;
    private ElapsedTime runtime = new ElapsedTime();

    static final double FORWARD_SPEED = 0.6;
    static final double REVERSE_SPEED = -0.6;

    // Ticks per inch (change for your wheel + encoder)
    public static final double TICKS_PER_INCH = 212.201591512;
    // 9.425 inches per revolution


    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        frontLeftDrive = hardwareMap.get(DcMotor.class, "FL");
        frontRightDrive = hardwareMap.get(DcMotor.class, "FR");
        backRightDrive = hardwareMap.get(DcMotor.class, "BL");



        backLeftDrive = hardwareMap.get(DcMotor.class, "BR");
        odometerAux = backLeftDrive;
        backLeftDrive.getCurrentPosition();
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        launcherLeft = hardwareMap.get(DcMotorEx.class, "LL");

        kicker = hardwareMap.get(Servo.class, "kicker");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct
        // drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Reset odometry encoder only
        // ---------------------
//odometerLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //      odometerRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odometerAux.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //  odometerRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // odometerLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odometerAux.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        telemetry.addLine("Initialized — waiting for start");
        telemetry.update();
        waitForStart();

        // Drive forward until odometry wheel reaches the target
        // ---------------------
        int targetTicks = (int) (12 * TICKS_PER_INCH);
                             // ^^inches

        // Start driving forward (simple open-loop power) main movement until while loop
        frontLeftDrive.setPower(REVERSE_SPEED);
        frontRightDrive.setPower(FORWARD_SPEED);
        backLeftDrive.setPower(FORWARD_SPEED);
        backRightDrive.setPower(REVERSE_SPEED);


        while (opModeIsActive() && Math.abs(odometerAux.getCurrentPosition()) < targetTicks) {

            telemetry.addData("Odometry Ticks", odometerAux.getCurrentPosition());
            //  telemetry.addData("Odometry Ticks", odometerRight.getCurrentPosition());
            // telemetry.addData("Odometry Ticks", odometerLeft.getCurrentPosition());
            telemetry.addData("Target Ticks", targetTicks);
            telemetry.update();
        }

        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backLeftDrive.setPower(0);

        telemetry.addLine("COMPLETE");
        telemetry.update();
        sleep(500);
        // Stop

    }
}