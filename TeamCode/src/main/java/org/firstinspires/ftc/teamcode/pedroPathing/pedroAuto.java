package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import static java.lang.Thread.sleep;

import com.pedropathing.geometry.BezierCurve;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "PedroAuto", group = "Examples")
public class pedroAuto extends OpMode {



    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private DcMotorEx intakeWheels;
    private DcMotorEx launcherLeft;
    private Servo kicker;

    private final Pose startPose = new Pose(20.8, 122, Math.toRadians(135)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(32.8, 111.2, Math.toRadians(135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup1Pose = new Pose(45, 84, Math.toRadians(0));// Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2Pose = new Pose(45, 60, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup3Pose = new Pose(45, 38, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose intake1Pose = new Pose (24,84,Math.toRadians(0));
    private final Pose intake2Pose = new Pose (24,60,Math.toRadians(0));
    private final Pose intake3Pose = new Pose (22, 38,Math.toRadians(0));
    private final Pose endPose = new Pose (23, 95, Math.toRadians(0));

    private Path scorePreload;

    //grab and score first line
    private PathChain grabPickup1;
    private PathChain pickupIntake1;
    private PathChain scorePickup1;

    // second line
    private PathChain grabPickup2;
    private PathChain pickupIntake2;
    private PathChain scorePickup2;

    // third line
    private PathChain grabPickup3;
    private PathChain pickupIntake3;
    private PathChain scorePickup3;

    // end
    private PathChain endChain;

    // ------------- SPUDGUN VELOCITY --------------
    public double currentVelocity;
    public double flyWheelVelocity = 1920;
    public double targetVelocity = 1700;
    public double F = 16;
    public double P = 60;

    public double error = targetVelocity - currentVelocity;


    ElapsedTime runTime = new ElapsedTime();
    double startTime;


    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()

                .addPath(new BezierLine(intake1Pose, scorePose))
                .setLinearHeadingInterpolation(intake1Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(intake2Pose, scorePose))
                .setLinearHeadingInterpolation(intake2Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(intake3Pose, scorePose))
                .setLinearHeadingInterpolation(intake3Pose.getHeading(), scorePose.getHeading())
                .build();

        // Picking up artifacts

        pickupIntake1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, intake1Pose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), intake1Pose.getHeading())
                .build();
        pickupIntake2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, intake2Pose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), intake2Pose.getHeading())
                .build();
        pickupIntake3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, intake3Pose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), intake3Pose.getHeading())
                .build();


        endChain = follower.pathBuilder()
                .addPath(new BezierLine(scorePose,endPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), endPose.getHeading())
                .build();
    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                launch(1);
                break;

            case 1:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup1,true);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    intakeWheels.setVelocity(1000);
                    follower.followPath(pickupIntake1, 0.35,true);
                    setPathState(3);
                }
            break;

            case 3:
                // SCORE POSITION
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup1,true);
                    launch(4);
                }
                break;

            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup2,true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(pickupIntake2, 0.35, true);
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup2,true);
                    launch(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup3,true);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(pickupIntake3, 0.35, true);
                    setPathState(9);
                }
                break;

            case 9:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup3, true);
                    launch(10);
                }
                break;
            case 10:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    follower.followPath(endChain, true);
                    setPathState(11);
                }
                break;




                // launching activation -------------------------------



            // START OF LAUNCH -----------------------------------
            case -10:
                launcherLeft.setVelocity(targetVelocity);
                setPathState(-20);
                break;

//             wait for velocity, lower kicker, run pulley
            case -20:
                if (launcherLeft.getVelocity() > targetVelocity - 5){
                    setPathState(-30);
                }
                break;
            case -30:
                kicker.setPosition(0.2);
                intakeWheels.setVelocity(1000);
                startTime = runTime.seconds();
                setPathState(-40);
                //wait for 5 seconds, stop motors
            case -40:
                if (runTime.seconds() >= startTime+5){
                    stopMotors();
                    setPathState(nextPathStateVar);
                }
                break;
        }

    }

    int nextPathStateVar;
    private void launch(int nextPathState) {
        nextPathStateVar = nextPathState;
        setPathState(-10);
    }

    private void stopMotors(){
        kicker.setPosition(0);
        launcherLeft.setVelocity(0);
        intakeWheels.setVelocity(0);
    }


    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        currentVelocity = launcherLeft.getVelocity();

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Current Velocity", "%.2f", currentVelocity);
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {

        runTime.reset();

        launcherLeft = hardwareMap.get(DcMotorEx.class, "LL");
        kicker = hardwareMap.get(Servo.class, "kicker");
        intakeWheels = hardwareMap.get(DcMotorEx.class, "IW");
        intakeWheels.setDirection(DcMotor.Direction.REVERSE);
        launcherLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(17.5, 0, 0, 16.5);
        launcherLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}
}
