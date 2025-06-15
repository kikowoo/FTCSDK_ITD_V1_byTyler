package org.firstinspires.ftc.teamcode.Robot.OpModes.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import java.lang.Math;

import org.firstinspires.ftc.teamcode.Robot.Structure.Library.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Autonomous(name = "Specimen_Auto", group = "Examples")
public class Specimens extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    /** Start Pose of our robot */
    private final Pose STP = new Pose(8, 61, Math.toRadians(180));

    /** Starting Scoring Pose */
    private final Pose STSCP = new Pose(30, 63, Math.toRadians(180));

    /** Highest (First) Sample from the Spike Mark */
    private final Pose P1P = new Pose(65, 27, Math.toRadians(180));

    private final Pose C1P1P = new Pose(34, 17, Math.toRadians(180));
    private final Pose C2P1P = new Pose(30, 40, Math.toRadians(180));
    private final Pose C3P1P = new Pose(65, 37, Math.toRadians(180));

    private final Pose P1PR = new Pose(12, 25, Math.toRadians(180));

    /** Middle (Second) Sample from the Spike Mark */
    private final Pose P2P = new Pose(61, 13, Math.toRadians(180));

    private final Pose CP2P = new Pose(60, 30, Math.toRadians(180));

    private final Pose P2PR = new Pose(12, 13, Math.toRadians(180));

    /** Lowest (Third) Sample from the Spike Mark */
    private final Pose P3P = new Pose(61, 6, Math.toRadians(180));

    private final Pose CP3P = new Pose(60, 19, Math.toRadians(180));

    private final Pose P3PR = new Pose(12, 6, Math.toRadians(180));

    /** Score Pose */
    private final Pose SCP = new Pose(30, 69, Math.toRadians(180));

    /** Human Player */
    private final Pose HP = new Pose(12, 32, Math.toRadians(180));

    private final Pose CHP = new Pose(34, 73, Math.toRadians(180));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload;
    private PathChain S1, S1R, S2, S2R, S3, S3R, SS, HPR1, HPS1, HPR2, HPS2, HPR3, HPS3, HPR4;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(new Point(STP), new Point(STSCP)));
        scorePreload.setConstantHeadingInterpolation(Math.toRadians(180));

        /* This is our S1 PathChain. We are using a single path with a BezierCurve, which is a curved line. */
        S1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(STSCP),
                                new Point(C1P1P),
                                new Point(C2P1P),
                                new Point(C3P1P),
                                new Point(P1P)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        /* This is our S1R PathChain. We are using a single path with a BezierLine, which is a straight line. */
        S1R = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(P1P),
                                new Point(P1PR)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        /* This is our S2 PathChain. We are using a single path with a BezierDurve, which is a curved line. */
        S2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(P1PR),
                                new Point(CP2P),
                                new Point(P2P)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        /* This is our S2R PathChain. We are using a single path with a BezierLine, which is a straight line. */
        S2R = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(P2P),
                                new Point(P2PR)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        /* This is our S3 PathChain. We are using a single path with a BezierCurve, which is a curved line. */
        S3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(P2PR),
                                new Point(CP3P),
                                new Point(P3P)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        /* This is our S3R PathChain. We are using a single path with a BezierLine, which is a straight line. */
        S3R = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(P3P),
                                new Point(P3PR)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        /* This is our SS PathChain. We are using a single path with a BezierLine, which is a straight line. */
        SS = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(P3PR),
                                new Point(SCP)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        /* This is our HPR1 PathChain. We are using a single path with a BezierCurve, which is a curved line. */
        HPR1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(SCP),
                                new Point(CHP),
                                new Point(HP)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        /* This is our HPS1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        HPS1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(HP),
                                new Point(SCP)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        /* This is our HPR2 PathChain. We are using a single path with a BezierCurve, which is a curved line. */
        HPR2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(SCP),
                                new Point(CHP),
                                new Point(HP)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        /* This is our HPS2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        HPS2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(HP),
                                new Point(SCP)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        /* This is our HPR3 PathChain. We are using a single path with a BezierCurve, which is a curved line. */
        HPR3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(SCP),
                                new Point(CHP),
                                new Point(HP)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        /* This is our HPS3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        HPS3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(HP),
                                new Point(SCP)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        /* This is our HPR4 PathChain. We are using a single path with a BezierCurve, which is a curved line. */
        HPR4 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(SCP),
                                new Point(CHP),
                                new Point(HP)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    follower.followPath(S1, false);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    follower.followPath(S1R, false);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    follower.followPath(S2, false);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    follower.followPath(S2R, false);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    follower.followPath(S3, false);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    follower.followPath(S3R, true);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    follower.followPath(SS, true);
                    setPathState(8);
                }
                break;
            case 8:
                if(!follower.isBusy()) {
                    follower.followPath(HPR1, true);
                    setPathState(9);
                }
                break;
            case 9:
                if(!follower.isBusy()) {
                    follower.followPath(HPS1, true);
                    setPathState(10);
                }
                break;
            case 10:
                if(!follower.isBusy()) {
                    follower.followPath(HPR2, true);
                    setPathState(11);
                }
                break;
            case 11:
                if(!follower.isBusy()) {
                    follower.followPath(HPS2, true);
                    setPathState(12);
                }
                break;
            case 12:
                if(!follower.isBusy()) {
                    follower.followPath(HPR3, true);
                    setPathState(13);
                }
                break;
            case 13:
                if(!follower.isBusy()) {
                    follower.followPath(HPS3, true);
                    setPathState(14);
                }
                break;
            case 14:
                if(!follower.isBusy()) {
                    follower.followPath(HPR4, true);
                    setPathState(15);
                }
                break;
            case 15:
                if(!follower.isBusy()) {
                    setPathState(-1);
                    PoseStorage.CurrentPose = follower.getPose();
                }
                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(STP);
        buildPaths();
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
    public void stop() {
        PoseStorage.CurrentPose = follower.getPose();
    }
}