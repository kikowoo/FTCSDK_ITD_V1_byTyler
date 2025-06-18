package org.firstinspires.ftc.teamcode.Robot.OpModes.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Robot.Structure.Library.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

/**
 * This is an example teleop that showcases movement and robot-centric driving.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 12/30/2024
 */

@TeleOp(name = "Robot-Centric Teleop", group = "TeleOp")
public class RobotCentricTeleOp extends OpMode {
    private Follower follower;
    private Path scoreBasket;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private final Pose startPose = PoseStorage.CurrentPose;
    private final Pose scorePose = new Pose(18, 130, Math.toRadians(315));
    private Gamepad currentGamepad1, previousGamepad1;

    private boolean isAutoDriving = false;

    /** This method is call once when init is played, it initializes the follower **/
    @Override
    public void init() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();
    }

    /** This method is called continuously after Init while waiting to be started. **/
    @Override
    public void init_loop() {
    }

    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start() {
        follower.startTeleopDrive();
        setPathState(0);
    }

    /** This is the main loop of the opmode and runs continuously after play **/
    @Override
    public void loop() {
        currentGamepad1.copy(gamepad1);
        previousGamepad1.copy(currentGamepad1);
        if(!isAutoDriving) {
            follower.setTeleOpMovementVectors(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true);
            follower.update();
        }

        if(currentGamepad1.a && !previousGamepad1.a) {
            isAutoDriving = true;
            scoreBasket = new Path(new BezierLine(new Point(follower.getPose()), new Point(scorePose)));
            scoreBasket.setLinearHeadingInterpolation(follower.getPose().getHeading(), scorePose.getHeading());
            paths();
            isAutoDriving = false;
        }

        /* Telemetry Outputs of our Follower */
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));

        /* Update Telemetry to the Driver Hub */
        telemetry.update();
    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void paths() {
        switch (pathState) {
            case 0:
                follower.followPath(scoreBasket);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    setPathState(-1);
                    PoseStorage.CurrentPose = follower.getPose();
                }
                break;
        }
    }

    /** We do not use this because everything automatically should disable **/
    @Override
    public void stop() {
    }
}