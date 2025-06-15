package org.firstinspires.ftc.teamcode.Robot.Structure.Hardware;
/*
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.ServoHubConfiguration;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import com.seattlesolvers.solverslib.solversHardware.SolversMotor;
import com.seattlesolvers.solverslib.solversHardware.SolversMotorEx;
import com.seattlesolvers.solverslib.solversHardware.SolversServo;
import com.seattlesolvers.solverslib.solversHardware.SolversCRServo;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import com.pedropathing.follower.FollowerConstants;

import java.util.List;

public class Robot {
    public SolversMotorEx leftFront;
    public SolversMotorEx rightFront;
    public SolversMotorEx leftBack;
    public SolversMotorEx rightBack;

    public SolversMotorEx leftHorizontalLift;
    public SolversMotorEx rightHorizontalLift;

    public SolversMotorEx leftVerticalLift;
    public SolversMotorEx rightVerticalLift;

    public SolversServo depositArmRotation;
    public SolversServo depositClawRotation;
    public SolversServo activeIntake;

    public Follower follower;

    private static Robot instance = new Robot();

    public boolean enabled;

    public static Robot getInstance() {
        if(instance == null) {
            instance = new Robot();
        }
        instance.enabled = true;
        return instance;
    }

    public void init(HardwareMap hardwaremap) {
        leftFront = new SolversMotorEx(hardwareMap.get(DcMotorEx.class, "FL"), 0.01);
        rightFront = new SolversMotorEx(hardwareMap.get(DcMotorEx.class, "FR"), 0.01);
        leftBack = new SolversMotorEx(hardwareMap.get(DcMotorEx.class, "BL"), 0.01);
        rightBack = new SolversMotorEx(hardwareMap.get(DcMotorEx.class, "BR"), 0.01);

        leftHorizontalLift = new SolversMotorEx(hardwareMap.get(DcMotorEx.class, "lhl"), 0.01);
        rightHorizontalLift = new SolversMotorEx(hardwareMap.get(DcMotorEx.class, "rhl"), 0.01);

        leftVerticalLift = new SolversMotorEx(hardwareMap.get(DcMotorEx.class, "lvl"), 0.01);
        rightVerticalLift = new SolversMotorEx(hardwareMap.get(DcMotorEx.class, "rvl"), 0.01);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftHorizontalLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightHorizontalLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftVerticalLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightVerticalLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftHorizontalLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightHorizontalLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftVerticalLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightVerticalLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            if (hub.isParent() && LynxConstants.isEmbeddedSerialNumber(hub.getSerialNumber())) {
                ControlHub = hub;
            }

        }

         if (opModeType.equals(OpModeType.TELEOP)) {
            follower.startTeleopDrive();
            follower.setStartingPose(autoEndPose);
        } else {
            follower.setStartingPose(new Pose(0, 0, 0));
        }
    }
}
*/