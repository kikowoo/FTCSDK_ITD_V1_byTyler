package org.firstinspires.ftc.teamcode.Robot.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Structure.Hardware.BTRobotV1;

@TeleOp(name = "SetupColor", group = "TeleOp")
public class Setup_Color extends LinearOpMode {
    BTRobotV1 robot = new BTRobotV1(this);
    private Gamepad currentGamepad1, previousGamepad1;

    @Override
    public void runOpMode(){
        while(opModeInInit()){
            telemetry.addData(">", "Setup_AllianceColor x = Blue b = Red");
            currentGamepad1 = new Gamepad();
            previousGamepad1 = new Gamepad();
        }
        while(opModeIsActive()){
            if(currentGamepad1.x && !previousGamepad1.x){
                robot.Color_Alliance = "Blue";
            } else if (currentGamepad1.b && !currentGamepad1.b) {
                robot.Color_Alliance = "Red";
            }
        }
    }
}
