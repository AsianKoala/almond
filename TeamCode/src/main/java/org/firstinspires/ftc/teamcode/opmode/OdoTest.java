package org.firstinspires.ftc.teamcode.opmode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class OdoTest extends LinearOpMode{
    DcMotorEx encoder1;
    DcMotorEx encoder2;
    public void runOpMode(){
        encoder1 = hardwareMap.get(DcMotorEx.class, "leftFront");//new DcMotorEx(hardwareMap.get(DcMotorEx.class, "leftFront"));
        encoder2 = hardwareMap.get(DcMotorEx.class, "rightFront");
        encoder1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        encoder2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        encoder1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        encoder2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        while(!isStopRequested()){
            telemetry.addData("encoder1 value: ", encoder1.getCurrentPosition());
            telemetry.addData("encoder2 value: ", encoder2.getCurrentPosition());
            telemetry.update();
        }
    }
}