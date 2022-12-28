package org.firstinspires.ftc.teamcode.opmode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class testTing extends LinearOpMode{
    Robot robot;
    public void runOpMode(){
        robot = new Robot(telemetry, hardwareMap);
        robot.init();
        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            robot.intake.clawServo.setPosition(1);
            telemetry.addData("SERVO POS: ", robot.intake.clawServo.getPosition());
            telemetry.update();
        }
    }
}

