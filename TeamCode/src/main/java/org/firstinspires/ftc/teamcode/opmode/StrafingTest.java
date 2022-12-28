package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

import com.qualcomm.robotcore.util.ElapsedTime;
@Autonomous
public class StrafingTest extends LinearOpMode {
    //    Pose2d START_POSE = new Pose2d(0, 0, 0);
    Drivetrain drivetrain;
    private ElapsedTime timer;
    public void runOpMode() throws InterruptedException {
        Drivetrain drivetrain;
        drivetrain = new Drivetrain(hardwareMap);
        timer = new ElapsedTime();
        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            if(timer.seconds() < 1) {
                drivetrain.leftFront.setPower(1);
                drivetrain.rightFront.setPower(-1);
                drivetrain.leftRear.setPower(-1);
                drivetrain.rightRear.setPower(1);
            }else {
                drivetrain.leftFront.setPower(0);
                drivetrain.rightFront.setPower(0);
                drivetrain.rightRear.setPower(0);
                drivetrain.leftRear.setPower(0);
            }
            telemetry.addData("time", timer);
            telemetry.update();

        }
    }