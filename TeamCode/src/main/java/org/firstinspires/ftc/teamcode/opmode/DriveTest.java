package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.controls.SlewRateLimiter;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class DriveTest extends LinearOpMode{
    private Robot robot;
    private final double dtRate = 0.5;
    private SlewRateLimiter SRL;
    public void runOpMode(){
        robot = new Robot(telemetry, hardwareMap);
        SRL = new SlewRateLimiter(dtRate);
        robot.init();

        waitForStart();
        //robot.lift.setArmPos(LiftConstants.IdleArm);
        telemetry.addData("loop:", "started");
        while(!isStopRequested()) {
            telemetry.addData("turret goal", robot.turret.getTargetAngle());

            robot.update();
            telemetry.update();
            robot.turret.setRotation(gamepad1.left_stick_x);

            robot.drive.setWeightedDrivePower(
                    new Pose2d(
                            -SRL.calculate(gamepad1.left_stick_y), //controls forward
                            -SRL.calculate(gamepad1.left_stick_x), //controls strafing
                            -SRL.calculate(gamepad1.right_stick_x) //controls turning
                    )
            );
            robot.drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad2.left_stick_y, //controls forward
                            -gamepad2.left_stick_x, //controls strafing
                            -gamepad2.right_stick_x //controls turning
                    )
            );
        }
    }

}