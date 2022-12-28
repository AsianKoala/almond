package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Config
@TeleOp
public class
SlidesPIDTest2 extends LinearOpMode{
    private Robot robot;
    private ElapsedTime timer;
    public enum Height {
        DOWN,
        LOW,
        MID,
        HIGH
    }

    FtcDashboard dashboard;

    public static int TARGET_POS = 0;

    public static double kP = 0, kI = 0, kD = 0, ff = 0;

    public void runOpMode() {
        robot = new Robot(telemetry, hardwareMap);
        timer = new ElapsedTime();
        robot.init();
        TelemetryPacket packet = new TelemetryPacket();

        MultipleTelemetry tele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        dashboard = FtcDashboard.getInstance();

        waitForStart();
        //robot.lift.setHeight(LiftConstants.IdleArm);
        tele.addData("loop:", "started");
        Height height = Height.DOWN;
        timer.reset();

        while (!isStopRequested()) {
            tele.addData("slide height", robot.lift.getCurrentHeight());
            tele.addData("lift PID", robot.lift.getPID());
            tele.addData("target pos", robot.lift.getTargetHeight());

            packet.put("slide height", robot.lift.getCurrentHeight());
            packet.put("lift PID", robot.lift.getPID());
            packet.put("target pos", robot.lift.getTargetHeight());
            dashboard.sendTelemetryPacket(packet);

            robot.update();
            telemetry.update();

            robot.lift.setTargetHeight(TARGET_POS);
            robot.lift.setFF(ff);
            robot.lift.setPID(kP, kI, kD);
        }
    }
}