package org.firstinspires.ftc.teamcode.opmode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.opmode.LiftConstants;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.opmode.SleeveDetection;
import org.firstinspires.ftc.teamcode.subsystems.SleeveDetector;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class preloadTuning extends LinearOpMode{
    Robot robot;
    Pose2d parkingPosition;
    SleeveDetection.Color color = SleeveDetection.Color.BLUE;
    double highJunction = 15;
    double intakeHeight = 5;
    double extendLength = 7;
    double scoreAngle = 55.0;
    SleeveDetector detector = new SleeveDetector();
    String currentTrajectory = "preload";




    public void runOpMode(){
        robot = new Robot(telemetry, hardwareMap);
        robot.init();
        detector.init(hardwareMap, telemetry);
        sleep(2000);
        robot.intake.closeClaw();
        robot.intake.centerArm();
        while(!isStarted() && !isStopRequested()){
            color = detector.getColor();
            telemetry.addData("COLOR: ", color);
            telemetry.update();
        }
        detector.stop();
        if(color == SleeveDetection.Color.MAGENTA){
            parkingPosition = new Pose2d(60, -12);
        }
        else if(color == SleeveDetection.Color.BLUE){
            parkingPosition = new Pose2d(36, -12);
        }
        else if(color == SleeveDetection.Color.RED){
            parkingPosition = new Pose2d(12, -12);
        }

        robot.drive.setPoseEstimate(new Pose2d(36, -64, Math.toRadians(180)));
        TrajectorySequence cycles = robot.drive.trajectorySequenceBuilder(new Pose2d(36, -64, Math.toRadians(180)))
                // Preload
                .lineTo(new Vector2d(36, -12))
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    robot.lift.setTargetHeight(highJunction);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    robot.turret.setTargetAngle(scoreAngle);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    robot.lift.setHorizontalPosition(extendLength);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    robot.intake.openClaw();
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    robot.lift.setHorizontalPosition(0.0);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    robot.turret.setTargetAngle(180.0);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    robot.lift.setTargetHeight(intakeHeight);
                })
                .build();


        robot.drive.followTrajectorySequenceAsync(cycles);
        while(!isStopRequested() && opModeIsActive()){
            robot.update();
            telemetry.update();
        }





    }
}
