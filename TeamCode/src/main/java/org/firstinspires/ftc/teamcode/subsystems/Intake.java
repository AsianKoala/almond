package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.threeten.bp.DayOfWeek;

public class Intake implements Subsystem {

    //Arm constants
    public double liftedArm = 1;
    public double droppedArm = 0.1;
    public double centeredArm = 0.5;

    //Claw constants
    private double openedClaw = 0.415;
    private double closedClaw = 0.242;
    private double fullyOpen = 0.53;

    public Servo clawServoB;
    public Servo clawServo;
    public Servo armServo1;
    public Servo armServo2;
    public CRServo intakeServo1;
    public CRServo intakeServo2;
//    public CRServo intakeServo3;
//    public CRServo intakeServo4;
    private RevColorSensorV3 colorRangeSensor;

    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        clawServo = hardwareMap.get(Servo.class, "claw");
        clawServoB = hardwareMap.get(Servo.class, "claw2");
        armServo1 = hardwareMap.get(Servo.class, "arm");
        armServo2 = hardwareMap.get(Servo.class, "arm2");
        intakeServo1 = hardwareMap.get(CRServo.class, "intake1");
        intakeServo2 = hardwareMap.get(CRServo.class, "intake2");
//        intakeServo3 = hardwareMap.get(CRServo.class, "intake3");
//        intakeServo4 = hardwareMap.get(CRServo.class, "intake4");
        colorRangeSensor = hardwareMap.get(RevColorSensorV3.class, "distanceSensor");
    }

    @Override
    public void init() {

    }

    @Override
    public void update() {

    }

    @Override
    public void update(TelemetryPacket packet) {

    }



    public void intake(double power){
        intakeServo1.setPower(power);
        intakeServo2.setPower(-power);
//        intakeServo3.setPower(-power);
//        intakeServo4.setPower(power);
    }

    public void stopIntake(){
        intakeServo1.setPower(0);
        intakeServo2.setPower(0);
//        intakeServo3.setPower(0);
//        intakeServo4.setPower(0);
    }

    public void setClawPos(double position){
        clawServo.setPosition(position);
        clawServoB.setPosition(1-position);
    }

    public void openClaw(){
        setClawPos(openedClaw);
    }

    public void closeClaw() {
        setClawPos(closedClaw);
    }

    public void fullyOpenClaw() {
        setClawPos(fullyOpen);
    }

    public void setArmPos(double position){
        armServo1.setPosition(1-position);
        armServo2.setPosition(position);
    }

    public void liftArm(){
        setArmPos(liftedArm);
    }

    public void centerArm(){
        setArmPos(centeredArm);
    }

    public void dropArm(){
        setArmPos(droppedArm);
    }

    public double getDistance() {
        return colorRangeSensor.getDistance(DistanceUnit.INCH);
    }

}
