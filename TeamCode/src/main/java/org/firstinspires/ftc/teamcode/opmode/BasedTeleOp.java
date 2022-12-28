package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

import robotuprising.lib.system.statemachine.StateMachine;
import robotuprising.lib.system.statemachine.StateMachineBuilder;

public class BasedTeleOp extends LinearOpMode {
    public enum RobotState {
        IDLE,
        INTAKING,
        OPEN_CLAW,
        BEFORE_GRAB_PAUSE,
        GRABBED,
        LIFTED,
        POST_LIFT,
        DROPPED,
        RESET,
    }

    public enum RobotMode {
        STACK,
        NORMAL,
        AUTOCYCLE
    }

    private enum SwitchToAutocycleStates {
        MOVE_SHIT,
        TURN_TURRET,
        FINALLY_DO_SHIT
    }

    private enum SwitchToStackStates {
        DO_SHIT,
        TURN_TURRET
    }

    private enum AutoCycleStates {
        SET_HORIZONTAL_TO_HORIZONTAL_EXTENDED,
        CLOSE_CLAW,
        SET_HORIZONTAL_TO_BACK_AND_DO_OTHER_SHIT,
        SET_LIFT_DROPPED_THING,
        DEPOSIT_AND_RESET
    }

    private enum  SwitchToNormalStates {
        SET_STUFF,
        TURN_TURRET
    }

    private enum StackStates {
        IDLE,
        INTAKING,
        GRABBED,
        LIFTED,
        POST_LIFT,
        DROPPED,
        RESET
    }

    private Robot robot;
    private RobotState robotState = RobotState.INTAKING;
    private RobotMode robotMode;
    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime turrettimer = new ElapsedTime();
    private final ElapsedTime armtimer = new ElapsedTime();
    private final ElapsedTime buttonDeltaTimer = new ElapsedTime();
    private boolean canTurn = false;
    private final double turretaddition = 10;
    public final double up = 1400;
    public final double mid = 1000;
    public final double low = 700;
    public final double ground = 0;
    public final double idle = 200;
    public final double intaking = 30;
    private final double front = 0;
    private final double back = 660;
    private final double right = 330;
    private final double left = -330;
    private final double droppedvalue = 150;
    private final double horizontalback = 0.4;
    private final double horizontallifted = 0.5;
    private final double horizontalmiddle = 0.65;
    private final double horizontalextended = 1;

    private double dtspeed = 1;
    private boolean armup = true;
    private boolean lowheight = false;
    private double turretPower = 0.0;
    private String horizontalpos = "back";
    private String liftedpos = "lifted";
    private boolean running = false;
    private double highposarm = 0.6;


    private final StateMachine<SwitchToAutocycleStates> switchToAutoCycleStateMachine = new StateMachineBuilder<SwitchToAutocycleStates>()
            .state(SwitchToAutocycleStates.MOVE_SHIT)
            .onEnter(() -> {
                robot.lift.setTargetHeight(idle);
                robot.intake.liftArm();
                robot.intake.closeClaw();
                robot.lift.setHorizontalPosition(horizontallifted);
            })
            .transitionTimed(0.7)
            .state(SwitchToAutocycleStates.TURN_TURRET)
            .onEnter(() -> robot.turret.setTargetAngle(front))
            .transitionTimed(0.2)
            .state(SwitchToAutocycleStates.FINALLY_DO_SHIT)
            .onEnter(() -> {
                robot.lift.setTargetHeight(intaking);
                robot.intake.centerArm();
                robot.intake.fullyOpenClaw();
                robotMode = RobotMode.AUTOCYCLE;
                timer.reset();
                buttonDeltaTimer.reset();
            })
            .build();


    private final StateMachine<SwitchToStackStates> switchToStackStatesStateMachine = new StateMachineBuilder<SwitchToStackStates>()
            .state(SwitchToStackStates.DO_SHIT)
            .onEnter(() -> {
                robot.lift.setTargetHeight(idle);
                robot.intake.liftArm();
                robot.intake.closeClaw();
                robot.lift.setHorizontalPosition(horizontallifted);
            })
            .transitionTimed(0.7)
            .state(SwitchToStackStates.TURN_TURRET)
            .onEnter(() -> {
                robot.turret.setTargetAngle(back);
                robotMode = RobotMode.STACK;
                robotState = RobotState.IDLE;
                timer.reset();
                buttonDeltaTimer.reset();
            })
            .build();


    private final StateMachine<SwitchToNormalStates> switchToNormalStatesStateMachine = new StateMachineBuilder<SwitchToNormalStates>()
            .state(SwitchToNormalStates.SET_STUFF)
            .onEnter(() -> {
                robot.lift.setTargetHeight(idle);
                robot.intake.liftArm();
                robot.intake.closeClaw();
                robot.lift.setHorizontalPosition(horizontallifted);
            })
            .transitionTimed(0.7)
            .state(SwitchToNormalStates.TURN_TURRET)
            .onEnter(() -> {
                robot.turret.setTargetAngle(front);
                timer.reset();
                robotMode = RobotMode.NORMAL;
                robotState = RobotState.IDLE;
            })
            .transition(() -> true)
            .build();

    private boolean shouldTransitionToLift = false;
    private boolean shouldTransitionToIntaking = false;

    private void idleEnter() {
        canTurn = true;
        robot.intake.closeClaw();
        robot.intake.liftArm();
        robot.lift.setHorizontalPosition(horizontallifted);
        robot.lift.setTargetHeight(idle);
    }

    private void grabbedLoop() {
        if (timer.milliseconds() > 300) {
            robot.intake.liftArm();
            robot.lift.setTargetHeight(idle);
        }

        if(timer.milliseconds() > 500) {
            shouldTransitionToIntaking = gamepad1.left_bumper;
            shouldTransitionToLift = gamepad2.dpad_up || gamepad2.dpad_right || gamepad2.dpad_left || gamepad2.dpad_down;

            if(shouldTransitionToIntaking) {
                robot.lift.setTargetHeight(intaking);
                timer.reset();
            }

            if(gamepad2.dpad_up) {
                robot.lift.setTargetHeight(up);
                robot.intake.setArmPos(highposarm);
                armup = true;
                lowheight = false;
            } else if(gamepad2.dpad_right) {
                robot.lift.setTargetHeight(mid);
                robot.intake.centerArm();
                armup = true;
                lowheight = false;
            } else if(gamepad2.dpad_left) {
                robot.lift.setTargetHeight(low);
                robot.intake.centerArm();
                armup = true;
                lowheight = false;
            } else if(gamepad2.dpad_down) {
                if (Math.abs(robot.turret.getCurrentAngle() - back) > 5) {
                    robot.turret.setTargetAngle(back);
                } else {
                    robot.lift.setTargetHeight(ground);
                    robot.intake.dropArm();
                    armup = false;
                    lowheight = true;
                }
            }
        }
    }

    private void liftedOnEnter() {
        canTurn = true;
        dtspeed = 0.4;
        horizontalpos = "middle";
        liftedpos = "lifted";
        robot.lift.setHorizontalPosition(horizontalmiddle);
    }

    private boolean shouldTransitionPostLiftedLoopToGrabbed = false;
    private boolean shouldTransitionPostLiftedLoopToDropped = false;

    private void postLiftedLoop() {
        if (gamepad2.dpad_down || gamepad1.dpad_down) {
            timer.reset();
            robot.turret.setTargetAngle(front);
            robot.lift.setTargetHeight(idle);
            shouldTransitionPostLiftedLoopToGrabbed = true;
        }
        if (gamepad1.left_bumper) {
            if (liftedpos.equals("lifted")) {
                robot.lift.setTargetHeight(robot.lift.getCurrentHeight() - droppedvalue);
            }
            timer.reset();
            shouldTransitionPostLiftedLoopToDropped = true;
        }
        if (gamepad2.dpad_up) {
            robot.lift.setTargetHeight(up);
            robot.intake.setArmPos(highposarm);
            timer.reset();
        }
        if (gamepad2.dpad_left) {
            robot.lift.setTargetHeight(low);
            robot.intake.centerArm();
        }
        if (gamepad2.dpad_right) {
            robot.lift.setTargetHeight(mid);
            robot.intake.centerArm();
        }
    }

    private final StateMachine<RobotState> normalMainStateMachine = new StateMachineBuilder<RobotState>()
            .state(RobotState.IDLE)
            .onEnter(this::idleEnter)
            .onExit(() -> {
                robot.lift.setTargetHeight(intaking);
                timer.reset();
            })
            .transition(() -> timer.milliseconds() > 500 && gamepad1.left_bumper)
            .state(RobotState.INTAKING) // TODO: check if this is correct
            .onEnter(() -> {
                canTurn = false;
                horizontalpos = "back";
                robot.lift.setHorizontalPosition(horizontalback);
                robot.intake.dropArm();
                robot.lift.setTargetHeight(intaking);
                robot.intake.intake(1);
            })
            .transitionTimed(0.150)
            .state(RobotState.OPEN_CLAW)
            .onEnter(() -> robot.intake.openClaw())
            .onExit(() -> {
                robot.intake.closeClaw();
                robot.intake.stopIntake();
            })
            .transition(() -> timer.milliseconds() > 500 && (gamepad1.left_bumper || robot.intake.getDistance() < 3))
            .state(RobotState.BEFORE_GRAB_PAUSE)
            .onExit(() -> timer.reset())
            .transitionTimed(0.3)
            .state(RobotState.GRABBED)
            .onEnter(() -> {
                canTurn = true;
                robot.lift.setHorizontalPosition(horizontallifted);
            })
            .loop(this::grabbedLoop)
            .onExit(() -> timer.reset())
            .transition(() -> shouldTransitionToLift)
            .state(RobotState.LIFTED)
            .onEnter(this::liftedOnEnter)
            .transitionTimed(0.750)
            .state(RobotState.POST_LIFT)
            .loop(this::postLiftedLoop)
            .state(RobotState.DROPPED)
            .onEnter(() -> {
                dtspeed = 1;
                canTurn = false;
                horizontalpos = "back";
                robot.intake.openClaw();
            })
            .transitionTimed(0.450)
            .state(RobotState.RESET)
            .onEnter(() -> {
                robot.lift.setTargetHeight(idle);
                robot.intake.liftArm();
                timer.reset();
                if(!lowheight) robot.turret.setTargetAngle(front);
            })
            .onExit(() -> {
                if(lowheight) robot.turret.setTargetAngle(front);
                timer.reset();
            })
            .transition(() -> !lowheight || timer.seconds() > 300)
            .build();


    private final StateMachine<StackStates> stackMainStateMachine = new StateMachineBuilder<StackStates>()
            .state(StackStates.IDLE)
            .onEnter(this::idleEnter)
            .onExit(() -> {
                robot.lift.setTargetHeight(intaking);
                timer.reset();
            })
            .transition(() -> timer.milliseconds() > 500 && gamepad1.left_bumper)
            .state(StackStates.INTAKING)
            .onEnter(() -> {
                canTurn = false;
                robot.lift.setHorizontalPosition(horizontalback);
                robot.intake.dropArm();
                robot.intake.openClaw();
                robot.intake.fullyOpenClaw();
                robot.lift.setTargetHeight(intaking);
            })
            .onExit(() -> {
                robot.intake.closeClaw();
                timer.reset();
            })
            .transition(() -> timer.milliseconds() > 500 && (gamepad1.left_bumper || robot.intake.getDistance() < 3))
            .state(StackStates.GRABBED)
            .state(StackStates.GRABBED)
            .onEnter(() -> {
                canTurn = true;
                robot.lift.setHorizontalPosition(horizontallifted);
            })
            .loop(this::grabbedLoop)
            .onExit(() -> timer.reset())
            .transition(() -> shouldTransitionToLift)
            .state(StackStates.LIFTED)
            .onEnter(this::liftedOnEnter)
            .transitionTimed(0.750)
            .state(StackStates.POST_LIFT)
            .loop(this::postLiftedLoop)
            .state(StackStates.DROPPED)
            .onEnter(() -> {
                dtspeed = 1;
                canTurn = false;
                robot.intake.openClaw();
            })
            .transitionTimed(0.450)
            .state(StackStates.RESET)
            .onEnter(() -> {
                robot.lift.setTargetHeight(idle);
                robot.intake.liftArm();
                timer.reset();
                if(!lowheight) robot.turret.setTargetAngle(back);
            })
            .onExit(() -> {
                if(lowheight) robot.turret.setTargetAngle(front);
                timer.reset();
            })
            .transition(() -> !lowheight || timer.seconds() > 300)
            .build();


    private final StateMachine<AutoCycleStates> autoCycleMainStateMachine = new StateMachineBuilder<AutoCycleStates>()
            .state(AutoCycleStates.SET_HORIZONTAL_TO_HORIZONTAL_EXTENDED)
            .onEnter(() -> robot.lift.setHorizontalPosition(horizontalextended))
            .transitionTimed(0.3)
            .state(AutoCycleStates.CLOSE_CLAW)
            .onEnter(robot.intake::closeClaw)
            .transitionTimed(0.2)
            .state(AutoCycleStates.SET_HORIZONTAL_TO_BACK_AND_DO_OTHER_SHIT)
            .onEnter(() -> {
                robot.lift.setHorizontalPosition(horizontalback);
                robot.lift.setTargetHeight(up);
                robot.intake.setArmPos(0.55);
                robot.turret.setTargetAngle(back);
            })
            .transitionTimed(0.5)
            .state(AutoCycleStates.SET_LIFT_DROPPED_THING)
            .onEnter(() -> robot.lift.setTargetHeight(robot.lift.getCurrentHeight() - droppedvalue))
            .transitionTimed(0.1)
            .state(AutoCycleStates.DEPOSIT_AND_RESET)
            .onEnter(() -> {
                robot.intake.openClaw();
                robot.intake.centerArm();
                robot.lift.setTargetHeight(200);
                robot.turret.setTargetAngle(front);
            })
            .transitionTimed(0.9)
            .build();

    private void driveControl() {
        robot.drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y * dtspeed, //  controls forward
                        -gamepad1.left_stick_x * dtspeed, +//controls strafing
                        -gamepad1.right_stick_x * dtspeed //controls turning
                )
        );

        //Slow-Mode
        if (gamepad1.right_bumper) dtspeed = 0.4;
        else dtspeed = 1;
    }

    private void manualControl() {
        turretPower = gamepad2.right_stick_x;
        //Vertical Slides Manual Control
        if (gamepad2.left_stick_y > 0.5) {
            robot.lift.setTargetHeight(robot.lift.getCurrentHeight() + 10);
        } else if (gamepad2.left_stick_y < -0.5) {
            robot.lift.setTargetHeight(robot.lift.getCurrentHeight() - 10);
        }

        //Turret Manual Control
        if (canTurn) {
            if (turretPower > 0.5) {
                robot.turret.setTargetAngle(robot.turret.getCurrentAngle() - turretaddition);
            } else if (turretPower < -0.5) {
                robot.turret.setTargetAngle(robot.turret.getCurrentAngle() + turretaddition);
            }
        }
    }

    private void horizontalSlidesControl() {
        if(robotState == RobotState.LIFTED && gamepad2.right_bumper) {
            if(horizontalpos.equals("middle")) {
                robot.lift.setHorizontalPosition(horizontalextended);
                liftedpos = "extended";
            } else if(horizontalpos.equals("extended")) {
                robot.lift.setHorizontalPosition(horizontalmiddle);
                liftedpos = "middle";
            }
        }
    }

    private void verticalSlideSlightDropSlightLift() {
        if(robotState == RobotState.LIFTED && gamepad2.left_bumper) {
            if(liftedpos.equals("lifted")) {
                robot.lift.setTargetHeight(robot.lift.getCurrentHeight() - droppedvalue);
                liftedpos = "dropped";
            } else if(liftedpos.equals("dropped")) {
                robot.lift.setTargetHeight(robot.lift.getCurrentHeight() + droppedvalue);
                liftedpos = "lifted";
            }
        }
    }

    private void turretControl() {
        if (canTurn) {
            if (turrettimer.milliseconds() > 500) {
                boolean shouldTransition = gamepad2.a || gamepad2.x || gamepad2.b || gamepad2.y;
                if (gamepad2.a) robot.turret.setTargetAngle(back);
                if (gamepad2.x) robot.turret.setTargetAngle(right);
                if (gamepad2.b) robot.turret.setTargetAngle(left);
                if (gamepad2.y) robot.turret.setTargetAngle(front);
                if(shouldTransition) timer.reset();
            }
        }
    }

    private void horizontalSlideExtendRetractForIntaking() {
        if(gamepad2.right_bumper && robotState == RobotState.INTAKING) {
            if(horizontalpos.equals("back")) {
                robot.lift.setHorizontalPosition(horizontalextended);
                horizontalpos = "extended";
            } else if(horizontalpos.equals("extended")) {
                robot.lift.setHorizontalPosition(horizontalmiddle);
            }
        }
    }

    private void updateTelemetryShit() {
        telemetry.addData("turret pos", robot.turret.getCurrentAngle());
        telemetry.addData("turret goal", robot.turret.getTargetAngle());
        telemetry.addData("State", robotState);
        telemetry.addData("Height", robot.lift.getCurrentHeight());
        telemetry.addData("TargetHeight", robot.lift.getTargetHeight());
        telemetry.addData("Distance", robot.intake.getDistance());
        telemetry.addData("Orientation", robot.drive.getOrientation());
        telemetry.addData("timer", timer.milliseconds());
        telemetry.addData("turrettimer", turrettimer.milliseconds());
        telemetry.addData("dtspeed", dtspeed);
        telemetry.addData("slide power", robot.lift.motor2.getPower());
        telemetry.addData("armup?", armup);
        telemetry.addData("lowheight?", lowheight);
        telemetry.addData("arm timer", armtimer.milliseconds());
        telemetry.addData("mode", robotMode);
        telemetry.addData("liftedpos", liftedpos);
        telemetry.addData("horizontalpos", horizontalpos);
        telemetry.addData("horzontalservopos", robot.lift.horizontalServo1.getPosition());
    }

    private void checkIfRunningAndRun(StateMachine<?> stateMachine) {
        if(stateMachine.getRunning()) stateMachine.update();
    }

    public void runOpMode() throws InterruptedException {
        robot = new Robot(telemetry, hardwareMap);
        robot.turret.tmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        robot.init();
        robot.intake.openClaw();
        armup = true;
        robot.turret.MAX_POWER = 1;
        robotMode = RobotMode.NORMAL;

        while (!isStopRequested() && opModeIsActive()) {
            driveControl();
            manualControl();

            switch (robotMode) {
                case NORMAL:
                    //Switching Modes
                    if (buttonDeltaTimer.milliseconds() > 500) {
                        if (gamepad1.x) {
                            switchToAutoCycleStateMachine.reset();
                            switchToAutoCycleStateMachine.start();
                        }
                        if (gamepad1.a) {
                            switchToStackStatesStateMachine.reset();
                            switchToStackStatesStateMachine.start();
                        }
                    }

                    horizontalSlidesControl();
                    verticalSlideSlightDropSlightLift();
                    turretControl();

                    if(normalMainStateMachine.getRunning()) {
                        normalMainStateMachine.update();
                        if(shouldTransitionToIntaking) {
                            normalMainStateMachine.resetTo(RobotState.INTAKING);
                            shouldTransitionToIntaking = false;
                        }
                    }


                case STACK:
                    //Switching Modes
                    if (buttonDeltaTimer.milliseconds() > 500) {
                        if (gamepad1.x) {
                            switchToAutoCycleStateMachine.reset();
                            switchToAutoCycleStateMachine.start();
                        }
                        if (gamepad1.y) {
                            switchToNormalStatesStateMachine.reset();
                            switchToNormalStatesStateMachine.start();
                        }
                    }

                    horizontalSlideExtendRetractForIntaking();
                    horizontalSlidesControl();
                    verticalSlideSlightDropSlightLift();
                    turretControl();

                    if(stackMainStateMachine.getRunning()) {
                        stackMainStateMachine.update();
                        if(shouldTransitionToIntaking) {
                            stackMainStateMachine.resetTo(StackStates.INTAKING);
                            shouldTransitionToIntaking = false;
                        }

                        if(shouldTransitionPostLiftedLoopToGrabbed) {
                            stackMainStateMachine.resetTo(StackStates.GRABBED);
                            shouldTransitionPostLiftedLoopToGrabbed = false;
                        }

                        if(shouldTransitionPostLiftedLoopToDropped) {
                            stackMainStateMachine.resetTo(StackStates.DROPPED);
                            shouldTransitionPostLiftedLoopToDropped = false;
                        }
                    }


                case AUTOCYCLE:
                    if (buttonDeltaTimer.milliseconds() > 500) {
                        if(gamepad1.y) {
                            switchToNormalStatesStateMachine.reset();
                            switchToNormalStatesStateMachine.start();
                        }
                        if (gamepad1.a) {
                            switchToStackStatesStateMachine.reset();
                            switchToStackStatesStateMachine.start();
                        }

                        if (gamepad1.left_bumper && !running) {
                            running = true;
                            buttonDeltaTimer.reset();
                        }
                        if (gamepad1.left_bumper && running) {
                            running = false;
                            buttonDeltaTimer.reset();
                        }
                    }

                    if (running) {
                        if(autoCycleMainStateMachine.getRunning()) {
                            autoCycleMainStateMachine.update();
                        } else {
                            autoCycleMainStateMachine.reset();
                            autoCycleMainStateMachine.start();
                        }
                    }
            }

            checkIfRunningAndRun(switchToNormalStatesStateMachine);
            checkIfRunningAndRun(switchToStackStatesStateMachine);
            checkIfRunningAndRun(switchToAutoCycleStateMachine);
            updateTelemetryShit();
            robot.update();
        }
    }
}
