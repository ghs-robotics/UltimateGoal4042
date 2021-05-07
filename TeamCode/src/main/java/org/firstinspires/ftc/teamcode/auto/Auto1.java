package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.data.FieldPositions;
import org.firstinspires.ftc.teamcode.robot_components.CVRobot;


@Autonomous
public class Auto1 extends LinearOpMode implements FieldPositions {

    // Declare OpMode members
    CVRobot robot;

    @Override
    public void runOpMode()
    {
        robot = new CVRobot(hardwareMap, telemetry);

        robot.initWithCV();
        robot.stack.activate();
        robot.wobble.activate();
        robot.startUp();
//        robot.towerXPID.setMinMax(-0.6, 0.6);
//        robot.towerWPID.setMinMax(-0.6, 0.6);
//        robot.towerXPID.k_P = 0.0400;
//        robot.towerXPID.k_P = 0.0015;
//        robot.towerWPID.k_P = 0.0450;
//        robot.towerWPID.k_P = 0.0010;

        telemetry.addData("Status", "Initialized");
        robot.telemetry.update();
        robot.powerLauncher.setLaunchAngleVertical();

        // Waiting for driver to hit PLAY
        waitForStart();

        robot.resetElapsedTime();
        robot.powerLauncher.setLaunchAngleLoading();
        robot.turnArmUpFull();

        // Determine how many rings in the starting ring stacks
        int config = robot.identifyRingConfig();

        robot.resetGyroAngle();
        robot.powerLauncher.setLaunchAnglePerfect();

        switch (config) {
            case 0:
                runZeroRingAuto();
                break;
            case 1:
                runOneRingAuto();
                break;
            case 4:
                runFourRingAuto();
                break;
        }

        robot.stopDrive();
    }

    private void dropWobble() {
        //once there, place down the wobble goal
        robot.turnArmDownFull();
        robot.toggleClaw();
        robot.wait(0.2);
        robot.turnArmUpFull();
        robot.wait(0.3);
        robot.move(1.0, 0, 0.25);
        robot.toggleClaw();
    }

    private void runZeroRingAuto() {

        robot.towerXPID.setMinMax(-0.7, 0.7);
        robot.towerWPID.setMinMax(-0.7, 0.7);

        // Shoot power shots with preloaded rings
        robot.moveToPos(MID_POWERSHOT_POS, 0.5, 1.4, 5.0, 1);
        robot.shootPowerShots();

        // Deliver first wobble goal
//        robot.turnArmDownDrag();
        robot.moveToPos(CONFIG_0_POS_I, 0.5, 1.2);
        robot.turnArmDownDrag();
        robot.wait(0.5);
        dropWobble();

        // Grab 2nd wobble goal
        robot.moveToPos(SECOND_WOBBLE_POS, 0.5, 1.0, 4.0, 1);
        robot.alignToWobble();
        robot.pickUpWobbleGoal("down");

        // Deliver 2nd wobble goal
        robot.moveToPos(PARK_0_POS, 0, 0, 1.0);
        robot.moveToPos(CONFIG_0_POS_II, 0.5, 1.5);
        dropWobble();

        // Move back to park over launch line
        if (robot.elapsedSecs() < 29) {
            robot.moveToPos(PARK_0_POS, 0.1, 1.0);
        }
        robot.stopDrive();
    }

    private void runOneRingAuto() {

        // Turn arm down so dropping wobble goal later is quicker
        robot.turnArmDownDrag();

        // Shoot power shots with preloaded rings
        robot.moveToPos(MID_POWERSHOT_POS, 0.3, 1.0, 4.0, 1);
        robot.shootPowerShots();

        // Deliver first wobble goal
        robot.moveToPos(PRE_CONFIG_1_POS_I, 0.1, 1.0);
        robot.move(0, 1.0, 0.6);
        dropWobble();
        robot.move(0, -1.0, 0.5);


        // Load rings from starter stack
        robot.moveToPos(PERFECT_LAUNCH_POS, 0.3, 0);
        robot.powerLauncher.setLaunchAngleLoading();
        robot.runIntake(0.9);
//        robot.move(0, -0.4, 1.0);


        // Grab 2nd wobble goal
        robot.moveToPos(SECOND_WOBBLE_POS, 0.5, 1.0, 4.0, 1);
        robot.alignToWobble();
        robot.pickUpWobbleGoal("down");

        // Shoot one ring
        robot.runIntake(0.0);
        robot.powerLauncher.toggleOn();
        robot.moveToPos(PERFECT_LAUNCH_POS, 0.1, 0.4, 4.0, 2, true);
        robot.setAssistedLaunchAngle();
        robot.indexRings(1);
        robot.powerLauncher.toggleOff();

        // Deliver 2nd wobble goal
        robot.moveToPos(PRE_CONFIG_1_POS_II, 0.3, 1.0, 2.0);
        robot.move(0, 1.0, 0.6);
        dropWobble();

        // Move back to park over launch line
        if (robot.elapsedSecs() <= 29.35) {
            robot.move(0, -1.0, 0.65);
        }
        robot.stopDrive();
    }

    private void runFourRingAutoNew() {

        // Move forward but avoid starter stack
        robot.moveToPos(NEXT_TO_STARTER_STACK_POS, 0, 0, 2.0);

        // Shoot 3 preloaded rings
        robot.autoShoot();

        // Load 1 or 2 rings from starter stack
        robot.powerLauncher.setLaunchAngleLoading();
        robot.moveToPos(PRE_CONFIG_1_POS_I, 0, 0, 0.5);
        robot.runIntake(0.9);
        robot.moveToPos(PERFECT_LAUNCH_POS, 0.2, 0.8, 2.0);
        robot.move(0, -0.4, 0.8);

        // Deliver first wobble goal
        robot.moveToPos(PRE_CONFIG_4_POS_I, 0.1, 0.7, 2.0);

        double t = robot.elapsedSecs();
        while (robot.elapsedSecs() - t < 0.4) {
            robot.calculateDrivePowers(-1.0, 0, -0.3, true);
            robot.sendDrivePowers();
        }

//        robot.rotateToPos(-70, 0.3);
        robot.runIntake(0.0);
        dropWobble();

        t = robot.elapsedSecs();
        robot.powerLauncher.toggleOn();
        while (robot.elapsedSecs() - t < 0.4) {
            robot.calculateDrivePowers(1.0, 0.5, 0.3, true);
            robot.sendDrivePowers();
        }

        // Shoot 1 ring
        robot.moveToPos(PERFECT_LAUNCH_POS, 0.2, 0.8, 4.0, 1, true);
        robot.indexRings(1);
        robot.powerLauncher.toggleOff();

        // Load remaining rings from starter stack
        robot.powerLauncher.setLaunchAngleLoading();
        robot.runIntake(0.9);
        robot.move(0, -0.4, 0.8);

        // Grab 2nd wobble goal
        robot.moveToPos(SECOND_WOBBLE_POS, 0.2, 0.8);
        robot.alignToWobble();
        robot.pickUpWobbleGoal("up");

        // Shoot remaining rings
        robot.powerLauncher.toggleOn();
        robot.moveToPos(PERFECT_LAUNCH_POS, 0.3, 0.5, 4.0, 1, true);
        robot.runIntake(0.0);
        robot.autoShoot();

        // Deliver 2nd wobble goal
//        robot.moveToPos(PRE_CONFIG_4_POS_II, 0.5, 1.0, 3.0);
//        robot.move(0, 0.6, 0.8);
//        robot.rotateToPos(-45, 0.5);
//        dropWobble();

        robot.stopDrive();

        // Move back to park over launch line
//        robot.move(0, 1.0, 0.45);
    }

    private void runFourRingAuto() {

        // Move forward but avoid starter stack
        robot.moveToPos(NEXT_TO_STARTER_STACK_POS, 0, 0, 2.0);
        robot.powerLauncher.setLaunchAnglePerfect();

        // Deliver first wobble goal
        robot.moveToPos(PRE_CONFIG_4_POS_I, 0.2, 1.5, 2.0);
        robot.move(0, 1.0, 0.57);
        robot.rotateToPos(-70, 0.3);
        robot.turnArmDownDrag();
        robot.wait(0.4);
        dropWobble();

        // Shoot 3 preloaded rings
        robot.rotateToPos(15, 0.4);
        robot.move(0, -1.0, 0.7, true);
        robot.powerLauncher.toggleOn();
        robot.moveToPos(PERFECT_LAUNCH_POS, 1.0, 2.5, 5.0, 2, true);
        robot.indexRings(3);

        // Load 1 or 2 rings from starter stack
        robot.powerLauncher.setLaunchAngleLoading();
        robot.runIntake(0.9);
        robot.powerLauncher.toggleOff();
        robot.move(0, -0.3, 0.8);
        robot.wait(1.4);

        // Shoot 1 ring
        robot.powerLauncher.toggleOn();
        robot.powerLauncher.setLaunchAnglePerfect();
        robot.moveToPos(PERFECT_LAUNCH_POS, 1.0, 2.5, 5.0, 2, true);
//        robot.wait(0.3);
        robot.runIntake(0.0);
        robot.indexRings(1);
        robot.powerLauncher.toggleOff();

        // Load remaining rings from starter stack
        robot.powerLauncher.setLaunchAngleLoading();
        robot.runIntake(0.9);
        robot.move(0, -0.3, 2.8);

        // Shoot remaining rings
        robot.powerLauncher.setLaunchAnglePerfect();

        robot.powerLauncher.toggleOn();
        robot.moveToPos(PERFECT_LAUNCH_POS, 0.2, 0.8);
        robot.runIntake(0.0);
        robot.setAssistedLaunchAngle();
        robot.wait(0.3);
        robot.indexRings(3);
        robot.powerLauncher.toggleOff();

        // Deliver 2nd wobble goal
//        robot.moveToPos(PRE_CONFIG_4_POS_I, 1.0, 1.5, 3.0);
//        robot.move(0, 0.6, 0.8);
//        robot.rotateToPos(-45, 0.5);
//        placeWobble();

        // Move back to park over launch line
        robot.move(0, 1.0, 0.45);
    }
}
