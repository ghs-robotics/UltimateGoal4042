package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.data.FieldPositions;
import org.firstinspires.ftc.teamcode.robot_components.Robot;


@Autonomous
public class Auto1 extends LinearOpMode implements FieldPositions {

    // Declare OpMode members
    Robot robot;

    @Override
    public void runOpMode()
    {
        robot = new Robot(hardwareMap, telemetry);
        robot.initWithCV();
        robot.stack.activate();
        telemetry.addData("Status: ", "Initialized");
        telemetry.addData("Config: ", robot.identifyRingConfig()); // TODO : COMMENT OUT
        robot.updateDrive(); // Display telemetry SHOULD THIS BE robot.telemetry.update();

        waitForStart();

        robot.resetElapsedTime();
        robot.powerLauncher.setLaunchAngleLoading();
        robot.wait(0.4); // wait for launcher to go down

        // Determine how many rings in the starting ring stacks
        int config = robot.identifyRingConfig();
        robot.stack.deactivate();

        robot.resetGyroAngle();
        robot.powerLauncher.setLaunchAnglePerfect();

        // Move forward but avoid starter stack
        robot.moveToPos(LEFT_POWERSHOT_POS, 0.5, 0.6, 3.0);

        if (config == 0) {
            // Deliver first wobble goal
            robot.moveToPos(CONFIG_0_POS_I, 1.0, 3.0);
            placeWobble();

            // Shoot 3 preloaded rings
            robot.moveToPos(PERFECT_LAUNCH_POS, 2.0, 3.0);
            robot.launchRings(3);
            madeIt("shot 3 goals");
        }
        else {
            if (config == 1) {
                // Deliver first wobble goal
                robot.moveToPos(PRE_CONFIG_1_POS_I, 1.0, 3.0, 3.0);
                robot.moveUsingWall(CONFIG_1_WALL_HEIGHT, 0.3, 2.0, 3.0);
                placeWobble();
            }

            // Shoot 3 preloaded rings
            robot.moveToPos(PERFECT_LAUNCH_POS, 2.0, 3.0);
            robot.launchRings(3);
            madeIt("shot 3 goals");

            // Load rings from starter stack
            robot.powerLauncher.setLaunchAngleLoading();
            robot.runIntake(1.0);
            if (config == 1) {
                robot.move(0, -0.3, 1.4);
            }
            else {
                robot.move(0, -0.17, 1.8);
            }
            madeIt("gathered rings");

            // Wait for rings to load fully
            robot.wait(2.7);

            robot.powerLauncher.setLaunchAnglePerfect();
            robot.runIntake(0.0);

            robot.moveToPos(PERFECT_LAUNCH_POS, 1.0, 2.0);
            if (config == 1) {
                robot.launchRings(1);
            } else {
                robot.launchRings(3);
            }
        }

        //move forward towards the desired target wobble goal zone
        //Check distance to tower goal and correct if necessary
        //Move left or right depending on target wobble goal

        // Move back to park over launch line
        robot.moveToPos(PARK_POS);
        robot.stopDrive();
    }

    private void getPowerShots() {
        // Shoot power shots
        robot.powerLauncher.setLaunchAnglePowershot();
        robot.moveToPos(LEFT_POWERSHOT_POS, 1.5, 3.0);
        robot.launchRings(1);
        robot.move(-0.6, 0, 0.6);
        robot.rotateToPos(0, 0.5);
        robot.launchRings(1);
        robot.move(-0.6, 0, 0.6);
        robot.rotateToPos(0, 0.5);
        robot.launchRings(1);
    }

    private void madeIt(String s) {
        robot.telemetry.addData("Made it! Status: ", s);
        robot.telemetry.update();
        robot.wait(0.0);
    }

    private void placeWobble() {
        //once there, place down the wobble goal
        robot.turnArm();
        robot.wait(0.3);
        robot.toggleClaw();
        robot.wait(0.3);
        robot.turnArm();
        robot.wait(0.5);
        robot.toggleClaw();
        robot.move(1.0, 0, 0.4);
    }
}
