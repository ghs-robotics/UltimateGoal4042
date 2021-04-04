package org.firstinspires.ftc.teamcode.auto_opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot_components.Config;
import org.firstinspires.ftc.teamcode.robot_components.FieldPositions;
import org.firstinspires.ftc.teamcode.robot_components.Robot;


@Autonomous
public class Auto1 extends LinearOpMode implements FieldPositions {

    // Declare OpMode members
    Robot robot;

    @Override
    public void runOpMode()
    {
        robot = new Robot(hardwareMap, telemetry);
        robot.init();
        robot.setTargetToStack();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        robot.updateObjectValues();
        robot.resetElapsedTime();

        // Determine how many rings in the starting ring stacks
        Config config = robot.identifyRingConfig();

        robot.resetGyroAngle();
        madeIt("config identified: " + config);
//        robot.wait(1.0);

        if (!config.equals(Config.ZERO)) {
            //Move forward 6-7 feet until at the edge of launch zone
            robot.setLauncherSideToBeForward();
            robot.moveToPos(NEXT_TO_STARTER_STACK_POS, 0.0, 0.0, 3.0);

            madeIt("next to starter stack");
            robot.setLauncherSideToBeForward();
            robot.moveToPos(new int[]{142, 65}, 1.0, 0.5, 1.0);
        }


        //Move sideways until in line with tower goal
        //aim the robot at the goal and make sure that the robot is within the launch zone
        //Shoot 3 rings
        robot.setLauncherSideToBeForward();
        robot.moveToPos(PERFECT_LAUNCH_POS, 2.0);
        robot.setLauncherSideToBeForward();
        robot.adjustAndShoot(3);
        madeIt("shot 3 goals");

        // if starterStack != 0, pickup the starter stack rings
        if (!config.equals(Config.ZERO)) {
            robot.setLauncherSideToBeForward();
            robot.runIntake(0.8);
            robot.move(0, -0.25, 3.1);
            madeIt("gathered rings");

            robot.wait(0.3);
            robot.setLauncherSideToBeForward();
            robot.runIntake(0.0);
            if (config.equals(Config.ONE)) {
                robot.adjustAndShoot(1);
            } else {
                robot.adjustAndShoot(3);
            }
            robot.runIntake(0.0);
        }

        robot.powerLauncher.setLaunchAngle(0.010);


        //move forward towards the desired target wobble goal zone
        //Check distance to tower goal and correct if necessary
        //Move left or right depending on target wobble goal
        robot.setLauncherSideToBeForward();
        if (config.equals(Config.ZERO)) {
            robot.moveToPos(CONFIG_0_POS_I);
        } else if (config.equals(Config.ONE)) {
            robot.moveToPos(CONFIG_1_POS_I, 3.0, 0.5, 3.0);
            robot.move(0, 0.6, 1.0);
        } else {
            robot.moveToPos(CONFIG_4_POS_I, 3.0, 0.5, 3.0);
            robot.move(0, 0.6, 1.0);
        }

        madeIt("next to wobble goal drop zone");

        //once there, place down the wobble goal
        robot.turnArm();
        robot.wait(0.3);
        robot.toggleClaw();
        robot.wait(0.3);
        robot.turnArm();
        robot.wait(0.3);
        robot.toggleClaw();

        madeIt("set down first wobble goal");


        //Move forward to park over launch line
        robot.setLauncherSideToBeForward();
        if (config.equals(Config.ONE)) {
            robot.move(0, -0.8, 0.5);
        } else if (config.equals(Config.FOUR)) {
            robot.move(-0.1, -0.8, 1.0);
        }
        robot.stopDrive();

        madeIt("parked");
    }

    public void madeIt(String s) {
        robot.telemetry.addData("Made it! Status: ", s);
        robot.telemetry.update();
        robot.wait(0.0);
    }
}
