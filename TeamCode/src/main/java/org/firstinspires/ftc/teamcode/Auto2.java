package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * This will be the main class that we will put all of our autonomous code into
 * This class is obviously not complete, and the structure of the code is not decided yet
 */
@Autonomous
public class Auto2 extends LinearOpMode {
    public static final int[] NEXT_TO_STARTER_STACK_POS = new int[]{170, 90}; // x position, width
    public static final int[] SHOOTER_POS = new int[]{100, 80};
    public static final int[] CONFIG_0_POS = new int[]{30, 107};
    public static final int[] CONFIG_1_POS = new int[]{103, 125};
    public static final int[] CONFIG_4_POS = new int[]{100, 80}; // TO DO: CAN'T SEE TOWER GOAL FROM THIS POS!
    public static final int[] SECOND_WOBBLE_POS = new int[]{53, 73};
    public static final int[] STARTER_STACK_BEFORE_POS = new int[]{100, 80};
    public static final int[] STARTER_STACK_AFTER_POS = new int[]{100, 70};
    public static final int[] PARK_POS = new int[]{100, 95};

    // Declare OpMode members
    Robot robot;
    int stage = 1;
    double t = 0;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        robot.init();
        robot.setTargetToTower();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        //Determine how many rings in the starting ring stacks
        robot.identifyRingConfig();
        robot.config = 1; // For testing purposes

        madeIt("config identified");

        robot.setTargetToTower(170, 90); // Setting targetX and targetWidth
        robot.updateObjectValues();

        while(opModeIsActive()) {
            if (stage == 1) {
                if (Math.abs(robot.targetWidth - robot.objectWidth) > 5 || Math.abs(robot.targetX - robot.objectX) > 5) {
                    robot.chaseTower();
                } else {
                    double t = robot.getElapsedTimeSeconds();
                    stage++;
                }
            }

            if (stage == 2) {
                if ((robot.leftRearPower != 0
                        || robot.rightRearPower != 0
                        || robot.leftFrontPower != 0
                        || robot.rightFrontPower != 0)
                        && robot.elapsedTime.seconds() - t < 3) {
                    robot.chaseTower();
                } else {
                    robot.stopDrive();
                    robot.setTargetToTower(95, 80);
                    stage++;
                }
            }

            if (stage == 3) { // First part of adjustAndShoot
                if (Math.abs(robot.targetWidth - robot.objectWidth) > 5
                        || Math.abs(robot.targetX - robot.objectX) > 5) {
                    robot.chaseTower();
                } else {
                    robot.toggleShooter();
                    t = robot.getElapsedTimeSeconds();
                    stage++;
                }
            }

            if (stage == 4) { // Second part of adjustAndShoot
                if ((robot.leftRearPower != 0
                        || robot.rightRearPower != 0
                        || robot.leftFrontPower != 0
                        || robot.rightFrontPower != 0)
                        && robot.elapsedTime.seconds() - t < 5) {
                    robot.chaseTower();
                } else {
                    robot.stopDrive();
                    for (int i = 0; i < 3; i++) {
                        robot.launchRing();
                    }
                    robot.toggleShooter();
                    stage++;
                }
            }
            sleep(50);
        }


        /*
        //Move forward 6-7 feet until at the edge of launch zone
        robot.moveToPos(NEXT_TO_STARTER_STACK_POS, 10);

        madeIt("next to starter stack");

        //Move sideways until in line with tower goal
        //aim the robot at the goal and make sure that the robot is within the launch zone
        //Shoot 3 rings
        robot.adjustAndShoot();

        madeIt("shot three goals");


        //move forward towards the desired target wobble goal zone
        //Check distance to tower goal and correct if necessary
        //Move left or right depending on target wobble goal
        robot.moveToPos(CONFIG_1_POS, 3);

        madeIt("next to wobble goal drop zone");

        //once there, place down the wobble goal
        robot.turnArm();
        robot.wait(1.0);
        robot.toggleGrab();
        robot.wait(0.5);
        robot.turnArm();
        robot.wait(0.1);
        robot.toggleGrab();

        madeIt("set down first wobble goal");

        //Head back to location where we shot the rings
        //Move left or right and then backward towards second wobble goal
        robot.moveToPos(SECOND_WOBBLE_POS, 5);

        madeIt("going for the second wobble goal");

        // Turn around
        robot.rotateToPos(180, 5);

        madeIt("turned around");

        //pick up second wobble goal
        robot.pickUpWobbleGoal(8);
        robot.targetAngle = 0;

        madeIt("picked up the second wobble goal");


        // if starterStack != 0, pickup the starter stack rings
        robot.moveToPos(STARTER_STACK_BEFORE_POS, 8);
        robot.moveToPos(STARTER_STACK_AFTER_POS, 8);

        madeIt("gathered rings");


        //Check that we're in shooting position
        //Shoot the 3 rings
        robot.wait(0.01);
        robot.adjustAndShoot();

        madeIt("shot more rings");

        //move forward towards the desired target wobble goal zone
        //Check distance to tower goal and correct if necessary
        //Move left or right depending on target wobble goal
        robot.wait(0.01);
        if (robot.config == 0) {
            robot.moveToPos(CONFIG_0_POS, 3);
        } else if (robot.config == 1) {
            robot.moveToPos(CONFIG_1_POS, 3);
        } else {
            robot.moveToPos(CONFIG_4_POS, 3);
        }

        madeIt("brought second wobble goal to drop zone");

        //once there, place down the wobble goal
        robot.turnArm();
        robot.wait(0.51);
        robot.toggleGrab();
        robot.wait(0.1);
        robot.turnArm();
        robot.wait(0.4);
        robot.toggleGrab();

        madeIt("delivered the second wobble");

        //Move forward to park over launch line
        robot.moveToPos(PARK_POS, 5);


        madeIt("parked");
        */
    }

    public void madeIt(String s) {
        robot.telemetry.addData("Made it! Status: ", s);
        robot.telemetry.update();
        robot.wait(0.5);
    }
}
