package org.firstinspires.ftc.teamcode.auto_opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot_components.Robot;

/**
 * This will be the main class that we will put all of our autonomous code into
 * This class is obviously not complete, and the structure of the code is not decided yet
 */
@Autonomous
public class Auto1 extends LinearOpMode {
    public static final int[] NEXT_TO_STARTER_STACK_POS = new int[]{170, 90}; // x position, width
    public static final int[] SHOOTER_POS = new int[]{100, 80};
    public static final int[] CONFIG_0_POS = new int[]{30, 107};
    public static final int[] CONFIG_1_POS = new int[]{103, 165};
    public static final int[] CONFIG_4_POS = new int[]{100, 80}; // TO DO: CAN'T SEE TOWER GOAL FROM THIS POS!
    public static final int[] SECOND_WOBBLE_POS = new int[]{53, 73};
    public static final int[] STARTER_STACK_BEFORE_POS = new int[]{100, 80};
    public static final int[] STARTER_STACK_AFTER_POS = new int[]{100, 70};
    public static final int[] PARK_POS = new int[]{100, 95};

    // Declare OpMode members
    Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        robot.init();
        robot.setTargetToTower();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        int stage = 1;
        waitForStart();

        // TO DO: remove the switch statement???
        while(opModeIsActive()) {
            switch (stage) {
                case 1 :
                    //Determine how many rings in the starting ring stacks
                    robot.identifyRingConfig();
                    robot.config = 1; // For testing purposes
                    stage++;
                    break;

                case 2 :
                    //Move forward 6-7 feet until at the edge of launch zone
                    robot.moveToPos(NEXT_TO_STARTER_STACK_POS, 10);
                    stage++;
                    break;

                case 3 :
                    //Move sideways until in line with tower goal
                    //aim the robot at the goal and make sure that the robot is within the launch zone
                    //Shoot 3 rings
                    robot.adjustAndShoot();
                    stage++;
                    break;

                case 4 :
                    //move forward towards the desired target wobble goal zone
                    //Check distance to tower goal and correct if necessary
                    //Move left or right depending on target wobble goal
                    if (robot.config == 0) {
                        robot.moveToPos(CONFIG_0_POS, 3);
                    } else if (robot.config == 1) {
                        robot.moveToPos(CONFIG_1_POS, 3);
                    } else {
                        robot.moveToPos(CONFIG_4_POS, 3);
                    }
                    stage++;
                    break;

                case 5 :
                    //once there, place down the wobble goal
                    robot.turnArm();
                    robot.wait(0.5);
                    robot.toggleGrab();
                    robot.wait(0.1);
                    robot.turnArm();
                    robot.wait(0.4);
                    robot.resetServos();
                    stage++;
                    break;

                case 6 :
                    //Head back to location where we shot the rings
                    //Move left or right and then backward towards second wobble goal
                    robot.moveToPos(SECOND_WOBBLE_POS, 3);
                    stage++;
                    break;

                case 7 :
                    // Turn around
                    robot.rotateToPos(180, 3);
                    stage++;
                    break;

                case 8 :
                    //pick up second wobble goal
                    robot.pickUpWobbleGoal(8);
                    stage++;
                    break;

                case 9 :
                    // if starterStack != 0, pickup the starter stack rings
                    if (robot.config == 4) {
                        robot.moveToPos(STARTER_STACK_BEFORE_POS, 5);
                        robot.moveToPos(STARTER_STACK_AFTER_POS, 5);
                    }
                    stage++;
                    break;

                case 10 :
                    //Check that we're in shooting position
                    //Shoot the 3 rings
                    robot.wait(0.01);
                    robot.adjustAndShoot();
                    stage++;
                    break;

                case 11 :
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
                    stage++;
                    break;

                case 12 :
                    //once there, place down the wobble goal
                    robot.turnArm();
                    robot.wait(0.51);
                    robot.toggleGrab();
                    robot.wait(0.1);
                    robot.turnArm();
                    robot.wait(0.4);
                    robot.resetServos();
                    stage++;
                    break;

                case 13 :
                    //Move forward to park over launch line
                    robot.moveToPos(PARK_POS, 5);
                    stage++;
                    break;
            }
            sleep(50);
        }
    }
}
