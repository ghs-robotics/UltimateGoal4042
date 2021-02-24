/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Tele3 extends LinearOpMode
{
    public static final int[] NEXT_TO_STARTER_STACK_POS = new int[]{170, 90}; // x position, width
    public static final int[] SHOOTER_POS = new int[]{100, 80};
    public static final int[] CONFIG_0_POS = new int[]{30, 107};
    public static final int[] CONFIG_1_POS = new int[]{103, 115}; //TO DO: CAN BARELY SEE TOWER GOAL FROM HERE!
    public static final int[] CONFIG_4_POS = new int[]{100, 80}; // TO DO: CAN'T SEE TOWER GOAL FROM THIS POS!
    public static final int[] SECOND_WOBBLE_POS = new int[]{53, 73};
    public static final int[] STARTER_STACK_BEFORE_POS = new int[]{100, 80};
    public static final int[] STARTER_STACK_AFTER_POS = new int[]{100, 70};
    public static final int[] PARK_POS = new int[]{100, 95};

    Robot robot;
    Controller controller1;

    @Override
    public void runOpMode()
    {
        robot = new Robot(hardwareMap, telemetry);
        controller1 = new Controller(gamepad1);
        robot.init();
        robot.setTargetToTower(95,80);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        robot.updateObjectValues();

        //Determine how many rings in the starting ring stacks
//        robot.identifyRingConfig();
        robot.config = 1; // For testing purposes

        madeIt("config identified");

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
        if (robot.config == 0) {
            robot.moveToPos(CONFIG_0_POS, 3);
        } else if (robot.config == 1) {
            robot.moveToPos(CONFIG_1_POS, 3);
        } else {
            robot.moveToPos(CONFIG_4_POS, 3);
        }

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
        robot.pickUpWobbleGoal(12);
        robot.targetAngle = 0;

        madeIt("picked up the second wobble goal");


        // if starterStack != 0, pickup the starter stack rings
        if (robot.config == 1) {
            robot.moveToPos(STARTER_STACK_BEFORE_POS, 8);
            robot.moveToPos(STARTER_STACK_AFTER_POS, 8);
        }

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

        while (opModeIsActive()) {
            controller1.update();
            if (controller1.a.equals("pressing")) {
                robot.moveToPos(NEXT_TO_STARTER_STACK_POS, 10);
            }
            if (controller1.b.equals("pressing")) {
                robot.adjustAndShoot();
            }
            if (controller1.y.equals("pressing")) {
                robot.moveToPos(new int[]{170, 90}, 3);
            }
            if (controller1.x.equals("pressing")) {
                robot.moveToPos(new int[]{100, 105}, 3);
            }

            /*
            if (controller1.dpad_down.equals("pressing")) {
                double[] list = Robot.lower.val;
                Robot.lower = new Scalar(list[0], list[1], list[2] - 2);
            }
            if (controller1.dpad_up.equals("pressing")) {
                double[] list = Robot.lower.val;
                Robot.lower = new Scalar(list[0], list[1], list[2] + 2);
            }

            robot.telemetry.addData("HSV MIN, MAX: ", Robot.lower + ", " + Robot.upper);
            robot.telemetry.update();
            */

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
    }

    public void madeIt(String s) {
        robot.telemetry.addData("Made it! Status: ", s);
        robot.telemetry.update();
        robot.wait(0.0);
    }
}

/*
while(opModeIsActive()) {
            if(stage == 2) {
                robot.moveToPos(NEXT_TO_STARTER_STACK_POS, 10);
                stage++;
            }
            /*switch (stage) {
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
 */