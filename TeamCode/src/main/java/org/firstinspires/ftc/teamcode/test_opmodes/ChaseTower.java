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

package org.firstinspires.ftc.teamcode.test_opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot_components.Controller;
import org.firstinspires.ftc.teamcode.robot_components.Robot;

@TeleOp
public class ChaseTower extends LinearOpMode
{
    Robot robot;
    Controller controller1;
    int stage = 1;
    double t = 0;

    @Override
    public void runOpMode()
    {
        robot = new Robot(hardwareMap, telemetry);
        controller1 = new Controller(gamepad1);
        robot.init();
        robot.setTargetToTower();

        waitForStart();

        robot.setTargetToTower(95,80);
        robot.updateObjectValues();

        robot.adjustAndShoot();

        while (opModeIsActive()) {
//            if (stage == 1) {
//                if (Math.abs(robot.targetWidth - robot.objectWidth) > 5
//                        || Math.abs(robot.targetX - robot.objectX) > 5) {
//                    robot.chaseTower();
//                } else {
//                    robot.toggleShooter();
//                    t = robot.getElapsedTimeSeconds();
//                    stage++;
//                }
//            }
//
//            if (stage == 2) {
//                if ((robot.leftRearPower != 0
//                        || robot.rightRearPower != 0
//                        || robot.leftFrontPower != 0
//                        || robot.rightFrontPower != 0)
//                        && robot.elapsedTime.seconds() - t < 5) {
//                    robot.chaseTower();
//                } else {
//                    robot.stopDrive();
//                    for (int i = 0; i < 3; i++) {
//                        robot.launchRing();
//                    }
//                    robot.toggleShooter();
//                    stage++;
//                }
//            }
            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
    }

}