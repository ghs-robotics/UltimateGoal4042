/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot_components.Controller;
import org.firstinspires.ftc.teamcode.robot_components.Robot;

@TeleOp(name="Tele1", group="Linear Opmode")
public class Tele1 extends LinearOpMode {

    // Declare OpMode members
    Robot robot;
    Controller controller1;
    Controller controller2;

    @Override
    public void runOpMode() {

        // Code to run ONCE when the driver hits INIT
        robot = new Robot(hardwareMap, telemetry);
        controller1 = new Controller(gamepad1); // Whoever presses start + a
        controller2 = new Controller(gamepad2); // Whoever presses start + b
        robot.init();
        robot.setTargetToTower();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot.switchDriveDirection(); // Default is when the front is the launcher side

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        robot.resetElapsedTime();

        while (opModeIsActive()) {
            // Registers controller input
            controller1.update();
            controller2.update();
            robot.updateObjectValues();

            // Constantly adjusts launch velocity
            if (robot.powerLauncher.running) {
                robot.powerLauncher.adjustShooterVelocity();
            }

            // -----------------------------------------------------------------------------------------
            // -----------------------------------------------------------------------------------------
            // ------------------------------   CONTROLLER 1 FUNCTIONS   -------------------------------
            // -----------------------------------------------------------------------------------------
            // -----------------------------------------------------------------------------------------

            // NOTE: TO USE THESE FUNCTIONS PRESS START A

            // Press "a" to toggle the intake motor
            if (controller1.a.equals("pressing")) {
                robot.toggleIntake();
            }

            // Press "b" to flip the drive direction
            if (controller1.b.equals("pressing")) {
                robot.switchDriveDirection();
            }

            // Press "x" to toggle speed between 100% and 50%
            if (controller1.x.equals("pressing")) {
                robot.toggleSpeed();
            }

            // Press "y" to launch 3 rings
            if (controller1.y.equals("pressing")) {
                robot.launchRings(3);
            }

            // Mecanum wheel drive
            robot.calculateDrivePowers(
                    controller1.left_stick_x,
                    controller1.left_stick_y,
                    controller1.right_stick_x
            );
            robot.updateDrive(); // Also updates telemetry

            // Rotate to face tower goal
            if (controller1.dpad_up.equals("pressing")) {
                robot.rotateToPos(0,1);
            }

            // Rotate to face away from tower goal
            if (controller1.dpad_down.equals("pressing")) {
                robot.rotateToPos(180,1);
            }

            /*
            //chases tower...?
            if(controller2.left_bumper.equals("pressing")); {
                robot.setTargetToTower();
                robot.chaseTower();
                robot.wait(0.05);
            }
            */

            // -----------------------------------------------------------------------------------------
            // -----------------------------------------------------------------------------------------
            // ------------------------------   CONTROLLER 2 FUNCTIONS   -------------------------------
            // -----------------------------------------------------------------------------------------
            // -----------------------------------------------------------------------------------------

            // NOTE: TO USE THESE FUNCTIONS, PRESS START B

            // Right bumper toggles the claw
            if (controller2.right_bumper.equals("pressing")) {
                robot.toggleClaw();
            }

            // Left bumper turns arm
            if (controller2.left_bumper.equals("pressing")) {
                robot.turnArm();
            }

            // Pressing x moves indexer servo
            if (controller2.x.equals("pressing")) {
//            robot.launchRings(1);
                robot.powerLauncher.index();
            }

            // Toggle launch motors on/off
            if (controller2.b.equals("pressing")) {
                robot.powerLauncher.toggle();
            }

            // Angle the launcher up a bit
            if (controller2.dpad_up.equals("pressing")) {
                robot.powerLauncher.changeLaunchAngle(0.002);
            }

            // Angle the launcher down a bit
            if (controller2.dpad_down.equals("pressing")) {
                robot.powerLauncher.changeLaunchAngle(-0.002);
            }
        }
    }
}
