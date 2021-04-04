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
import org.firstinspires.ftc.teamcode.robot_components.DriveMode;
import org.firstinspires.ftc.teamcode.robot_components.FieldPositions;
import org.firstinspires.ftc.teamcode.robot_components.Robot;

@TeleOp(name="Tele1", group="Linear Opmode")
public class Tele1 extends LinearOpMode implements FieldPositions {

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
        int queue = 0; // Keeps track of how many rings are "in line" to be shot

        robot.init();
        robot.setTargetToStack(); // TODO : CHANGE BACK TO TOWER
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot.setForwardDirection("intake"); // Default is when the front is the launcher side
        DriveMode.setController(controller1);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        robot.resetGyroAngle();
        robot.resetElapsedTime();

        while (opModeIsActive()) {

            // -----------------------------------------------------------------------------------------
            // -----------------------------------------------------------------------------------------
            // --------------------------------   CONTINUOUS UPDATES   ---------------------------------
            // -----------------------------------------------------------------------------------------
            // -----------------------------------------------------------------------------------------

            // Registers controller input
            controller1.update();
            controller2.update();
            robot.updateObjectValues();

            // Constantly adjusts launch velocity
            if (robot.powerLauncher.running) {
                robot.powerLauncher.adjustShooterVelocity();
            }

            // Checks if any rings need to be shot and takes care of indexing
            if (queue > 0) {
                queue = robot.powerLauncher.handleQueue(queue);
            }

            // -----------------------------------------------------------------------------------------
            // -----------------------------------------------------------------------------------------
            // ------------------------------   CONTROLLER 1 FUNCTIONS   -------------------------------
            // -----------------------------------------------------------------------------------------
            // -----------------------------------------------------------------------------------------

            // NOTE: TO USE THESE FUNCTIONS PRESS START A

            //
            if (controller1.a.equals("pressing")) {
            }

            //
            if (controller1.b.equals("pressing")) {
            }

            // Toggle speed between 100% and 50%
            if (controller1.x.equals("pressing")) {
                robot.toggleSpeed();
            }

            //
            if (controller1.y.equals("pressing")) {
            }

            // Go to perfect launch position and set launch angle
            if (controller1.left_bumper.equals("pressing")) {
                robot.moveToPos(PERFECT_LAUNCH_POS, 2.0);
                robot.powerLauncher.setPerfectLaunchAngle();
                robot.setForwardDirection("intake");
            }

            // Adjust and shoot
            if (controller1.right_bumper.equals("pressing")) {
                robot.adjustAndShoot(3);
                robot.setForwardDirection("intake");
            }

            // Mecanum wheel drive
            DriveMode.update();
            robot.calculateDrivePowers(
                    controller1.left_stick_x * DriveMode.getFactor(),
                    controller1.left_stick_y * DriveMode.getFactor(),
                    controller1.right_stick_x * DriveMode.getFactor()
            );

            robot.updateDrive(); // Also updates telemetry

            // Flip the drive direction
            if (controller1.right_stick_button.equals("pressing")) {
                robot.switchDriveDirection();
            }

            // Toggles between strong/smooth mode
            if (controller1.left_stick_button.equals("pressing")) {
                DriveMode.switchMode();
            }

            // Rotate to face tower goal (north)
            if (controller1.dpad_up.equals("pressing")) {
                robot.rotateToPos(180,1);
            }

            // Rotate to face away from tower goal (south)
            if (controller1.dpad_down.equals("pressing")) {
                robot.rotateToPos(0,1);
            }

            // Rotate to face east
            if (controller1.dpad_right.equals("pressing")) {
                robot.rotateToPos(90,1);
            }

            // Rotate to face west
            if (controller1.dpad_left.equals("pressing")) {
                robot.rotateToPos(-90,1);
            }

            // Reset gyro in case of emergency
            if (controller1.left_trigger + controller1.right_trigger > 1.8) {
                robot.resetGyroAngle();
            }

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

            // Launch one ring
            if (controller2.a.equals("pressing")) {
                queue++;
            }

            // Toggle launcher
            if (controller2.b.equals("pressing")) {
                robot.powerLauncher.toggle();
            }

            // Index TODO : NOT NEEDED
            if (controller2.x.equals("pressing")) {
                robot.powerLauncher.index();
            }

            //
            if (controller2.y.equals("pressing")) {
            }

            // Run intake with right joystick
            robot.runIntake(controller2.right_stick_y);

            // Angle the launcher up a tiny bit and set to default angle
            if (controller2.dpad_up.equals("pressing")) {
                robot.powerLauncher.changeLaunchAngle(-0.001);
                robot.powerLauncher.setCurrentAngleToDefault();
            }

            // Angle the launcher down a tiny bit and set to default angle
            if (controller2.dpad_down.equals("pressing")) {
                robot.powerLauncher.changeLaunchAngle(0.001);
                robot.powerLauncher.setCurrentAngleToDefault();
            }

            // Set current launch angle to the default TODO : REPEATED
            if (controller2.dpad_right.equals("pressing")) {
                robot.powerLauncher.setCurrentAngleToDefault();
            }

            // Set current launch angle to the default TODO : REPEATED
            if (controller2.dpad_left.equals("pressing")) {
                robot.powerLauncher.setCurrentAngleToDefault();
            }

            // Change current launch Angle (but keep default the same)
            if (controller2.left_stick_y != 0) {
                robot.powerLauncher.changeLaunchAngleGradually(controller2.left_stick_y / 200);
            }

            // Go to default launch angle
            if (controller2.left_stick_button.equals("pressed")) {
                robot.powerLauncher.setPerfectLaunchAngle();
            }
        }
    }
}
