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

package org.firstinspires.ftc.teamcode.test_opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot_components.Controller;
import org.firstinspires.ftc.teamcode.robot_components.Robot;

// THIS CLASS IS FOR TESTING PURPOSES!!

@TeleOp(name="Tele2", group="Iterative Opmode")
public class Tele2 extends OpMode {
    //Declare OpMode members
    Robot robot;
    Controller controller1;
    //Controller controller2;
    String target = "none";

    //Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);
        controller1 = new Controller(gamepad1);
        robot.init();
        robot.setIntakeSideToBeForward();

        robot.setTargetToRing();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    //Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    @Override
    public void init_loop() {}

    //Code to run ONCE when the driver hits PLAY
    @Override
    public void start() { robot.resetElapsedTime(); }

    //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        //Registers controller input
        controller1.update();
        robot.updateObjectValues();

        //Mecanum wheel drive
        robot.calculateDrivePowers(
                controller1.left_stick_x,
                controller1.left_stick_y,
                controller1.right_stick_x
        );

        if (controller1.x.equals("pressing")) {
            target = "none";
        }

        if (controller1.a.equals("pressing")) {
            target = "ring";
            robot.setTargetToRing(33, 80);
        }

        if (controller1.y.equals("pressing")) {
            target = "ring";
            robot.setTargetToRing(33, 80);
        }

        if (controller1.b.equals("pressing")) {
            target = "ring";
            robot.setTargetToRing(63, 60);
        }

        if (controller1.left_bumper.equals("pressing")) { robot.wPID.k_P -= 0.005; }
        if (controller1.right_bumper.equals("pressing")) { robot.wPID.k_P += 0.005; }

        if (controller1.dpad_down.equals("pressing")) { robot.wPID.k_I -= 0.0005; }
        if (controller1.dpad_up.equals("pressing")) { robot.wPID.k_I += 0.0005; }

        if (controller1.dpad_left.equals("pressing")) { robot.wPID.k_D -= 0.0005; }
        if (controller1.dpad_right.equals("pressing")) { robot.wPID.k_D += 0.0005; }


        if (target.equals("none")){ robot.updateDrive(); }
        if (target.equals("wobble")){ robot.chaseWobble(); }
        if (target.equals("ring")){ robot.chaseRing(); }
        if (target.equals("tower")){ robot.chaseTower(); }
    }

    //Code to run ONCE after the driver hits STOP
    @Override
    public void stop(){}
}