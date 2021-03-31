package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot_components.Controller;
import org.firstinspires.ftc.teamcode.robot_components.Robot;

@TeleOp(name="TeleKenny", group="Iterative Opmode")
public class TeleKenny extends OpMode
{
    //Declare OpMode members
    Robot robot;
    Controller controller1;
    Controller controller2;

    //Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);
        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);
        robot.resetServos();
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
        controller2.update();

        //Press "x" to toggle speed between 100% and 30%
        if (controller1.x.equals("pressing")) {
            robot.toggleSpeed();
        }

        //Mecanum wheel drive
        robot.calculateDrivePowers(
                controller1.left_stick_x,
                controller1.left_stick_y,
                controller1.right_stick_x
        );
        robot.updateDrive();

        //Press left bumper to turn on/off the shooter motor
        if (controller2.right_bumper.equals("pressing")) {
            robot.toggleShooter();
            robot.wait(0.5);
            robot.launchRing();
        }

        //Press "y" to turn on/off the intake motor
        if (controller1.y.equals("pressing")) {
            robot.toggleIntake();
        }

        //Press "b" to toggle the wobble gripper, could group together toggle wobble and turn arm?
        if (controller2.b.equals("pressing")) {
            robot.toggleGrab();
        }

        //Press "a" to turn the arm,
        if (controller2.a.equals("pressing")) {
            robot.turnArm();
        }

        //turn 180 might work?
        if(controller2.x.equals("pressing")); {
            robot.rotateToPos(180, 1);
        }

        //turn to face goal might work??
        if(controller2.y.equals("pressing")); {
            robot.rotateToPos(0, 1);
        }
        //chases tower...?
        if(controller2.left_bumper.equals("pressing")); {
            robot.setTargetToTower();
            robot.chaseTower();
            robot.wait(0.05);
        }

    }

    //Code to run ONCE after the driver hits STOP
    @Override
    public void stop(){}
}

//TO DO: functions that turns robot to face goal, adujust and shoot?
//2 shits: if button press, turn to face goal; if button press turn 180