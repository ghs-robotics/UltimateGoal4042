package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

/**
 * This will be the main class that we will put all of our autonomous code into
 * This class is obviously not complete, and the structure of the code is not decided yet
 */
@Autonomous
public class Auto1 extends LinearOpMode {
    //Declare OpMode members
    Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        robot.init();
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();

        robot.identifyRingConfig();

        while(opModeIsActive()){
            robot.identifyRingConfig();
            robot.updateDrive();
        }

        if (opModeIsActive()) {
            //move towards the desired target wobble goal zone

            //once there, place down the wobble goal

            //aim the robot at the goal and make sure that the robot is within the launch zone

            //shoot 3 shots

            //pickup 3 rings

            telemetry.addData("opmode is working","");
            telemetry.update();
        }
    }
}
