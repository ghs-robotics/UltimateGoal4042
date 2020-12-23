package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

class Robot {
    double leftFrontPower = 0;
    double rightFrontPower = 0;
    double leftRearPower = 0;
    double rightRearPower = 0;
    double shooterPower = 0;
    double intakePower = 0;
    double speed = 1;

    DcMotor leftFrontDrive;
    DcMotor rightFrontDrive;
    DcMotor leftRearDrive;
    DcMotor rightRearDrive;
    DcMotor shooterMotor;
    DcMotor intakeMotor;

    ElapsedTime elapsedTime;
    Gyro gyro;
    Telemetry telemetry;

    //Creates a robot object with methods that we can use in both Auto and TeleOp
    Robot(HardwareMap hardwareMap, Telemetry telemetry) {

        //These are the names to use in the phone config (in quotes below)
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftRearDrive = hardwareMap.get(DcMotor.class, "leftRearDrive");
        rightRearDrive = hardwareMap.get(DcMotor.class, "rightRearDrive");
        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "shooterMotor");

        //Defines the forward direction for each of our motors
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftRearDrive.setDirection(DcMotor.Direction.FORWARD);
        rightRearDrive.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        //Declares some other useful tools for our robot (the gyroscope, the timer, etc.)
        gyro = new Gyro(hardwareMap);
        gyro.resetAngle();
        elapsedTime = new ElapsedTime();
        elapsedTime.reset();
        this.telemetry = telemetry;
    }

/*    //In case we actually need to use encoders on the drive motors
    void resetDrive() {
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontPower = 0;
        rightFrontPower = 0;
        leftRearPower = 0;
        rightRearPower = 0;
        updateDrive();
    } */

    //Updates the powers being sent to the drive motors
    void updateDrive() {
        //Adjusts powers for speed
        double LF = speed * leftFrontPower;
        double RF = speed * rightFrontPower;
        double LR = speed * leftRearPower;
        double RR = speed * rightRearPower;

        //Displays motor powers on the phone
        telemetry.addData("leftFrontPower", "" + LF);
        telemetry.addData("rightFrontPower", "" + RF);
        telemetry.addData("leftRearPower", "" + LR);
        telemetry.addData("rightRearPower", "" + RR);
        telemetry.addData("shooterPower", "" + shooterPower);
        telemetry.addData("intakePower", "" + intakePower);
        telemetry.update();

        //Sends desired power to drive motors
        leftFrontDrive.setPower(speed * leftFrontPower);
        rightFrontDrive.setPower(speed * rightFrontPower);
        leftRearDrive.setPower(speed * leftRearPower);
        rightRearDrive.setPower(speed * rightRearPower);
    }

    //Sets the drive speed to 30%
    void toggleSpeed() { speed = (speed == 1 ? 0.3 : 1); }

    //Turns the shooter motor on or off
    void toggleShooter() {
        shooterPower = (shooterPower == 0 ? 0.9 : 0);
        shooterMotor.setPower(shooterPower);
    }

    //Turns the intake motor on or off
    void toggleIntake() {
        intakePower = (intakePower == 0 ? 0.9 : 0);
        intakeMotor.setPower(intakePower);
    }

    //Resets the timer
    void resetElapsedTime() { elapsedTime.reset(); }

    //Returns how many seconds have passed since the timer was last reset
    double getElapsedTimeSeconds() { return elapsedTime.seconds(); }

    //Makes the robot wait (i.e. do nothing) for a specified number of seconds
    void wait(double seconds) {
        double start = getElapsedTimeSeconds();
        while (getElapsedTimeSeconds() - start < seconds) {}
    }
}

//https://ftctechnh.github.io/ftc_app/doc/javadoc/index.html