package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
//questions: what angle to set/move servos to
class RobotButWorse {
    double leftFrontPower = 0;
    double rightFrontPower = 0;
    double leftRearPower = 0;
    double rightRearPower = 0;
    double intakePower = 0;
    double speed = 1;
    double grabAngle = 0;

    DcMotor leftFrontDrive;
    DcMotor rightFrontDrive;
    DcMotor leftRearDrive;
    DcMotor rightRearDrive;
    DcMotor intakeMotor;
    Servo grabServo;

    ElapsedTime elapsedTime;
    Telemetry telemetry;

    RobotButWorse(HardwareMap hardwareMap, Telemetry telemetry) {
        //@Imants, these are the names you should use in the config on the phones!!! (i.e. "leftFrontDrive" etc.)
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftRearDrive = hardwareMap.get(DcMotor.class, "leftRearDrive");
        rightRearDrive = hardwareMap.get(DcMotor.class, "rightRearDrive");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftRearDrive.setDirection(DcMotor.Direction.FORWARD);
        rightRearDrive.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        grabServo.setDirection(Servo.Direction.FORWARD);

        elapsedTime = new ElapsedTime();
        elapsedTime.reset();
        this.telemetry = telemetry;
    }

    void updateDrive() {
        //displays power of motors on phone
        telemetry.addData("leftFrontPower", "LF = " + leftFrontPower);
        telemetry.addData("rightFrontPower", "RF = " + rightFrontPower);
        telemetry.addData("leftRearPower", "LR = " + leftRearPower);
        telemetry.addData("rightRearPower", "RR = " + rightRearPower);
        telemetry.addData("intakePower", "intakePower = " + intakePower);
        telemetry.update();

        leftFrontDrive.setPower(speed * leftFrontPower);
        rightFrontDrive.setPower(speed * rightFrontPower);
        leftRearDrive.setPower(speed * leftRearPower);
        rightRearDrive.setPower(speed * rightRearPower);
    }

    void toggleSpeed() {
        if (speed == 1) {
            speed = 0.3;
        } else {
            speed = 1;
        }
    }

    void toggleIntake() {
        if (intakePower == 0) {
            intakePower = 1;
        } else {
            intakePower = 0;
        }
        intakeMotor.setPower(intakePower);
    }
//values need calibration
     void toggleGrab() {
        if (grabAngle == 0) {
            grabAngle = 0.5;
        } else {
            grabAngle = 0;
        }
        grabServo.setPosition(grabAngle);
    }

    void resetElapsedTime() {
        elapsedTime.reset();
    }

    double getElapsedTimeSeconds() {
        return elapsedTime.seconds();
    }

    void wait(double seconds) {
        double start = getElapsedTimeSeconds();
        while (getElapsedTimeSeconds() - start < seconds) {}
    }
}

//https://ftctechnh.github.io/ftc_app/doc/javadoc/index.html