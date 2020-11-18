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
    double intakePower = 0;
    double speed = 1;

    DcMotor leftFrontDrive;
    DcMotor rightFrontDrive;
    DcMotor leftRearDrive;
    DcMotor rightRearDrive;
    DcMotor intakeMotor;

    ElapsedTime elapsedTime;
    Telemetry telemetry;

    Robot(HardwareMap hardwareMap, Telemetry telemetry) {
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

        elapsedTime = new ElapsedTime();
        elapsedTime.reset();
        this.telemetry = telemetry;
    }

    void updateBallDrive() {
        telemetry.addData("leftFrontPower", "LF = " + leftFrontPower);
        telemetry.addData("rightFrontPower", "RF = " + rightFrontPower);
        telemetry.addData("leftFrontPower", "LR = " + leftRearPower);
        telemetry.addData("rightFrontPower", "RR = " + rightRearPower);
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