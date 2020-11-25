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

    static final double X_TICKS_PER_INCH = 54.000; // MUST BE CALIBRATED!!
    static final double Y_TICKS_PER_INCH = 59.529; // MUST BE CALIBRATED!!

    DcMotor leftFrontDrive;
    DcMotor rightFrontDrive;
    DcMotor leftRearDrive;
    DcMotor rightRearDrive;
    DcMotor intakeMotor;

    ElapsedTime elapsedTime;
    Gyro gyro;
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

        gyro = new Gyro(hardwareMap);
        gyro.resetAngle();
        elapsedTime = new ElapsedTime();
        elapsedTime.reset();
        this.telemetry = telemetry;
    }

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

    //moves to the (x,y) position relative to current location
    void move(double x, double y){
        resetDrive();
        double targetX = x * X_TICKS_PER_INCH;
        double targetY = y * Y_TICKS_PER_INCH;
        double forwardPower;
        while (true) {
            double dy = targetY - leftFrontDrive.getCurrentPosition();
            forwardPower = 0.0013 * dy;
            //do stuff
            updateDrive();
            if (dy < 20) {
                break;
            }
        }
        resetDrive();
        wait(0.1);

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