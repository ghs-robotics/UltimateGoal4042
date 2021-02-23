package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

class Robot {
    //HSV constants
    public static final Scalar LOWER_RING_HSV = new Scalar(74, 153, 144); //original values: 74, 153, 144
    public static final Scalar UPPER_RING_HSV = new Scalar(112, 242, 255); //original values: 112, 242, 255
    public static final Scalar LOWER_TOWER_HSV = new Scalar(0, 124, 30); //original value: V = 60, V=40 works well
    public static final Scalar UPPER_TOWER_HSV = new Scalar(54, 212, 255);
    public static final Scalar LOWER_WOBBLE_HSV = new Scalar(0, 117, 0);
    public static final Scalar UPPER_WOBBLE_HSV = new Scalar(77, 255, 97);

    //CV detection variables
    public static Scalar lower = LOWER_RING_HSV;
    public static Scalar upper = UPPER_RING_HSV;
    public static double cover = 0;

    int targetX = 100;
    int targetY = 140;
    int targetWidth = 95;
    int objectX = 0;
    int objectY = 0;
    int objectWidth = 0;
    int objectHeight = 0;
    double x = 0;
    double y = 0;
    double targetAngle = 0; //gyroscope will target this angle

    String currentTargetObject = "ring";

    //Robot variables and objects
    double leftFrontPower = 0;
    double rightFrontPower = 0;
    double leftRearPower = 0;
    double rightRearPower = 0;
    double shooterPower = 0;
    double intakePower = 0;
    double armAngle = 0.5;
    double grabAngle = 0.25; //Angle of 0.25 means closed
    double shooterAngle = 0.05;
    double speed = 1;
    double config = 0;

    double previousShooterMotorTicks = 0;
    double previousElapsedTime = 0;

    DcMotor leftFrontDrive;
    DcMotor rightFrontDrive;
    DcMotor leftRearDrive;
    DcMotor rightRearDrive;
    DcMotor shooterMotor;
    DcMotor intakeMotor;
    Servo armServo;
    Servo grabServo;
    Servo shooterServo;

    ElapsedTime elapsedTime;
    Gyro gyro;
    Telemetry telemetry;

    //PID controllers
    PIDController xPID;
    PIDController yPID;
    PIDController wPID;
    PIDController gyroPID;

    //CV objects
    OpenCvInternalCamera phoneCam;
    ObjectDeterminationPipeline pipeline;

    //Creates a robot object with methods that we can use in both Auto and TeleOp
    Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        //These are the names to use in the phone config (in quotes below)
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftRearDrive = hardwareMap.get(DcMotor.class, "leftRearDrive");
        rightRearDrive = hardwareMap.get(DcMotor.class, "rightRearDrive");
        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        armServo = hardwareMap.get(Servo.class, "armServo");
        grabServo = hardwareMap.get(Servo.class, "grabServo");
        shooterServo = hardwareMap.get(Servo.class, "shooterServo");

        //Defines the forward direction for each of our motors/servos
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftRearDrive.setDirection(DcMotor.Direction.FORWARD);
        rightRearDrive.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        armServo.setDirection(Servo.Direction.FORWARD);
        grabServo.setDirection(Servo.Direction.FORWARD);
        shooterServo.setDirection(Servo.Direction.FORWARD);

        //Declares some other useful tools for our robot (the gyroscope, the timer, etc.)
        gyro = new Gyro(hardwareMap);
        gyro.resetAngle();
        elapsedTime = new ElapsedTime();
        elapsedTime.reset();
        this.telemetry = telemetry;

        //Initiating PID objects
        xPID = new PIDController(0.0120, 0.0022, 0.0015, 3);
        yPID = new PIDController(0.0200, 0.0025, 0.0010, 3);
        wPID = new PIDController(0.0450, 0.0015, 0.0020, 2); //0.0440, 0.0016, 0.0010
        gyroPID = new PIDController(0.0330, 0.0000, 0.0020, 2); //works best when Ki = 0

        //Initiating some CV variables/objects
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId",
                "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(
                OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new ObjectDeterminationPipeline();
        phoneCam.setPipeline(pipeline);
    }

    //To use at the start of each OpMode that uses CV
    void init() {
        resetServos();
        initCamera();
    }

    //Initialize the phone camera
    void initCamera() {
        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                startStreaming();
            }
        });
    }

    //Start streaming frames on the phone camera
    void startStreaming() {
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_RIGHT);
    }

    //Stop streaming frames on the phone camera
    void stopStreaming() {
        phoneCam.stopStreaming();
    }

    //Updates the coordinates of the object being detected on the screen
    void updateObjectValues() {
        objectX = pipeline.objectX;
        objectY = pipeline.objectY;
        objectWidth = pipeline.objectWidth;
        objectHeight = pipeline.objectHeight;
    }

    //Switches the object that the robot is trying to detect to a ring
    void setTargetToRing(int x, int y) {
        currentTargetObject = "ring";
        xPID.resetValues();
        yPID.resetValues();
        cover = 0;
        lower = LOWER_RING_HSV;
        upper = UPPER_RING_HSV;
        targetX = x;
        targetY = y;
    }

    //Switches the object that the robot is trying to detect to the wobble goal
    void setTargetToWobble(int x, int y) {
        currentTargetObject = "wobble";
        xPID.resetValues();
        yPID.resetValues();
        cover = 0;
        lower = LOWER_WOBBLE_HSV;
        upper = UPPER_WOBBLE_HSV;
        targetX = x;
        targetY = y;
    }

    //Switches the object that the robot is trying to detect to the tower goal
    void setTargetToTower(int x, int w) {
        currentTargetObject = "tower";
        xPID.resetValues();
        wPID.resetValues();
        cover = 0;
        lower = LOWER_TOWER_HSV;
        upper = UPPER_TOWER_HSV;
        targetX = x;
        targetWidth = w;
    }

    void setTargetToRing() { setTargetToRing(100, 140); }
    void setTargetToWobble() { setTargetToWobble(60, 160); }
    void setTargetToTower() { setTargetToTower(65, 95); }

    //Sets servos to starting positions
    void resetServos() {
        armServo.setPosition(armAngle);
        grabServo.setPosition(grabAngle);
        shooterServo.setPosition(shooterAngle);
    }

    //Makes the robot stop driving
    void stopDrive() {
        leftFrontPower = 0;
        rightFrontPower = 0;
        leftRearPower = 0;
        leftRearPower = 0;
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftRearDrive.setPower(leftRearPower);
        rightRearDrive.setPower(rightRearPower);
    }

    //Calculates powers for mecanum wheel drive
    void calculateDrivePowers(double x, double y, double rotation) {
        double r = Math.hypot(x, y);
        double robotAngle = Math.atan2(y, x) - Math.PI / 4;
        leftFrontPower = Range.clip(r * Math.cos(robotAngle) + rotation, -1.0, 1.0) * speed;
        rightFrontPower = Range.clip(r * Math.sin(robotAngle) - rotation, -1.0, 1.0) * speed;
        leftRearPower = Range.clip(r * Math.sin(robotAngle) + rotation, -1.0, 1.0) * speed;
        rightRearPower = Range.clip(r * Math.cos(robotAngle) - rotation, -1.0, 1.0) * speed;
    }

    //Sends desired power to drive motors
    void sendDrivePowers() {
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftRearDrive.setPower(leftRearPower);
        rightRearDrive.setPower(rightRearPower);
    }

    //Updates the powers being sent to the drive motors
    void updateDrive() {
        //Displays motor powers on the phone
        telemetry.addData("shooterPower", "" + shooterPower);
        telemetry.addData("armServo", "" + armAngle);
        telemetry.addData("grabServo", "" + grabAngle);
        telemetry.addData("shooterAngle", "" + shooterAngle);
        telemetry.addData("angle", "" + gyro.getAngle());
        telemetry.addData("config: ", "" + config);
        telemetry.update();
        sendDrivePowers();
    }

//    public void adjustAngle() {
//        calculateDrivePowers(0, 0, -gyroPID.calcVal(targetAngle - gyro.getAngle()));
//        sendDrivePowers();
//        telemetry.addData("angle: ", gyro.getAngle() + " ( target: " + targetAngle + " )");
//        telemetry.addData("Kp: ", gyroPID.k_P);
//        telemetry.addData("Ki: ", gyroPID.k_I);
//        telemetry.addData("Kd: ", gyroPID.k_D);
//        telemetry.update();
//    }

    //Displays important values on the phone screen; don't ever call this method from another class
    void chaseObject(double x, double y) {
        calculateDrivePowers(x, y, 0);
        sendDrivePowers();

        String t = currentTargetObject;

        if (t.equals("tower")) {
            telemetry.addData("towerX = ", objectX + " (target = " + targetX + ")");
            telemetry.addData("towerY = ", objectY);
            telemetry.addData("towerWidth = ", objectWidth + " (target = " + targetWidth + ")");
            telemetry.addData("towerHeight = ", objectHeight);
        } else {
            telemetry.addData(t + "X = ", objectX + " (target = " + targetX + ")");
            telemetry.addData(t + "Y = ", objectY + " (target = " + targetY + ")");
            telemetry.addData("width = ", objectWidth);
            telemetry.addData("height = ", objectHeight);
        }

        telemetry.addData("(x, y)", "( " + x + ", " + y + " )");
        telemetry.addData("HSV MIN, MAX: ", lower + ", " + upper);
        telemetry.addData("cover: ", cover);
        telemetry.update();
    }

    //Makes the robot chase the closest ring
    void chaseRing() {
        if (!currentTargetObject.equals("ring")) {
            setTargetToRing();
        }
        updateObjectValues();

        x = xPID.calcVal(targetX - objectX);
        y = -yPID.calcVal(targetY - objectY);

        double h = objectHeight;
        double w = objectWidth;
        double r = 1.0 * w / h;

        //Testing to make sure the detected object is a ring
        if (!(h > 10 && h < 45 && w > 22 && w < 65 && r > 1.2 && r < 2.5)) {
            x = 0;
            y = 0;
        }

        chaseObject(x, y);
    }

    //Makes the robot chase the wobble goal
    void chaseWobble() {
        if (!currentTargetObject.equals("wobble")) {
            setTargetToWobble();
        }
        updateObjectValues();

        double dy = targetY - objectY;
        double dx = targetX - objectX;

        //Adjust x and y values according to how far away the robot is from the target
        if (Math.abs(dx) < 10) {
            x = 0;
        } else {
            x += Range.clip(dx / 400.0, -0.2, 0.2);
        }
        if (Math.abs(dy) < 10) {
            y = 0;
        } else {
            y -= Range.clip(dy / 400.0, -0.2, 0.2);
        }

        //Make sure the robot doesn't go too fast
        y = Range.clip(y, -0.6, 0.6);
        x = Range.clip(x, -0.6, 0.6);

        double h = objectHeight;
        double w = objectWidth;

        //Testing to make sure the detected object is a wobble goal
        if (!(h > 10 && h < 200 && w > 10 && w < 200)) {
            x = 0;
            y = 0;
        }

        chaseObject(x, y);
    }

    public void chaseTower() {
        if (!currentTargetObject.equals("tower")) {
            setTargetToTower();
        }
        updateObjectValues();

        x = xPID.calcVal(targetX - objectX);
        y = -wPID.calcVal(targetWidth - objectWidth);

        if (!(objectWidth > 60 && objectWidth < 220)) {
            x = 0;
            y = 0;
        }

        calculateDrivePowers(0, y, -gyroPID.calcVal(targetAngle - gyro.getAngle()));
        sendDrivePowers();

        String t = currentTargetObject;

        telemetry.addData("(x, y)", "( " + x + ", " + y + " )");
//        telemetry.addData("Kp_x: ", xPID.k_P);
//        telemetry.addData(t + "X = ", objectX + " (target = " + targetX + ")");
        telemetry.addData("Kp: ", wPID.k_P);
        telemetry.addData("Ki: ", wPID.k_I);
        telemetry.addData("Kd: ", wPID.k_D);
        telemetry.addData(t + "W = ", objectWidth + " (target = " + targetWidth + ")");
        telemetry.addData("y = ", objectY);
        telemetry.addData("height = ", objectHeight);
        telemetry.addData("HSV MIN, MAX: ", lower + ", " + upper);
        telemetry.update();
    }

    //Makes the robot align with the tower goal
    void chaseTower2() {
        if (!currentTargetObject.equals("tower")) {
            setTargetToTower();
        }
        updateObjectValues();
//        if (objectHeight < 10 && cover > 0) { cover -= 0.004; }
//        else if (objectHeight > 15) { cover += 0.004; }
//        if (cover > 0.5) { cover = 0.2; }

        double dw = targetWidth - objectWidth;
        double dx = targetX - objectX;

        if (Math.abs(dx) < 3) {
            x = 0;
        } else {
            x += Range.clip(dx / 400.0, -0.2, 0.2);
        }
        if (Math.abs(dw) < 3) {
            y = 0;
        } else {
            y -= Range.clip(dw / 240.0, -0.2, 0.2);
        }

        y = Range.clip(y, -0.6, 0.6);
        x = Range.clip(x, -0.6, 0.6);

        if (!(objectWidth > 60 && objectWidth < 220)) {
            x = 0;
            y = 0;
        }

        chaseObject(x, y);
    }

    //Sets the drive speed to 30%
    void toggleSpeed() {
        speed = (speed == 1 ? 0.5 : 1);
    }

    //Turns the shooter motor on or off
    void toggleShooter() {
        shooterPower = (shooterPower == 0 ? 1.0 : 0);
        shooterMotor.setPower(shooterPower);
    }

    //Turns the intake motor on or off
    void toggleIntake() {
        intakePower = (intakePower == 0 ? 1 : 0);
        intakeMotor.setPower(intakePower);
    }

    //Turns the arm
    void turnArm() {
        armAngle = (armAngle == 0.88 ? 0.5 : 0.88);
        armServo.setPosition(armAngle);
    }

    //Toggles the wobble gripper
    void toggleGrab() {
        grabAngle = (grabAngle == 0.25 ? 0 : 0.25);
        grabServo.setPosition(grabAngle);
    }

    //Launches a ring by moving the shooterServo
    void launchRing() {
        shooterAngle = 0.25;
        shooterServo.setPosition(shooterAngle);
        wait(0.5);
        shooterAngle = 0.05;
        shooterServo.setPosition(shooterAngle);
    }

    void increaseArmAngle() {
        armAngle += 0.1;
        armServo.setPosition(armAngle);
    }

    void decreaseArmAngle() {
        armAngle -= 0.1;
        armServo.setPosition(armAngle);
    }

    //Calculates shooter motor speed in ticks per second
    double findShooterVelocity() {
        double deltaTicks = (shooterMotor.getCurrentPosition() - previousShooterMotorTicks);
        double deltaTime = elapsedTime.seconds() - previousElapsedTime;
        previousShooterMotorTicks = shooterMotor.getCurrentPosition();
        previousElapsedTime = elapsedTime.seconds();
        return (deltaTicks / deltaTime);
    }

    void pickUpWobbleGoal(double distance) {
        double motorPower = -0.4 * (distance / Math.abs(distance));
        double moveTime = distance / 100; //Adjust this later
        turnArm();
        toggleGrab();
        calculateDrivePowers(0, motorPower, 0);
        wait(0.6);
        sendDrivePowers();
        wait(moveTime);
        stopDrive();
        toggleGrab();
        wait(0.6);
        turnArm();
    }

    //Resets the timer
    void resetElapsedTime() {
        elapsedTime.reset();
    }

    //Returns how many seconds have passed since the timer was last reset
    double getElapsedTimeSeconds() {
        return elapsedTime.seconds();
    }

    //Makes the robot wait (i.e. do nothing) for a specified number of seconds
    void wait(double seconds) {
        double start = getElapsedTimeSeconds();
        while (getElapsedTimeSeconds() - start < seconds) {
        }
    }

    //Classifies the starter stack
    void identifyRingConfig() {
        setTargetToRing();
        updateObjectValues();
        if (!(objectWidth == 0)) {
            config = 1.0 * objectHeight / objectWidth;
        }
    }
}
//https://ftctechnh.github.io/ftc_app/doc/javadoc/index.html