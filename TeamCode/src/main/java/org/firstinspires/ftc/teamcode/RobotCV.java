package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

class RobotCV {
    //HSV constants
    public static final Scalar LOWER_RING_HSV = new Scalar(74, 153, 144);
    public static final Scalar UPPER_RING_HSV = new Scalar(112, 242, 255);
    public static final Scalar LOWER_TOWER_HSV = new Scalar(0, 0, 0);
    public static final Scalar UPPER_TOWER_HSV = new Scalar(255, 255, 20);
    public static final Scalar LOWER_WOBBLE_HSV = new Scalar(0, 123, 25);
    public static final Scalar UPPER_WOBBLE_HSV = new Scalar(25, 210, 101);

    //CV detection variables
    int objectX = 0;
    int objectY = 0;
    int objectWidth = 0;
    int objectHeight = 0;

    //robot variables
    double leftFrontPower = 0;
    double rightFrontPower = 0;
    double leftRearPower = 0;
    double rightRearPower = 0;
    double shooterPower = 0;
    double intakePower = 0;
    double armAngle = 0.5;
    double grabAngle = 0.2;
    double shooterAngle = 0.05;
    double speed = 1;

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

    //CV objects
    OpenCvInternalCamera phoneCam;
    RingDeterminationPipeline pipeline;

    //Creates a robot object with methods that we can use in both Auto and TeleOp
    RobotCV(HardwareMap hardwareMap, Telemetry telemetry) {

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

        //initiating some CV variables/objects
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new RingDeterminationPipeline();
        phoneCam.setPipeline(pipeline);
    }

    void initCamera(){
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened() { startStreaming(); }
        });
    }

    void startStreaming(){ phoneCam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_RIGHT); }

    void stopStreaming(){ phoneCam.stopStreaming(); }

    void updateObjectValues(){
        objectX = pipeline.objectX;
        objectY = pipeline.objectY;
        objectWidth = pipeline.objectWidth;
        objectHeight = pipeline.objectHeight;
    }

    //Sets servos to starting positions
    void resetServos(){
        armServo.setPosition(armAngle);
        grabServo.setPosition(grabAngle);
        shooterServo.setPosition(shooterAngle);
    }

    String identifyRingConfig() {
        updateObjectValues();
        if (!(objectWidth == 0)) {
            return "" + Math.round((6.7 * objectHeight) / objectWidth);
        } else {
            return "0";
        }
    }

    //Updates the powers being sent to the drive motors
    void updateDrive() {
        //Adjusts powers for speed
        double LF = speed * leftFrontPower;
        double RF = speed * rightFrontPower;
        double LR = speed * leftRearPower;
        double RR = speed * rightRearPower;

        //Displays motor powers on the phone
//        telemetry.addData("leftFrontPower", "" + LF);
//        telemetry.addData("rightFrontPower", "" + RF);
//        telemetry.addData("leftRearPower", "" + LR);
//        telemetry.addData("rightRearPower", "" + RR);
        telemetry.addData("shooterPower", "" + shooterPower);
        telemetry.addData("armServo", "" + armAngle);
        telemetry.addData("grabServo", "" + grabAngle);
        telemetry.addData("shooterAngle", "" + shooterAngle);
        telemetry.update();

        //Sends desired power to drive motors
        leftFrontDrive.setPower(LF);
        rightFrontDrive.setPower(RF);
        leftRearDrive.setPower(LR);
        rightRearDrive.setPower(RR);
    }

    void chaseObject(double x, double y, int targetX, int targetY){
        double r = Math.hypot(x, y);
        double robotAngle = Math.atan2(y, x) - Math.PI / 4;
        double rotate = 0;
        leftFrontPower = r * Math.cos(robotAngle) + rotate;
        rightFrontPower = r * Math.sin(robotAngle) - rotate;
        leftRearPower = r * Math.sin(robotAngle) + rotate;
        rightRearPower = r * Math.cos(robotAngle) - rotate;

        //Adjusts powers for speed
        double LF = speed * leftFrontPower;
        double RF = speed * rightFrontPower;
        double LR = speed * leftRearPower;
        double RR = speed * rightRearPower;

        //Displays motor powers on the phone
//        telemetry.addData("leftFrontPower", "" + LF);
//        telemetry.addData("rightFrontPower", "" + RF);
//        telemetry.addData("leftRearPower", "" + LR);
//        telemetry.addData("rightRearPower", "" + RR);
        updateObjectValues();
        telemetry.addData("width, height", "( " + objectWidth + ", " + objectHeight + " )");
        telemetry.addData("objectX, targetX", "( " + objectX + ", " + targetX + " )");
        telemetry.addData("objectY, targetY", "( " + objectY + ", " + targetY + " )");
        telemetry.addData("x, y", "( " + x + ", " + y + " )");
        telemetry.update();

        //Sends desired power to drive motors
        leftFrontDrive.setPower(LF);
        rightFrontDrive.setPower(RF);
        leftRearDrive.setPower(LR);
        rightRearDrive.setPower(RR);
    }

    //Sets the drive speed to 30%
    void toggleSpeed() { speed = (speed == 1 ? 0.5 : 1); }

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

    void increaseArmAngle(){
        armAngle += 0.1;
        armServo.setPosition(armAngle);
    }

    void decreaseArmAngle(){
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

    //Resets the timer
    void resetElapsedTime() { elapsedTime.reset(); }

    //Returns how many seconds have passed since the timer was last reset
    double getElapsedTimeSeconds() { return elapsedTime.seconds(); }

    //Makes the robot wait (i.e. do nothing) for a specified number of seconds
    void wait(double seconds) {
        double start = getElapsedTimeSeconds();
        while (getElapsedTimeSeconds() - start < seconds) {}
    }

    public static class RingDeterminationPipeline extends OpenCvPipeline
    {
        //HSV constants
        public static final Scalar LOWER_RING_HSV = new Scalar(74, 153, 144);
        public static final Scalar UPPER_RING_HSV = new Scalar(112, 242, 255);
        public static final Scalar LOWER_TOWER_HSV = new Scalar(0, 0, 0);
        public static final Scalar UPPER_TOWER_HSV = new Scalar(255, 255, 20);
        public static final Scalar LOWER_WOBBLE_HSV = new Scalar(0, 123, 25);
        public static final Scalar UPPER_WOBBLE_HSV = new Scalar(25, 210, 101);

        public static final Scalar GREEN = new Scalar(0, 255, 0);
        public static final int SCREEN_HEIGHT = 240;
        public static final int SCREEN_WIDTH = 320;

        public Mat mask;
        public int objectX = 0;
        public int objectY = 0;
        public int objectWidth = 0;
        public int objectHeight = 0;

        @Override
        public void init(Mat firstFrame) {}

        @Override
        public Mat processFrame(Mat input)
        {
            //update ring coordinates
            int[] coords = getObjectCoordinates(input, LOWER_RING_HSV, UPPER_RING_HSV);
            objectX = coords[0];
            objectY = coords[1];
            objectWidth = coords[2];
            objectHeight = coords[3];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    new Point(objectX, objectY), // First point which defines the rectangle
                    new Point(objectX + objectWidth, objectY + objectHeight), // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }

        public int[] getObjectCoordinates(Mat input, Scalar lower, Scalar upper) {
            Scalar GREEN = new Scalar(0, 255, 0);

            Mat dst = new Mat();
            Mat src = input;
            Imgproc.resize(src, src, new Size(320, 240));

            Imgproc.rectangle(src, new Point(0,0), new Point(320, 0), GREEN, -1);

            Imgproc.cvtColor(src, dst, Imgproc.COLOR_BGR2HSV);
            Imgproc.GaussianBlur(dst, dst, new Size(5, 5), 80, 80);

            //adding a mask to the dst mat
            Core.inRange(dst, lower, upper, dst);

            //dilate the ring to make it easier to detect
            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.dilate(dst, dst, kernel);

            //get the contours of the ring
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(dst, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            //draw a contour on the src image
            Imgproc.drawContours(src, contours, -1, GREEN, 2, Imgproc.LINE_8, hierarchy, 2, new Point());

            for (int i = 0; i < contours.size(); i++) {
                Rect rect = Imgproc.boundingRect(contours.get(i));

                //don't draw a square around a spot that's too small
                //to avoid false detections
                if (rect.area() > 7_000) { Imgproc.rectangle(src, rect, GREEN, 5); }
            }

            Rect largest = new Rect();
            for (int i = 0; i < contours.size(); i++) {
                Rect rect = Imgproc.boundingRect(contours.get(i));

                if (largest.area() < rect.area()) { largest = rect; }
            }

            //draws largest rect
            Imgproc.rectangle(src, largest,GREEN, 5);

            return new int[]{largest.x,largest.y, largest.width, largest.height};
        }
    }
}

//https://ftctechnh.github.io/ftc_app/doc/javadoc/index.html