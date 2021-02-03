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

class Robot
{
    //HSV constants
    public static final Scalar LOWER_RING_HSV = new Scalar(74, 133, 130); //original values: 74, 153, 144
    public static final Scalar UPPER_RING_HSV = new Scalar(112, 242, 255); //original values: 112, 242, 255
    public static final Scalar LOWER_TOWER_HSV = new Scalar(0, 0, 0);
    public static final Scalar UPPER_TOWER_HSV = new Scalar(255, 255, 12);
    public static final Scalar LOWER_WOBBLE_HSV = new Scalar(0,117,0);
    public static final Scalar UPPER_WOBBLE_HSV = new Scalar(77,255,97);

    //CV detection variables
    public static Scalar lower = LOWER_RING_HSV;
    public static Scalar upper = UPPER_RING_HSV;
    public static double cover = 0;

    int targetX = 100;
    int targetY = 140;
    int targetWidth = 75;
    int objectX = 0;
    int objectY = 0;
    int objectWidth = 0;
    int objectHeight = 0;
    double x = 0;
    double y = 0;

    String currentTargetObject = "ring";

    //Robot variables and objects
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

    //CV objects
    OpenCvInternalCamera phoneCam;
    ObjectDeterminationPipeline pipeline;

    //Creates a robot object with methods that we can use in both Auto and TeleOp
    Robot(HardwareMap hardwareMap, Telemetry telemetry)
    {
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
        xPID = new PIDController(0.0120, 0.0022, 0.0015, 3, -1.0, 1.0); //0.0120, 0.0022, 0.0015
        yPID = new PIDController(0.0200, 0.0025, 0.0010, 3, -1.0, 1.0); //Kp = 0.0200, 0.0025, 0.0010

        //Initiating some CV variables/objects
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new ObjectDeterminationPipeline();
        phoneCam.setPipeline(pipeline);
    }

    //To use at the start of each OpMode that uses CV
    void init()
    {
        resetServos();
        initCamera();
    }

    //Initialize the phone camera
    void initCamera()
    {
        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened() { startStreaming(); }
        });
    }

    //Start streaming frames on the phone camera
    void startStreaming(){ phoneCam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_RIGHT); }

    //Stop streaming frames on the phone camera
    void stopStreaming(){ phoneCam.stopStreaming(); }

    //Updates the coordinates of the object being detected on the screen
    void updateObjectValues()
    {
        objectX = pipeline.objectX;
        objectY = pipeline.objectY;
        objectWidth = pipeline.objectWidth;
        objectHeight = pipeline.objectHeight;
    }

    //Switches the object that the robot is trying to detect with computer vision
    void setTargetTo(String s)
    {
        currentTargetObject = s;
        cover = 0;
        xPID.resetValues();
        yPID.resetValues();

        if (s.equals("ring")){
            lower = LOWER_RING_HSV;
            upper = UPPER_RING_HSV;
            targetX = 100;
            targetY = 140;
        } else if (s.equals("tower")) {
            lower = LOWER_TOWER_HSV;
            upper = UPPER_TOWER_HSV;
            targetX = 65; //95
            targetWidth = 95; //75
            cover = 0.23;
        } else if (s.equals("wobble")) {
            lower = LOWER_WOBBLE_HSV;
            upper = UPPER_WOBBLE_HSV;
            targetX = 60;
            targetY = 160;
        }
    }

    //Sets servos to starting positions
    void resetServos()
    {
        armServo.setPosition(armAngle);
        grabServo.setPosition(grabAngle);
        shooterServo.setPosition(shooterAngle);
    }

    //Makes the robot stop driving
    void stopDrive()
    {
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
    void calculateDrivePowers(double x, double y, double rotation)
    {
        double r = Math.hypot(x, y);
        double robotAngle = Math.atan2(y, x) - Math.PI / 4;
        leftFrontPower = Range.clip(r * Math.cos(robotAngle) + rotation, -1.0, 1.0) * speed;
        rightFrontPower = Range.clip(r * Math.sin(robotAngle) - rotation, -1.0, 1.0) * speed;
        leftRearPower = Range.clip(r * Math.sin(robotAngle) + rotation, -1.0, 1.0) * speed;
        rightRearPower = Range.clip(r * Math.cos(robotAngle) - rotation, -1.0, 1.0) * speed;
    }

    //Sends desired power to drive motors
    void sendDrivePowers()
    {
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftRearDrive.setPower(leftRearPower);
        rightRearDrive.setPower(rightRearPower);
    }

    //Updates the powers being sent to the drive motors
    void updateDrive()
    {
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

    //Displays important values on the phone screen; don't ever call this method from another class
    void chaseObject(double x, double y)
    {
        calculateDrivePowers(x, y, 0);
        sendDrivePowers();

        String t = currentTargetObject;

        if (t.equals("tower")){
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
    void chaseRing()
    {
        if (!currentTargetObject.equals("ring")) { setTargetTo("ring"); }
        updateObjectValues();

        x = xPID.calcVal(targetX - objectX);
        y = -yPID.calcVal(targetY - objectY);

        double h = objectHeight;
        double w = objectWidth;
        double r = 1.0 * w / h;

        //Testing to make sure the detected object is a ring
        if ( !(h > 10 && h < 45 && w > 22 && w < 65 && r > 1.2 && r < 2.5) ) {
            x = 0;
            y = 0;
        }

        calculateDrivePowers(x, y, 0);
        sendDrivePowers();

        String t = currentTargetObject;

        telemetry.addData("(x, y)", "( " + x + ", " + y + " )");
        telemetry.addData("Kd_x: ", xPID.k_D);
        telemetry.addData("Kd_y: ", yPID.k_D);
        telemetry.addData(t + "X = ", objectX + " (target = " + targetX + ")");
        telemetry.addData(t + "Y = ", objectY + " (target = " + targetY + ")");
        telemetry.addData("width = ", objectWidth);
        telemetry.addData("height = ", objectHeight);
        telemetry.addData("HSV MIN, MAX: ", lower + ", " + upper);
        telemetry.addData("cover: ", cover);
        telemetry.update();
    }

    //Makes the robot chase the wobble goal
    void chaseWobble()
    {
        if (!currentTargetObject.equals("wobble")) { setTargetTo("wobble"); }
        updateObjectValues();

        double dy = targetY - objectY;
        double dx = targetX - objectX;

        //Adjust x and y values according to how far away the robot is from the target
        if (Math.abs(dx) < 10) { x = 0; } else { x += Range.clip(dx / 400.0, -0.2, 0.2); }
        if (Math.abs(dy) < 10) { y = 0; } else { y -= Range.clip(dy / 400.0, -0.2, 0.2); }

        //Make sure the robot doesn't go too fast
        y = Range.clip(y, -0.6, 0.6);
        x = Range.clip(x, -0.6, 0.6);

        double h = objectHeight;
        double w = objectWidth;

        //Testing to make sure the detected object is a wobble goal
        if ( !(h > 10 && h < 200 && w > 10 && w < 200)){
            x = 0;
            y = 0;
        }

        chaseObject(x, y);
    }

    public void chaseTower()
    {
        if (!currentTargetObject.equals("tower")) { setTargetTo("tower"); }
        updateObjectValues();

        x = xPID.calcVal(targetX - objectX);
        y = -yPID.calcVal(targetWidth - objectWidth);

        if (!(objectWidth > 60 && objectWidth < 220)) {
            x = 0;
            y = 0;
        }

        chaseObject(x, y);
    }

    //Makes the robot align with the tower goal
    void chaseTower2()
    {
        if (!currentTargetObject.equals("tower")) { setTargetTo("tower"); }
        updateObjectValues();
//        if (objectHeight < 10 && cover > 0) { cover -= 0.004; }
//        else if (objectHeight > 15) { cover += 0.004; }
//        if (cover > 0.5) { cover = 0.2; }

        double dw = targetWidth - objectWidth;
        double dx = targetX - objectX;

        if (Math.abs(dx) < 3) { x = 0; } else { x += Range.clip(dx / 400.0, -0.2, 0.2); }
        if (Math.abs(dw) < 3) { y = 0; } else { y -= Range.clip(dw / 240.0, -0.2, 0.2); }

        y = Range.clip(y, -0.6, 0.6);
        x = Range.clip(x, -0.6, 0.6);

        if (!(objectWidth > 60 && objectWidth < 220)) {
            x = 0;
            y = 0;
        }

        chaseObject(x, y);
    }

    //Sets the drive speed to 30%
    void toggleSpeed() { speed = (speed == 1 ? 0.5 : 1); }

    //Turns the shooter motor on or off
    void toggleShooter()
    {
        shooterPower = (shooterPower == 0 ? 1.0 : 0);
        shooterMotor.setPower(shooterPower);
    }

    //Turns the intake motor on or off
    void toggleIntake()
    {
        intakePower = (intakePower == 0 ? 1 : 0);
        intakeMotor.setPower(intakePower);
    }

    //Turns the arm
    void turnArm()
    {
        armAngle = (armAngle == 0.88 ? 0.5 : 0.88);
        armServo.setPosition(armAngle);
    }

    //Toggles the wobble gripper
    void toggleGrab()
    {
        grabAngle = (grabAngle == 0.25 ? 0 : 0.25);
        grabServo.setPosition(grabAngle);
    }

    //Launches a ring by moving the shooterServo
    void launchRing()
    {
        shooterAngle = 0.25;
        shooterServo.setPosition(shooterAngle);
        wait(0.5);
        shooterAngle = 0.05;
        shooterServo.setPosition(shooterAngle);
    }

    void increaseArmAngle()
    {
        armAngle += 0.1;
        armServo.setPosition(armAngle);
    }

    void decreaseArmAngle()
    {
        armAngle -= 0.1;
        armServo.setPosition(armAngle);
    }

    //Calculates shooter motor speed in ticks per second
    double findShooterVelocity()
    {
        double deltaTicks = (shooterMotor.getCurrentPosition() - previousShooterMotorTicks);
        double deltaTime = elapsedTime.seconds() - previousElapsedTime;
        previousShooterMotorTicks = shooterMotor.getCurrentPosition();
        previousElapsedTime = elapsedTime.seconds();
        return (deltaTicks / deltaTime);
    }

    void pickUpWobbleGoal() {
        turnArm();
        toggleGrab();
        toggleGrab();
        calculateDrivePowers(0, -0.4, 0);
        wait(0.6);
        sendDrivePowers();
        wait(1.6);
        stopDrive();
        toggleGrab();
        turnArm();
    }

    //Resets the timer
    void resetElapsedTime() { elapsedTime.reset(); }

    //Returns how many seconds have passed since the timer was last reset
    double getElapsedTimeSeconds() { return elapsedTime.seconds(); }

    //Makes the robot wait (i.e. do nothing) for a specified number of seconds
    void wait(double seconds)
    {
        double start = getElapsedTimeSeconds();
        while (getElapsedTimeSeconds() - start < seconds) {}
    }

    //Classifies the starter stack
    void identifyRingConfig()
    {
        setTargetTo("ring");
        updateObjectValues();
        if (!(objectWidth == 0)) { config = 1.0 * objectHeight / objectWidth; }
    }

    //Detects the position of the target object on the screen and returns an array with those values
    public static int[] getObjectCoordinates(Mat input)
    {
        Scalar GREEN = new Scalar(0, 255, 0);

        Mat dst = new Mat();
        Mat src = input;
        Imgproc.resize(src, src, new Size(320, 240));

        //Cover up background noise
        Imgproc.rectangle(src, new Point(0,0), new Point(320, (int) (Robot.cover * 240)), GREEN, -1);

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

        for (int i = 0; i < contours.size(); i++)
        {
            Rect rect = Imgproc.boundingRect(contours.get(i));

            //don't draw a square around a spot that's too small
            //to avoid false detections
            //if (rect.area() > 7_000) { Imgproc.rectangle(src, rect, GREEN, 5); }
        }

        Rect largest = new Rect();
        for (int i = 0; i < contours.size(); i++)
        {
            Rect rect = Imgproc.boundingRect(contours.get(i));
            if (largest.area() < rect.area()) { largest = rect; }
        }

        //draws largest rect
        Imgproc.rectangle(src, largest, GREEN, 5);

        return new int[]{largest.x, largest.y, largest.width, largest.height};
    }

    //A class that does a lot of background processing for CV
    public static class ObjectDeterminationPipeline extends OpenCvPipeline
    {
        public static final Scalar GREEN = new Scalar(0, 255, 0);
        public static final int SCREEN_HEIGHT = 240;
        public static final int SCREEN_WIDTH = 320;

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
            int[] coords = Robot.getObjectCoordinates(input);
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
    }

    public class PIDController
    {
        public double k_P; //change to private later
        public double k_I;
        public double k_D;
        private double p_error;
        private double i_error;
        private double d_error;
        private double toleranceRadius; //PID won't adjust within this range
        private double min;
        private double max;
        private double prevError;
        private double prevTime; //measured in seconds!
        private ElapsedTime time;

        public PIDController(double Kp, double Ki, double Kd, double tolerance, double min, double max)
        {
            k_P = Kp;
            k_I = Ki;
            k_D = Kd;
            toleranceRadius = tolerance;
            this.min = min;
            this.max = max;
            time = new ElapsedTime();
            resetValues();
        }

        public void resetValues(){
            p_error = 0;
            i_error = 0;
            d_error = 0;
            prevError = 0;
            prevTime = 0;
            time.reset();
        }

        public double calcVal(double error)
        {
            //If the error is small enough, the robot won't adjust
            if (Math.abs(error) < toleranceRadius) { return 0; }

            //Calculate the different errors
            double deltaTime = time.seconds() - prevTime;
            p_error = error;
            i_error += error * deltaTime;
            d_error = (error - prevError) / deltaTime;

            //Update the "prev" variables for the next loop
            prevError = error;
            prevTime = time.seconds();

            //Return the PID value
            return (k_P * p_error + k_I * i_error + k_D * d_error);
        }
    }
}

//https://ftctechnh.github.io/ftc_app/doc/javadoc/index.html