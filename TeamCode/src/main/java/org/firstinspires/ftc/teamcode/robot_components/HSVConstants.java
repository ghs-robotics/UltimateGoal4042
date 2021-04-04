package org.firstinspires.ftc.teamcode.robot_components;

import org.opencv.core.Scalar;

// Interface containing HSV constants for the robot class
public interface HSVConstants {

    // Ring HSV values
    Scalar LOWER_RING_HSV = new Scalar(74, 123, 94); // original values: 74, 153, 144
    Scalar UPPER_RING_HSV = new Scalar(112, 255, 255); // original values: 112, 242, 255

    // Ring stack HSV values (they're slightly different because it's the webcam, not the phonecam)
    Scalar LOWER_STACK_HSV = new Scalar(65, 93, 64); // original values: 74, 153, 144
    Scalar UPPER_STACK_HSV = new Scalar(120, 255, 255); // original values: 112, 242, 255

    // Tower goal HSV values
    Scalar LOWER_TOWER_HSV = new Scalar(0, 80, 100);
    Scalar UPPER_TOWER_HSV = new Scalar(80, 255, 220);

    // Wobble goal HSV values
    Scalar LOWER_WOBBLE_HSV = new Scalar(0, 0, 0);
    Scalar UPPER_WOBBLE_HSV = new Scalar(255, 255, 25);

    // For image processing (e.g. drawing rectangles)
    Scalar GREEN_BGR = new Scalar(0, 255, 0); // THIS ONE IS BGR

}

