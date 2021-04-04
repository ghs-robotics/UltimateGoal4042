package org.firstinspires.ftc.teamcode.robot_components;

import org.opencv.core.Scalar;

// Interface containing HSV constants for the robot class
public interface HSVConstants {

    // Ring HSV values
    Scalar LOWER_RING_HSV = new Scalar(74, 153, 144); // original values: 74, 153, 144
    Scalar UPPER_RING_HSV = new Scalar(112, 242, 255); // original values: 112, 242, 255

    // Tower goal HSV values
    Scalar LOWER_TOWER_HSV = new Scalar(0, 80, 100); // original values: 0, 124, 60
    Scalar UPPER_TOWER_HSV = new Scalar(80, 255, 220); // original values: 54, 212, 255

    // Wobble goal HSV values
    Scalar LOWER_WOBBLE_HSV = new Scalar(0, 117, 0);
    Scalar UPPER_WOBBLE_HSV = new Scalar(77, 255, 97);

}

