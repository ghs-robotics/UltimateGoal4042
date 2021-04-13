package org.firstinspires.ftc.teamcode.data;

import org.opencv.core.Scalar;

// Interface containing HSV constants for the robot class
public interface HSVConstants {

    // Floor HSV values
    Scalar LOWER_FLOOR_HSV = new Scalar(0, 0, 0);
    Scalar UPPER_FLOOR_HSV = new Scalar(255, 85, 175);

    // Ring HSV values
    Scalar LOWER_RING_HSV = new Scalar(74, 123, 94); // original values: 74, 153, 144
    Scalar UPPER_RING_HSV = new Scalar(112, 255, 255); // original values: 112, 242, 255

    // Ring stack HSV values (they're slightly different because it's the webcam, not the phonecam)
    Scalar LOWER_STACK_HSV = new Scalar(60, 103, 54); // original values: 74, 153, 144
    Scalar UPPER_STACK_HSV = new Scalar(130, 255, 255); // original values: 112, 242, 255

    // Tower goal HSV values
    Scalar LOWER_TOWER_HSV = new Scalar(0, 80, 100);
    Scalar UPPER_TOWER_HSV = new Scalar(80, 255, 220); // 80, 255, 220

    // Wall HSV values TODO : THESE VALUES ARE PLACEHOLDERS
    Scalar LOWER_WALL_HSV = new Scalar(70, 75, 95);
    Scalar UPPER_WALL_HSV = new Scalar(130, 255, 255);

    // Wobble goal HSV values
//    Scalar LOWER_WOBBLE_HSV = new Scalar(0, 0, 0);
//    Scalar UPPER_WOBBLE_HSV = new Scalar(255, 255, 25);
    Scalar LOWER_WOBBLE_HSV = new Scalar(0, 80, 100); // TODO : TEST THESE VALUES
    Scalar UPPER_WOBBLE_HSV = new Scalar(80, 255, 220);

    // For image processing (e.g. drawing rectangles)
    Scalar GREEN_BGR = new Scalar(0, 255, 0); // THIS ONE IS BGR

    // Some constants for the screen size
    int SCREEN_HEIGHT = 240;
    int SCREEN_WIDTH = 320;
}

