package org.firstinspires.ftc.teamcode.data;

// Interface containing HSV constants for the robot class
public interface HSVConstants {

    // Tower goal HSV values
    MyScalar LOWER_BLACK_TOWER_HSV = new MyScalar(0, 0, 0);
    MyScalar UPPER_BLACK_TOWER_HSV = new MyScalar(255, 255, 80);

    // Tower goal HSV values
    MyScalar LOWER_BLUE_TOWER_HSV = new MyScalar(0, 80, 100);
    MyScalar UPPER_BLUE_TOWER_HSV = new MyScalar(80, 255, 220); // 80, 255, 220

    // Floor HSV values
    MyScalar LOWER_FLOOR_HSV = new MyScalar(0, 0, 0);
    MyScalar UPPER_FLOOR_HSV = new MyScalar(255, 85, 175);

    // Ring HSV values
    MyScalar LOWER_RING_HSV = new MyScalar(74, 123, 94); // original values: 74, 153, 144
    MyScalar UPPER_RING_HSV = new MyScalar(112, 255, 255); // original values: 112, 242, 255

    // Ring stack HSV values (they're slightly different because it's the webcam, not the phonecam)
    MyScalar LOWER_STACK_HSV = new MyScalar(60, 103, 54); // original values: 74, 153, 144
    MyScalar UPPER_STACK_HSV = new MyScalar(130, 255, 255); // original values: 112, 242, 255

    // Blue column part of the tower goal
    MyScalar LOWER_TWIN_HSV = new MyScalar(0, 80, 60);
    MyScalar UPPER_TWIN_HSV = new MyScalar(80, 255, 220);

    // Wall HSV values TODO : THESE VALUES ARE PLACEHOLDERS
    MyScalar LOWER_WALL_HSV = new MyScalar(70, 75, 95);
    MyScalar UPPER_WALL_HSV = new MyScalar(130, 255, 255);

    // Wobble goal HSV values
//    MyScalar LOWER_WOBBLE_HSV = new MyScalar(0, 0, 0);
//    MyScalar UPPER_WOBBLE_HSV = new MyScalar(255, 255, 25);
    MyScalar LOWER_WOBBLE_HSV = new MyScalar(0, 80, 100); // TODO : TEST THESE VALUES
    MyScalar UPPER_WOBBLE_HSV = new MyScalar(80, 255, 220);

    // For image processing (e.g. drawing rectangles)
    MyScalar GREEN_BGR = new MyScalar(0, 255, 0); // THIS ONE IS BGR

    // Some constants for the screen size
    int SCREEN_HEIGHT = 240;
    int SCREEN_WIDTH = 320;
}

