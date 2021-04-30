package org.firstinspires.ftc.teamcode.data;

// Contains the coordinates (x position + tower width) for useful locations on the field
public interface FieldPositions {

    // Smaller x values mean the robot will be further to the right (away from driver)
    // Larger x values mean the robot will be further to the left (closer to driver)

    int xOffset = 0;

    // Generally useful positions
    int[] PERFECT_LAUNCH_POS = new int[]{135, 58}; // Perfect launch position
    int[] SECOND_PERFECT_LAUNCH_POS = new int[]{136, 48}; // Perfect launch position
    int[] NEXT_TO_STARTER_STACK_POS = new int[]{94 + xOffset, 67}; // Where to go to avoid first starter stack

    int[] LEFT_MOVING_POWERSHOT_POS = new int[]{108, 58};
    int[] LEFT_POWERSHOT_POS = new int[]{97, 58};
    int[] MID_POWERSHOT_POS = new int[]{80, 60};
    int[] RIGHT_POWERSHOT_POS = new int[]{67, 58}; // - 6.5 degrees

    // Park positions (where to park during autonomous)
    int[] PARK_0_POS = new int[]{96 + xOffset, 74};
    int[] PARK_1_POS = new int[]{80 + xOffset, 74};
    int[] PARK_4_POS = new int[]{96 + xOffset, 74};

    // Position pertaining to the wobble goals
    int[] CONFIG_0_POS_I = new int[]{174 + xOffset, 85}; // Where to drop first wobble goal
    int[] CONFIG_0_POS_II = new int[]{154 + xOffset, 85}; // Where to drop second wobble goal

    int[] PRE_CONFIG_1_POS_I = new int[]{93 + xOffset, 85};
    int[] PRE_CONFIG_1_POS_II = new int[]{66 + xOffset, 85};
    int CONFIG_1_WALL_HEIGHT = 24;

    int[] PRE_CONFIG_4_POS_I = new int[]{180 + xOffset, 90};
    int[] PRE_CONFIG_4_POS_II = new int[]{157 + xOffset, 90};
    int CONFIG_4_WALL_HEIGHT = 24;

    int[] SECOND_WOBBLE_POS = new int[]{140 + xOffset, 45}; // Where to go before moving forward and grabbing 2nd wobble
    int SECOND_WOBBLE_WALL_HEIGHT = 75;

    int[] PRE_SECOND_WOBBLE_POS = new int[]{136, 52};
}
