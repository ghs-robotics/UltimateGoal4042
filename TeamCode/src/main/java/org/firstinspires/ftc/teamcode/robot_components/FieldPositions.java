package org.firstinspires.ftc.teamcode.robot_components;

public enum FieldPositions {

    // Generally useful positions
    PERFECT_LAUNCH_POS (140, 58),
    NEXT_TO_STARTER_STACK_POS (190, 60),
    PARK_POS (134, 73),
    RIGHT_POWERSHOT_POS (180, 58),

    // Position pertaining to the wobble goals
    CONFIG_0_POS_I (100, 80),
    CONFIG_0_POS_II (110, 80),

    CONFIG_1_POS_I (186, 110),
    CONFIG_1_POS_II (196, 110),

    CONFIG_4_POS_I (100, 110),
    CONFIG_4_POS_II (90, 110),

    SECOND_WOBBLE_POS (154, 45);

    private int blueTowerX;
    private int blueTowerW;
    private int frontGrayTileY;
    private int rearGrayTileY;

    FieldPositions(int blueTowerX, int blueTowerW) {
        this.blueTowerX = blueTowerX;
        this.blueTowerW = blueTowerW;

        // TODO : UPDATE
        this.frontGrayTileY = 0;
        this.rearGrayTileY = 0;
    }

    public int[] getBlueXWPos() {
        return new int[]{blueTowerX, blueTowerW};
    }

    public int getFrontGrayTileY() {
        return frontGrayTileY;
    }

    public int getRearGrayTileY() {
        return rearGrayTileY;
    }
}
