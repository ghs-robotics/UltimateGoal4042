package org.firstinspires.ftc.teamcode.cv_objects;

import org.firstinspires.ftc.teamcode.robot_components.CVDetectionPipeline;

public class FieldWall extends CVObject {

    public FieldWall(CVDetectionPipeline pipeline) {
        super("wall", pipeline);
        lowerHSV = LOWER_WALL_HSV;
        upperHSV = UPPER_WALL_HSV;
        cover = 0;
    }

    @Override
    protected void coverBackground() {
        super.coverBackground();
    }

    // Testing to make sure the detected object is the wall close up
    @Override
    protected boolean isReasonable(int x, int y, int w, int h) {
        return (w > 1 && w < 12 && h > 8 && h < 23); // TODO : THESE ARE PLACEHOLDERS
    }
}
