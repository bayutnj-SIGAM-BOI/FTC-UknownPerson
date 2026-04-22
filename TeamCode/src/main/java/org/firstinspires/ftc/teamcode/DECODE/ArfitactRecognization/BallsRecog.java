package org.firstinspires.ftc.teamcode.DECODE.ArfitactRecognization;

import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvPipeline;

public class BallsRecog extends OpenCvPipeline {
    static final Scalar GREEN_LOW = new Scalar(35, 80, 60);
    static final Scalar GREEN_HIGH = new Scalar(85, 255, 255);
    static final Scalar PURPLE_LOW = new Scalar(125, 60, 60);
    static final Scalar PURPLE_HIGH = new Scalar(160, 255, 255);
    private volatile BallTarget bestTarget = null;
    @Override
    public Mat processFrame(Mat input) {


        return input;
    }

    private BallTarget ClosesBall() {
        BallTarget best = null;
        return best;
    }

    static class BallTarget {
        final String color;
        final double area, centerX, centerY;
        BallTarget(String color, double area, double cx, double cy) {
            this.color = color; this.area = area;
            this.centerX = cx; this.centerY = cy;
        }
    }
}
