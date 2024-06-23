package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class PixelDetectorBB implements VisionProcessor {

    private static int spike_position = 0;

    OpMode OpMode;
    Mat leftRegion = new Mat();
    Mat rightRegion = new Mat();
    int leftRegion_avg = -1, rightRegion_avg = -1;
    Mat YCrCb = new Mat();
    Mat Y = new Mat();

    int frameWidth;
    int frameHeight;

    private final int LEFT_REGION_START_X = 400;
    private final int LEFT_REGION_START_Y = 80;

    private final int RIGHT_REGION_START_X = 930;
    private final int RIGHT_REGION_START_Y = 130;

    private final int REGIONS_WIDTH = 100;
    private final int REGIONS_HEIGHT = 150;

    private final int BRIGHTNESS_DIFFERENCE = 17;

    public int getLeftRegion_avg() {
        return this.leftRegion_avg;
    }
    public int getRightRegion_avg() {
        return this.rightRegion_avg;
    }
    public PixelDetectorBB(OpMode opMode){
        this.OpMode = opMode;
    }
    public static int getSpike_position() {
        return spike_position;
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        frameWidth = frame.width();
        frameHeight = frame.height();

        Imgproc.cvtColor(frame, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Y, 2); // 0 - white | 1 - red | 2 -



        Imgproc.rectangle(frame, new Point(LEFT_REGION_START_X,LEFT_REGION_START_Y), new Point(LEFT_REGION_START_X+REGIONS_WIDTH,
                LEFT_REGION_START_Y+REGIONS_HEIGHT), new Scalar(255, 0, 0)); // left

        Imgproc.rectangle(frame, new Point(RIGHT_REGION_START_X, RIGHT_REGION_START_Y),new Point(RIGHT_REGION_START_X+REGIONS_WIDTH
                , RIGHT_REGION_START_Y +REGIONS_HEIGHT), new Scalar(255, 0, 0)); // right

        leftRegion = Y.submat(new Rect(LEFT_REGION_START_X, LEFT_REGION_START_Y, REGIONS_WIDTH, REGIONS_HEIGHT));
        rightRegion = Y.submat(new Rect(RIGHT_REGION_START_X, RIGHT_REGION_START_Y, REGIONS_WIDTH, REGIONS_HEIGHT));

        leftRegion_avg = (int) Core.mean(leftRegion).val[0];
        rightRegion_avg = (int) Core.mean(rightRegion).val[0];

        if (leftRegion_avg - rightRegion_avg >= BRIGHTNESS_DIFFERENCE)  {
            spike_position = 0;
        }
        else if (rightRegion_avg - leftRegion_avg >= BRIGHTNESS_DIFFERENCE) {
            spike_position = 2;
        }
        else {
            spike_position = 1;
        }

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }
}
