
package org.firstinspires.ftc.teamcode.vision;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class ColorDetection implements VisionProcessor {

    /*
     * These are our variables that will be
     * modifiable from the variable tuner.
     *
     * Scalars in OpenCV are generally used to
     * represent color. So our values in the
     * lower and upper Scalars here represent
     * the Y, Cr and Cb values respectively.
     *
     * YCbCr, like most color spaces, range
     * from 0-255, so we default to those
     * min and max values here for now, meaning
     * that all pixels will be shown.
     */
    private static final int W = 1920;
    private static final int H = 1080;

    public static int lowY = 30;
    public static int lowCr = 50;
    public static int lowCb = 50;
    public static int highY = 50;
    public static int highCr = 120;
    public static int highCb = 250;
    public Scalar lower = new Scalar(lowY, lowCr, lowCb);
    public Scalar upper = new Scalar(highY, highCr, highCb);

    /**
     * This will allow us to choose the color
     * space we want to use on the live field
     * tuner instead of hardcoding it
     */
    public ColorSpace colorSpace = ColorSpace.YCrCb;
    private Mat ycrcbMat = new Mat();
    private Mat binaryMat = new Mat();
    private Mat maskedInputMat = new Mat();

    private Telemetry telemetry = null;

    public enum Location {
        LEFT, MIDDLE, RIGHT
    }

    Location location;

    enum ColorSpace {
        /*
         * Define our "conversion codes" in the enum
         * so that we don't have to do a switch
         * statement in the processFrame method.
         */
        RGB(Imgproc.COLOR_RGBA2RGB),
        HSV(Imgproc.COLOR_RGB2HSV),
        YCrCb(Imgproc.COLOR_RGB2YCrCb),
        Lab(Imgproc.COLOR_RGB2Lab);

        //store cvtCode in a public var
        public int cvtCode = 0;

        //constructor to be used by enum declarations above
        ColorSpace(int cvtCode) {
            this.cvtCode = cvtCode;
        }
    }

    public ColorDetection(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Mat processFrame(Mat frame, long captureTimeNanos) {

        Scalar lower = new Scalar(lowY, lowCr, lowCb);
        Scalar upper = new Scalar(highY, highCr, highCb);


        Imgproc.cvtColor(frame, ycrcbMat, colorSpace.cvtCode);

        Core.inRange(ycrcbMat, lower, upper, binaryMat);

        maskedInputMat.release();

        Core.bitwise_and(frame, frame, maskedInputMat, binaryMat);

        //use binary mat from here
        List<MatOfPoint> countersList = new ArrayList<>();
        Imgproc.findContours(binaryMat, countersList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(binaryMat, countersList, 0, new Scalar(0, 0, 255));

        Rect hat = new Rect(new Point(0, 0), new Point(1, 1));

        for (MatOfPoint countor : countersList) {

            Rect rect = Imgproc.boundingRect(countor);
            if (rect.area() > hat.area()) {
                hat = rect;
            }

        }
        Imgproc.rectangle(maskedInputMat, hat, new Scalar(255, 255, 255));
        //Imgproc.rectangle(Mat.zeros( H, W, CvType.CV_8UC3 ), hat, new Scalar(255, 255, 255));
        line( Mat.zeros( H, W, CvType.CV_8UC3 ), new Point( hat.x, hat.y ), new Point( hat.x+hat.width, hat.y+hat.width ) );

        int theta;
        int centerX = hat.x + hat.width;

        telemetry.update();

        maskedInputMat.copyTo(frame);
        return frame;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
    }

    public Location getLocation() {
        return location;
    }

    private void line( Mat img, Point start, Point end ) {
        int thickness = 2;
        int lineType = 8;
        int shift = 0;

        Imgproc.line( img,
                start,
                end,
                new Scalar( 0, 0, 0 ),
                thickness,
                lineType,
                shift );
    }
}
