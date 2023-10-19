/*
 * Copyright (c) 2023 Sebastian Erives
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.CodeabotCommon;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class TeamGameElementDetermination implements VisionProcessor {
    public Scalar blueLower = new Scalar(0, 0, 140);
    public Scalar blueUpper = new Scalar(255, 130, 255);
    public Scalar redLower = new Scalar(0, 150, 0);
    public Scalar redUpper = new Scalar(255, 255, 130);
    private Scalar lower = new Scalar(0, 0, 0);
    private Scalar upper = new Scalar(255, 255, 255);

    public enum Position {
        LEFT,
        CENTER,
        RIGHT
    }

    private class DetectionRegion {
        public Rect regionRect;
        public double score = 0;
        public double totalScore = 0;

        private final Mat cvtMat = new Mat();
        private final Mat binaryMat = new Mat();
        private final Mat maskedMat = new Mat();

        public DetectionRegion(Rect regionRect) {
            this.regionRect = regionRect;
        }


        public void processFrame(Mat frame) {
            Mat submat = frame.submat(regionRect);

            Imgproc.cvtColor(submat, cvtMat, Imgproc.COLOR_RGB2YCrCb);
            Core.inRange(cvtMat, lower, upper, binaryMat);

            maskedMat.release();
            Core.bitwise_and(submat, submat, maskedMat, binaryMat);
            maskedMat.copyTo(submat);

            score = (double) Core.countNonZero(binaryMat) / binaryMat.total();
            totalScore += score;
        }
    }

    public Rect leftRect = new Rect(24, 160, 106, 110);
    public Rect centerRect = new Rect(280, 180, 80, 100);
    public Rect rightRect = new Rect(540, 210, 95, 110);

    DetectionRegion leftRegion = new DetectionRegion(leftRect);
    DetectionRegion centerRegion = new DetectionRegion(centerRect);
    DetectionRegion rightRegion = new DetectionRegion(rightRect);

    Telemetry telemetry;
    public CodeabotCommon.Alliance alliance = CodeabotCommon.Alliance.BLUE;
    int width;
    int height;

    public TeamGameElementDetermination(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public TeamGameElementDetermination(Telemetry telemetry, CodeabotCommon.Alliance alliance) {
        this.telemetry = telemetry;
        this.alliance = alliance;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        this.width = width;
        this.height = height;
        if (alliance == CodeabotCommon.Alliance.BLUE) {
            lower = blueLower;
            upper = blueUpper;
        } else {
            lower = redLower;
            upper = redUpper;
        }
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        leftRegion.processFrame(frame);
        centerRegion.processFrame(frame);
        rightRegion.processFrame(frame);

        Position position = getPosition();

        telemetry.addData("Position", position);
        telemetry.addData("Left Score", leftRegion.score);
        telemetry.addData("Center Score", centerRegion.score);
        telemetry.addData("Right Score", rightRegion.score);
        telemetry.update();

        return position;
    }

    public Position getPosition() {
        if (leftRegion.totalScore > centerRegion.totalScore && leftRegion.totalScore > rightRegion.totalScore) {
            return Position.LEFT;
        } else if (centerRegion.totalScore > leftRegion.totalScore && centerRegion.totalScore > rightRegion.totalScore) {
            return Position.CENTER;
        } else {
            return Position.RIGHT;
        }
    }

    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }

    private void drawRegion(Canvas canvas, float scale, DetectionRegion region, boolean selected) {
        final Paint paint = new Paint();
        paint.setColor(Color.BLUE);
        paint.setStyle(Paint.Style.STROKE);
        if (selected) {
            paint.setColor(Color.GREEN);
            paint.setStrokeWidth(5);
        }

        canvas.drawRect(makeGraphicsRect(region.regionRect, scale), paint);
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx,
                            float scaleCanvasDensity, Object userContext) {
        Position position = (Position) userContext;
        drawRegion(canvas, scaleBmpPxToCanvasPx, leftRegion, position == Position.LEFT);
        drawRegion(canvas, scaleBmpPxToCanvasPx, centerRegion, position == Position.CENTER);
        drawRegion(canvas, scaleBmpPxToCanvasPx, rightRegion, position == Position.RIGHT);
    }
}
