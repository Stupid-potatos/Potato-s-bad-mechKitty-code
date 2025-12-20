package org.firstinspires.ftc.teamcode.Potato_Assets;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.util.concurrent.atomic.AtomicReference;

/**
 * A class mode for streaming the camera vision feed to the Panels.
 * This is primary used for testing to see what the camera sees.
 *
 * Took from an example source on a website (forgot where i got it from)
 */
public class PanelsStreamProcessor implements VisionProcessor, CameraStreamSource {

    private final AtomicReference<Bitmap> lastFrame =
            new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Bitmap bitmap = Bitmap.createBitmap(
                frame.width(),
                frame.height(),
                Bitmap.Config.RGB_565
        );

        Utils.matToBitmap(frame, bitmap);
        lastFrame.set(bitmap);
        return null;
    }

    @Override
    public void onDrawFrame(
            Canvas canvas,
            int onscreenWidth,
            int onscreenHeight,
            float scaleBmpPxToCanvasPx,
            float scaleCanvasDensity,
            Object userContext
    ) {
        // Not needed for PanelsCameraStream
    }

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(consumer -> consumer.accept(lastFrame.get()));
    }
}
