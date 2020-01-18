package com.edinaftc.library.vision;

import android.app.Activity;
import android.graphics.Bitmap;
import android.widget.FrameLayout;
import android.widget.LinearLayout;

import com.qualcomm.robotcore.util.ThreadPool;
import com.vuforia.CameraCalibration;
import com.vuforia.CameraDevice;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import java.nio.ByteBuffer;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.ExecutorService;

public class VuforiaCamera2 extends VuforiaCamera {
    public static final String TAG = "VuforiaCamera2";

    public static final String VUFORIA_LICENSE_KEY = "ASA9XvT/////AAABmUnq30r9sU3Nmf/+RS+Xx0CHgJj/JtD5ycahnuM/0B2SFvbMRPIZCbLi4LeOkfse9Dymor5W7vNMYI+vmqVx9kpEaKE8VM7cFMUb/T1LLwlCPdX9QKOruzTcRdlYswR7ULh4K11GuFZDO/45pSks+Nf25kT5cnV+IN3TsscA0o7I6XPIeUoAJJPsjw+AycsmRk2uffr3Bnupexr93iRfHylniqP+ss4cRcT1lOqS5Zhh7FQaoelR58qL/RUorGpknjy9ufCn9ervc6Mz01u3ZkM/EOa5wUPT8bDzPZ6nMDaadqumorT5Py+GtJSUosUgz4Gd3iR++fdEk6faFZq3L9xfBSagNykwhiyYx+oqwVqe";

    private VuforiaLocalizer vuforia;
    private FrameLayout cameraLayout;
    private OverlayView overlayView;
    private ExecutorService frameConsumerExecutor;

    private class FrameConsumer implements Runnable {
        private BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue;
        private Mat frame;
        private byte[] frameBuffer;

        private FrameConsumer(BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue) {
            this.frameQueue = frameQueue;
        }

        @Override
        public void run() {
            while (!Thread.currentThread().isInterrupted()) {
                // grab frames and process them
                if (!frameQueue.isEmpty()) {
                    VuforiaLocalizer.CloseableFrame vuforiaFrame = null;
                    try {
                        vuforiaFrame = frameQueue.take();
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    }

                    if (vuforiaFrame == null) {
                        continue;
                    }

                    for (int i = 0; i < vuforiaFrame.getNumImages(); i++) {
                        Image image = vuforiaFrame.getImage(i);
                        if (image.getFormat() == PIXEL_FORMAT.RGB565) {
                            int imageWidth = image.getWidth(), imageHeight = image.getHeight();
                            Bitmap bm = Bitmap.createBitmap(imageWidth, imageHeight, Bitmap.Config.RGB_565);
                            bm.copyPixelsFromBuffer(image.getPixels());

                            if (this.frame == null) {
                                this.frame = new Mat(bm.getHeight(), bm.getWidth(), CvType.CV_8UC3);

                                if (overlayView != null) {
                                    overlayView.setImageSize(imageWidth, imageHeight);
                                }
                            }

                            Utils.bitmapToMat(bm, frame);

                            onFrame(this.frame, vuforiaFrame.getTimeStamp());
                        }
                    }
                    vuforiaFrame.close();
                } else {
                    try {
                        Thread.sleep(1);
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    }
                }
            }
        }
    }

    public VuforiaCamera2() {
        this(new Parameters());
    }

    public VuforiaCamera2(Parameters parameters) {
        super(parameters);
    }

    @Override
    public synchronized void addTracker(Tracker tracker) {
        super.addTracker(tracker);

        if (overlayView != null) {
            this.overlayView.addTracker(tracker);
        }
    }

    @Override
    protected void onFrame(Mat frame, double timestamp) {
        super.onFrame(frame, timestamp);

        if (overlayView != null) {
            overlayView.postInvalidate();
        }
    }

    @Override
    protected void doInitialize() {
        VuforiaLocalizer.Parameters vuforiaParams = new VuforiaLocalizer.Parameters(parameters.cameraMonitorViewId);
        vuforiaParams.vuforiaLicenseKey = VUFORIA_LICENSE_KEY;
        vuforiaParams.cameraDirection = parameters.cameraDirection;
        vuforia = ClassFactory.getInstance().createVuforia(vuforiaParams);

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        vuforia.setFrameQueueCapacity(1);

        if (parameters.cameraMonitorViewId != 0) {
            this.overlayView = new OverlayView(activity);

            for (Tracker tracker : trackers) {
                overlayView.addTracker(tracker);
            }

            final Activity activity = appUtil.getActivity();
            activity.runOnUiThread(() -> {
                try {
                    LinearLayout cameraMonitorView = activity.findViewById(parameters.cameraMonitorViewId);
                    cameraLayout = (FrameLayout) cameraMonitorView.getParent().getParent();
                    cameraLayout.addView(overlayView);
                } catch (Exception ex) {
                    ex.getMessage();
                }
            });
        }

        frameConsumerExecutor = ThreadPool.newSingleThreadExecutor("Vuforia frame consumer");
        frameConsumerExecutor.execute(new FrameConsumer(vuforia.getFrameQueue()));
    }

    public VuforiaLocalizer getVuforia() {
        return this.vuforia;
    }

    public void close() {
        if (overlayView != null) {
            appUtil.runOnUiThread(() -> {
                cameraLayout.removeView(overlayView);
                overlayView = null;
            });
        }

        if (frameConsumerExecutor != null) {
            frameConsumerExecutor.shutdownNow();
            frameConsumerExecutor = null;
        }
    }

    @Override
    public Properties getProperties() {
        return new VuforiaProperties();
    }

    private class VuforiaProperties implements Properties {
        @Override
        public double getHorizontalFocalLengthPx(double imageWidth) {
            CameraCalibration cameraCalibration = CameraDevice.getInstance().getCameraCalibration();
            double fov = cameraCalibration.getFieldOfViewRads().getData()[0];
            return (imageWidth * 0.5) / Math.tan(0.5 * fov);
        }
    }
}
