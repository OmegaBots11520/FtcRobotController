/*
 * Copyright (c) 2019 OpenFTC Team
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
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.os.Environment;
import android.widget.Toast;

import com.google.mlkit.vision.common.InputImage;
import com.opencsv.CSVReader;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.io.File;
import java.io.FileReader;

@Autonomous(name="Robot: autobot4 ", group="Robot")
//@Disabled

public class autobot4 extends LinearOpMode
{
    OpenCvWebcam webcam;
    barcodescanner scanner = new barcodescanner();
    String data = "waiting";

    private DcMotor FrontLeft   = null;
    private DcMotor FrontRight  = null;
    private DcMotor RearLeft   = null;
    private DcMotor RearRight  = null;
    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 312 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 19 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    double[][] DriveData1 = new double[9][7];
    double[] DriveData = new double[7];

    public autobot4() {

    }

    @Override
    public void runOpMode()
    {
         /*
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "color cam"), cameraMonitorViewId);
        // OR...  Do Not Activate the Camera Monitor View
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        */
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "color cam"));

        FrontLeft  = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        RearLeft  = hardwareMap.get(DcMotor.class, "RearLeft");
        RearRight = hardwareMap.get(DcMotor.class, "RearRight");

        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        FrontRight.setDirection(DcMotor.Direction.FORWARD);
        RearLeft.setDirection(DcMotor.Direction.REVERSE);
        RearRight.setDirection(DcMotor.Direction.FORWARD);

        /*
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        */

        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        webcam.setPipeline(new SamplePipeline());

        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            { webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT); }
            @Override
            public void onError(int errorCode)
            { }
        });

        //telemetry.addLine("no of lines read");

        try {
            //File csvfile = new File("/storage/sdcard0/FIRST/data.csv");
            File fileDirectory = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS);
            File csvfile = new File(fileDirectory,"data.csv");

            //if(csvfile.exists() && !csvfile.isDirectory()) {
            //    telemetry.addData("filedata", csvfile);
            //}
           // else
            //    telemetry.addData("filedata", "file not found");

            CSVReader reader = new CSVReader(new FileReader(csvfile.getAbsolutePath()));
            String[] nextLine;
            int i = 0 ;
            while ((nextLine = reader.readNext()) != null) {
                // nextLine[] is an array of values from the line
                //telemetry.addData("filedata", nextLine);
               // telemetry.update();
                for(int j = 0 ; j<7;j++) DriveData1[i][j] = Double.parseDouble(nextLine[j]);
                i++;
            }

            telemetry.addData("no", i);

        } catch (Exception e) {
            e.printStackTrace();
            //Toast.makeText(this, "The specified file was not found", Toast.LENGTH_SHORT).show();
            telemetry.addData("error", "file  noy found");

        }

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();
        int parkingposition = 0;
        while (opModeIsActive())
        {
            telemetry.addData("QR_Code", data);
            telemetry.update();

            //DriveData= new double[]{0.2, 1, 1, 1, 1, 1.0,parkingposition};
            //encoderDrive();
            runtime.reset();
            parkingposition = 2;
            while (runtime.seconds()<10){

                telemetry.addData("QR_Code", data);
                telemetry.update();

                if (data.equals("one"))
                    parkingposition=1;
                else if(data.equals("two"))
                    parkingposition=2;
                else if(data.equals("three"))
                    parkingposition=3;
                sleep(100);

            }
            webcam.stopStreaming();
            telemetry.addData("Parking Position", parkingposition);
            telemetry.update();
            if(parkingposition==3){
            DriveData = DriveData1[0];
            encoderDrive();
            DriveData = DriveData1[1];
            encoderDrive();
            DriveData = DriveData1[2];
            encoderDrive();
            DriveData = DriveData1[3];
            encoderDrive();
            }
            else if (parkingposition==2) {
                DriveData = DriveData1[4];
                encoderDrive();
            }
            else if(parkingposition==1){
            DriveData = DriveData1[5];
            encoderDrive();
            DriveData = DriveData1[6];
            encoderDrive();
            DriveData = DriveData1[7];
            encoderDrive();
            DriveData = DriveData1[8];
            encoderDrive();
            }
            telemetry.addData("QR_Code", "Done");
            telemetry.addData("QR_Code1", data);
            telemetry.addData("QR_Code2",parkingposition);
            telemetry.update();
            while(opModeIsActive()){
                sleep(100);
            }
        }
    }

    class SamplePipeline extends OpenCvPipeline
    {
        boolean viewportPaused;
        Bitmap bmp = Bitmap.createBitmap(640, 480, Bitmap.Config.ARGB_8888);
        //Imgproc.cvtColor(input, mYuvFrameData, Imgproc.COLOR_RGBA2YUV_YV12);
        @Override
        public Mat processFrame(Mat input)
        {
            //Mat mYuvFrameData = new Mat();
            Utils.matToBitmap(input, bmp);
            //byte[] return_buff = new byte[(int) (mYuvFrameData.total() *
            //        mYuvFrameData.channels())];
           // mYuvFrameData.get(0, 0, return_buff);

           // InputImage image = InputImage.fromByteBuffer(ByteBuffer.wrap(return_buff),
           //         /* image width */ 640,
           //         /* image height */ 480,
           //         0,
           //         InputImage.IMAGE_FORMAT_YV12 // or IMAGE_FORMAT_YV12
           // );
            InputImage image = InputImage.fromBitmap(bmp, 0);
            data = scanner.scanBarcodes(image);
            return input;
        }
    }

    private void encoderDrive() {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newRearLeftTarget;
        int newRearRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            /*
            FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            newFrontLeftTarget = FrontLeft.getCurrentPosition() + (int)(DriveData[1] * COUNTS_PER_INCH);
            newFrontRightTarget = FrontRight.getCurrentPosition() + (int)(DriveData[2] * COUNTS_PER_INCH);
            newRearLeftTarget = RearLeft.getCurrentPosition() + (int)(DriveData[3] * COUNTS_PER_INCH);
            newRearRightTarget = RearRight.getCurrentPosition() + (int)(DriveData[4] * COUNTS_PER_INCH);

            FrontLeft.setTargetPosition(newFrontLeftTarget);
            FrontRight.setTargetPosition(newFrontRightTarget);
            RearLeft.setTargetPosition(newRearLeftTarget);
            RearRight.setTargetPosition(newRearRightTarget);

            // Turn On RUN_TO_POSITION

            FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            */
            // reset the timeout time and start motion.
            runtime.reset();

            FrontLeft.setPower(Math.abs(DriveData[0])*DriveData[1]);
            FrontRight.setPower(Math.abs(DriveData[0])*DriveData[2]);
            RearLeft.setPower(Math.abs(DriveData[0])*DriveData[3]);
            RearRight.setPower(Math.abs(DriveData[0])*DriveData[4]);
           /*
            FrontLeft.setPower(Math.abs(DriveData[0]));
            FrontRight.setPower(Math.abs(DriveData[0]));
            RearLeft.setPower(Math.abs(DriveData[0]));
            RearRight.setPower(Math.abs(DriveData[0]));
            */
            while (opModeIsActive() &&
                    (runtime.seconds() < DriveData[5]) ){
                    //&&
                    //(FrontLeft.isBusy() || FrontRight.isBusy()||RearLeft.isBusy() || RearRight.isBusy())) {

                telemetry.addLine("target");
                /*
                telemetry.addData("FLTGT", newFrontLeftTarget);
                telemetry.addData("FRTGT", newFrontRightTarget);
                telemetry.addData("RFTGT", newRearLeftTarget);
                telemetry.addData("RRTGT", newRearRightTarget);
                 */
                telemetry.addData("pkp", DriveData[6]);
                telemetry.addLine("power");
                telemetry.addData("FLpower", FrontRight.getPower());
                telemetry.addData("FRpower", FrontRight.getPower());
                telemetry.addData("RFpower", RearLeft.getPower());
                telemetry.addData("RRpower", RearRight.getPower());
                telemetry.update();

                sleep(10);
            }

            // Stop all motion;
            FrontLeft.setPower(0);
            FrontRight.setPower(0);
            RearLeft.setPower(0);
            RearRight.setPower(0);

            // Turn off RUN_TO_POSITION
           /*
            FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
           */
        }
    }
}
