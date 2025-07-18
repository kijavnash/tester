/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;
import com.acmerobotics.dashboard.FtcDashboard;


/*
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Iterative OpMode", group="Iterative OpMode")
//@Disabled
public class vision_strafe extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontRight = null;
    private DcMotor frontLeft = null;
    private DcMotor backRight = null;
    private DcMotor backLeft = null;
    public int timer = 0;
    public int averageTA = 0;

    public static int[] addX(int n, int arr[], int x) {
        int newarr[] = new int[n + 1];
        for (int i = 0; i < n; i++) {
            newarr[i] = arr[i];
        }

        newarr[n] = x;

        return newarr;
    }
    public int[] results = new int[1];
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontRight  = hardwareMap.get(DcMotor.class, "front_right");
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        backRight = hardwareMap.get(DcMotor.class, "back_right");
        backLeft = hardwareMap.get(DcMotor.class, "back_left");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        /*
         * Starts polling for data.
         */
        limelight.start();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    private Limelight3A limelight;
    @Override
    public void loop() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvWebcam camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        FtcDashboard.getInstance().startCameraStream(camera, 0);
        // TIMER is here to sample average of data samples from the last 5 ticks (to eliminate noise potentially)
        timer += 1;
        // Setup a variable for each drive wheel to save power level for telemetry
        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        //double drive = -gamepad1.left_stick_y;
        //double turn  =  gamepad1.right_stick_x;
        double drive = 0;
        double turn = 0;
        double strafe = 0;

        LLResult result = limelight.getLatestResult();
        //telemetry.addData("result", result);
        if (result != null) {
            if (result.isValid()) {
                telemetry.addData("valid", "true");
                Pose3D botpose = result.getBotpose();
                turn = result.getTx() / 20;
                /*if (result.getTy() > -2) {
                    if (result.getTy() < -2) {
                        telemetry.addData("what", "the hell is even that");
                    }
                    drive = (result.getTy()) / 10;
                }
                else {
                    drive = 0;
                }
                else {
                   drive = -(result.getTy() / 2 - 1) / 20;
                }*/
                //if (drive < 0.2 && drive > -0.2) { drive = 0; }
                if (result.getTa() != 0) {
                    averageTA += result.getTa();
                }
                /*telemetry.addData("tx", result.getTx());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("ta", result.getTa());
                telemetry.addData("DRIVE", drive);
                telemetry.addData("red detected", "here");*/
            }
        }

        if (timer >= 20) {
            averageTA = averageTA / 5;
            telemetry.addData("average ta", averageTA);
            results = addX(results.length, results, averageTA);
            timer = 0;
            // do drive calculations
        }
        else {
            telemetry.addData("timer", timer);
        }
        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        // leftPower  = -gamepad1.left_stick_y ;
        // rightPower = -gamepad1.right_stick_y ;

        // Send calculated power to wheels
        turn = 0;
        double frPower = drive - turn - strafe;
        double flPower = drive + turn + strafe;
        double brPower = drive - turn + strafe;
        double blPower = drive + turn - strafe;
        if (frPower < 0.1 && frPower > -0.1) { frPower = 0; }
        if (flPower < 0.1 && flPower > -0.1) { flPower = 0; }
        if (brPower < 0.1 && brPower > -0.1) { brPower = 0; }
        if (blPower < 0.1 && blPower > -0.1) { blPower = 0; }
        frontRight.setPower(frPower);
        frontLeft.setPower(flPower);
        backRight.setPower(brPower);
        backLeft.setPower(blPower);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "time: " + runtime.toString());


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        telemetry.addData("results", results);
    }


}
