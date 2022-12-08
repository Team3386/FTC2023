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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.Tfod;

import java.util.List;

/**
 * This file illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 * <p>
 * The code assumes that you do NOT have encoders on the wheels,
 * otherwise you would use: RobotAutoDriveByEncoder;
 * <p>
 * The desired path in this example is:
 * - Drive forward for 3 seconds
 * - Spin right for 1.3 seconds
 * - Drive Backward for 1 Second
 * <p>
 * The code is written in a simple form with no optimizations.
 * However, there are several ways that this type of sequence could be streamlined,
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name = "Robot: Auto Drive By Time", group = "Robot", preselectTeleOp = "FTC-2023 1.0")
//@Disabled
public class AutonomousTEST extends LinearOpMode {
    private RevBlinkinLedDriver light;
    RevBlinkinLedDriver.BlinkinPattern pattern;

    static final double FORWARD_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    static final int CMD_TURN = 0;
    static final int CMD_ELEVATOR_UP = 1;
    static final int CMD_ELEVATOR_DOWN = 2;
    static final int CMD_IDLE = 3;
    static final int CMD_TURN_RIGHT = 4;
    static final int CMD_MOVE_FORWARD = 5;
    static final int CMD_MOVE_BACKWARD = 6;
    static final int CMD_PINCE_CLOSE = 7;
    static final int CMD_PINCE_OPEN = 8;
    static final int CMD_MOVE_RIGHT = 9;
    static final int CMD_MOVE_LEFT = 10;
    private VuforiaCurrentGame vuforiaPOWERPLAY;
    private Tfod tfod;

    Recognition recognition;


    @Override
    public void runOpMode() throws InterruptedException {
        List<Recognition> recognitions;
        int index;
        vuforiaPOWERPLAY = new VuforiaCurrentGame();
        tfod = new Tfod();

        // Initialize Vuforia.
        vuforiaPOWERPLAY.initialize(
                "", // vuforiaLicenseKey
                hardwareMap.get(WebcamName.class, "webcam"), // cameraName
                "", // webcamCalibrationFilename
                false, // useExtendedTracking
                false, // enableCameraMonitoring
                VuforiaLocalizer.Parameters.CameraMonitorFeedback.NONE, // cameraMonitorFeedback
                0, // dx
                0, // dy
                0, // dz
                AxesOrder.XYZ, // axesOrder
                0, // firstAngle
                90, // secondAngle
                90, // thirdAngle
                true); // useCompetitionFieldTargetLocations
        tfod.useDefaultModel();
        // Set min confidence threshold to 0.7
        tfod.initialize(vuforiaPOWERPLAY, (float) 0.65, true, true);


        // Initialize TFOD before waitForStart.
        // Activate TFOD here so the object detection labels are visible
        // in the Camera Stream preview window on the Driver Station.
        tfod.activate();




        // Enable following block to zoom in on target.
        tfod.setZoom(2, 16 / 9);
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Press Play to start");
        telemetry.update();


        // Declare our motors
        // Make sure your ID's match your configuration
        CRServo motorBackLeft = hardwareMap.crservo.get("leftBack");
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("leftFront");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("rightFront");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("rightBack");

        DcMotor elevator = hardwareMap.dcMotor.get("elevator");

        // CONSTANTS
        int elevatorPOS = 0;
        int elevatorSpeed = 50;
        double deadZone = 0.01;
        int coElevatorSpeed = 80;


        //int maximumHeight = 4500;

        Servo pince = hardwareMap.servo.get("pince");
        boolean pinceState = false;
        boolean isPressed_a = false;

        light = hardwareMap.get(RevBlinkinLedDriver.class, "light");

        pattern = RevBlinkinLedDriver.BlinkinPattern.CP1_2_COLOR_GRADIENT; //Starting color
        light.setPattern(pattern);


        // Reverse left motors because it's SPARK mini
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set ZERO Power to brake
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Elevator Configuration
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setDirection(DcMotorSimple.Direction.REVERSE);
        elevator.setTargetPosition(0);

        // Retrieve the IMU from the hardware map
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);


        elevatorPOS = elevator.getCurrentPosition();


        if (isStopRequested()) return;
        waitForStart();
        if (opModeIsActive()) {
            elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elevator.setPower(0.7);

            // Alliance selection
            if (gamepad1.x) {
                pattern = RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE; //Set blue alliance
            } else if (gamepad1.b) {
                pattern = RevBlinkinLedDriver.BlinkinPattern.RED; //Set red alliance
            }
            light.setPattern(pattern);

            // Initialization and Declaration of Joystick Variables
            double x = 0;
            double y = 0;
            double rx = 0;
            int STATE_MACHINE = 0;
            int CURRENT_STATE = 0;
            int PROGRAM_STATE = 0;

            double STATE_DURATION = 1.0;


            int PROGRAM[] =
                    {

                        CMD_IDLE


                    };
            int PROGRAM_OBJECT_DETECTED[] =
                    {
                            CMD_PINCE_OPEN,
                            CMD_MOVE_FORWARD,
                            CMD_PINCE_CLOSE,
                            CMD_MOVE_BACKWARD,
                            CMD_MOVE_FORWARD,
                            CMD_MOVE_BACKWARD
                    };

            STATE_MACHINE = PROGRAM[0];
            ElapsedTime runtime = new ElapsedTime();
            boolean sawCone = false;

            while (opModeIsActive()) {

                recognitions = tfod.getRecognitions();
                // If list is empty, inform the user. Otherwise, go
                // through list and display info for each recognition.
                if (JavaUtil.listLength(recognitions) == 0) {
                    telemetry.addData("TFOD", "No items detected.");
                } else {
                    index = 0;
                    // Iterate through list and call a function to
                    // display info for each recognized object.
                    for (Recognition recognition_item : recognitions) {

                        recognition = recognition_item;
                        if (!sawCone) {

                            sawCone = true;
                            STATE_MACHINE = PROGRAM_OBJECT_DETECTED[0];
                            PROGRAM_STATE = 0;
                        }

                        // Display info.
                        displayInfo(index);
                        // Increment index.
                        index = index + 1;
                    }
                }


                // AI
                if (runtime.seconds() > STATE_DURATION) {

                    switch (STATE_MACHINE) {  //if STATEMACHINE = 0
                        case CMD_TURN: // 0
                            x = 0;
                            y = 0;
                            rx = .65;
                            break;

                        case CMD_ELEVATOR_UP: // 1  // else    if STATEMACHINE = 1
                            if ((elevator.getTargetPosition() < 4500)) {
                                elevatorPOS = 4500;
                            }
                            break;
                        case CMD_ELEVATOR_DOWN: // 2
                            elevatorPOS = 0;
                            break;

                        case CMD_IDLE: // 3
                            x = 0;
                            y = 0;
                            rx = 0;
                            break;

                        case CMD_TURN_RIGHT: //4
                            x = 0;
                            y = 0;
                            rx = -.65;
                            break;
                        case CMD_MOVE_FORWARD:
                            rx = 0;
                            y = -.65;
                            break;
                        case CMD_MOVE_BACKWARD:
                            rx = 0;
                            y = .65;
                            break;
                        case CMD_MOVE_RIGHT:
                            rx = 0;
                            x = -.65;
                            break;
                        case CMD_MOVE_LEFT:
                            rx = 0;
                            x = .65;
                            break;
                        case CMD_PINCE_OPEN:
                            pince.setPosition(1);
                            break;
                        case CMD_PINCE_CLOSE:
                            pince.setPosition(0.675);
                            break;
                    }
                    if (PROGRAM_STATE >= PROGRAM_OBJECT_DETECTED.length) {
                        PROGRAM_STATE = 0;
                        STATE_MACHINE = PROGRAM[CURRENT_STATE];
                        sawCone = false;
                    }

                    if (!sawCone) {
                        STATE_MACHINE = PROGRAM[CURRENT_STATE];
                        CURRENT_STATE++;
                    } else {
                        STATE_MACHINE = PROGRAM_OBJECT_DETECTED[PROGRAM_STATE];
                        PROGRAM_STATE++;
                    }

                    if (STATE_MACHINE > 10) {
                        STATE_MACHINE = 0;
                    }



                    if (CURRENT_STATE >= PROGRAM.length) {
                        CURRENT_STATE = 0;
                    }
                    runtime.reset();
                }
                // Read inverse IMU heading, as the IMU heading is CW positive
                double botHeading = -imu.getAngularOrientation().firstAngle;

                double rotX = x * Math.cos(botHeading) - y * Math.sin(0);
                double rotY = x * Math.sin(botHeading) + y * Math.cos(0);

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio, but only when
                // at least one is out of the range [-1, 1]
                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                double frontLeftPower = (rotY + rotX + rx) / denominator;
                double backLeftPower = (rotY - rotX + rx) / denominator;
                double frontRightPower = (rotY - rotX - rx) / denominator;
                double backRightPower = (rotY + rotX - rx) / denominator;


                telemetry.addData("RotX", rotX);
                telemetry.addData("RotY", rotY);
                telemetry.addData("jX", x);
                telemetry.addData("jY", y);
                telemetry.addData("rx", rx);
                telemetry.addData("Current State", CURRENT_STATE);

                motorFrontLeft.setPower(frontLeftPower);
                motorBackLeft.setPower(backLeftPower);
                motorFrontRight.setPower(frontRightPower);
                motorBackRight.setPower(backRightPower);


                //Elevator
//                if (gamepad1.dpad_up || gamepad1.dpad_down) {
//                    if (gamepad1.dpad_up && (elevator.getTargetPosition() < 4500)) {
//                        elevatorPOS += elevatorSpeed;
//                    } else if (gamepad1.dpad_down && (elevator.getTargetPosition() > 0)) {
//                        elevatorPOS -= elevatorSpeed;
//                    }
//                } else {
//                    elevatorPOS = elevator.getCurrentPosition();
//                }

                elevator.setTargetPosition(elevatorPOS);

                telemetry.addData("ElevatorPOS", elevatorPOS);
                telemetry.addData("CurrentPOS", elevator.getCurrentPosition());

                //Pince
                if (gamepad1.a) {
                    if (!isPressed_a) {
                        pinceState = !pinceState;
                        isPressed_a = true;
                    }
                    if (pinceState) {
                        pince.setPosition(1);
                        pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
                        pattern = RevBlinkinLedDriver.BlinkinPattern.HOT_PINK;
                    } else {
                        pince.setPosition(0.7);
                        pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
                        pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
                    }
                } else {
                    isPressed_a = false;
                }

                // COPILOT CONTROLS
                // Co-Elevator
            /*    if (gamepad2.dpad_up || gamepad2.dpad_down) {
                    if (gamepad2.dpad_up && (elevator.getTargetPosition() < 4500)) {
                        elevatorPOS += coElevatorSpeed;
                    } else if (gamepad2.dpad_down && (elevator.getTargetPosition() > 0)) {
                        elevatorPOS -= coElevatorSpeed;
                    }
                } else {
                    elevatorPOS = elevator.getCurrentPosition();
                }*/
                telemetry.addData("CURRENT STATE", STATE_MACHINE);
                light.setPattern(pattern);
                telemetry.update();
            }
        }
        // Deactivate TFOD.
        tfod.deactivate();

        vuforiaPOWERPLAY.close();
        tfod.close();

    }

    private void displayInfo(int i) {
        // Display label info.
        // Display the label and index number for the recognition.
        telemetry.addData("label " + i, recognition.getLabel());
        // Display upper corner info.
        // Display the location of the top left corner
        // of the detection boundary for the recognition
        telemetry.addData("Left, Top " + i, Double.parseDouble(JavaUtil.formatNumber(recognition.getLeft(), 0)) + ", " + Double.parseDouble(JavaUtil.formatNumber(recognition.getTop(), 0)));
        // Display lower corner info.
        // Display the location of the bottom right corner
        // of the detection boundary for the recognition
        telemetry.addData("Right, Bottom " + i, Double.parseDouble(JavaUtil.formatNumber(recognition.getRight(), 0)) + ", " + Double.parseDouble(JavaUtil.formatNumber(recognition.getBottom(), 0)));
    }

}
