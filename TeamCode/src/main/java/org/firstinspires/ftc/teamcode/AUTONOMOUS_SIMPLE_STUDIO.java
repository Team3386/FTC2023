package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.List;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.Tfod;

@Autonomous(name = "AUTONOMOUS SIMPLE STUDIO", preselectTeleOp = "FTC-2023 1.2")
public class AUTONOMOUS_SIMPLE_STUDIO extends LinearOpMode {

    private ElapsedTime timer = new ElapsedTime();

    // Pince
    Servo pince;

    // Elevator
    DcMotor elevator;

    // Base
    CRServo motorBackLeft;
    DcMotor motorFrontLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackRight;

    BNO055IMU imu;

    // LEDs runs when OP mode is selected
    RevBlinkinLedDriver light;
    RevBlinkinLedDriver.BlinkinPattern pattern;

    //Sensor
    ColorSensor colorrev;
    ColorSensor colorrev2;
    DistanceSensor distrev;
    private VoltageSensor voltSensor;

    Boolean lookForColor = true;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */

    @Override
    public void runOpMode() {

        light = hardwareMap.get(RevBlinkinLedDriver.class, "light"); // INIT LEDs

        pince = hardwareMap.servo.get("pince"); // INIT PINCE

        elevator = hardwareMap.dcMotor.get("elevator"); // INIT ELEVATOR
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setDirection(DcMotorSimple.Direction.REVERSE);
        elevator.setTargetPosition(0);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.7);

        motorBackLeft = hardwareMap.crservo.get("leftBack"); // INIT BASE
        motorFrontLeft = hardwareMap.dcMotor.get("leftFront");
        motorFrontRight = hardwareMap.dcMotor.get("rightFront");
        motorBackRight = hardwareMap.dcMotor.get("rightBack");

        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        double botHeading = -imu.getAngularOrientation().firstAngle;

        colorrev = hardwareMap.get(ColorSensor.class, "colorrev"); // INIT SENSOR
        colorrev2 = hardwareMap.get(ColorSensor.class, "colorrev2");
        distrev = hardwareMap.get(DistanceSensor.class, "2mrev");
        voltSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        // Wait for start command from Driver Station.
        waitForStart();

        String detectedImage = "none lmao";


        //------------------------------------------ C O D E -------------------------------------//


        if (opModeIsActive()) {
            // Put run blocks here.

            //Image detected
            cmd_pinceOpen();



            //Take back cone
            cmd_setElevatorPOS(0, 0);
            cmd_pinceClose();
            cmd_setElevatorPOS(3000, 1);

            //NEW
            cmd_move(0,0.2,0,1.5);
            resetRuntime();
            while (getRuntime() < 3){
                cmd_move(0, 0.4, 0, -1);
                if (lookForColor) {
                    detectedImage = coneDetectionColor();

                }else{
                    break;
                }
            }

            cmd_move(0,0,0,0);
            cmd_visionPosition(detectedImage);

            cmd_setElevatorPOS(0,5);
            //cmd_pinceOpen();

            requestOpModeStop();
        }
    }


    //-------------------------------------- C O D E   E N D -------------------------------------//




    public void cmd_pinceClose() {
        pince.setPosition(.0);
        sleep(500);
    }

    public void cmd_pinceOpen() {
        pince.setPosition(.3);
        sleep(500);
    }

    public void cmd_setElevatorPOS(int pos, double time) {
        pos = Math.min(Math.max(pos, 0), 4600);
        elevator.setTargetPosition(pos);

        if (time > 0) {
            sleep((long) (time * 1000));
        }

    }

    public String coneDetectionColor() {
        NormalizedRGBA normalizedColors;
        int color;
        float hue;

        normalizedColors = ((NormalizedColorSensor) colorrev2).getNormalizedColors();
        color = normalizedColors.toColor();
        hue = JavaUtil.colorToHue(color);

        if (hue < 10) {
            lookForColor = true;
        } else if (hue < 90) {
            cmd_setLED(RevBlinkinLedDriver.BlinkinPattern.RED);
            lookForColor = false;
            return "RED";
        } else if (hue < 210) {
            cmd_setLED(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            lookForColor = false;
            return "GREEN";
        } else if (hue < 275) {
            cmd_setLED(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            lookForColor = false;
            return "BLUE";
        } else {
        }
        return "none lmao";
    }


    // Move the robot.
    // x and y are the speed to apply to their respective local axis

    public void cmd_move(double x, double y, double rx, double time) {
        //resetRuntime();

        // Read inverse IMU heading, as the IMU heading is CW positive
        double botHeading = -imu.getAngularOrientation().firstAngle;
        double maximumVoltage =  13.39; // Constant
        double minimumVoltage = 10.0; // Constant
        double currentVoltage = voltSensor.getVoltage(); // Current Voltage



        // If the voltage is less than the minimumVoltage...
        if(currentVoltage <= minimumVoltage){
            // It mean there is an error
            // Set the currentVoltage to maximum, so the ratio is always 1
            // This make sure we do not divided by zero
            currentVoltage = maximumVoltage;
        }

        // If the voltage is more than maximumVoltage....
        if(currentVoltage >= maximumVoltage){
            // it means we are over volt, or there is something cringe
            // Therefore, we set the ratio to 1.00
            currentVoltage = maximumVoltage;
        }


        // Ratio is MaximumVoltage / CurrentVoltage
        // So if we have less voltage, we go further
        double qualibration =  .85f;
        double ratio = (maximumVoltage / currentVoltage) * ( qualibration);


        telemetry.addData("Current Voltage", currentVoltage);
        telemetry.addData("maximumVoltage", qualibration);
        telemetry.addData("Current Ratio", ratio);
        telemetry.addData("Qualibration", qualibration);

        telemetry.update();

        x = -x * ratio;
        y = -y * ratio;
        rx = -rx;

        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        if (time <= -1) {
            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);
        } else {
            resetRuntime();
            while (getRuntime() < time) {
                motorFrontLeft.setPower(frontLeftPower);
                motorBackLeft.setPower(backLeftPower);
                motorFrontRight.setPower(frontRightPower);
                motorBackRight.setPower(backRightPower);
            }
            motorFrontLeft.setPower(0);
            motorBackLeft.setPower(0);
            motorFrontRight.setPower(0);
            motorBackRight.setPower(0);
        }


    }

    public void cmd_setLED(RevBlinkinLedDriver.BlinkinPattern color) {
        pattern = color;
        light.setPattern(pattern);

    }

    public void cmd_visionPosition(String label) {
        if (label == "GREEN") {
            cmd_move(-0.5,0,0,1.8);
        }else if (label == "BLUE") {
            cmd_move(0.5,0,0,1.8);
        } else if (label == "RED"){
            return;
        } else{
        }
    }
}