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

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "AUTONOMOUS 2 STUDIO", preselectTeleOp = "FTC-2023 STUDIO")
public class AUTONOMOUS_2_STUDIO extends LinearOpMode {

    private ElapsedTime timer = new ElapsedTime();

    static final int NULL_COLOR = 0;
    static final int GREEN_COLOR = 1;
    static final int RED_COLOR = 2;
    static final int BLUE_COLOR = 3;

    static final int ELEVATOR_CLEARANCE = 3000;
    static final int ELEVATOR_MAX_POSITION = 4600;
    static final int ELEVATOR_MIN_POSITION = 0;

    static final int PAUSE_BETWEEN_PINCE = 500;
    static final double PINCE_OPEN_POSITION = .30;
    static final double PINCE_CLOSE_POSITION = .0;

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

    Boolean lookForColor = false;

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

        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        // Wait for start command from Driver Station.
        waitForStart();

        int detectedImage = 0;


        //------------------------------------------ C O D E -------------------------------------//


        if (opModeIsActive()) {
            // Put run blocks here.

            cmd_pinceOpen();
            //Take back cone
            cmd_setElevatorPOS(0, 0);
            //requestOpModeStop();
            cmd_pinceClose();
            cmd_setElevatorPOS(4800, 1);

            //MOVE
            resetRuntime();
            while (distrev.getDistance(DistanceUnit.CM) > 30) {
                cmd_move(0, 0.3, 0, -1);
                if (lookForColor) {
                    detectedImage = coneDetectionColor();
                }
                if (getRuntime() > 6) {
                    cmd_move(0, 0, 0, 0);
                    cmd_setElevatorPOS(0, 1);
                    cmd_visionPosition(detectedImage);
                    cmd_pinceOpen();
                    requestOpModeStop();
                    break; // FAILSAFE
                }
            }


            cmd_move(0, 0, 0, 1);

            resetRuntime();
            while (botHeading > -1.55) {
                botHeading = -imu.getAngularOrientation().firstAngle;
                cmd_move(0, 0, -0.25, -1);
                telemetry.addData("Angle", botHeading);
                telemetry.update();
                if (getRuntime() > 4) break;//                  FAILSAFE

            }
            // END OF TURN
            resetRuntime();
            while (((DistanceSensor) colorrev).getDistance(DistanceUnit.CM) > 8.5) {
                cmd_move(-0.25, 0, 0, -1);
                if (getRuntime() > 2) break; //      FAILSAFE

            }
            cmd_move(0, 0, 0, 0);
            cmd_pinceOpen();

            //CODE FOR PARKING
            cmd_move(0.2, 0, 0, 1.1);
            cmd_move(0, -0.3, 0, 2);
            cmd_visionPosition(detectedImage);
            cmd_setElevatorPOS(0, 5);

            requestOpModeStop();
        }
    }


    //-------------------------------------- C O D E   E N D -------------------------------------//

    public void cmd_pinceClose() {
        pince.setPosition(PINCE_CLOSE_POSITION);
        sleep(PAUSE_BETWEEN_PINCE);
    }

    public void cmd_pinceOpen() {
        pince.setPosition(PINCE_OPEN_POSITION);
        sleep(PAUSE_BETWEEN_PINCE);
    }

    public void cmd_setElevatorPOS(int pos, double time) {
        pos = Math.min(Math.max(pos, ELEVATOR_MIN_POSITION), ELEVATOR_MAX_POSITION);
        elevator.setTargetPosition(pos);

        if (time > 0) {
            sleep((long) (time * 1000));
        }

    }

    public int coneDetectionColor() {
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
            return RED_COLOR;
        } else if (hue < 210) {
            cmd_setLED(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            lookForColor = false;
            return GREEN_COLOR;
        } else if (hue < 275) {
            cmd_setLED(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            lookForColor = false;
            return BLUE_COLOR;
        } else {
        }
        return NULL_COLOR;
    }

    public void cmd_move(double x, double y, double rx, double time) {
        //resetRuntime();

        // Read inverse IMU heading, as the IMU heading is CW positive
        double botHeading = -imu.getAngularOrientation().firstAngle;
        x = -x;
        y = -y;
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

    public void cmd_visionPosition(int color) {
        if (color == GREEN_COLOR) {
            cmd_move(-0.5, 0, 0, 1.8);
        } else if (color == BLUE_COLOR) {
            cmd_move(0.5, 0, 0, 1.8);
        } else if (color == RED_COLOR) {
            return;
        } else {


        }
    }
}