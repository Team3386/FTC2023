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


@Autonomous(name = "AUTONOMOUS BETA STUDIO", preselectTeleOp = "FTC-2023 STUDIO")
public class AUTONOMOUS_BETA_STUDIO extends LinearOpMode {

    private ElapsedTime timer = new ElapsedTime();

    static final int NULL_COLOR = 0;
    static final int GREEN_COLOR = 1;
    static final int RED_COLOR = 2;
    static final int BLUE_COLOR = 3;

    static final int ELEVATOR_CLEARANCE = 5900;
    static final int ELEVATOR_MAX_POSITION = 6100; // Was 4600
    static final int ELEVATOR_MIN_POSITION = 0;

    static final double MAX_COLLISION_DISTANCE = 13.5;
    static final double MIN_COLLISION_DISTANCE = 2.5;
    static final int PAUSE_BETWEEN_PINCE = 500;
    static final double PINCE_OPEN_POSITION = 0.25;
    static final double PINCE_CLOSE_POSITION = 0.5;


    // Pince
    Servo pince;

    Servo linear; // For high peg extension

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

    Boolean lookForColor = true;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */

    @Override
    public void runOpMode() {

        light = hardwareMap.get(RevBlinkinLedDriver.class, "light"); // INIT LEDs

        pince = hardwareMap.servo.get("pince"); // INIT PINCE

        // Servo linear for the elevator
        linear = hardwareMap.servo.get("centreUp");

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

        linear.setPosition(0.7);
        sleep(1000);
        linear.setPosition(0.2);//LOWER BOOST
        sleep(1000);

        // Use a lot of data and Ram to send something we do not need
        telemetry.addData("STATUS", "Gyroscope is on");
        telemetry.addData(">", "Press Play to start");
        telemetry.addData("BOOSTPOS",linear.getPosition());
        telemetry.update();


        // Wait for start command from Driver Station.
        waitForStart();

        int detectedImage = 0;
        double pegCollisionTimer = 0;


        //------------------------------------------ C O D E -------------------------------------//

        if (opModeIsActive()) {

            cmd_pinceOpen();
            //Take back cone
            cmd_setElevatorPOS(0, 0);
            cmd_pinceClose();
            cmd_setElevatorPOS(ELEVATOR_CLEARANCE, 1);

            cmd_move(0, 0.2, 0, 1.5);
            resetRuntime();

            while (getRuntime() < 3) {
                cmd_move(0, 0.4, 0, -1);
                if (lookForColor) {
                    detectedImage = coneDetectionColor();

                } else {
                    break;
                }
            }

            cmd_move(0, 0, 0, 1);

            //NEW PROCESS
            resetRuntime();
            while (botHeading < 0.83) {
                botHeading = -imu.getAngularOrientation().firstAngle;
                cmd_move(0, 0, 0.25, -1);
                telemetry.addData("Angle 1", botHeading);
                telemetry.update();
                if (getRuntime() > 4) break;//                  FAILSAFE

            }

            cmd_move(0,0,0,1);


            resetRuntime(); // going forward until collision detected
            while ((MIN_COLLISION_DISTANCE > distrev.getDistance(DistanceUnit.CM))
                    || (distrev.getDistance(DistanceUnit.CM) > MAX_COLLISION_DISTANCE )) {
                cmd_move(0.2, 0.2, 0, -1);
                pegCollisionTimer = getRuntime();

                if (getRuntime() > 3) {
                    cmd_move(-0.2, -0.2, 0, 3);
                    cmd_visionPosition(detectedImage);
                    cmd_setElevatorPOS(0, 5);
                    cmd_pinceOpen();
                    requestOpModeStop();
                    break; // FAILSAFE
                }

            }

            cmd_move(0,0,0,1);
            cmd_pinceOpen();
            cmd_move(-0.2, -0.2, 0, pegCollisionTimer + 0.5);

            resetRuntime();
            while (botHeading > -1.571) {
                botHeading = -imu.getAngularOrientation().firstAngle;
                cmd_move(0, 0, -0.25, -1);
                telemetry.addData("Angle 2", botHeading);
                telemetry.update();
                if (getRuntime() > 8) break;//    FAILSAFE

            }

            cmd_move(0,0,0, 1);



            //END NEW PROCESS
            cmd_setElevatorPOS(0,0);
            cmd_visionPosition(detectedImage);
            cmd_setElevatorPOS(0, 5);
            cmd_pinceOpen();

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
        } else if ((hue == 120) || (hue == 180)){} // hue always snaps to 120 or 180 for some reason
        else if (hue < 90) {
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