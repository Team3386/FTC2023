package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;


// Set the name that'll show up on the Rev Controller
@TeleOp(name = "FTC-2023 STUDIO")

// We extends LinearOpMode as it's a LinearOpMode
public class FTC_2023_TELEOP_STUDIO extends LinearOpMode {

    // LEDs runs when OP mode is selected
    private RevBlinkinLedDriver light;

    // Pattern
    RevBlinkinLedDriver.BlinkinPattern pattern;

    // 1 = 100% SPEED 0.5 = 50% SPEED. LOWER THIS TO REDUCE SPEED
    static final double SPEED_MODIFIER = 1;
    static final int ELEVATOR_MAX_POSITION = 5000;
    static final int ELEVATOR_MIN_POSITION = 0;
    static final int ELEVATOR_SEEK_POSITION_TOP = ELEVATOR_MAX_POSITION;
    static final int ELEVATOR_SEEK_POSITION_BOTTOM = ELEVATOR_MIN_POSITION;
    static final double ELEVATOR_DEADZONE = 0.1;

    static final double PINCE_OPEN_POSITION = .30;
    static final double PINCE_CLOSE_POSITION = .0;

    static final double ELEVATOR_SPEED = 180.0;
    static final double ELEVATOR_POWER = 0.9; // CAUTION DON'T RAISE THIS, CAN DESTROY THE ELEVATOR

    // JOYSTICK PILOT SETTINGS
    static final double JOYSTICK_PRECISION_MODIFIER = 0.8; // REDUCE TO RAISE PRECISION
    static final double JOYSTICK_DEADZONE = 0.006;


    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        // Motors for the wheels or the mouvement
        // We are using a CRServo for motorBackLeft due to the number of DcMotor(4), motors in total(5)
        //DcMotor for the wheels
        CRServo motorBackLeft = hardwareMap.crservo.get("leftBack");
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("leftFront");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("rightFront");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("rightBack");
        // DcMotor for the elevator
        DcMotor elevator = hardwareMap.dcMotor.get("elevator");

        // Servo linear for the elevator
        Servo Linear = hardwareMap.servo.get("centreUp");
        // Servo pince for the grabber
        Servo pince = hardwareMap.servo.get("pince");


        double elevatorPOS = 0;  // Current


        boolean isPinceOpen = false;
        boolean isPressed_a = false;
        boolean gotoPOS = false;
        double SeekPosition = 0;

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
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // ??
        elevator.setDirection(DcMotorSimple.Direction.REVERSE); // ??  
        elevator.setTargetPosition(0); // ??

        //
        // Retrieve the IMU from the hardware map
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        //imu.initialize(parameters);


        // [  ---------------------------------------------------------- C O D E -------------------------------------------------------------  ] \\
        waitForStart();
        elevatorPOS = elevator.getCurrentPosition();

        if (isStopRequested()) return;

        if (opModeIsActive()) {


            elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elevator.setPower(ELEVATOR_POWER);
            Linear.setPosition(0);

            // Initialization and Declaration of Joystick Variables
            double x = 0;
            double y = 0;
            double rx = 0;
            boolean boostMode = false;
            double pincePosition = PINCE_CLOSE_POSITION;

            int directionY = 0;
            int directionX = 0;

            Gamepad PlayerWithBase = null;
            Gamepad PlayerWithElevator = null;

            directionY = gamepad1.left_stick_y > 0 ? 1 : -1;
            directionX = gamepad1.left_stick_x > 0 ? 1 : -1;


            while (opModeIsActive()) {

                PlayerWithBase = gamepad1; // CHANGE THIS TO CONTROLLING PLAYER
                PlayerWithElevator = gamepad2; // CHANGE THIS TO ELEVATOR PLAYER


                // Joystick
                if (Math.abs(PlayerWithBase.left_stick_y) > JOYSTICK_DEADZONE) {
                    y = .1 * directionY + PlayerWithBase.left_stick_y * JOYSTICK_PRECISION_MODIFIER;// Remember, this is reversed!
                } else {
                    y = 0;
                }

                if (Math.abs(PlayerWithBase.left_stick_x) > JOYSTICK_DEADZONE) {
                    x = -(.1 * directionX + PlayerWithBase.left_stick_x * JOYSTICK_PRECISION_MODIFIER); // Counteract imperfect strafing
                } else {
                    x = 0;
                }

                if (Math.abs(PlayerWithBase.right_stick_x) > JOYSTICK_DEADZONE) {
                    rx = -(PlayerWithBase.right_stick_x * 1);
                } else {
                    rx = 0;
                }


                // Read inverse IMU heading, as the IMU heading is CW positive
                double botHeading = -imu.getAngularOrientation().firstAngle;

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
                double boostModifier = 1.2; //BOOST MODE REVERSED FOR NASSIM 4/12/2023 by Ghanais


                if ((PlayerWithBase.left_trigger > .3) || (PlayerWithBase.left_bumper) || (PlayerWithBase.right_bumper)) {
                    boostMode = true;
                } else {
                    boostMode = false;
                } //FOR NASSIM CONVENIENCE BOOST 4/12/2023 by Ghanais

                if (boostMode) {
                    boostModifier = 0.6;
                }

                motorFrontLeft.setPower(frontLeftPower * SPEED_MODIFIER * boostModifier);
                motorBackLeft.setPower(backLeftPower * SPEED_MODIFIER * boostModifier);
                motorFrontRight.setPower(frontRightPower * SPEED_MODIFIER * boostModifier);
                motorBackRight.setPower(backRightPower * SPEED_MODIFIER * boostModifier);


                if (PlayerWithBase.right_trigger > .3) {

                    // Debouncing prevention
                    if (!isPressed_a) {
                        isPinceOpen = !isPinceOpen;
                        isPressed_a = true;
                    }
                } else {
                    // Reset the debouncing flag
                    isPressed_a = false;
                }

                if (!isPinceOpen) {
                    pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE_GREEN;
                    pincePosition = PINCE_CLOSE_POSITION;
                } else {
                    pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
                    pincePosition = PINCE_OPEN_POSITION;
                }

                pince.setPosition(pincePosition);

                if (PlayerWithElevator.x) {

                    SeekPosition = ELEVATOR_SEEK_POSITION_BOTTOM;
                    gotoPOS = true;
                    pattern = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED;
                }
                if (PlayerWithElevator.y) {
                    gotoPOS = true;
                    SeekPosition = ELEVATOR_SEEK_POSITION_TOP; // CHECK CONSTANT
                }

                // Ajout de 3ème niveau
                if (PlayerWithElevator.a) {
                    Linear.setPosition(0.0);
                }
                if (PlayerWithElevator.b) {
                    Linear.setPosition(1);
                }
                telemetry.addData("Servo Position", Linear.getPosition());


                if ((PlayerWithElevator.dpad_up || PlayerWithElevator.dpad_down)
                        ||
                        ((Math.abs(PlayerWithElevator.right_trigger) > .1) || (Math.abs(PlayerWithElevator.left_trigger) > ELEVATOR_DEADZONE)) // DEADZONE CHANGE (.1)
                ) {
                    gotoPOS = false;

                    if ((Math.abs(PlayerWithElevator.right_trigger) > .1) && (elevator.getTargetPosition() < ELEVATOR_MAX_POSITION)) {

                        elevatorPOS += ELEVATOR_SPEED * Math.abs(PlayerWithElevator.right_trigger) * 3; // CHANGE (3) TO ACCELERATE ELEVATOR SPEED

                    } else if ((Math.abs(PlayerWithElevator.left_trigger) > .1) && (elevator.getTargetPosition() > ELEVATOR_MIN_POSITION)) {

                        elevatorPOS -= ELEVATOR_SPEED * Math.abs(PlayerWithElevator.left_trigger) * 3;


                    } else if (PlayerWithElevator.dpad_up && (elevator.getTargetPosition() < ELEVATOR_MAX_POSITION)) { // REMOVE IF NOT USED BY PILOTS
                        elevatorPOS += ELEVATOR_SPEED;

                    } else if (PlayerWithElevator.dpad_down && (elevator.getTargetPosition() > ELEVATOR_MIN_POSITION)) {

                        double dxSpeed = 0;

                        if (isPinceOpen) {
                            dxSpeed = 200;
                        }
                        elevatorPOS -= ELEVATOR_SPEED + Math.abs(dxSpeed);
                    }
                } else {

                    if (gotoPOS) {
                        elevatorPOS = SeekPosition;
                    } else {
                        elevatorPOS = elevator.getCurrentPosition();
                    }

                }

                elevator.setTargetPosition((int) elevatorPOS);

                light.setPattern(pattern);
                telemetry.update();
            }
        }
    }
}
