
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

// Set the name that'll show up on the Rev Controller
@TeleOp(name = "FTC-2023 STUDIO")

// We extends LinearOpMode as it's a LinearOpMode
public class FTC_2023_TELEOP_STUDIO extends LinearOpMode {

    // LEDs runs when OP mode is selected
    private RevBlinkinLedDriver light;

    // Pattern
    RevBlinkinLedDriver.BlinkinPattern pattern;

    private ElapsedTime runtime = new ElapsedTime();
    static final double FORWARD_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;
    static final double FRICTION_COMPENSATION = 1; // 1.2  ( 120%  MAX POWER )
    static final boolean ANALOG_HOLD = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        // Motors for the wheels or the mouvement
        // We are using a CRServo for motorBackLeft due to the number of DcMotor(4), motors in total(5)
        CRServo motorBackLeft = hardwareMap.crservo.get("leftBack");
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("leftFront");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("rightFront");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("rightBack");

        // DcMotor for the elevator
        DcMotor elevator = hardwareMap.dcMotor.get("elevator");

        // Constants Used
        int elevatorPOS = 0;  // Current
        final int elevatorSpeed = 180; //120;
        final double deadZone = 0.006;//0.01;
        int coElevatorSpeed = 80;

        int maximumHeight = 5000;

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
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // ??
        elevator.setDirection(DcMotorSimple.Direction.REVERSE); // ??  
        elevator.setTargetPosition(0); // ??

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
            elevator.setPower(0.9);

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
            boolean boostMode = false;

            int directionY = 0;
            int directionX = 0;
            int directionR = 0;

            directionY = gamepad1.left_stick_y > 0 ? 1 : -1;
            directionX = gamepad1.left_stick_x > 0 ? 1 : -1;


            while (opModeIsActive()) {
                // Joystick
                if (Math.abs(gamepad1.left_stick_y) > deadZone) {
                    y = .1 * directionY + gamepad1.left_stick_y * 0.8;// Remember, this is reversed!
                } else {
                    y = 0;
                }

                if (Math.abs(gamepad1.left_stick_x) > deadZone) {
                    x = -(.1 * directionX + gamepad1.left_stick_x * 0.8); // Counteract imperfect strafing
                } else {
                    x = 0;
                }

                if (Math.abs(gamepad1.right_stick_x) > deadZone) {
                    rx = -(gamepad1.right_stick_x * 1);
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


//                boostMode = gamepad1.left_bumper;

                if ((gamepad1.left_trigger > .3) || (gamepad1.left_bumper) || (gamepad1.right_bumper)){ boostMode = true;}else{boostMode = false;} //FOR NASSIM CONVENIENCE BOOST 4/12/2023 by Ghanais

                if (boostMode) {
                    boostModifier = 0.6;
                }
                motorFrontLeft.setPower(frontLeftPower * FRICTION_COMPENSATION * boostModifier);
                motorBackLeft.setPower(backLeftPower * FRICTION_COMPENSATION * boostModifier);
                motorFrontRight.setPower(frontRightPower * FRICTION_COMPENSATION * boostModifier);
                motorBackRight.setPower(backRightPower * FRICTION_COMPENSATION * boostModifier);

// .0 ouvert
// .3 close
                double pincePosition = (1 - gamepad1.right_trigger);

                if (ANALOG_HOLD) {

                  /*  if (pincePosition > .50) {
                        isPressed_a = true;
                        pincePosition = .50;

                    }

                    if (pincePosition < .13) {
                        pincePosition = .13;
                        isPressed_a = false;
                    }*/
                } else {

                    if (gamepad1.right_trigger > .3) {
                        if (!isPressed_a) {
                            pinceState = !pinceState;
                            isPressed_a = true;
                        }
                    } else {
                        isPressed_a = false;
                    }

// OUVERT
                    if (pinceState) {
                        pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE_GREEN;
                        pincePosition = .0;
                    } else {
                        pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
                        pincePosition = .30;
                    }

                }

                telemetry.addData("Pince",pincePosition );
                pince.setPosition(pincePosition);



                //Pince pilot (gamepad1)
                //  isPressed_a = false;

                //Elevator copilot(gamepad2)
                Gamepad PlayerWithElevator = gamepad2; // CHANGE THIS TO CONTROLLING PLAYER

                if (PlayerWithElevator.y) {
                    elevatorPOS = 0;
                    pattern = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED;
                }
                if (PlayerWithElevator.x) {
                    elevatorPOS = maximumHeight; // CHECK CONSTANT
                }

                if ((PlayerWithElevator.dpad_up || PlayerWithElevator.dpad_down)
                        ||
                        ((Math.abs(PlayerWithElevator.right_trigger) > .1) || (Math.abs(PlayerWithElevator.left_trigger) > .1)) // DEADZONE CHANGE (.1)
                ) {

                    if ((Math.abs(PlayerWithElevator.right_trigger) > .1) && (elevator.getTargetPosition() < maximumHeight)) {

                        elevatorPOS += elevatorSpeed * Math.abs(PlayerWithElevator.right_trigger) * 3; // CHANGE (3) TO ACCELERATE ELEVATOR SPEED

                    } else if ((Math.abs(PlayerWithElevator.left_trigger) > .1) && (elevator.getTargetPosition() > 0)) {

                        elevatorPOS -= elevatorSpeed * Math.abs(PlayerWithElevator.left_trigger) * 3;


                    } else if (PlayerWithElevator.dpad_up && (elevator.getTargetPosition() < maximumHeight)) { // REMOVE IF NOT USED BY PILOTS
                        double dxSpeed = 0;

                        if (pinceState) {
                            dxSpeed = 200;
                        }
                        elevatorPOS += elevatorSpeed + Math.abs(dxSpeed);

                    } else if (PlayerWithElevator.dpad_down && (elevator.getTargetPosition() > 0)) {

                        double dxSpeed = 0;

                        if (pinceState) {
                            dxSpeed = 200;
                        }
                        elevatorPOS -= elevatorSpeed + Math.abs(dxSpeed);
                    }
                } else {
                    elevatorPOS = elevator.getCurrentPosition();
                }

                elevator.setTargetPosition(elevatorPOS);


                //   telemetry.addData("ElevatorPOS", elevatorPOS);
                //   telemetry.addData("CurrentPOS", elevator.getCurrentPosition());

                light.setPattern(pattern);
                telemetry.update();
            }
        }
    }
}
