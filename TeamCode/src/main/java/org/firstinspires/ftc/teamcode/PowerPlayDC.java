package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "PowerPlayDCJava")
@Disabled
public class PowerPlayDC extends LinearOpMode {
    static final double ARM_DRIVE_REDUCTION = 2;
    static final double ARM_WHEEL_DIAMETER_INCHES = 2.5;
    static final double ARM_MOTOR_RPM = 300;
    static final double ARM_COUNTS_PER_MOTOR_REV = 480;   // TorqueNADO
    static final double ARM_COUNTS_PER_WHEEL_REV = (ARM_COUNTS_PER_MOTOR_REV * ARM_DRIVE_REDUCTION);
    static final double ARM_COUNTS_PER_INCH = ARM_COUNTS_PER_WHEEL_REV / (ARM_WHEEL_DIAMETER_INCHES * 3.1415);
    static final int LOW_JUNCTION = 14;
    static final int MEDIUM_JUNCTION = 24;
    static final int HIGH_JUNCTION = 34;
    static final int HOME_POSITION = 1;
    static final int CONE_HEIGHT = 5;
    static final int ADJUST_ARM_INCREMENT = 1;
    // Calculate velocity for arm movement.
    private double TPS = ((ARM_MOTOR_RPM * .75) / 60) * ARM_COUNTS_PER_WHEEL_REV;

    private DcMotorEx armMotor = null;
    private ElevatorFeedforward armFeedForward;
    private GamepadEx gamePadArm;
    private int armTarget = 0;
    private int armPosition = 0;
    private ArmPosition selectedPosition;
    private double armVelocity = 0;
    private double armCurrent = 0;
    private double feedForwardCalculate = 0;
    private boolean isBusy = false;

    private DcMotor Frontleft;
    private DcMotor Backleft;
    private DcMotor Frontright;
    private DcMotor Backright;
    private Servo GripperServo;
//Whole Code Notes: Look at block code for denominator info.

    public void runOpMode() {
        double maxPower;
        double y;
        double x;
        double rx;
        double denominator;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        armMotor = hardwareMap.get(DcMotorEx.class, "ArmMotor");
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armFeedForward = new ElevatorFeedforward(12, 20, .9);

        gamePadArm = new GamepadEx(gamepad2);

        Frontleft = hardwareMap.get(DcMotor.class, "Frontleft");
        Backleft = hardwareMap.get(DcMotor.class, "Backleft");
        Frontright = hardwareMap.get(DcMotor.class, "Frontright");
        Backright = hardwareMap.get(DcMotor.class, "Backright");
        GripperServo = hardwareMap.get(Servo.class, "GripperServo");

        maxPower = 0.7;

        waitForStart();
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
        Backleft.setDirection(DcMotorSimple.Direction.REVERSE);

        if (opModeIsActive()) {
            while (opModeIsActive() && !isStopRequested()) {
                gamePadArm.readButtons();
                ProcessArm();

                // Drive Code
                y = -gamepad1.left_stick_y;
                x = gamepad1.left_stick_x;
                // Counteract imperfect strafing
                rx = gamepad1.right_stick_x * 1.1;
                denominator = JavaUtil.maxOfList(JavaUtil.createListWith(JavaUtil.sumOfList(JavaUtil.createListWith(Math.abs(y), Math.abs(x), Math.abs(rx))), 1));
                Frontleft.setPower(((y + x + rx) / denominator) * maxPower);
                Backleft.setPower((((y - x) + rx) / denominator) * maxPower);
                Frontright.setPower((((y - x) - rx) / denominator) * maxPower);
                Backright.setPower((((y + x) - rx) / denominator) * maxPower);

                //} This is commented out on purpose.
                //if (gamepad1.right_bumper) {
                //Frontleft.
                //BackLeft.
                //FrontRight.
                //BackRight.

                //}
                //if (gamepad1.left_bumper) {
                //Frontleft.
                //BackLeft.
                //FrontRight.
                //BackRight.

                //Stuff past here should be uncommented out.

            }
        }

    }

    public void sendTelemetry(String location) {
        telemetry.addData("Called from", location);
        telemetry.addData("Selected Position", selectedPosition);
        telemetry.addData("Target Position", "%d", armTarget);
        telemetry.addData("Current Position", armPosition);
        telemetry.addData("Velocity", armVelocity);
        telemetry.addData("Current", armCurrent);
        telemetry.addData("Busy", isBusy);
        telemetry.addData("Feed Forward", feedForwardCalculate);
        telemetry.addData("Pos Tolerance", armMotor.getTargetPositionTolerance());
        telemetry.update();
    }


    public void ProcessArm() {
        ArmPosition position = null;
        // Adjust position
        if (gamePadArm.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            position = ArmPosition.ADJUST_DOWN;
        }

        if (gamePadArm.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            position = ArmPosition.ADJUST_UP;
        }

        // Low junction
        if (gamePadArm.wasJustPressed(GamepadKeys.Button.A)) {
            position = ArmPosition.LOW;
        }

        // medium junction
        if (gamePadArm.wasJustPressed(GamepadKeys.Button.B)) {
            position = ArmPosition.MEDIUM;
        }

        // high junction
        if (gamePadArm.wasJustPressed(GamepadKeys.Button.Y)) {
            position = ArmPosition.HIGH;
        }

        // ground junction
        if (gamePadArm.wasJustPressed(GamepadKeys.Button.X)) {
            position = ArmPosition.HOME;
        }

        if (position != null) {
            moveArm(position);
        }
    }


    public void moveArm(ArmPosition position) {
        selectedPosition = position;
        armPosition = armMotor.getCurrentPosition();
        switch (position) {
            case HOME:
                armTarget = HOME_POSITION * (int) ARM_COUNTS_PER_INCH;
                break;
            case LOW:
                armTarget = (LOW_JUNCTION * (int) ARM_COUNTS_PER_INCH);
                break;
            case MEDIUM:
                armTarget = (MEDIUM_JUNCTION * (int) ARM_COUNTS_PER_INCH);
                break;
            case HIGH:
                armTarget = (HIGH_JUNCTION * (int) ARM_COUNTS_PER_INCH);
                break;
            case ADJUST_UP:
                armTarget = armPosition + (ADJUST_ARM_INCREMENT * (int) ARM_COUNTS_PER_INCH);
                break;
            case ADJUST_DOWN:
                armTarget = armPosition - (ADJUST_ARM_INCREMENT * (int) ARM_COUNTS_PER_INCH);
                break;
            default:
                return;
        }

        // Prevent arm moving below HOME_POSITION
        armTarget = Math.max(armTarget, HOME_POSITION * (int) ARM_COUNTS_PER_INCH);
        armMotor.setTargetPosition(armTarget);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setVelocityPIDFCoefficients(1.26, 0.126, 0, 12.6);
        armMotor.setPositionPIDFCoefficients(10);
        armMotor.setTargetPositionTolerance(5);
        armMotor.setVelocity(TPS);

        while (armMotor.isBusy() && !isStopRequested()) {
            armVelocity = armMotor.getVelocity();
            feedForwardCalculate = armFeedForward.calculate(armVelocity);
            armMotor.setPower(feedForwardCalculate);
            armPosition = armMotor.getCurrentPosition();
            armCurrent = armMotor.getCurrent(CurrentUnit.AMPS);
            isBusy = armMotor.isBusy();
            sendTelemetry("Move Arm");
        }
    }

    public enum ArmPosition {
        HOME,
        LOW,
        MEDIUM,
        HIGH,
        ADJUST_UP,
        ADJUST_DOWN
    }


}
