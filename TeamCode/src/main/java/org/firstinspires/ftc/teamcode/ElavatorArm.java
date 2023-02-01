package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ElavatorArm {
    private final DcMotorEx armMotor;
    LinearOpMode opMode;
    Telemetry telemetry;

    public ElavatorArm(DcMotorEx armMotor, Telemetry telemetry, LinearOpMode opMode) {
        this.armMotor = armMotor;
        this.opMode = opMode;
        this.telemetry = telemetry;

        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    static final double ARM_DRIVE_REDUCTION = 2;
    static final double ARM_WHEEL_DIAMETER_INCHES = 2.5;
    static final double ARM_MOTOR_RPM = 300;
    static final double ARM_COUNTS_PER_MOTOR_REV = 480;   // TorqueNADO 20:1
    static final double ARM_COUNTS_PER_WHEEL_REV = (ARM_COUNTS_PER_MOTOR_REV * ARM_DRIVE_REDUCTION);
    static final double ARM_COUNTS_PER_INCH = ARM_COUNTS_PER_WHEEL_REV / (ARM_WHEEL_DIAMETER_INCHES * 3.1415);
    private double TPS = ((ARM_MOTOR_RPM * .75) / 60) * ARM_COUNTS_PER_WHEEL_REV;
    static final int LOW_JUNCTION = 14;
    static final int MEDIUM_JUNCTION = 24;
    static final int HIGH_JUNCTION = 34;
    static final int HOME_POSITION = 0;
    static final int CONE_HEIGHT = 5;
    static final int ADJUST_ARM_INCREMENT = 1;
    // position in ticks.
    private int armPosition;
    private int armTarget = 0;
    private double armVelocity = 0;
    private double feedForwardCalculate;
    private ElevatorFeedforward armFeedForward;

    public void moveArm(ArmPosition position) {

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
            case CONE_HEIGHT:
                armTarget = armPosition + CONE_HEIGHT * (int) ARM_COUNTS_PER_INCH;
                break;
            default:
                armTarget = 0;
        }

        // Prevent arm moving below HOME_POSITION
        armTarget = Math.max(armTarget, HOME_POSITION * (int) ARM_COUNTS_PER_INCH);
        armMotor.setTargetPosition(armTarget);
        armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        armMotor.setVelocityPIDFCoefficients(1.26, 0.126, 0, 12.6);
        armMotor.setPositionPIDFCoefficients(25);
        armMotor.setTargetPositionTolerance(20);
        armMotor.setVelocity(TPS);

        while (armMotor.isBusy() && !opMode.isStopRequested()) {
            armVelocity = armMotor.getVelocity();
            feedForwardCalculate = armFeedForward.calculate(armVelocity);
            armMotor.setPower(feedForwardCalculate);
            armPosition = armMotor.getCurrentPosition();
        }
    }

    public enum ArmPosition {
        HOME,
        LOW,
        MEDIUM,
        HIGH,
        ADJUST_UP,
        ADJUST_DOWN,
        CONE_HEIGHT
    }
}
