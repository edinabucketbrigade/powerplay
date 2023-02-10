package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class ElevatorArm {
    private DcMotorEx armMotor;
    private DigitalChannel digitalTouch;
    OpMode opMode;

    public ElevatorArm(DcMotorEx armMotor, OpMode opMode) {
        this.armMotor = armMotor;
        this.opMode = opMode;

        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armFeedForward = new ElevatorFeedforward(12, 20, 5);
//        digitalTouch = opMode.hardwareMap.get(DigitalChannel.class, "touch");
    }

    static final double ARM_DRIVE_REDUCTION = 1.75;
    static final double ARM_WHEEL_DIAMETER_INCHES = 2.5;
    static final double ARM_MOTOR_RPM = 300;
    static final double ARM_COUNTS_PER_MOTOR_REV = 480;   // TorqueNADO 20:1
    static final double ARM_COUNTS_PER_WHEEL_REV = (ARM_COUNTS_PER_MOTOR_REV * ARM_DRIVE_REDUCTION);
    static final double ARM_COUNTS_PER_INCH = ARM_COUNTS_PER_WHEEL_REV / (ARM_WHEEL_DIAMETER_INCHES * 3.1415);
    private double TPS = ((ARM_MOTOR_RPM * .75) / 60) * ARM_COUNTS_PER_WHEEL_REV;
    static final int LOW_JUNCTION = 17;
    static final int MEDIUM_JUNCTION = 27;
    static final int HIGH_JUNCTION = 37;
    static final int HOME_POSITION = 0;
    static final int CONE_HEIGHT = 5;
    static final int ADJUST_ARM_INCREMENT = 1;
    // position in ticks.
    private int armPosition;
    private int armTarget = 0;
    private double armVelocity = 0;
    private double armCurrent = 0;
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

        // Prevent arm moving below HOME_POSITION or above HIGH_JUNCTION
        armTarget = Math.max(armTarget, HOME_POSITION * (int) ARM_COUNTS_PER_INCH);
        armTarget = Math.min(armTarget, HIGH_JUNCTION * (int) ARM_COUNTS_PER_INCH);
        armMotor.setTargetPosition(armTarget);
        armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//        armMotor.setVelocityPIDFCoefficients(1.09, 0.109, 0, 10.9);
//        armMotor.setPositionPIDFCoefficients(10);
        armMotor.setTargetPositionTolerance(25);
        armMotor.setVelocity(TPS);

        while (armMotor.isBusy()) {
            armVelocity = armMotor.getVelocity();
//            feedForwardCalculate = armFeedForward.calculate(armVelocity);
//            armMotor.setVelocity(feedForwardCalculate);
            armMotor.setVelocity(TPS);
            armVelocity = armMotor.getVelocity();
            armPosition = armMotor.getCurrentPosition();
            armCurrent = armMotor.getCurrent(CurrentUnit.AMPS);
        }
    }

    public double getArmPosition() {
        return armPosition;
    }

    public PIDFCoefficients getVelocityPidfoefficients() {
        return armMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public PIDFCoefficients getPositionPIDFCoefficients() {
        return armMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public int getPositionTolerance() {
        return armMotor.getTargetPositionTolerance();
    }

    /*
        Get the cuurent the motor is currently using.
     */
    public double getArmCurrent() {
        return armCurrent;
    }

    public void resetEncoder() {
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /*
        Move the arm down until touch sensor is triggered or current is too high (something failed).
        That is the lowest (starting) position.
        resetEncoder() is also called to match the defined height position values.
     */
    public void resetArmPosition() {
        double CURRENT_LIMIT = 3;

        // NOTE: getState() = true means NOT pressed.
        if (digitalTouch.getState()) {
            armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armMotor.setPower(-.2);
            while (digitalTouch.getState() &&
                    armMotor.getCurrent(CurrentUnit.AMPS) < CURRENT_LIMIT) {
            }

            armMotor.setPower(0);
        }

        // Set the encoder to 0 so the junction heights work.
        resetEncoder();
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
