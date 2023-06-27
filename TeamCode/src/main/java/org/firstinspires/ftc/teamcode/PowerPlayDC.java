package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "PowerPlayDCJava")
//@Disabled
public class PowerPlayDC extends LinearOpMode {
    static final double GRIPPER_MIN_ANGLE = 0;
    static final double GRIPPER_MAX_ANGLE = 180;
    // These set the open and close positions
    static final double GRIPPER_OPEN = 153;
    static final double GRIPPER_CLOSED = 23;
    private DcMotorEx armMotor = null;
    private ElevatorArm elevatorArm;
    private GamepadEx gamePadArm;
    private int armTarget = 0;
    private int armPosition = 0;
    private ElevatorArm.ArmPosition selectedPosition;
    private double armVelocity = 0;
    private double armCurrent = 0;
    private boolean isBusy = false;
    private DcMotor Frontleft;
    private DcMotor Backleft;
    private DcMotor Frontright;
    private DcMotor Backright;
    private SimpleServo GripperServo;
//Whole Code Notes: Look at block code for denominator info.

    public void runOpMode() {
        double maxPower = 1;
        double y;
        double x;
        double rx;
        double denominator;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        armMotor = hardwareMap.get(DcMotorEx.class, "ArmMotor");
        // elevatorArm manages the arm
        elevatorArm = new ElevatorArm(armMotor, this);
//TODO: Enable this line after testing.
//        elevatorArm.resetArmPosition();

        gamePadArm = new GamepadEx(gamepad2);

        Frontleft = hardwareMap.get(DcMotor.class, "Frontleft");
        Backleft = hardwareMap.get(DcMotor.class, "Backleft");
        Frontright = hardwareMap.get(DcMotor.class, "Frontright");
        Backright = hardwareMap.get(DcMotor.class, "Backright");
        GripperServo = new SimpleServo(hardwareMap, "GripperServo",
                GRIPPER_MIN_ANGLE, GRIPPER_MAX_ANGLE);

        waitForStart();
        elevatorArm.resetEncoder();
        Frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
        Backleft.setDirection(DcMotorSimple.Direction.REVERSE);

        if (opModeIsActive()) {
            while (opModeIsActive() && !isStopRequested()) {
                gamePadArm.readButtons();
                ProcessArm();
                sendEncoderTelemetry();

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

                if (gamePadArm.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
                    openGripper();
                }
                if (gamePadArm.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
                    closeGripper();
                }
            }
        }
    }

    public void sendEncoderTelemetry(){
        telemetry.addData("BL",Backleft.getCurrentPosition());
        telemetry.addData("BR",Backright.getCurrentPosition());
        telemetry.addData("FL",Frontleft.getCurrentPosition());
        telemetry.addData("FR",Frontright.getCurrentPosition());
        telemetry.update();
    }
    public void sendTelemetry(String location) {
        telemetry.addData("Called from", location);
        telemetry.addData("Selected Position", selectedPosition);
        telemetry.addData("Target Position", "%d", armTarget);
        telemetry.addData("Current Position", armPosition);
        telemetry.addData("Velocity", armVelocity);
        telemetry.addData("Current", armCurrent);
        telemetry.addData("Busy", isBusy);
        telemetry.update();
    }


    public void ProcessArm() {
        ElevatorArm.ArmPosition position = null;
        // Adjust position
        if (gamePadArm.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            position = ElevatorArm.ArmPosition.ADJUST_DOWN;
        }

        if (gamePadArm.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            position = ElevatorArm.ArmPosition.ADJUST_UP;
        }

        // Low junction
        if (gamePadArm.wasJustPressed(GamepadKeys.Button.A)) {
            position = ElevatorArm.ArmPosition.LOW;
        }

        // medium junction
        if (gamePadArm.wasJustPressed(GamepadKeys.Button.B)) {
            position = ElevatorArm.ArmPosition.MEDIUM;
        }

        // high junction
        if (gamePadArm.wasJustPressed(GamepadKeys.Button.Y)) {
            position = ElevatorArm.ArmPosition.HIGH;
        }

        // ground junction
        if (gamePadArm.wasJustPressed(GamepadKeys.Button.X)) {
            position = ElevatorArm.ArmPosition.HOME;
        }

        if (position != null) {
            elevatorArm.moveArm(position);
        }
    }

    public void openGripper() {
        GripperServo.turnToAngle(GRIPPER_OPEN);
    }

    public void closeGripper() {
        GripperServo.turnToAngle(GRIPPER_CLOSED);
        sleep(100);
        elevatorArm.moveArm(ElevatorArm.ArmPosition.CONE_HEIGHT);
    }
}
