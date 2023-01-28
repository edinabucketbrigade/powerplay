package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Servo Tune", group = "test")
//@Disabled
public class RHSServoTune extends OpMode {
    SimpleServo servo;
    boolean isInverted = false;
    private final double MIN_RANGE = 0;
    private final double MAX_RANGE = 180;
    private double minAngle = 0;
    private double maxAngle = 180;
    private double range = 0;
    private double turnToAngle = 0;
    private GamepadEx gamePadDrive;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        gamePadDrive = new GamepadEx(gamepad1);
        servo = new SimpleServo(hardwareMap, "servo1", minAngle, maxAngle);
        servo.setInverted(isInverted);
        servo.setRange(minAngle, maxAngle);
    }

    @Override
    public void init_loop() {
    }

    /*
     * This method will be called ONCE when start is pressed
     */
    @Override
    public void start() {

    }

    /*
     * This method will be called repeatedly in a loop
     */
    @Override
    public void loop() {
        gamePadDrive.readButtons();

        if (gamePadDrive.wasJustPressed(GamepadKeys.Button.BACK)) {
            isInverted = !isInverted;
        }

        if (gamePadDrive.wasJustReleased(GamepadKeys.Button.DPAD_LEFT)) {
            if (minAngle > MIN_RANGE) {
                minAngle -= 1;
                updateRange();
            }
        }

        if (gamePadDrive.wasJustReleased(GamepadKeys.Button.DPAD_RIGHT)) {
            if (minAngle < MAX_RANGE) {
                minAngle += 1;
                updateRange();
            }
        }

        if (gamePadDrive.wasJustReleased(GamepadKeys.Button.DPAD_UP)) {
            if (maxAngle < MAX_RANGE) {
                maxAngle += 1;
                updateRange();
            }
        }

        if (gamePadDrive.wasJustReleased(GamepadKeys.Button.DPAD_DOWN)) {
            if (maxAngle > MIN_RANGE) {
                maxAngle -= 1;
                updateRange();
            }
        }

        if (gamePadDrive.wasJustReleased(GamepadKeys.Button.X)) {
            if (turnToAngle > MIN_RANGE) {
                turnToAngle -= 1;
                servo.turnToAngle(turnToAngle);
            }
        }

        if (gamePadDrive.wasJustReleased(GamepadKeys.Button.Y)) {
            if (turnToAngle < MAX_RANGE) {
                turnToAngle += 1;
                servo.turnToAngle(turnToAngle);
            }
        }

        if (gamePadDrive.wasJustReleased(GamepadKeys.Button.A)) {
            servo.turnToAngle(minAngle);
        }

        if (gamePadDrive.wasJustReleased(GamepadKeys.Button.B)) {
            servo.turnToAngle(maxAngle);
        }

        telemetry.addData("Inverted", isInverted);
        telemetry.addData("Min Angle", minAngle);
        telemetry.addData("Max Angle", maxAngle);
        telemetry.addData("Range", range);
        telemetry.addData("Turn to", turnToAngle);
        telemetry.addData("Servo angle", servo.getAngle());
        telemetry.addData("Servo position", servo.getPosition());
        telemetry.addData("Servo range", servo.getAngleRange());
        telemetry.update();
    }

    @Override
    public void stop() {

    }

    public void updateRange() {
        range = maxAngle - minAngle;
        servo.setRange(minAngle, maxAngle);
    }
}
