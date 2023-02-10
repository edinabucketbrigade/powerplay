package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "OldSTele")
//@Disabled
public class PowerPlayDC1 extends LinearOpMode {

    private DcMotor ArmMotor;
    private DcMotor Frontleft;
    private DcMotor Backleft;
    private DcMotor Frontright;
    private DcMotor Backright;
    private Servo GripperServo;
//Whole Code Notes: Look at block code for denominator info.

    public void runOpMode() {
        boolean goToPosition;
        int lowJunction;
        int mediumJunction;
        int highJunction;
        int positionHome;
        int coneHeight;
        double maxPower;
        double y;
        double x;
        double rx;
        double denominator;

        ArmMotor = hardwareMap.get(DcMotor.class, "ArmMotor");
        Frontleft = hardwareMap.get(DcMotor.class, "Frontleft");
        Backleft = hardwareMap.get(DcMotor.class, "Backleft");
        Frontright = hardwareMap.get(DcMotor.class, "Frontright");
        Backright = hardwareMap.get(DcMotor.class, "Backright");
        GripperServo = hardwareMap.get(Servo.class, "GripperServo");

        ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        ArmMotor.setTargetPosition(0);
        ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        goToPosition = false;
        lowJunction = 1700;
        mediumJunction = 2800;
        highJunction = 3800;
        positionHome = 0;
        coneHeight = 750;
        maxPower = 0.7;
        waitForStart();
        if (opModeIsActive()) {
            Frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
            Backleft.setDirection(DcMotorSimple.Direction.REVERSE);
            while (opModeIsActive()) {
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

                // Arm Code
                if (gamepad2.dpad_down) {
                    ArmMotor.setTargetPosition(ArmMotor.getCurrentPosition() + -100);
                    ArmMotor.setPower(1);
                }
                if (gamepad2.dpad_up) {
                    ArmMotor.setTargetPosition(ArmMotor.getCurrentPosition() + 100);
                    ArmMotor.setPower(1);
                }
                if (gamepad2.a) {
                    while (gamepad2.a) {
                    }
                    if (ArmMotor.getCurrentPosition() > 3 && GripperServo.getPosition() == 0) {
                        ArmMotor.setTargetPosition(ArmMotor.getCurrentPosition() + (lowJunction - ArmMotor.getCurrentPosition()));
                        ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        ArmMotor.setPower(1);
                    } else {
                        ArmMotor.setTargetPosition(positionHome);
                        ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        ArmMotor.setPower(0.8);
                    }
                    goToPosition = true;
                }
                if (gamepad2.b) {
                    while (gamepad2.b) {
                    }
                    if (ArmMotor.getCurrentPosition() > 3 && GripperServo.getPosition() == 0) {
                        ArmMotor.setTargetPosition(ArmMotor.getCurrentPosition() + (mediumJunction - ArmMotor.getCurrentPosition()));
                        ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        ArmMotor.setPower(1);
                    } else {
                        ArmMotor.setTargetPosition(positionHome);
                        ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        ArmMotor.setPower(0.8);
                    }
                    goToPosition = true;
                }
                if (gamepad2.y) {
                    while (gamepad2.y) {
                    }
                    if (ArmMotor.getCurrentPosition() > 3 && GripperServo.getPosition() == 0) {
                        ArmMotor.setTargetPosition(ArmMotor.getCurrentPosition() + (highJunction - ArmMotor.getCurrentPosition()));
                        ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        ArmMotor.setPower(1);
                    } else {
                        ArmMotor.setTargetPosition(positionHome);
                        ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        ArmMotor.setPower(0.8);
                    }
                    goToPosition = true;
                }
                if (gamepad2.x) {
                    while (gamepad2.x) {
                        ArmMotor.setTargetPosition(positionHome);
                        ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        ArmMotor.setPower(0.8);
                    }
                    goToPosition = true;
                }
                // Gripper Code
                if (gamepad2.left_bumper) {
                    GripperServo.setPosition(0);
                    sleep(100);
                    ArmMotor.setTargetPosition(ArmMotor.getCurrentPosition() + coneHeight);
                    ArmMotor.setPower(1);
                }
                if (gamepad2.right_bumper) {
                    GripperServo.setPosition(.85);
                }
                telemetry.addData("Ticks", ArmMotor.getCurrentPosition());
                telemetry.addData("key", GripperServo.getPosition());
                telemetry.update();
            }
        }
    }
}
