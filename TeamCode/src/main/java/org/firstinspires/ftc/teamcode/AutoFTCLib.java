package org.firstinspires.ftc.teamcode;


import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;

@Autonomous(name = "State Auto", group = "FtcLib")
//@Autonomous(name = "State Auto", group = "FtcLib", preselectTeleOp = "PowerPlayDc")
public class AutoFTCLib extends LinearOpMode {
    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 3.778;     // For figuring circumference
    static final double DRIVE_SPEED = 0.4;         // Max driving speed for better distance accuracy.
    static final double TURN_SPEED = 0.4;
    // These are for GoBilda 435 motors
    static final double MOTOR_RPM = 425;
    static double COUNTS_PER_MOTOR_REV = 383.6;

    //TODO: These are for TorqweNado 20:1
    //    static final double MOTOR_RPM = 300;
    //    static double COUNTS_PER_MOTOR_REV = 480;
    static double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static double MOTOR_TPS = ((MOTOR_RPM * .75) / 60) * COUNTS_PER_MOTOR_REV;

    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double P_TURN_GAIN = 0.0009;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_GAIN = 0.02;     // Larger is more responsive, but also less stable
    // How close must the heading get to the target before moving to next step.
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    static final double HEADING_THRESHOLD = .8;
    static final double MOTOR_POSITION_COEFFICIENT = 30;
    static final int MOTOR_TARGET_POSITION_TOLERANCE = 20;
    private ElevatorArm elevatorArm;
    //     These set the range for the gripper servo.
    static final double GRIPPER_MIN_ANGLE = 0;
    static final double GRIPPER_MAX_ANGLE = 180;
    // These set the open and close positions
    static final double GRIPPER_OPEN = 25;
    static final double GRIPPER_CLOSED = 73;
    static final double MAX_POWER = 0.4;
    private int pathSegment;
    private StartPosition startPosition = StartPosition.NONE;
    private int STRAFE_TIMEOUT = 3;     //Time to wait for strafing to finish.
    private SleeveDetection sleeveDetection;
    private OpenCvWebcam camera;
    private IMU imu;
    private GamepadEx gamePadDrive;
    private String WEB_CAM_NAME = "Webcam 1";
    private SleeveDetection.ParkingPosition parkLocation;
    private DcMotorEx backLeftDrive;
    private DcMotorEx backRightDrive;
    private DcMotorEx frontLeftDrive;
    private DcMotorEx frontRightDrive;
    private DcMotorEx armMotor = null;
    //TODO: Test this to see if it adds anything
    //    private SimpleMotorFeedforward simpleFeedForward;
    private SimpleServo gripperServo;
    private int backLeftTarget = 0;
    private int backRightTarget = 0;
    private int frontLeftTarget = 0;
    private int frontRightTarget = 0;
    private int backLeftPosition = 0;
    private int backRightPosition = 0;
    private int frontLeftPosition = 0;
    private int frontRightPosition = 0;
    private double backLeftVelocity = 0;
    private double backRightVelocity = 0;
    private double frontLeftVelocity = 0;
    private double frontRightVelocity = 0;
    private double turnSpeed = 0;
    private double driveSpeed = 0;
    private double leftSpeed = 0;
    private double rightSpeed = 0;
    private double headingError = 0;
    private double targetHeading = 0;
    private double robotHeading = 0;
    private double headingOffset = 0;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        // Bulk reads
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        // Important: Set all Expansion hubs to use the AUTO Bulk Caching mode
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, WEB_CAM_NAME), cameraMonitorViewId);
        sleeveDetection = new SleeveDetection();
        camera.setPipeline(sleeveDetection);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_RIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        gamePadDrive = new GamepadEx(gamepad1);

        armMotor = hardwareMap.get(DcMotorEx.class, "ArmMotor");
        // Arm functions are handled by elevatorArm
        elevatorArm = new ElevatorArm(armMotor, this);

        backLeftDrive = hardwareMap.get(DcMotorEx.class, "Backleft");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "Backright");
        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "Frontleft");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "Frontright");
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        setMotorsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        setMotorsPositionCoefficients(MOTOR_POSITION_COEFFICIENT);

        //TODO: Maybe this will help. Test it.
        // simpleFeedForward = new SimpleMotorFeedforward(2, 10);

        gripperServo = new SimpleServo(hardwareMap, "GripperServo", GRIPPER_MIN_ANGLE, GRIPPER_MAX_ANGLE);
        openGripper();

        // Choose your start position.
        while (!isStarted() &&
                !gamePadDrive.wasJustPressed(GamepadKeys.Button.START) &&
                !isStopRequested()) {
            gamePadDrive.readButtons();
            if (gamePadDrive.wasJustPressed(GamepadKeys.Button.X)) {
                startPosition = StartPosition.LEFT;
                telemetry.speak(startPosition.toString());
                telemetry.update();
            }

            if (gamePadDrive.wasJustPressed(GamepadKeys.Button.B)) {
                startPosition = StartPosition.RIGHT;
                telemetry.speak(startPosition.toString());
                telemetry.update();
            }

            telemetry.addLine("Select start position");
            telemetry.addLine("X=Left, B=Right, Start=Adjust Camera");
            telemetry.addData("Position:", startPosition);
            telemetry.update();
        }

        // Adjust the camera here.
        while (!isStarted() && !isStopRequested()) {
            parkLocation = getParkLocation();
            telemetry.addData("Start Position:", startPosition);
            telemetry.addData("Park Location:", parkLocation);
            telemetry.update();
        }

        waitForStart();
        // Assume arm is physically at it's lowest position.
        elevatorArm.resetEncoder();
        // Save processing time, we are finished with the camera.
        camera.stopStreaming();
        camera.closeCameraDevice();

        pathSegment = 1;
        while (opModeIsActive() && !isStopRequested()) {
            switch (pathSegment) {
                case 1:
                    closeGripper();
                    driveStraight(DRIVE_SPEED, 36, 0, 3);
                    sleep(750);
                    pathSegment = 7;
                    break;
                case 2:
                    if (startPosition == StartPosition.RIGHT) {
                        turnToHeading(TURN_SPEED, -90, 3);
                    }
                    if (startPosition == StartPosition.LEFT) {
                        turnToHeading(TURN_SPEED, 90, 3);
                    }
                    sleep(750);
                    pathSegment = 3;
                    break;
                case 3:
                    if (startPosition == StartPosition.RIGHT) {
                        driveStraight(DRIVE_SPEED, 24, -90, 3);
                    }
                    if (startPosition == StartPosition.LEFT) {
                        driveStraight(DRIVE_SPEED, 24, 90, 3);
                    }
                    sleep(750);
                    pathSegment = 7;
                    break;
                case 4:
                    elevatorArm.moveArm(ElevatorArm.ArmPosition.HIGH);
                    openGripper();
                    elevatorArm.moveArm(ElevatorArm.ArmPosition.HOME);
                    pathSegment = 6;
                    break;
                case 5:
                    elevatorArm.moveArm(ElevatorArm.ArmPosition.HOME);
                    sleep(500);
                    if (startPosition == StartPosition.RIGHT) {
                        turnToHeading(TURN_SPEED, 90, 3);
                    }
                    if (startPosition == StartPosition.LEFT) {
                        turnToHeading(TURN_SPEED, -90, 3);
                    }
                    sleep(750);
                    driveStraight(DRIVE_SPEED, -60, 0, 3);
                    pathSegment = 6;
                    break;
                case 6:
                    // Where did the camera tell us to park?
                    switch (parkLocation) {
                        case LEFT:
                            strafeRobot(DRIVE_SPEED, 24, 270, STRAFE_TIMEOUT);
                            break;
                        case CENTER:
                            break;
                        case RIGHT:
                            strafeRobot(DRIVE_SPEED, 24, 90, STRAFE_TIMEOUT);
                            break;
                    }
                    pathSegment = 7;
                    break;
                case 7:
                    telemetry.addData("Status", "Path complete.");
                    telemetry.update();
                    break;
                default:
                    throw new IllegalStateException("Unexpected value: " + pathSegment);
            }
        }
    }

    /**
     * Get the result of the camera sleeve detection.
     */
    private SleeveDetection.ParkingPosition getParkLocation() {
        return sleeveDetection.getPosition();
    }
    /*
     * ====================================================================================================
     * Driving "Helper" functions are below this line.
     * These provide the high and low level methods that handle driving straight and turning.
     * ====================================================================================================
     */

    // **********  HIGH Level driving functions.  ********************

    /**
     * Method to drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the desired position
     * 2) Driver stops the opmode running.
     *
     * @param maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
     * @param distance      Distance (in inches) to move from current position.
     * @param heading       Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                      0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                      If a relative angle is required, add/subtract from the current robotHeading.
     * @param driveTime     Time limit for drive operation. Seconds.
     */
    public void driveStraight(double maxDriveSpeed,
                              double distance,
                              double heading,
                              double driveTime) {

        ElapsedTime driveTimer = new ElapsedTime();
        driveTimer.reset();
        setMotorsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Determine new target position, and pass to motor controller
        int moveCounts = (int) (distance * COUNTS_PER_INCH);
        getCurrentPositionsFromMotors();
        backLeftTarget = backLeftPosition + moveCounts;
        backRightTarget = backRightPosition + moveCounts;
        frontLeftTarget = frontLeftTarget + moveCounts;
        frontRightTarget = frontRightTarget + moveCounts;

//        telemetry.addData("backLeftTarget", backLeftTarget);
//        telemetry.addData("backRightTarget", backRightTarget);
//        telemetry.addData("frontLeftTarget", frontLeftTarget);
//        telemetry.addData("frontRightTarget", frontRightTarget);
//        telemetry.update();

        // Set Target FIRST, then turn on RUN_TO_POSITION
        backLeftDrive.setTargetPosition(backLeftTarget);
        backRightDrive.setTargetPosition(backRightTarget);
        frontLeftDrive.setTargetPosition(frontLeftTarget);
        frontRightDrive.setTargetPosition(frontRightTarget);

        // Set SDK Pidf values
        setMotorsMode(DcMotor.RunMode.RUN_TO_POSITION);
//        setMotorVelocityCoefficients();
        setMotorsPositionTolerance();
//        setMotorsPositionCoefficients(MOTOR_POSITION_COEFFICIENT);

        // Set the required driving speed  (must be positive for RUN_TO_POSITION)
        // Start driving straight, and then enter the control loop
        maxDriveSpeed = Math.abs(maxDriveSpeed);
        moveRobot(maxDriveSpeed, 0);

        // keep looping while we are still active, and any motors are running.
        while (opModeIsActive() &&
                !isStopRequested() &&
                (backLeftDrive.isBusy() || backRightDrive.isBusy() || frontLeftDrive.isBusy() || frontRightDrive.isBusy()) &&
                (driveTimer.time() < driveTime)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

            // if driving in reverse, the motor correction also needs to be reversed
            if (distance < 0)
                turnSpeed *= -1.0;

            // Apply the turning correction to the current driving speed.
            moveRobot(maxDriveSpeed, turnSpeed);
        }
    }

    /**
     * Method to spin on central axis to point in a new direction.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the heading (angle)
     * 2) Driver stops the opmode running.
     *
     * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
     * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                     0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                     If a relative angle is required, add/subtract from current heading.
     * @param turnTime     Time limit (seconds) to complete turn.
     */
    public void turnToHeading(double maxTurnSpeed, double heading, double turnTime) {
        ElapsedTime turnTimer = new ElapsedTime();
        turnTimer.reset();

        setMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setMotorVelocityCoefficients();
        setMotorsPositionCoefficients(MOTOR_POSITION_COEFFICIENT);

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() &&
                !isStopRequested() &&
                (Math.abs(headingError) > HEADING_THRESHOLD) &&
                turnTimer.time() < turnTime) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);
        }

        // Stop all motion;
        stopAllMotors();
    }

    /**
     * Method to obtain & hold a heading for a finite amount of time
     * Move will stop once the requested time has elapsed
     * This function is useful for giving the robot a moment to stabilize it's heading between movements.
     *
     * @param maxTurnSpeed Maximum differential turn speed (range 0 to +1.0)
     * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                     0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                     If a relative angle is required, add/subtract from current heading.
     * @param holdTime     Length of time (in seconds) to hold the specified heading.
     */
    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        setMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // keep looping while we have time remaining.
        while (opModeIsActive() &&
                !isStopRequested() &&
                (holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);
        }

        // Stop all motion;
//        stopAllMotors();
//        moveRobot(0, 0);
    }

    // **********  LOW Level driving functions.  ********************

    /**
     * This method uses a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading   The desired absolute heading (relative to last heading reset)
     * @param proportionalGain Gain factor applied to heading error to obtain turning power.
     * @return Turning power needed to get to required heading.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Get the robot heading by applying an offset to the IMU heading
        robotHeading = getRawHeading() - headingOffset;

        // Determine the heading current error
        headingError = targetHeading - robotHeading;

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    /**
     * This method takes separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     *
     * @param drive forward motor speed
     * @param turn  clockwise turning motor speed.
     */
    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed = turn;      // save this value as a class member so it can be used by telemetry.

        leftSpeed = drive - turn;
        rightSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0) {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        backLeftDrive.setVelocity(powerToTPS(leftSpeed));
        backRightDrive.setVelocity(powerToTPS(rightSpeed));
        frontLeftDrive.setVelocity(powerToTPS(leftSpeed));
        frontRightDrive.setVelocity(powerToTPS(rightSpeed));

//        getVelocityFromMotors();
//        backLeftDrive.setVelocity(simpleFeedForward.calculate(backLeftVelocity));
//        backRightDrive.setVelocity(simpleFeedForward.calculate(backRightVelocity));
//        frontLeftDrive.setVelocity(simpleFeedForward.calculate(frontLeftVelocity));
//        frontRightDrive.setVelocity(simpleFeedForward.calculate(frontRightVelocity));

        //sendTelemetry();
    }

    /**
     * Strafe left or right.
     *
     * @param strafeSpeed - Drive speed.
     * @param distance    - Distance in inches..
     * @param heading     - Strafe direction. Field-centric.
     * @param strafeTime  - Timeout seconds.
     */
    public void strafeRobot(double strafeSpeed, double distance, double heading, int strafeTime) {
        ElapsedTime strafeTimer = new ElapsedTime();
        strafeTimer.reset();
        setMotorsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        targetHeading = heading;
        int moveCounts = (int) (distance * COUNTS_PER_INCH);
        getCurrentPositionsFromMotors();
        backLeftTarget = backLeftPosition + moveCounts;
        backRightTarget = backRightPosition + moveCounts;
        frontLeftTarget = frontLeftPosition + moveCounts;
        frontRightTarget = frontRightPosition + moveCounts;

        if (heading > 180) {
            frontLeftTarget *= -1;
            backRightTarget *= -1;
        } else {
            frontRightTarget *= -1;
            backLeftTarget *= -1;
        }

        // Set Target FIRST, then turn on RUN_TO_POSITION
        backLeftDrive.setTargetPosition(backLeftTarget);
        backRightDrive.setTargetPosition(backRightTarget);
        frontLeftDrive.setTargetPosition(frontLeftTarget);
        frontRightDrive.setTargetPosition(frontRightTarget);

        setMotorsMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() &&
                !isStopRequested() &&
                (backLeftDrive.isBusy() &&
                        backRightDrive.isBusy() &&
                        frontLeftDrive.isBusy() &&
                        frontRightDrive.isBusy()) &&
                (strafeTimer.seconds() < strafeTime)) {
            backLeftDrive.setPower(strafeSpeed);
            backRightDrive.setPower(strafeSpeed);
            frontLeftDrive.setPower(strafeSpeed);
            frontRightDrive.setPower(strafeSpeed);
            sendTelemetry();
        }
    }

    /**
     * Display the various control parameters while driving
     */
    private void sendTelemetry() {
        telemetry.addData("Path Segment", pathSegment);
        telemetry.addData("Target FL:FR",
                "%7d:%7d\n       BL:BR %7d:%7d",
                frontLeftTarget,
                frontRightTarget,
                backLeftTarget,
                backRightTarget);
        telemetry.addData("Actual FL:FR",
                "%7d:%7d\n       BL:BR %7d:%7d",
                frontLeftPosition,
                frontRightPosition,
                backLeftPosition,
                backRightPosition);
        telemetry.addData("Angle Target:Current", "%5.2f:%5.0f", targetHeading, robotHeading);
        telemetry.addData("Error:Steer", "%5.1f:%5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds L:R.", "%5.2f : %5.2f", leftSpeed, rightSpeed);
        telemetry.update();
    }

    // Get postions from individual motors.
    public void getCurrentPositionsFromMotors() {
        backLeftPosition = backLeftDrive.getCurrentPosition();
        backRightPosition = backRightDrive.getCurrentPosition();
        frontLeftPosition = frontLeftDrive.getCurrentPosition();
        frontRightPosition = frontRightDrive.getCurrentPosition();
    }

    public void getVelocityFromMotors() {
        backLeftVelocity = backLeftDrive.getVelocity();
        backRightVelocity = backRightDrive.getVelocity();
        frontLeftVelocity = frontLeftDrive.getVelocity();
        frontRightVelocity = frontRightDrive.getVelocity();
    }

    public void setMotorsMode(DcMotor.RunMode mode) {
        backLeftDrive.setMode(mode);
        backRightDrive.setMode(mode);
        frontLeftDrive.setMode(mode);
        frontRightDrive.setMode(mode);
    }

    public void setMotorsPositionCoefficients(double PositionCoefficent) {
        backLeftDrive.setPositionPIDFCoefficients(PositionCoefficent);
        backRightDrive.setPositionPIDFCoefficients(PositionCoefficent);
        frontLeftDrive.setPositionPIDFCoefficients(PositionCoefficent);
        frontRightDrive.setPositionPIDFCoefficients(PositionCoefficent);
    }

    public void setMotorsPositionTolerance() {
        backLeftDrive.setTargetPositionTolerance(MOTOR_TARGET_POSITION_TOLERANCE);
        backRightDrive.setTargetPositionTolerance(MOTOR_TARGET_POSITION_TOLERANCE);
        frontLeftDrive.setTargetPositionTolerance(MOTOR_TARGET_POSITION_TOLERANCE);
        frontRightDrive.setTargetPositionTolerance(MOTOR_TARGET_POSITION_TOLERANCE);

    }

    public void setMotorVelocityCoefficients() {
        backLeftDrive.setVelocityPIDFCoefficients(1.26, 0.126, 0, 12.6);
        backRightDrive.setVelocityPIDFCoefficients(1.26, 0.126, 0, 12.6);
        frontLeftDrive.setVelocityPIDFCoefficients(1.26, 0.126, 0, 12.6);
        frontRightDrive.setVelocityPIDFCoefficients(1.26, 0.126, 0, 12.6);
    }

    public void stopAllMotors() {
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
    }

    /**
     * read the raw (un-offset Gyro heading) directly from the IMU
     */
    public double getRawHeading() {
        Orientation angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    /**
     * Reset the "offset" heading back to zero
     */
    public void resetHeading() {
        // Save a new heading offset equal to the current raw heading.
        headingOffset = getRawHeading();
        robotHeading = 0;
    }

    /**
     * Convert power to velocity.
     * double TPS = (power/60) * COUNTS_PER_MOTOR_REV;
     * ! For drive motors, not arm.
     *
     * @param power A motor power within -1 to 1.
     * @return Ticks per second for use in velocity.
     */
    private double powerToTPS(double power) {
        return ((power * MOTOR_RPM) / 60) * COUNTS_PER_MOTOR_REV;
    }

    public void openGripper() {
        gripperServo.turnToAngle(GRIPPER_OPEN);
    }

    public void closeGripper() {
        gripperServo.turnToAngle(GRIPPER_CLOSED);
        elevatorArm.moveArm(ElevatorArm.ArmPosition.CONE_HEIGHT);
    }

    public enum StartPosition {
        RIGHT,
        LEFT,
        NONE
    }
}