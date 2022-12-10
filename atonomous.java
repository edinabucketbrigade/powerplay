package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name = "Autonomous (Blocks to Java)", group = "Auto", preselectTeleOp = "Power Play DC")
public class JavaAutonomous extends LinearOpMode {

  private IMU imu_IMU;
  private Servo GripperServo;
  private DcMotor Backleft;
  private DcMotor Backright;
  private DcMotor Frontleft;
  private DcMotor Frontright;
  private ColorSensor sensorColorRange_REV_ColorRangeSensor;

  private double robotHeading = 0;
  private double headingOffset = 0;
  private double headingError = 0;
  private double targetHeading = 0;
  int blpos;
  int brpos;
  String Final_Color;
  int flpos;
  int Permanent_color;
  int frpos;
  int Step_;
  int Final_Hue;
  int active;
  private double leftSpeed = 0;
  private double rightSpeed = 0;
  private double turnSpeed = 0.2;     // Max Turn speed to limit turn rate
  static final double HEADING_THRESHOLD = 1.0;    // How close must the heading get to the target before moving to next step.
 // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double P_TURN_GAIN = 0.02;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_GAIN = 0.03;     // Larger is more responsive, but also less stable
    /* Declare OpMode members. */

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    int Inch;
    int Foot;
    int Tile_length;
    int my_1_2_Tile_Length;
    int Shift;
    double speed_value;

    imu_IMU = hardwareMap.get(IMU.class, "imu");
    GripperServo = hardwareMap.get(Servo.class, "GripperServo");
    Backleft = hardwareMap.get(DcMotor.class, "Backleft");
    Backright = hardwareMap.get(DcMotor.class, "Backright");
    Frontleft = hardwareMap.get(DcMotor.class, "Frontleft");
    Frontright = hardwareMap.get(DcMotor.class, "Frontright");
    sensorColorRange_REV_ColorRangeSensor = hardwareMap.get(ColorSensor.class, "sensorColorRange");
    // Initialize the IMU.
    // Initializes the IMU with non-default settings. To use this block,
    // plug one of the "new IMU.Parameters" blocks into the parameters socket.
    // Creates a Parameters object for use with an IMU in a REV Robotics Control Hub or Expansion Hub, specifying the hub's orientation on the robot via the direction that the REV Robotics logo is facing and the direction that the USB ports are facing.
    imu_IMU.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP)));
    // Put initialization blocks here.
    Permanent_color = 0;
    Final_Hue = 0;
    GripperServo.setPosition(0.71);
    Backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    Backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    Frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    Frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    Backleft.setDirection(DcMotorSimple.Direction.REVERSE);
    Frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
    Inch = 40;
    active = 0;
    Foot = Inch * 12;
    Tile_length = Inch * 23;
    my_1_2_Tile_Length = Tile_length / 2;
    blpos = 0;
    brpos = 0;
    flpos = 0;
    frpos = 0;
    Step_ = 1;
    Final_Color = "Blank";
    Shift = Tile_length + Inch * 5;
    speed_value = 0.75;
    resetHeading();
    while (opModeInInit()) {
      resetHeading();
      telemetry.addData("Hub orientation", headingOffset);
      telemetry.update();
    }
    waitForStart();
    // move away from wall to the left
    while (opModeIsActive()) {
      /*while (Step_ == 0) {
        // go forward 1 tile
        drive((Tile_length) * 4, (Tile_length) * 4, (Tile_length) * 4, (Tile_length) * 4, 0.25, 0);
        //driveStrafe((Tile_length) * 4, (Tile_length) * -4, (Tile_length) * -4, (Tile_length) * 4, 0.25);
        Step_ = 5;
      }*/
      while (Step_ == 1) {
        // go forward 1 tile
        drive((Tile_length - Inch * 2) * 1, (Tile_length - Inch * 2) * 1, (Tile_length - Inch * 2) * 1, (Tile_length - Inch * 2) * 1, 0.25, 0);
        Step_ = 2;
      }
      while (Step_ == 2) {
        // Color looking
        Look_for_color_values(25);
        Step_ = 3;
      }
      while (Step_ == 3) {
        Final_Hue = Permanent_color / 25;
        check_for_hue_to_color();
        drive(Inch * 7, Inch * 7, Inch * 7, Inch * 7, speed_value, 0);
        Step_ = 4;
      }
      while (Step_ == 4) {
        if (Final_Color.equals("Blue") || Final_Color.equals("Purple")) {
          driveStrafe(Shift * 1, Shift * -1, Shift * -1, Shift * 1, 0.5);
          drive(Inch * 3, Inch * 3, Inch * 3, Inch * 3, 1, 0);
          Step_ = 5;
          //requestOpModeStop();
        } else if (Final_Color.equals("Green")) {
          drive(Inch * 3, Inch * 3, Inch * 3, Inch * 3, 1, 0);
          Step_ = 5;
          //requestOpModeStop();
        } 
        else if (Final_Color.equals("Red")) {
          drive(Inch * -1, Inch * -1, Inch * -1, Inch * -1, 1, 0);
          driveStrafe(Shift * -1, Shift * 1, Shift * 1, Shift * -1, 0.2);
          drive(Inch * 4, Inch * 4, Inch * 4, Inch * 4, 1, 0);
          Step_ = 5;
          //requestOpModeStop();
        }
        while (Step_ == 5){
          if (active == 0) {
            requestOpModeStop();
            active = 1;
          }
        }
      }
      telemetry.update();
    }
  }
  /**
   * Describe this function...
   */
  private void drive(double blTarget, double brTarget, double flTarget, double frTarget, double speed, double heading) {
    blpos += blTarget;
    brpos += brTarget;
    flpos += flTarget;
    frpos += frTarget;
    Backleft.setTargetPosition(blpos);
    Backright.setTargetPosition(brpos);
    Frontleft.setTargetPosition(flpos);
    Frontright.setTargetPosition(frpos);
    Backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    Backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    Frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    Frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    Backleft.setPower(speed);
    Backright.setPower(speed);
    Frontleft.setPower(speed);
    Frontright.setPower(speed);
    while (opModeIsActive() && Backleft.isBusy() && Backright.isBusy() && Frontleft.isBusy() && Frontright.isBusy())
    {
      turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);
      moveRobot(speed, turnSpeed);
    }
  }
   private void driveStrafe(double blTarget, double brTarget, double flTarget, double frTarget, double speed) {
    blpos += blTarget;
    brpos += brTarget;
    flpos += flTarget;
    frpos += frTarget;
    Backleft.setTargetPosition(blpos);
    Backright.setTargetPosition(brpos);
    Frontleft.setTargetPosition(flpos);
    Frontright.setTargetPosition(frpos);
    Backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    Backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    Frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    Frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    Backleft.setPower(speed);
    Backright.setPower(speed);
    Frontleft.setPower(speed);
    Frontright.setPower(speed);
    while (opModeIsActive() && Backleft.isBusy() && Backright.isBusy() && Frontleft.isBusy() && Frontright.isBusy())
    {
      idle();
    }
  }

  /**
   * Describe this function...
   */
  private void Look_for_color_values(int How_many_times) {
    int gain;
    NormalizedRGBA normalizedColors;
    int color;
    float hue;
    float saturation;
    float value;

    // This op mode demonstrates the color and distance features of the REV sensor.
    gain = 2;
    for (int count = 0; count < How_many_times; count++) {
      // Display distance info.
      telemetry.addData("Dist to tgt (cm)", ((DistanceSensor) sensorColorRange_REV_ColorRangeSensor).getDistance(DistanceUnit.CM));
      // Display reflected light.
      telemetry.addData("Light detected", ((OpticalDistanceSensor) sensorColorRange_REV_ColorRangeSensor).getLightDetected());
      ((NormalizedColorSensor) sensorColorRange_REV_ColorRangeSensor).setGain(gain);
      telemetry.addData("Gain", ((NormalizedColorSensor) sensorColorRange_REV_ColorRangeSensor).getGain());
      // Read color from the sensor.
      normalizedColors = ((NormalizedColorSensor) sensorColorRange_REV_ColorRangeSensor).getNormalizedColors();
      telemetry.addData("Red", Double.parseDouble(JavaUtil.formatNumber(normalizedColors.red, 3)));
      telemetry.addData("Green", Double.parseDouble(JavaUtil.formatNumber(normalizedColors.green, 3)));
      telemetry.addData("Blue", Double.parseDouble(JavaUtil.formatNumber(normalizedColors.blue, 3)));
      // Convert RGB values to Hue, Saturation, and Value.
      // See https://en.wikipedia.org/wiki/HSL_and_HSV for details on HSV color model.
      color = normalizedColors.toColor();
      hue = JavaUtil.colorToHue(color);
      saturation = JavaUtil.colorToSaturation(color);
      value = JavaUtil.colorToValue(color);
      telemetry.addData("Hue", Double.parseDouble(JavaUtil.formatNumber(hue, 0)));
      telemetry.addData("Saturation", Double.parseDouble(JavaUtil.formatNumber(saturation, 3)));
      telemetry.addData("Value", Double.parseDouble(JavaUtil.formatNumber(value, 3)));
      telemetry.addData("Alpha", Double.parseDouble(JavaUtil.formatNumber(normalizedColors.alpha, 3)));
      // Use hue to determine if it's red, green, blue, etc..
      if (hue < 30) {
        telemetry.addData("Color", "Red");
      } else if (hue < 60) {
        telemetry.addData("Color", "Orange");
      } else if (hue < 90) {
        telemetry.addData("Color", "Yellow");
      } else if (hue < 150) {
        telemetry.addData("Color", "Green");
      } else if (hue < 225) {
        telemetry.addData("Color", "Blue");
      } else if (hue < 350) {
        telemetry.addData("Color", "Purple");
      } else {
        telemetry.addData("Color", "Blank");
      }
      // Check to see if it might be black or white.
      if (saturation < 0.2) {
        telemetry.addData("Check Sat", "Is surface white?");
        if (value < 0.16) {
          telemetry.addData("Check Val", "Is surface black?");
        }
      }
      Permanent_color += hue;
      telemetry.update();
      // Show white on the Robot Controller screen.
    }
    Step_ = 3;
  }

  /**
   * Describe this function...
   */
  private void check_for_hue_to_color() {
    if (Final_Hue <= 60 || Final_Hue >= 300) {
      Final_Color = "Red";
    } else if (Final_Hue <= 180 && Final_Hue >= 60) {
      Final_Color = "Green";
    } else {
      if (Final_Hue >= 180 && Final_Hue <= 285) {
        Final_Color = "Blue";
      } else {
      }
    }
  }
/*  private void sendTelemetry(boolean straight) {

      if (straight) {
        backright.getCurrentPosition();
      } else {
          telemetry.addData("Motion", "Turning");
      }
        // Retrieve Rotational Angles and Velocities
        YawPitchRollAngles orientation = IMU.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = IMU.getRobotAngularVelocity(AngleUnit.DEGREES);
    }
*/
    /**
   * describe this function...
   **/
   public double getRawHeading() {
     Orientation angles = imu_IMU.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
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
        leftSpeed = drive - turn;
        rightSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0) {
            leftSpeed /= max;
            rightSpeed /= max;
        }
        Frontright.setPower(rightSpeed);
        Frontleft.setPower(leftSpeed);
        Backleft.setPower(leftSpeed);
        Backright.setPower(rightSpeed);
    }

}
