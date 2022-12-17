package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "JavaAutonomous", group = "Auto")
public class JavaAutonomous extends LinearOpMode {

  private DcMotor BackleftAsDcMotor;
  private DcMotor Backright;
  private DcMotor FrontleftAsDcMotor;
  private DcMotor Frontright;
  private Servo GripperServoAsServo;
  private ColorSensor sensorColorRangeAsREVColorRangeSensor;

  
  int blpos;
  int brpos;
  int Permanent_color;
  String Final_Color;
  int flpos;
  int Final_Hue;
  int frpos;
  int Step_;

  /**
   * Describe this function...
   */
  private void drive(double blTarget, double brTarget, double flTarget, double frTarget, double speed) {
    blpos += blTarget;
    brpos += brTarget;
    flpos += flTarget;
    frpos += frTarget;
    BackleftAsDcMotor.setTargetPosition(blpos);
    Backright.setTargetPosition(brpos);
    FrontleftAsDcMotor.setTargetPosition(flpos);
    Frontright.setTargetPosition(frpos);
    BackleftAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    Backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    FrontleftAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    Frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    BackleftAsDcMotor.setPower(speed);
    Backright.setPower(speed);
    FrontleftAsDcMotor.setPower(speed);
    Frontright.setPower(speed);
    while (opModeIsActive() && BackleftAsDcMotor.isBusy() && Backright.isBusy() && FrontleftAsDcMotor.isBusy() && Frontright.isBusy()) {
      idle();
    }
  }

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    double Inch;
    double Foot;
    double Tile_length;
    double my_1_2_Tile_Length;
    double Shift;
    double speed_value;

    BackleftAsDcMotor = hardwareMap.get(DcMotor.class, "BackleftAsDcMotor");
    Backright = hardwareMap.get(DcMotor.class, "Backright");
    FrontleftAsDcMotor = hardwareMap.get(DcMotor.class, "FrontleftAsDcMotor");
    Frontright = hardwareMap.get(DcMotor.class, "Frontright");
    GripperServoAsServo = hardwareMap.get(Servo.class, "GripperServoAsServo");
    sensorColorRangeAsREVColorRangeSensor = hardwareMap.get(ColorSensor.class, "sensorColorRangeAsREVColorRangeSensor");

    // Put initialization blocks here.
    Permanent_color = 0;
    Final_Hue = 0;
    GripperServoAsServo.setPosition(0.71);
    BackleftAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    Backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    FrontleftAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    Frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    BackleftAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    FrontleftAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    Inch = 76.9230769231;
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
    waitForStart();
    // move away from wall to the left
    while (opModeIsActive()) {
      while (Step_ == 1) {
        // go forward 1 tile
        drive((Tile_length - Inch * 2) * 1, (Tile_length - Inch * 2) * 1, (Tile_length - Inch * 2) * 1, (Tile_length - Inch * 2) * 1, 0.25);
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
        drive(Inch * 7, Inch * 7, Inch * 7, Inch * 7, speed_value);
        Step_ = 4;
      }
      while (Step_ == 4) {
        if (Final_Color.equals("Blue") || Final_Color.equals("Purple")) {
          drive(Shift * 1, Shift * -1, Shift * -1, Shift * 1, 0.5);
          drive(Inch * 3, Inch * 3, Inch * 3, Inch * 3, 1);
          requestOpModeStop();
        } else if (Final_Color.equals("Green")) {
          drive(Inch * 3, Inch * 3, Inch * 3, Inch * 3, 1);
          requestOpModeStop();
        } else {
          if (Final_Color.equals("Red")) {
            drive(Inch * -1, Inch * -1, Inch * -1, Inch * -1, 1);
            drive(Shift * -1, Shift * 1, Shift * 1, Shift * -1, 0.2);
            drive(Inch * 4, Inch * 4, Inch * 4, Inch * 4, 1);
            requestOpModeStop();
          } else {
            requestOpModeStop();
          }
        }
        Step_ = 5;
      }
      telemetry.update();
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
      telemetry.addData("Dist to tgt (cm)", ((DistanceSensor) sensorColorRangeAsREVColorRangeSensor).getDistance(DistanceUnit.CM));
      // Display reflected light.
      telemetry.addData("Light detected", ((OpticalDistanceSensor) sensorColorRangeAsREVColorRangeSensor).getLightDetected());
      ((NormalizedColorSensor) sensorColorRangeAsREVColorRangeSensor).setGain(gain);
      telemetry.addData("Gain", ((NormalizedColorSensor) sensorColorRangeAsREVColorRangeSensor).getGain());
      // Read color from the sensor.
      normalizedColors = ((NormalizedColorSensor) sensorColorRangeAsREVColorRangeSensor).getNormalizedColors();
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
}
