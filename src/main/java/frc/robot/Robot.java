/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Robot extends TimedRobot {
  private XboxController xboxController;
  private Joystick joystick;
  private DifferentialDrive drivePlatform;
  private SpeedController camMotor, grabberMotor, lifterMotor;

  @Override
  public void robotInit() {
    xboxController = new XboxController(0);

    joystick = new Joystick(1);

    SpeedControllerGroup rightMotorGroup = new SpeedControllerGroup(new CANSparkMax(1, MotorType.kBrushless),
        new CANSparkMax(3, MotorType.kBrushless));
    rightMotorGroup.setInverted(true);

    SpeedControllerGroup leftMotorGroup = new SpeedControllerGroup(new CANSparkMax(2, MotorType.kBrushless),
        new CANSparkMax(4, MotorType.kBrushless));
    leftMotorGroup.setInverted(true);

    drivePlatform = new DifferentialDrive(leftMotorGroup, rightMotorGroup);

    camMotor = new CANTalonSRX(25);

    grabberMotor = new CANTalonSRX(21);

    lifterMotor = new CANSparkMax(12, MotorType.kBrushless);
    lifterMotor.setInverted(true);

    // proximitySensor = new AnalogInput(0);
    // accum_cm = countToCM(proximitySensor.getValue());
  }

  private double countToCM(int count) {
    return -0.0000000457 * Math.pow(count, 3) + 0.0001943165 * Math.pow(count, 2) - 0.2725554439 * count
        + 143.7771684066;
  }

  @Override
  public void teleopPeriodic() {
    double rightSpeed = xboxController.getY(Hand.kRight);
    double leftSpeed = xboxController.getY(Hand.kLeft);

    drivePlatform.tankDrive(leftSpeed, rightSpeed, true);

    if (xboxController.getTriggerAxis(Hand.kRight) > 0.05)
      camMotor.set(-xboxController.getTriggerAxis(Hand.kRight));
    else if (xboxController.getTriggerAxis(Hand.kLeft) > 0.05)
      camMotor.set(xboxController.getTriggerAxis(Hand.kLeft));
    else
      camMotor.stopMotor();

    double joystickY = joystick.getY();
    if (Math.abs(joystickY) > 0.05)
      lifterMotor.set(joystickY);
    else
      lifterMotor.stopMotor();

    if (joystick.getRawButton(5))
      grabberMotor.set(0.5);
    else if (joystick.getRawButton(3))
      grabberMotor.set(-0.5);
    else
      grabberMotor.stopMotor();

    // int reading_count = proximitySensor.getValue();

    // int frac = 1;
    // double reading_cm = countToCM(reading_count);
    // accum_cm = (reading_cm + frac * accum_cm) / (frac + 1);

    // System.out.println((accum_cm) + "cm");
  }
}
