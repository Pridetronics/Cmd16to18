
package org.usfirst.frc3853.CompetitionRobot20160120.subsystems;

import org.usfirst.frc3853.CompetitionRobot20160120.RobotMap;
import org.usfirst.frc3853.CompetitionRobot20160120.commands.DriveTeleop;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveTrain extends Subsystem {

  private final SpeedController rightMotor = RobotMap.DriveTrainRightMotor;
  private final SpeedController leftMotor = RobotMap.DriveTrainLeftMotor;
  private final RobotDrive robotDrive21 = RobotMap.DriveTrainRobotDrive21;
  private final Encoder driveTrainEncoderRight = RobotMap.DriveTrainDriveTrainEncoderRight;
  private final Encoder driveTrainEncoderLeft = RobotMap.DriveTrainDriveTrainEncoderLeft;

  // Add by hand because robot builder wouldn't let us do it.

  // Initialize your subsystem here
  public DriveTrain() {

    // Use these to get going:
    // setSetpoint() - Sets where the PID controller should move the system
    // to
    // enable() - Enables the PID controller.
    robotDrive21.stopMotor();
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new DriveTeleop());
  }

  // Set the default command for a subsystem here.
  // setDefaultCommand(new MySpecialCommand());
  public void RobotDrive21(Joystick joystick1, Joystick joystick2) {
    robotDrive21.arcadeDrive(joystick1, true);
  }

  protected double returnPIDInput() {
    // Return your input value for the PID loop
    // e.g. a sensor, like a potentiometer:
    // yourPot.getAverageVoltage() / kYourMaxVoltage;
    return driveTrainEncoderRight.pidGet();
  }

  // protected void usePIDOutput(double output) {
  // Use output to drive your system, like a motor
  // e.g. yourMotor.set(output);

  // rightMotor.pidWrite(output);

  // }

  // public void Drive(double left,double right){
  // robotDrive21.tankDrive(left, right);

  // }
  public void Drive(Joystick Joystick1) {
    robotDrive21.arcadeDrive(Joystick1);

  }

  public void TDrive(Joystick Joystick1, Joystick Joystick2) {
    robotDrive21.tankDrive(Joystick1, Joystick2);

  }

  public void left() {
    leftMotor.set(0.5);
    rightMotor.set(-0.5);
  }

  public void right() {
    leftMotor.set(-0.5);
    rightMotor.set(0.5);
  }

  public void stop() {
    robotDrive21.stopMotor();
  }

  public void DriveFoward(double speed) {
    robotDrive21.arcadeDrive(speed, speed);
    robotDrive21.tankDrive(speed, speed);
  }

  public double getRightEncoder() {
    return this.driveTrainEncoderRight.getDistance();
  }

  public double getLeftEncoder() {
    return this.driveTrainEncoderLeft.getDistance();

  }

  public void sdData() {
    SmartDashboard.putNumber("Drive Left Encoder", driveTrainEncoderLeft.get());
    SmartDashboard.putNumber("Drive Right Encoder", driveTrainEncoderRight.get());
  }

}
