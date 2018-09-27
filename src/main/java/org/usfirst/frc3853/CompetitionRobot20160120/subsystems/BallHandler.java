
package org.usfirst.frc3853.CompetitionRobot20160120.subsystems;

import org.usfirst.frc3853.CompetitionRobot20160120.RobotMap;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class BallHandler extends Subsystem {
  private final SpeedController motor = RobotMap.ballHandlerMotor;
  private final DigitalInput limitSwitchBallHandler = RobotMap.ballHandlerLimitSwitchBallHandler;

  // Joystick stick = new Joystick(0);
  // Preferences prefs;

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {

    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public BallHandler() {
    // LiveWindow.addActuator("BallHandler", "frontMotor", );
  }

  public void limitSwitchballhander() {
    motor.set(.5);
  }

  public void Load() {
    // double backup = 0;
    // double speed = prefs.getDouble("BallGrabSpeed", backup);
    motor.set(1);
    SmartDashboard.putNumber("BallLoad", 1);
  }

  public void Shoot() {
    // double backup = 0;
    // double speed = prefs.getDouble("BallShootSpeed", backup);
    motor.set(-1);
    SmartDashboard.putNumber("BallShoot", -1);
  }

  public void Release() {
    motor.set(-0.1);
    SmartDashboard.putNumber("BallRelease", -0.1);
  }

  public void stop() {
    motor.set(0);
    SmartDashboard.putNumber("BallStop", 0);
  }

  public boolean limitSwitchBallHandler() {
    // TODO Auto-generated method stub
    return !limitSwitchBallHandler.get();
  }

  public void sdData() {
    SmartDashboard.putBoolean("BallSwitch", limitSwitchBallHandler.get());
    SmartDashboard.putNumber("BallMotorSpeed", motor.get());
  }
}
