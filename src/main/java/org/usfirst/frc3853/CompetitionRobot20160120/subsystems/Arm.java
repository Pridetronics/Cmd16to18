// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

package org.usfirst.frc3853.CompetitionRobot20160120.subsystems;

import org.usfirst.frc3853.CompetitionRobot20160120.Robot;
import org.usfirst.frc3853.CompetitionRobot20160120.RobotMap;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Arm extends PIDSubsystem {

  // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

  // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

  // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
  private final SpeedController armMotorTalon = RobotMap.armArmMotorTalon;
  private final Encoder armEncoder = RobotMap.armArmEncoder;
  private final DigitalInput limitSwitchArm = RobotMap.armLimitSwitchArm;

  // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

  //private double setPointHome = 0;
  //private double setPointEnd = -7126;
  //private double setPointVertical = 90;
  //private double setPointAngle = 130;
  private double setPointCheval = 7.5;
  private double setPointPortcullis = 6.5;
  
  // Initialize your subsystem here
  public Arm() {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=PID
    super("Arm", .25, 0.0, 0.0);
    setAbsoluteTolerance(0.2);
    getPIDController().setContinuous(false);
    LiveWindow.addActuator("Arm", "PIDSubsystem Controller", getPIDController());

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=PID

    // Use these to get going:
    // setSetpoint() - Sets where the PID controller should move the system
    // to
    // enable() - Enables the PID controller.
    armMotorTalon.stopMotor();
  }

  @Override
  public void initDefaultCommand() {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  @Override
  public double returnPIDInput() {
    return armEncoder.pidGet();

  }

  @Override
  protected void usePIDOutput(double output) {
    armMotorTalon.pidWrite(output);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=OUTPUT
  }

  public void Up(double speed) {
    speed = Math.abs(speed);
    armMotorTalon.set(-speed);

  }

  public void Down(double speed) {
    speed = Math.abs(speed);
    armMotorTalon.set(speed);
  }
  

  public void End() {
    armMotorTalon.stopMotor();
  }

  public boolean limitSwitchOpen() {
    // TODO Auto-generated method stub
    return this.limitSwitchArm.get();
  }

  public double getArmEncoderDistance() {

    return armEncoder.getDistance();
  }

  public void resetEncoder() {
    armEncoder.reset();
  }

  public void setPoint(double set) {
    this.setSetpoint(set);
  } 
  public void setPoint(double set,double speed) {
    this.setOutputRange(-speed, speed);
    this.setSetpoint(set);
  }

  public void sdData() {
    SmartDashboard.putBoolean("armLimit", limitSwitchArm.get());
    SmartDashboard.putNumber("armEncoder", armEncoder.get());
  }
  public void goToCheval() {
    this.setSetpoint(setPointCheval);
  }
  public void goToPortcullis() {
    this.setSetpoint(setPointPortcullis);
      
  }
  //public void goToAngle() {
    //this.setSetpoint(setPointAngle);
  //}
  //public void goToVertical() {
   // this.setSetpoint(setPointVertical);
  //}
}
