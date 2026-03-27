// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.intakeConstants;



public class intake extends SubsystemBase {
  boolean intakeOut = false;
  boolean intakeForward = false;
  boolean  intakeReverse = false;
  /** Creates a new ExampleSubsystem. */
  public static SparkMax intakeMotor = new SparkMax(Constants.intakeConstants.intakeMotorID,MotorType.kBrushless);
    SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();
    public static SparkMax intakeMotorSecond = new SparkMax(Constants.intakeConstants.intakeMotorSecond,MotorType.kBrushless);
    public RelativeEncoder intakeMotorEncoder = intakeMotor.getEncoder();
    public static PneumaticHub m_pH = new PneumaticHub(Constants.intakeConstants.pnumaticID);
      DoubleSolenoid m_doubleSolenoidLeft = m_pH.makeDoubleSolenoid(Constants.intakeConstants.intakeNuIDLeftF, Constants.intakeConstants.intakeNuIDLeftR);
      DoubleSolenoid m_doubleSolenoidRight = m_pH.makeDoubleSolenoid(Constants.intakeConstants.intakeNuIDRightF, Constants.intakeConstants.intakeNuIDRightR);

  public intake() {

      m_pH.enableCompressorDigital();
  }

    public void spinIntake (){

      intakeMotorConfig
    .inverted(false)
    .idleMode(IdleMode.kCoast);
intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); 

intakeMotorSecond.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); 

//  elevatorPID.setReference(-32,SparkBase.ControlType.kPosition);
          /* 
            intakePID.setSetpoint(
              targetSpeed,
              SparkBase.ControlType.kVelocity, 
              ClosedLoopSlot.kSlot0,
              0, 
              ArbFFUnits.kVoltage);*/
        intakeMotor.set(0.7);
        intakeForward = true;
        intakeReverse = false;
            
    }
    public void stopIntake(){
      intakeMotor.stopMotor();
      intakeForward = false;
      intakeReverse = false;
    }

    public void reverseINtake(){
      intakeMotor.set(-0.7);
      intakeReverse = true;
      intakeForward = false;
    } 



  public void intakeFoldOut(){
    m_doubleSolenoidLeft.set(Value.kForward);
    m_doubleSolenoidRight.set(Value.kForward);
    intakeOut = true;
  }
  public void intakeFoldIn(){
    m_doubleSolenoidLeft.set(Value.kReverse);
    m_doubleSolenoidRight.set(Value.kReverse);
    intakeOut = false;
  } 

  /**
   * Example command factory method.
   *
   * @return a command
   */
  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */

  @Override
  public void periodic() {
  switch (m_doubleSolenoidLeft.get()) {
      case kOff:
        SmartDashboard.putString("Get Solenoid", "kOff");
        break;
      case kForward:
        SmartDashboard.putString("Get Solenoid", "kForward");
        break;
      case kReverse:
        SmartDashboard.putString("Get Solenoid", "kReverse");
        break;
      default:
        SmartDashboard.putString("Get Solenoid", "N/A");
        break;
  }

  SmartDashboard.putBoolean("Intake Forward", intakeForward);
  SmartDashboard.putBoolean("Intake Reverse", intakeReverse);
  SmartDashboard.putBoolean("Intake Out", intakeOut);

   switch (m_doubleSolenoidRight.get()) {
      case kOff:
        SmartDashboard.putString("Get Solenoid", "kOff");
        break;
      case kForward:
        SmartDashboard.putString("Get Solenoid", "kForward");
        break;
      case kReverse:
        SmartDashboard.putString("Get Solenoid", "kReverse");
        break;
      default:
        SmartDashboard.putString("Get Solenoid", "N/A");
        break;

  }
        }


  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public static Object getInstance() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getInstance'");
  }
}
