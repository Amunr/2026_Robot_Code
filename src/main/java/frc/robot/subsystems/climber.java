// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.CANifier.LEDChannel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.config.*;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.climberConstants;


import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
//import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class climber extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public static SparkMax deployMotor = new SparkMax(Constants.climberConstants.deployMotorID,MotorType.kBrushless);
  SparkMaxConfig deployMotorConfig = new SparkMaxConfig();
  static SparkClosedLoopController deployPID = deployMotor.getClosedLoopController();
  public RelativeEncoder deployMotorEncoder = deployMotor.getEncoder();
  double targetRotations = 0.0;
  public static SparkMax climbMotor = new SparkMax(Constants.climberConstants.climberMotorID,MotorType.kBrushless);
  SparkMaxConfig climbMotorConfig = new SparkMaxConfig();
  static SparkClosedLoopController climbPID = climbMotor.getClosedLoopController();
  public RelativeEncoder climbMotorEncoder = climbMotor.getEncoder();
  double targetRotationsClimb = 0.0;
  public climber() {
    deployMotorConfig
      .inverted(true)
      .idleMode(IdleMode.kCoast)
      .smartCurrentLimit(40)
      ;

      deployMotorConfig.encoder
      .positionConversionFactor(1)
      .velocityConversionFactor(1);

      deployMotorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(0.01, 0, 0)
      .maxOutput(0.1)
      ;
    
    deployMotor.configure(deployMotorConfig, ResetMode.kResetSafeParameters, null);
      
    climbMotorConfig
      .inverted(true)
      .idleMode(IdleMode.kCoast)
      .smartCurrentLimit(40);

      climbMotorConfig.encoder
      .positionConversionFactor(1)
      .velocityConversionFactor(1);

      climbMotorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(0.01, 0, 0)
      .maxOutput(0.1)
      ;
    
    climbMotor.configure(climbMotorConfig, ResetMode.kResetSafeParameters, null);
      


  }


  public void movePOS (){
    
     //  elevatorPID.setReference(-32,SparkBase.ControlType.kPosition);
          if(targetRotations == 0.0){
            setForward();
          }else{
            setBackward();
          }
            deployPID.setSetpoint(
              targetRotations,
              SparkBase.ControlType.kPosition, 
              ClosedLoopSlot.kSlot0,
              0, 
              ArbFFUnits.kVoltage);
            
    }
    public void moveClimber(){
      climbPID.setSetpoint(
            targetRotationsClimb,
            SparkBase.ControlType.kPosition, 
            ClosedLoopSlot.kSlot0,
            0, 
            ArbFFUnits.kVoltage);
    }

  public void setBackward (){
      targetRotations = 0.0; 
      //SmartDashboard.putString("Elevator Set Level", "level One");
    }

  public void setForward (){
      targetRotations = climberConstants.forwardRotations; 
      //SmartDashboard.putString("Elevator Set Level", "level One");
    }

  public double getEnc (){
      return deployMotorEncoder.getPosition();
   }

    public void resetEnc (){
       deployMotorEncoder.setPosition(0);
  }
  public void setL1(){
      targetRotationsClimb = climberConstants.climbLevel1; 
      //SmartDashboard.putString("Elevator Set Level", "level One");
    }
    public void setL1Over(){
      targetRotationsClimb = climberConstants.climbLevel1+climberConstants.overShoot; 
    }
    public void setL2(){
      targetRotationsClimb = climberConstants.climbLevel2; 
      //SmartDashboard.putString("Elevator Set Level", "level Two");
    }
     public void setL2Over(){
      targetRotationsClimb = climberConstants.climbLevel2+climberConstants.overShoot; 
    }
    public void setL3(){
      targetRotationsClimb = climberConstants.climbLevel3; 
      //SmartDashboard.putString("Elevator Set Level", "level Three");
    }
     public void setL3Over(){
      targetRotationsClimb = climberConstants.climbLevel3+climberConstants.overShoot; 
    }

      //SmartDashboard.putString("elavator forward", "false");
        // this.m_elevatorSys = m_elevatorSys;
        
      
  /**
   * Example command factory method.
   * 
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
