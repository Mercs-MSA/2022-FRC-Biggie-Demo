// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.TimedRobot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  //private final Subsystems my_subsystem = new Subsystems();
  private final WPI_TalonFX shooter_motor1 = new WPI_TalonFX(7);
  private final WPI_TalonFX shooter_motor2 = new WPI_TalonFX(8);
  private final WPI_TalonFX driver_leftmotor1 = new WPI_TalonFX(2);
  private final WPI_TalonFX driver_leftmotor2 = new WPI_TalonFX(3);
  private final WPI_TalonFX driver_rightmotor1 = new WPI_TalonFX(4);
  private final WPI_TalonFX driver_rightmotor2 = new WPI_TalonFX(5);
  private final WPI_TalonFX climber_motor1 = new WPI_TalonFX(14);
  private final WPI_TalonFX climber_motor2 = new WPI_TalonFX(15);
  private final WPI_TalonFX intake_motor1 = new WPI_TalonFX(19);
  private final WPI_TalonFX conveyer1 = new WPI_TalonFX(20);

  private final DigitalInput climberEncoderData = new DigitalInput(1);
  private final DutyCycleEncoder climberEncoder = new DutyCycleEncoder(climberEncoderData);
  //Joysticks
  private final XboxController driver_joystick = new XboxController(0);
  private final XboxController copilot_joystick = new XboxController(1);
  //DifferentialDrive tarzan_robot = new DifferentialDrive(driver_leftmotor1, driver_rightmotor1);

  private final DigitalInput conveyor_loc_1 = new DigitalInput(0);

  // Setup the pneumatics devices, 
  Compressor phCompressor = new Compressor(1, PneumaticsModuleType.REVPH);
  DoubleSolenoid IntakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 15); /* Make sure channel number associates with kReverse and Forward Ex: Channel 6 brings down (kReverse) and vice versa with channel 7*/
  DoubleSolenoid RightClimberSolenoid1 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 4, 11); //Kforward is Retract and KReverse is Extend
  DoubleSolenoid LeftClimberSolenoid1 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 14);
  DoubleSolenoid RightClimberSolenoid2 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 2, 13);
  DoubleSolenoid LeftClimberSolenoid2 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 3, 12);

  // private Robot_Cargo_State cargo_status = Robot_Cargo_State.Idle;
  // private Intake_Deployment_State intake_status = Intake_Deployment_State.up;
  // private Climber_State Climber_status = Climber_State.start;
  private final Timer state4_Timer = new Timer();
  private final Timer state2_Timer = new Timer();
  private final Timer deBounce = new Timer();
  private double tx_angle;
  private double ty_angle = -1000.0; //target degrees above the center of the camera
  private double climberArmCommand = 0.0; //variable used to actively control the position of the climber arms; units are in counts at the climber1 motor shaft

  private final double cameraPitch = 18.104; //degrees above horizon ||
  private final double pupilCameraHeight = 32.5; //inches above the ground ||
  private final double goalHeight = 104; //inches above the ground to the top of the goal
  private final double goalRadius = 26.7716535; //inches 
  private final double pupilDistanceToShooter = -6; //inches, in relation to distance from goal ||
  private final double desiredDistanceFromGoal = 150; //inches, distance from the shooter to the center of goal (114.75in - 24in) || //drive team distance from goal
  private final double minimum_climber_limit = -850000; // this is the absolute minimum safe climber arm rotation limit
  private final double maximum_climber_limit = 28500; // this is the absolute maximum safe climber arm rotation limit
  private final double hapticFeedbackPercent = 0.0;
  private boolean button_toggle_1 = false;
  private double shooter_setpoint = 16250;

  private Trigger shootTrig = new Trigger(copilot_joystick::getAButton);
  private Trigger shootStopTrig = new Trigger(copilot_joystick::getYButton);
  private Trigger intakeTrig = new Trigger(copilot_joystick::getBButton);
  private Trigger stopAllTrig = new Trigger(copilot_joystick::getXButton);
  
  @Override
  public void robotInit() {
    System.out.println("==========================");
    System.out.println("FRC 6369 Robot Software");
    System.out.println("Project: Week2_New_Controls");
    System.out.println("Version: v1.0");
    System.out.println("==========================");

    shootTrig.whenActive(new InstantCommand(() -> {shooter_motor1.set(copilot_joystick.getLeftTriggerAxis());}), false);
    shootStopTrig.whenActive(new InstantCommand(() -> {shooter_motor1.set(0);}), false);
    intakeTrig.whenActive(new InstantCommand(() -> {intake_motor1.set(0.7);conveyer1.set(0.7);}), false);
    stopAllTrig.whenActive(new InstantCommand(() -> {intake_motor1.set(0);conveyer1.set(0);}), false);

    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    shooter_motor1.configFactoryDefault();
    shooter_motor2.configFactoryDefault();
    driver_leftmotor1.configFactoryDefault();
    driver_leftmotor2.configFactoryDefault();
    driver_rightmotor1.configFactoryDefault();
    driver_rightmotor2.configFactoryDefault();
    climber_motor1.configFactoryDefault();
    climber_motor2.configFactoryDefault();
    intake_motor1.configFactoryDefault();
    conveyer1.configFactoryDefault();

    driver_leftmotor1.setNeutralMode(NeutralMode.Brake);
    driver_leftmotor2.setNeutralMode(NeutralMode.Brake);
    driver_rightmotor1.setNeutralMode(NeutralMode.Brake);
    driver_rightmotor2.setNeutralMode(NeutralMode.Brake);

    //Mechanism Inversion settings 
    shooter_motor2.setInverted(true);
    driver_rightmotor1.setInverted(true);
    driver_rightmotor2.setInverted(true);
    climber_motor2.setInverted(true);
    conveyer1.setInverted(true);
    intake_motor1.setInverted(true);

    //Shooter, Driver, Climber Follow
    shooter_motor2.follow(shooter_motor1);
    driver_rightmotor2.follow(driver_rightmotor1);
    driver_leftmotor2.follow(driver_leftmotor1);
    climber_motor2.follow(climber_motor1);

    // Drive Base Velocity Control
		driver_rightmotor1.config_kP(0, 0.015, 30);
		driver_rightmotor2.config_kP(0, 0.015, 30);
    driver_leftmotor1.config_kP(0, 0.015, 30);
		driver_leftmotor2.config_kP(0, 0.015, 30);
    driver_rightmotor1.configClosedloopRamp(0.75);
    driver_rightmotor2.configClosedloopRamp(0.75);
    driver_leftmotor1.configClosedloopRamp(0.75);
    driver_leftmotor2.configClosedloopRamp(0.75);
    driver_rightmotor1.configOpenloopRamp(0.75);
    driver_rightmotor2.configOpenloopRamp(0.75);
    driver_leftmotor1.configOpenloopRamp(0.75);
    driver_leftmotor2.configOpenloopRamp(0.75);

    // Shooter Control Loop Settings
    shooter_motor1.configNeutralDeadband(0.001);
		shooter_motor1.config_kP(0, 0.01, 30);
		shooter_motor1.config_kI(0, 0.0, 30);
		shooter_motor1.config_kD(0, 0.0017, 30);
    shooter_motor1.config_kF(0, 0.048141, 30);
    shooter_motor2.configNeutralDeadband(0.001);
		shooter_motor2.config_kP(0, 0.01, 30);
		shooter_motor2.config_kI(0, 0.0, 30);
		shooter_motor2.config_kD(0, 0.0017, 30);
    shooter_motor2.config_kF(0, 0.048141, 30);


    climber_motor1.configNeutralDeadband(0.001);
		climber_motor1.config_kP(0, 0.015, 30);
		climber_motor1.config_kI(0, 0.000, 30);
		climber_motor1.config_kD(0, 0, 30);
    climber_motor2.configNeutralDeadband(0.001);
		climber_motor2.config_kP(0, 0.015, 30);
		climber_motor2.config_kI(0, 0.000, 30);
		climber_motor2.config_kD(0, 0, 30);

    climber_motor1.set(ControlMode.Position, 0.0);

    //Setup compressor controls for analog pressure transducer
    phCompressor.enableAnalog(119, 120);

    //Initializes Solenoids on position 'A'
    IntakeSolenoid.set(Value.kForward);
    RightClimberSolenoid1.set(Value.kForward);
    RightClimberSolenoid2.set(Value.kForward);
    LeftClimberSolenoid1.set(Value.kForward);
    LeftClimberSolenoid2.set(Value.kForward);

    
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("High Side Pressure", phCompressor.getPressure());
    SmartDashboard.putData("Comp", phCompressor);
    // SmartDashboard.putBoolean("Cargo Detected on Conveyor", conveyor_loc_1.get());
    // SmartDashboard.putNumber("Climber Arm Angle relative to robot base", climberEncoder.get());
    // SmartDashboard.putNumber("Distance to Goal", camAngletoDistance(ty_angle));
    // SmartDashboard.putNumber("Climber 1 Encoder (counts)", climber_motor1.getSelectedSensorPosition());
    // SmartDashboard.putBoolean("Robot Idle State", (cargo_status == Robot_Cargo_State.Idle));
    // SmartDashboard.putBoolean("Cargo Being Intaked", (cargo_status == Robot_Cargo_State.Cargo_being_intaked));
    // //SmartDashboard.putNumber("Timer 2", state2_Timer.get());
    // SmartDashboard.putNumber("shooter speed", shooter_motor1.getSelectedSensorVelocity());
    // SmartDashboard.putBoolean("SPEED OK", ((shooter_motor1.getSelectedSensorVelocity() > (shooter_setpoint - 500)) && (shooter_motor1.getSelectedSensorVelocity() < shooter_setpoint + 500)));
    
    // tx_angle = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    // ty_angle = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    CommandScheduler.getInstance().run();
  }
    
  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
  }

  @Override
  public void teleopPeriodic() {
    newDrive2(-1*driver_joystick.getRawAxis(1), -1*driver_joystick.getRawAxis(5));
    /**
    *          ntake function while holding (after 4 seconds of no pressing, it cancels)
    +
    COPILOT JOYSTICK
    * Back (button 7) AND X (button 3) = Prepare for Middle Rung Climb
    * Back (button 7) AND A (button 1) = Perform Middle Rung Climb
    * A button (button 1) = Run AutoAim function while holding
    * Right Bumper (button 6) = Perform Autoshoot function while holding (completes after 1.5 seconds)
    * Start (button 8) = Force robot back to Idle
    *           PILOT JOYSTICK
    * Left Stick Up/Down (raw axis 1) = Move Robot left side
    * Right Stick Up/Down (raw axis 5) = Move Robot right side
    * Back (button 7) = Move Intake Up
    * Start (button 8) = Move Intake Down
    * Left Bumper (button 5) = Perform Auto
    **/
        //distanceHaptic(driver_joystick);
        // if (driver_joystick.getRawButton(6)){
        //   autoAim();
        // }
        // else{
        //    //newDrive(-40000*driver_joystick.getRawAxis(1), -40000*driver_joystick.getRawAxis(5));
        //    newDrive2(-1*driver_joystick.getRawAxis(1), -1*driver_joystick.getRawAxis(5));
        // }
        
        // // expelCargo(copilot_joystick, 2); // b hold
        // flyWheelToggle(copilot_joystick, 6); // right bumper toggle
        // intakeToggle(copilot_joystick, 5); // right bumper toggle
        // // shootCargo(copilot_joystick, 3); // right trigger hold
        // // intakeCargo(copilot_joystick, 2); // left trigger
        // // stopCargo(copilot_joystick, 5); // left bumper

        // // climberManualAdjustment(copilot_joystick);
        // // climberTest2();
        // // if (copilot_joystick.getRawButton(7) && copilot_joystick.getRawButton(3)){
        // //   initiateMiddleRungClimb();
        // //   finalizeMiddleRungClimb();
        // //   part1ClimbTopRung();
        // //   part2ClimbTopRung();
        // //   part3ClimbTopRung();
        // //   part1ClimbTraversal();
        // //   part2ClimbTraversal();
        // //   part3ClimbTraversal();
        // //   part4ClimbTraversal();
        // //   part5ClimbTraversal();        
        // // }
  }

  @Override
  public void disabledInit() {

    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {
    // cargo_status = Robot_Cargo_State.Idle;
    // updateClimberMotorPosition(); // Test it first without this, then add it in
  }

  @Override
  public void testPeriodic() {
    //tarzan_robot.tankDrive(-0.5*driver_joystick.getRawAxis(1), -0.5*driver_joystick.getRawAxis(5));
    // newDrive(-35000*driver_joystick.getRawAxis(1), -35000*driver_joystick.getRawAxis(5));

    // //         COPILOT JOYSTICK
    // // Back Button (raw button 7) = Move Intake from Down to Up.
    // if (copilot_joystick.getRawButton(7)){
    //   moveIntakeDowntoUp();
    // }
    // //         COPILOT JOYSTICK
    // // Start Button (raw button 8) = Move Intake from Up to Down.
    // if (copilot_joystick.getRawButton(8)){
    //   moveIntakeUptoDown();
    // }
    // climberTest2();
    // updateClimberMotorPosition(); // Test it first without this, then add it in
  }

  void newDrive2(double leftControl, double rightControl) {
    // if (Math.abs(leftControl) < 0.07) {
    //   leftControl = 0.0;
    // }
    // if (Math.abs(rightControl) < 0.07) {
    //   rightControl = 0.0;
    // }    
    driver_leftmotor1.set(leftControl);
    driver_rightmotor1.set(rightControl); 
  }
}
