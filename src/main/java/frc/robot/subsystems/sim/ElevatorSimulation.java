// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.sim;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.GlobalConstants;
import frc.robot.Constants.SimConstants;

/** Add your docs here. */
public class ElevatorSimulation {
  
  private static final double kCarriageMass = 4.0; // kg
  
  private final DCMotor m_elevatorGearbox = DCMotor.getNEO(1);
  
  private CANSparkMax motorA, motorB;
  private RelativeEncoder encoderA, encoderB;
  
  private final Field2d m_elevatorTrajectorySim;
  Trajectory t = null;
  
  //ElevatorSimulations help us simulate what's going on, including gravity.
    private final ElevatorSim m_elevatorXSim =
    new ElevatorSim(
      m_elevatorGearbox,
      ElevatorConstants.kElevatorGearing,
      kCarriageMass,
      ElevatorConstants.kElevatorDrumRadius,
      ElevatorConstants.kMinElevatorReach,
      ElevatorConstants.kMaxElevatorReach,
      false,
      VecBuilder.fill(0.0001));
    private final ElevatorSim m_elevatorZSim =
    new ElevatorSim(
      m_elevatorGearbox,
      ElevatorConstants.kElevatorGearing,
      kCarriageMass,
      ElevatorConstants.kElevatorDrumRadius,
      ElevatorConstants.kMinElevatorHeight,
      ElevatorConstants.kMaxElevatorHeight,
      false,
      VecBuilder.fill(0.0001));//TODO Make gravety true

    public ElevatorSimulation(CANSparkMax motorA, CANSparkMax motorB){
      this.motorA = motorA;
      this.motorB = motorB;
      this.encoderA = motorA.getEncoder();
      this.encoderB = motorB.getEncoder();
      REVPhysicsSim.getInstance().addSparkMax(motorA, m_elevatorGearbox);
      REVPhysicsSim.getInstance().addSparkMax(motorB, m_elevatorGearbox);
      
      m_elevatorTrajectorySim = new Field2d(); //Screen Width 3.885m and Screen Height 2.000m

      // Publish Mechanism2d to SmartDashboard
      // To view the Elevator Sim in the simulator, select Network Tables -> SmartDashboard ->
      // Elevator Sim
      SmartDashboard.putData("Elevator Sim", m_mech2d);
      SmartDashboard.putData("Elevator Trajectory Sim", m_elevatorTrajectorySim);
    }

    // Create a Mechanism2d visualization of the elevator
    private final Double screenWidth = SimConstants.kChargeStationWidth*0.5 + SimConstants.kCommunitWidth + SimConstants.kGridDepth; //m
    private final double screenHeight = 2.00; //m
    private final double m_endEffectorOffsetX = ElevatorConstants.kElevatorSetbackFromOrigin - (0.5 * ElevatorConstants.kEndEffectorLength); //m
    private final double m_endEffectorOffsetZ = 0.10; //m
    private final double m_elevatorOriginX = (screenWidth - SimConstants.kGridDepth - ElevatorConstants.kRobotBumperThickness);
    private final double m_elevatorPosition = m_elevatorOriginX - ElevatorConstants.kElevatorSetbackFromOrigin; 

    private final Mechanism2d m_mech2d = new Mechanism2d(screenWidth, screenHeight);
    private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Elevator Root", m_elevatorPosition, 0);
    

    private final MechanismLigament2d m_elevatorLiftMech2d =
      m_mech2dRoot.append( new MechanismLigament2d("ElevatorLift", 
      1, 90,5,new Color8Bit(Color.kRed)));
    
    private final MechanismLigament2d m_elevatorArmMech2d =
      m_elevatorLiftMech2d.append( new MechanismLigament2d("ElevatorArm",
      1, -90,5,  new Color8Bit(Color.kPurple)));

    private final MechanismLigament2d m_elevatorArm2Mech2d =
      m_elevatorArmMech2d.append( new MechanismLigament2d("ElevatorArm2",
      0.10, -90,5,  new Color8Bit(Color.kPurple)));

    private final MechanismLigament2d m_endEffectorMech2d =
      m_elevatorArm2Mech2d.append( new MechanismLigament2d("End Effector",
      ElevatorConstants.kEndEffectorLength, 90,5,  new Color8Bit(Color.kPurple)));

    public void update(Double m_setposX, Double m_setposZ){
        REVPhysicsSim.getInstance().run();

        // In this method, we update our simulation of what our elevator is doing
        // First, we set our "inputs" (voltages)
        m_elevatorXSim.setInput(0.5*(motorA.getAppliedOutput() + motorB.getAppliedOutput()));
        m_elevatorZSim.setInput(0.5*(motorA.getAppliedOutput() - motorB.getAppliedOutput()));
      
        // Next, we update it. The standard loop time is 20ms.
        m_elevatorXSim.update(GlobalConstants.kLoopTime);
        m_elevatorZSim.update(GlobalConstants.kLoopTime);
    
        // Finally, we set our simulated encoder's readings and simulated battery voltage
        encoderA.setPosition(m_elevatorXSim.getPositionMeters() + m_elevatorZSim.getPositionMeters());
        encoderB.setPosition(m_elevatorXSim.getPositionMeters() - m_elevatorZSim.getPositionMeters());

        // Update elevator visualization with simulated position
        m_elevatorLiftMech2d.setLength(m_elevatorZSim.getPositionMeters() + m_endEffectorOffsetZ);
        m_elevatorArmMech2d.setLength(m_elevatorXSim.getPositionMeters() + m_endEffectorOffsetX);

        Pose2d elevatorPose = new Pose2d(
          new Translation2d(m_elevatorOriginX + m_setposX, m_setposZ), new Rotation2d());

        m_elevatorTrajectorySim.setRobotPose(elevatorPose);
        if(t != null) m_elevatorTrajectorySim.getRobotObject().setTrajectory(t);
    }

    public void drawTrajectory(Trajectory t){
      this.t = t;
    }
}
