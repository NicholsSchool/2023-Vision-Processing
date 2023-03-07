// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.lang.invoke.VarHandle.VarHandleDesc;
import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {

  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  
  NetworkTable table = inst.getTable("test");

  NetworkTable pieces = inst.getTable("Vision");

  private final PWMSparkMax m_leftDrive = new PWMSparkMax(0);
  private final PWMSparkMax m_rightDrive = new PWMSparkMax(1);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftDrive, m_rightDrive);
  private final XboxController m_controller = new XboxController(0);
  private final Timer m_timer = new Timer();
  ArrayList<Double> angle = new ArrayList<Double>(); 

  Localizer location = new Localizer();
  File file = new File( "angle.txt" );

  PhotonCamera camera = new PhotonCamera( "Microsoft_LifeCam_HD-3000" );

  //StringSubscriber name = pieces.getStringTopic("piece").subscribe("Nothing");
  IntegerSubscriber ymin = pieces.getIntegerTopic("yMin").subscribe(0);
  IntegerSubscriber xmin = pieces.getIntegerTopic("xMin").subscribe(0);
  IntegerSubscriber ymax = pieces.getIntegerTopic("yMax").subscribe(0);
  IntegerSubscriber xmax = pieces.getIntegerTopic("xMax").subscribe(0);


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightDrive.setInverted(true);
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_timer.reset();
    m_timer.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // Drive for 2 seconds
    if (m_timer.get() < 2.0) {
      // Drive forwards half speed, make sure to turn input squaring off
      m_robotDrive.arcadeDrive(0.5, 0.0, false);
    } else {
      m_robotDrive.stopMotor(); // stop robot
    }
  }

  long lt;

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() 
  {

    lt = System.currentTimeMillis();

  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() 
  {

    //String name = table.getEntry("Test").getString("Fail")

    //String piece = name.get();
    long yMin = ymin.get(); 
    long xMin = xmin.get(); 
    long yMax = ymax.get(); 
    long xMax = xmax.get(); 
    

    long timeNow = System.currentTimeMillis();
    m_robotDrive.arcadeDrive(-m_controller.getLeftY(), -m_controller.getRightX());

    var result = camera.getLatestResult(); 
if( result.hasTargets() )
    {
      var target = result.getBestTarget();

      var id = target.getFiducialId();
      var yaw = Math.toDegrees( target.getYaw() );
      var pitch = Math.toDegrees( target.getPitch() );
      var x = target.getBestCameraToTarget().getX();
      var y = target.getBestCameraToTarget().getY();
      var rot = target.getBestCameraToTarget().getRotation().toRotation2d().getDegrees(); 

      var theta = target
    .getBestCameraToTarget().getRotation().getAngle(); 
      var ambiguity = target.getPoseAmbiguity(); 
      double time = timeNow - location.getTimeAtDetection() / 1000;
      double certainty = location.getConfidence( ambiguity, x, y, time ); 
      //var range = PhotonUtils.calculateDistanceToTargetMeters( .51, .51, 0, pitch );

      //PhotonUtils.estimateFieldToCamera(null, null)
      

      if( System.currentTimeMillis() - lt > 3000 ) {

      System.out.println( "Yaw: " + yaw );
      System.out.println( "pitch: " + pitch );
      System.out.println( "ID: " + id );
      System.out.println( "X: " + x );
      System.out.println( "y: " + y );

      System.out.println( "rot: " + rot ); 

      System.out.println( "theta: " + Math.toDegrees( Math.atan2( y, x ) ) ); 

      // System.out.println( "localX: " + location.localizePos( id , x, y )[0] );
      // System.out.println( "localY: " + location.localizePos( id , x, y )[1] ); 
      // System.out.println( "localAngle: "  + Math.toDegrees( location.localizeAngle( id, rot ) ) );
      System.out.println( "ambiguity: " + ambiguity ); 
      System.out.println( "certainty: " + certainty ); 

      //System.out.println("piece:1 " + piece);
      System.out.println("ymin:2 " + yMin);
      System.out.println("xmin:3 " + xMin);
      System.out.println("ymax:4 " + yMax);
      System.out.println("xmax:5 " + xMax);

      

      lt = System.currentTimeMillis();

      }

      //System.out.println(name);

    }

  }


  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
