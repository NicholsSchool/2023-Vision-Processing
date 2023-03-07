package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/*
* Localizer localizes our robots position and angle on the field relative to ( 0, 0 ) bottom left
* for position and ( 0, 0 ) for angle relative to center of field with the red alliance station being 
* 0 degrees and the blue alliance station being 180 degrees. Position localization is done in meters
* and angle localization done in radians
*/
public class Localizer{
    
    PhotonCamera camera = new PhotonCamera( "Microsoft_LifeCam_HD-3000" );

    
    double[] location = new double[]{ 0.0, 0.0 }; 
    double fieldAngle = 0; 
    long timeDetected = 0; 
    int id; 
    double x, y, rot;

    PhotonPipelineResult result = camera.getLatestResult(); 


    final double[][] APRIL_TAG_LOCATIONS = new double[][]{ { 15.51, 1.07 }, { 15.51, 2.75 }, { 15.51, 4.42 },
    { 16.18, 6.75 }, { .36, 6.75 }, { 1.03, 4.42 }, { 1.03, 2.75 }, { 1.03, 1.07 } };
    
    public void runDetection()
    {
        PhotonPipelineResult result = camera.getLatestResult(); 

        if( result.hasTargets() )
        {
            PhotonTrackedTarget target = result.getBestTarget();

            id = target.getFiducialId();
            
            x = target.getBestCameraToTarget().getX();
            y = target.getBestCameraToTarget().getY();
            rot = target.getBestCameraToTarget().getRotation().toRotation2d().getDegrees();
        }
           
    }
    
    /**
     * Returns a confidence from 0 --> 1 based on how accurate we believe our position and angle are
     * @return
     */
    public double getConfidence( double amb, double x, double y, double t )
    {
        double ambiguity = 1 - amb; 
        double distance = ( 1 - ( x / 16.54 ) + ( 1 - ( y / 8.02 ) ) ) / 2;
        double time = Math.pow( .8, t );

        return ambiguity * distance * time; 
    }


    /**
     * Returns the System's time when an Apriltag was detected
     * 
     * @return the System time when an Apriltag was dicovered
     */
    public long getTimeAtDetection()
    {
        return timeDetected; 
    }

    /**
     * Localize your position on the field relative to (0, 0) located in the bottom left of the field
     */
    public double[] localizePos()
    {
        runDetection();

        double localX;
        double localY;
        
        if( id == -1 )
        {
            return location; 
        }
        else if( id <= 4 )
        {
             localX = APRIL_TAG_LOCATIONS[ id - 1 ][ 0 ] - x;  
             localY = APRIL_TAG_LOCATIONS[ id - 1 ][ 1 ] + y; 
        }
        else
        {
             localX = APRIL_TAG_LOCATIONS[ id - 1 ][ 0 ] + x;  
             localY = APRIL_TAG_LOCATIONS[ id - 1 ][ 1 ] + y; 
        }

        location[ 0 ] = localX;
        location[ 1 ] = localY; 

        timeDetected = System.currentTimeMillis(); 

        return location;
    }

    /*
    * Finds the angle of the robot's heading relative to the field with 0 degrees
    * at the red alliance station and 180 degrees at the blue alliance station. This
    * angle is measured in radians.  
    */
    public double localizeAngle()
    {
        runDetection();
    
        rot = ( -rot + 360 ) % 360; 

        if( id == -1 )
        {
            return rot;
        }
        else if( id <= 4 )
        {
            fieldAngle = ( rot + 180 ) % 360;
        }
        else
        {
            fieldAngle = rot; 
        }
     
       return Math.toRadians( fieldAngle ); 
    }
    /*
     * Gets X location relative to bottom left of robot on field in meters
     */
    public double getXPos()
    {
        return location[ 0 ]; 
    }

    
    /*
     * Gets Y location relative to bottom left of robot on field in meters
     */
    public double getYPos()
    {
        return location[ 1 ]; 
    }

    /*
    * Returns the angle of the robot's heading with 0 degrees at the red alliance station
    * and 180 degrees at the blue alliance station. This angle is measured in radians. 
    */
    public double getAngle()
    {
        return fieldAngle; 
    }
}