/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.library.Util;

/**
 * Add your docs here.
 */
public class Util {
    public static int pos_x1;
    public static int pos_x2;
    public static int pos_newX;
    public static int pos_y1;
    public static int pos_y2;
    public static int pos_newY;
    public static double deltaOverall_x;
    public static double deltaOverall_y;
    public static int pos_y;
    public static double distanceToObjective;
    public static double delta1_x;
    public static double delta1_y;
    public static double delta2_x;
    public static double delta2_y;
    public static double vel_x;
    public static double vel_y;
    public static double acc_x;
    public static double acc_y;
    public static double angleRobotToField;
    public static double angleToObjective;
    public static double angleCorrection;

  public static double delta1_x (double pos_x1, double pos_x2) {
    delta1_x =  (pos_x2 - pos_x1); 
    return delta1_x;
  }
  public static double delta1_y (double pos_y1, double pos_y2){
    delta1_y = (pos_y2 - pos_y1);
    return delta1_y;
  }
  public static double delta2_x (double pos_newX, double pos_x1) {
    delta2_x = (pos_newX - pos_x1);  
    return delta2_x;
  }
  public static double delta2_y (double pos_newY, double pos_y1) {
    delta2_y =  (pos_newY - pos_y1); 
    return delta2_y;
  }
  public static double deltaOverall_x(){
    deltaOverall_x = (delta1_x - delta2_x);  
    return deltaOverall_x;
  }
  public static double deltaOverall_y(){
    deltaOverall_y = (delta1_y - delta2_y); 
    return deltaOverall_y;
  }
  public static double distanceToObjective() {
    distanceToObjective = Math.sqrt(((Math.pow((delta1_x - delta2_x), 2.0)) - (Math.pow(((delta1_x - delta2_x)), 2.0))));
    return distanceToObjective;
  }
  public static double angleRobotToField() {
    angleRobotToField = Math.atan2(delta1_y, delta1_x);
    return angleRobotToField;
  }
  public static double angleToObjective() {
    angleToObjective = Math.atan2(delta2_y, delta2_x);
    return angleToObjective;
  }
  public static double angleCorrection () {
    angleCorrection = (angleRobotToField - angleToObjective);  
    return angleCorrection;
  }
         

/**
 * Below is the raw formulas of the different constants used for the MotionProfileGenerator
 * 
 *  public void MotionFormulas() {
    delta1_x = (pos_x2 - pos_x1);
    delta1_y = (pos_y2 - pos_y1);
    delta2_x = (pos_newX - pos_x1);
    delta2_y = (pos_newY - pos_y1);
    deltaOverall_x = (delta1_x - delta2_x);
    deltaOverall_y = (delta1_y - delta2_y);
    distanceToObjective = (int) Math.sqrt(((Math.pow((delta1_x - delta2_x), 2.0)) - (Math.pow(((delta1_x - delta2_x)), 2.0))));
    angleRobotToField = (int) Math.atan2(delta1_y, delta1_x);
    angleToObjective =(int) Math.atan2(delta2_y, delta2_x);
    angleCorrection = angleRobotToField - angleToObjective;
  }

  */
}