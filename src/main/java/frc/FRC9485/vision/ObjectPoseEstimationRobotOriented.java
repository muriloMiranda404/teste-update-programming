package frc.FRC9485.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.FRC9485.utils.CoordinatesTransform;
import frc.FRC9485.utils.logger.CustomDoubleLog;

public class ObjectPoseEstimationRobotOriented {
  ObjectDetection object;

  ObjectDetectionCamera camera;

  CustomDoubleLog cameraAngleLogger;

  CustomDoubleLog tyLogger;

  CustomDoubleLog txLogger;

  CustomDoubleLog txDegreesLogger;

  CustomDoubleLog tyDegreesLogger;

  CustomDoubleLog distanceToObjectLogger;

  CustomDoubleLog xObjectSDistanceLogger;

  CustomDoubleLog yObjectDistanceLogger;


  double angleObjectInMaxTy;

  double angleObjectInMaxTx;

  public ObjectPoseEstimationRobotOriented(ObjectDetection object, ObjectDetectionCamera camera,
      double angleObjectInMaxTy, double angleObjectInMaxTx) {
    this.object = object;
    this.camera = camera;
    this.angleObjectInMaxTy = angleObjectInMaxTy;
    this.angleObjectInMaxTx = angleObjectInMaxTx;
    setupLogs();
  }

  private void setupLogs() {
    cameraAngleLogger = new CustomDoubleLog("ObjectDetection/Camera Angle");
    tyLogger = new CustomDoubleLog("ObjectDetection/Raw tY Value");
    txLogger = new CustomDoubleLog("ObjectDetection/Raw tX Value");
    tyDegreesLogger = new CustomDoubleLog("ObjectDetection/tY In Degrees");
    txDegreesLogger = new CustomDoubleLog("ObjectDetection/tX In Degrees");
    distanceToObjectLogger = new CustomDoubleLog("ObjectDetection/Camera Distance To Object");
    xObjectSDistanceLogger = new CustomDoubleLog("ObjectDetection/X Object Distance To Robot");
    yObjectDistanceLogger = new CustomDoubleLog("ObjectDetection/Y Object Distance To Robot");
  }

  public Pose2d getObjectPoseInField(Pose2d robotPose, Pose2d objectPose) {
    Pose2d poseInTheField2d = CoordinatesTransform.fromRobotRelativeToF(robotPose, objectPose);
    Pose3d poseInTheField3d = new Pose3d(poseInTheField2d);
    return poseInTheField2d;
  }

  public Pose2d getObjectPoseFromRobotCenter() {
    return new Pose2d(getXnoteDistanceFromRobotCenter(), -getYnoteDistanceFromRobotCenter(), new Rotation2d());
  }

  public double getXnoteDistanceFromRobotCenter() {
    double calculation = (getDistanceFromObject() * Math.cos(
        Units.radiansToDegrees(camera.robotToCamera.getRotation().getZ()) + Units.degreesToRadians(getObjectAngleX())))
        + camera.robotToCamera.getX();
    xObjectSDistanceLogger.append(calculation);
    return calculation;
  }

  public double getYnoteDistanceFromRobotCenter() {
    double calculation = (getDistanceFromObject() * Math.sin(
        Units.radiansToDegrees(camera.robotToCamera.getRotation().getZ()) + Units.degreesToRadians(getObjectAngleX())))
        + camera.robotToCamera.getY();
    yObjectDistanceLogger.append(calculation);
    return calculation;
  }

  public double getDistanceFromObject() {
    cameraAngleLogger.append(camera.getCameraAngle());
    double calculation = camera.cameraHeight
        * Math.tan(Units.degreesToRadians(getObjectAngleY()) + Units.degreesToRadians(camera.getCameraAngle()));
    distanceToObjectLogger.append(calculation);
    return calculation;
  }

  private double getObjectAngleY() {
    tyLogger.append(object.ty);
    double calculation = (object.ty * angleObjectInMaxTy) / camera.maxTy;
    tyDegreesLogger.append(calculation);
    return calculation;
  }

  private double getObjectAngleX() {
    txLogger.append(object.tx);
    double calculation = (object.tx * angleObjectInMaxTx) / camera.maxTx;
    txDegreesLogger.append(calculation);
    return calculation;
  }

}