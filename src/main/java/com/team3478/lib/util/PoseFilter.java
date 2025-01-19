package com.team3478.lib.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.ArrayList;
import java.util.List;

public class PoseFilter {
  private List<Pose3d> poseList = null;

  // Add new value to the list
  public void add(Pose3d newPose) {
    if (poseList == null) {
      poseList = new ArrayList<>();
    }
    // de momento esta en uno para no filtrar(ya que dejamos los fps bajos)
    if (poseList.size() >= 1) {
      poseList.remove(0);
    }
    poseList.add(newPose);
  }

  // Function to reset the list
  public void clear() {
    poseList = null;
  }

  // Function to the list size
  public int getSize() {
    if (poseList == null) {
      return 0;
    }
    return poseList.size();
  }

  // Function to get the average value
  public Pose3d getValue() {
    double xPos = 0;
    double yPos = 0;
    double zPos = 0;
    double xAngle = 0;
    double yAngle = 0;
    double zAngle = 0;
    if (poseList == null) {
      return new Pose3d();
    }
    int counter = 0;
    for (Pose3d pose3d : poseList) {
      xPos += pose3d.getTranslation().getX();
      yPos += pose3d.getTranslation().getY();
      zPos += pose3d.getTranslation().getZ();
      xAngle += pose3d.getRotation().getX();
      yAngle += pose3d.getRotation().getY();
      zAngle += pose3d.getRotation().getZ();
      counter += 1;
    }
    return new Pose3d(
        new Translation3d(xPos / counter, yPos / counter, zPos / counter),
        new Rotation3d(xAngle / counter, yAngle / counter, zAngle / counter));
  }
}
