//
// Created by victor on 9/22/20.
//

#ifndef NBVEPLANNER_CAMERA_MODEL_H_
#define NBVEPLANNER_CAMERA_MODEL_H_

#include <nbveplanner/common.h>

namespace nbveplanner {

class Plane {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Plane() : normal_(Point::Identity()), distance_(0) {}
  virtual ~Plane() = default;

  void setFromPoints(const Point& p1, const Point& p2, const Point& p3);
  void setFromDistanceNormal(const Point& normal, double distance);

  bool isPointCorrectSide(const Point& point) const;

  Point normal() const { return normal_; }
  double distance() const { return distance_; }

 private:
  Point normal_;
  double distance_;
};

class CameraModel {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CameraModel() : initialized_(false), horizontal_limit_(true) {}
  virtual ~CameraModel() = default;

  /// Set up the camera model, intrinsics and extrinsics.
  void setIntrinsicsFromFoV(double horizontal_fov_deg, double vertical_fov_deg,
                            double min_distance, double max_distance);
  /**
   * Get and set the current poses for the camera (should be called after
   * the camera is properly set up).
   */
  Transformation getCameraPose() const;
  /// Set camera pose actually computes the new bounding plane positions.
  void setCameraPose(const Transformation& cam_pose);

  void setBodyPose(const Transformation& body_pose);

  void setExtrinsics(const Transformation& T_C_B);

  /// Check whether a point belongs in the current view.
  bool isPointInView(const Point& point) const;

  bool isPointInSector(size_t sector_idx, const Point& point) const;

  void getAabb(Point* aabb_min, Point* aabb_max) const;

  /**
   * Accessor functions for visualization (or other reasons).
   * Bounding planes are returned in the global coordinate frame.
   */
  const AlignedVector<Plane>& getBoundingPlanes() const {
    return bounding_planes_;
  }

  void getBoundingLines(AlignedVector<Point>* lines) const;

  /**
   * Get the 3 points definining the plane at the back (far end) of the camera
   * frustum. Expressed in global coordinates.
   */
  void getFarPlanePoints(size_t sector_idx, AlignedVector<Point>* points) const;

  void setBoundingBox(Point& bbx_min, Point& bbx_max);

  bool hasHorizontalLimit() { return horizontal_limit_; }

 private:
  void calculateBoundingPlanes();

  bool initialized_;
  bool horizontal_limit_;

  /// Current pose of the camera.
  Transformation T_G_C_;
  /// Extrinsic calibration to body.
  Transformation T_C_B_;

  /**
   * The original vertices of the frustum, in the axis-aligned coordinate frame
   * (before rotation).
   */
  AlignedVector<Point> corners_C_;

  /**
   * The 6 bounding planes for the current camera pose, and their corresponding
   * AABB (Axis Aligned Bounding Box). Expressed in global coordinate frame.
   */
  AlignedVector<Plane> bounding_planes_;
  Point aabb_min_;
  Point aabb_max_;

  Point bbx_min_;
  Point bbx_max_;
};
}  // namespace nbveplanner

#endif  // NBVEPLANNER_CAMERA_MODEL_H_
