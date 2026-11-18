/**
 ******************************************************************************
 * @file    AxisAlignment.hpp
 * @brief   Rotates sensor-axis vectors into the ParaGuardian BODY frame.
 *
 * BODY FRAME (FRD, right-handed):
 *     +X  forward (direction the pilot faces)
 *     +Y  right
 *     +Z  down
 *   This matches the aerospace NED navigation convention, so a future
 *   attitude filter's rotation R_nb (body -> NED) is the standard one. At
 *   rest and level, the accelerometer (specific force) reads (0, 0, -g) in FRD.
 *
 * NAVIGATION FRAME (ENU) - for later consumers
 *   The project's navigation outputs are East-North-Up. A body vector is
 *   taken to ENU in two explicit steps; the frames are never mixed silently:
 *       v_NED = R_nb * v_body            (R_nb from the future attitude filter)
 *       v_ENU = C_enu_ned * v_NED,  C_enu_ned = [[0,1,0],[1,0,0],[0,0,-1]]
 *   (east = NED y, north = NED x, up = -NED z). C_enu_ned is its own inverse
 *   and has determinant +1: swapping two axes (-1) combined with negating the
 *   third (-1) is a proper rotation (180 deg about the bisector of N and E),
 *   as it must be between two right-handed frames.
 *
 * MOUNTING
 *   v_body = R_body_from_sensor * v_sensor  for each sensor (3 IMUs, the
 *   magnetometer). The SAME R is applied to the accelerometer and the gyro of
 *   one IMU (they share one silicon die and one axis triad).
 *   The PCB/enclosure orientation is NOT KNOWN: every mounting defaults to the
 *   identity matrix with verified = false, and every processed frame carries
 *   that "unverified" flag until real values are entered from the mechanical
 *   drawing and checked on hardware (gravity / rotation tests).
 *
 * VALIDITY
 *   A mounting matrix must be a proper rotation: R * R^T = I (orthonormal,
 *   preserves lengths and angles) and det(R) = +1 (no mirror). A single axis
 *   sign flip has det = -1: it would turn the right-handed chip frame into a
 *   left-handed one, which no physical mounting can do, so it is rejected.
 *   Flipping TWO axes (= 180 deg rotation, e.g. chip on the underside of the
 *   board) is a proper rotation and accepted.
 *
 * Rotation builders: mat::rotationX/Y/Z (VectorMath.hpp) are single-axis
 * active rotations; compose them explicitly (R = Rz * Ry * Rx is "rotate
 * about X first, then Y, then Z, all about the fixed body axes"). No Euler
 * angle API is offered, to avoid order ambiguity.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_PROCESSING_AXIS_ALIGNMENT_HPP
#define PARAGUARDIAN_PROCESSING_AXIS_ALIGNMENT_HPP

#include <cstdint>

#include "Processing/ProcessingTypes.hpp"
#include "Processing/VectorMath.hpp"

namespace pg {

/// NED -> ENU relabelling matrix (see file header).
constexpr Mat3f kEnuFromNed{{{0.0f, 1.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, -1.0f}}};

struct SensorMounting {
    Mat3f bodyFromSensor;   ///< v_body = bodyFromSensor * v_sensor
    bool verified;          ///< true only once checked against the real hardware
};

/// One axis of an axis map: body axis k = sign * sensor axis `axis`.
struct AxisMapEntry {
    uint8_t axis;   ///< 0 = sensor X, 1 = sensor Y, 2 = sensor Z
    int8_t sign;    ///< +1 or -1
};

class AxisAlignment {
public:
    /// Tolerances for "is a rotation": |R R^T - I| entries and |det - 1|.
    static constexpr float kOrthoTolerance = 1e-4f;
    static constexpr float kDetTolerance = 1e-4f;

    AxisAlignment();

    /// True if @p r is a proper rotation within tolerance.
    static bool isProperRotation(const Mat3f& r);

    /**
     * Builds a matrix from an axis map (body X/Y/Z = signed sensor axes), the
     * usual way a datasheet "pin 1" orientation is translated. Returns false
     * if the map is not a proper rotation (repeated axis, bad sign, mirror).
     */
    static bool fromAxisMap(const AxisMapEntry (&map)[3], Mat3f& out);

    /**
     * Installs a mounting for @p id. Rejects (returns false, keeps the
     * previous mounting) matrices that are not proper rotations or sensors
     * that have no vector output (only IMUs and the magnetometer).
     */
    bool setMounting(SensorId id, const Mat3f& bodyFromSensor, bool verified);

    const SensorMounting& mounting(SensorId id) const;
    bool allVerified() const;

    /// v_body = R * v_sensor. Unsupported ids return the input unchanged.
    Vec3f toBody(SensorId id, const Vec3f& v_sensor) const;

    uint32_t rejectedConfigurations() const { return rejected_; }

private:
    static bool supports(SensorId id);
    SensorMounting mountings_[kSensorCount];
    uint32_t rejected_ = 0u;
};

/// NED vector -> ENU vector.
inline Vec3f nedToEnu(const Vec3f& ned) { return mat::mul(kEnuFromNed, ned); }

}  // namespace pg

#endif  // PARAGUARDIAN_PROCESSING_AXIS_ALIGNMENT_HPP
