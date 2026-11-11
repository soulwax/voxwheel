import 'dart:math' as math;

/// Lightweight immutable 3D vector for [voxwheel].
final class WheelVec3 {
  final double x;
  final double y;
  final double z;

  const WheelVec3(this.x, this.y, this.z);

  static const WheelVec3 zero = WheelVec3(0.0, 0.0, 0.0);
  static const WheelVec3 unitX = WheelVec3(1.0, 0.0, 0.0);
  static const WheelVec3 unitY = WheelVec3(0.0, 1.0, 0.0);
  static const WheelVec3 unitZ = WheelVec3(0.0, 0.0, 1.0);

  WheelVec3 operator +(WheelVec3 o) => WheelVec3(x + o.x, y + o.y, z + o.z);
  WheelVec3 operator -(WheelVec3 o) => WheelVec3(x - o.x, y - o.y, z - o.z);
  WheelVec3 operator -() => WheelVec3(-x, -y, -z);
  WheelVec3 operator *(double s) => WheelVec3(x * s, y * s, z * s);
  WheelVec3 operator /(double s) => WheelVec3(x / s, y / s, z / s);

  double dot(WheelVec3 o) => x * o.x + y * o.y + z * o.z;

  WheelVec3 cross(WheelVec3 o) => WheelVec3(
        y * o.z - z * o.y,
        z * o.x - x * o.z,
        x * o.y - y * o.x,
      );

  double get lengthSquared => x * x + y * y + z * z;
  double get length => math.sqrt(lengthSquared);

  WheelVec3 get normalized {
    final lenSq = lengthSquared;
    if (lenSq <= 1e-16) return zero;
    final inv = 1.0 / math.sqrt(lenSq);
    return WheelVec3(x * inv, y * inv, z * inv);
  }

  WheelVec3 projectOntoPlane(WheelVec3 unitNormal) =>
      this - unitNormal * dot(unitNormal);

  static WheelVec3 lerp(WheelVec3 a, WheelVec3 b, double t) =>
      WheelVec3(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t);

  @override
  String toString() => 'WheelVec3($x, $y, $z)';
}

/// Unit quaternion used to orient vehicle bodies and steering frames.
final class WheelQuat {
  final double x;
  final double y;
  final double z;
  final double w;

  const WheelQuat(this.x, this.y, this.z, this.w);

  static const WheelQuat identity = WheelQuat(0.0, 0.0, 0.0, 1.0);

  factory WheelQuat.fromAxisAngle(WheelVec3 axis, double angleRad) {
    final norm = axis.normalized;
    if (norm.lengthSquared <= 1e-12) return identity;
    final half = angleRad * 0.5;
    final s = math.sin(half);
    return WheelQuat(norm.x * s, norm.y * s, norm.z * s, math.cos(half));
  }

  WheelQuat get normalized {
    final lenSq = x * x + y * y + z * z + w * w;
    if (lenSq <= 1e-16) return identity;
    final inv = 1.0 / math.sqrt(lenSq);
    return WheelQuat(x * inv, y * inv, z * inv, w * inv);
  }

  WheelQuat get conjugate => WheelQuat(-x, -y, -z, w);

  WheelQuat operator *(WheelQuat o) => WheelQuat(
        w * o.x + x * o.w + y * o.z - z * o.y,
        w * o.y - x * o.z + y * o.w + z * o.x,
        w * o.z + x * o.y - y * o.x + z * o.w,
        w * o.w - x * o.x - y * o.y - z * o.z,
      );

  WheelVec3 rotate(WheelVec3 v) {
    final qv = WheelVec3(x, y, z);
    final t = qv.cross(v) * 2.0;
    return v + t * w + qv.cross(t);
  }

  /// Integrates this orientation by world-space angular velocity vector over [dt].
  WheelQuat integrateAngularVelocity(WheelVec3 omegaWorld, double dt) {
    final speed = omegaWorld.length;
    if (speed <= 1e-12 || dt <= 0.0) return this;
    final delta = WheelQuat.fromAxisAngle(omegaWorld / speed, speed * dt);
    return (delta * this).normalized;
  }

  @override
  String toString() => 'WheelQuat($x, $y, $z, $w)';
}
