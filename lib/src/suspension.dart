import 'dart:math' as math;

import 'math.dart';

/// Identifies which axle a wheel belongs to for steering and drive torque distribution.
enum WheelAxle {
  front,
  rear,
  other,
}

/// Static configuration for one suspension strut and wheel assembly.
final class WheelConfig {
  final String id;
  final WheelVec3 localAnchor;
  final WheelAxle axle;

  /// Left wheels use `-1.0`, right wheels `+1.0`, center `0.0`.
  final double sideSign;
  final double restLength;
  final double maxTravel;
  final double wheelRadius;
  final double springStiffness;
  final double damperCompression;
  final double damperRebound;
  final double wheelInertia;
  final bool steerable;
  final bool braked;
  final bool handbraked;
  final double peakGripMu;

  const WheelConfig({
    required this.id,
    required this.localAnchor,
    required this.axle,
    this.sideSign = 0.0,
    this.restLength = 0.45,
    this.maxTravel = 0.22,
    this.wheelRadius = 0.34,
    this.springStiffness = 32000.0,
    this.damperCompression = 3600.0,
    this.damperRebound = 4200.0,
    this.wheelInertia = 1.35,
    this.steerable = false,
    this.braked = true,
    this.handbraked = false,
    this.peakGripMu = 1.15,
  });

  double get minLength => math.max(0.02, restLength - maxTravel);
}

/// Links two opposite-side wheels with an anti-roll bar stiffness (`N/m` of differential compression).
final class AntiRollPair {
  final int leftWheelIndex;
  final int rightWheelIndex;
  final double stiffness;

  const AntiRollPair({
    required this.leftWheelIndex,
    required this.rightWheelIndex,
    this.stiffness = 8500.0,
  });
}

/// Per-wheel runtime telemetry updated every simulation step.
final class WheelSuspensionState {
  final String id;
  bool isGrounded = false;
  double suspensionLength = 0.45;
  double compression = 0.0;
  double compressionVelocity = 0.0;
  double compressionRatio = 0.0;
  double normalForce = 0.0;
  double antiRollForce = 0.0;
  double steerAngleRad = 0.0;
  double spinAngleRad = 0.0;
  double angularVelocityRadPerSec = 0.0;
  double slipRatio = 0.0;
  double slipAngleRad = 0.0;
  double longitudinalForce = 0.0;
  double lateralForce = 0.0;
  WheelVec3 worldAnchor = WheelVec3.zero;
  WheelVec3 worldContactPoint = WheelVec3.zero;
  WheelVec3 worldContactNormal = WheelVec3.unitY;

  WheelSuspensionState({
    required this.id,
    double initialRestLength = 0.45,
  }) : suspensionLength = initialRestLength;
}

/// Spring-damper strut force evaluation utilities.
abstract final class WheelSuspension {
  /// Evaluates unilateral upward spring-damper strut force (`>= 0` N).
  static double computeSpringDamperForce({
    required WheelConfig config,
    required double compression,
    required double compressionVelocity,
  }) {
    if (compression <= 0.0) return 0.0;
    final springForce = compression * config.springStiffness;
    final damperRate = compressionVelocity >= 0.0
        ? config.damperCompression
        : config.damperRebound;
    return math.max(0.0, springForce + compressionVelocity * damperRate);
  }

  /// Evaluates Ackermann inner/outer steering angles `(leftAngleRad, rightAngleRad)`
  /// for a positive right-turn steer angle (`> 0`) or negative left-turn (`< 0`).
  static ({double leftAngleRad, double rightAngleRad}) computeAckermannAngles({
    required double centerSteerAngleRad,
    required double wheelbase,
    required double trackWidth,
  }) {
    if (centerSteerAngleRad.abs() <= 1e-6 ||
        wheelbase <= 1e-4 ||
        trackWidth <= 1e-4) {
      return (
        leftAngleRad: centerSteerAngleRad,
        rightAngleRad: centerSteerAngleRad,
      );
    }
    final sign = centerSteerAngleRad.sign;
    final absAngle = centerSteerAngleRad.abs();
    final tanA = math.tan(absAngle);
    final turnRadius = wheelbase / math.max(1e-5, tanA);
    final halfTrack = trackWidth * 0.5;
    final innerDenom = math.max(0.15, turnRadius - halfTrack);
    final outerDenom = turnRadius + halfTrack;
    final innerAngle = math.atan2(wheelbase, innerDenom);
    final outerAngle = math.atan2(wheelbase, outerDenom);
    if (sign > 0) {
      // Turning right: right wheel is inner, left wheel is outer.
      return (leftAngleRad: outerAngle, rightAngleRad: innerAngle);
    }
    // Turning left: left wheel is inner (negative), right wheel is outer (negative).
    return (leftAngleRad: -innerAngle, rightAngleRad: -outerAngle);
  }
}
