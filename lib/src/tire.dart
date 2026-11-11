import 'dart:math' as math;

/// Result of evaluating a [TireBrushModel] contact patch.
final class TireForceSample {
  final double longitudinalForce;
  final double lateralForce;
  final double slipRatio;
  final double slipAngleRad;
  final double frictionLimitN;

  const TireForceSample({
    required this.longitudinalForce,
    required this.lateralForce,
    required this.slipRatio,
    required this.slipAngleRad,
    required this.frictionLimitN,
  });
}

/// Analytic brush tire model with smooth slip curves, peak/sliding grip transition,
/// friction-ellipse combined slip clamping, and handbrake drift scaling.
final class TireBrushModel {
  /// Slope of longitudinal force wrt slip ratio (`tanh(longitudinalStiffness * slip)`).
  final double longitudinalStiffness;

  /// Slope of lateral force wrt slip angle (`tanh(lateralStiffness * slipAngle)`).
  final double lateralStiffness;

  /// Peak tire friction coefficient at reference nominal load (`~4000 N`).
  final double peakMu;

  /// Reduced kinetic friction coefficient during saturated slip / locked slide.
  final double slidingMu;

  /// Lateral grip multiplier applied when handbrake locks rear wheels.
  final double handbrakeLateralGripScale;

  /// Load sensitivity exponent factor reducing mu gently on overloaded tires.
  final double loadSensitivity;

  /// Reference vertical wheel load (N) for load sensitivity scaling.
  final double referenceLoadN;

  /// Low-speed regularization threshold (`m/s`) preventing static jitter at rest.
  final double lowSpeedThresholdMps;

  const TireBrushModel({
    this.longitudinalStiffness = 11.0,
    this.lateralStiffness = 8.5,
    this.peakMu = 1.15,
    this.slidingMu = 0.88,
    this.handbrakeLateralGripScale = 0.42,
    this.loadSensitivity = 0.05,
    this.referenceLoadN = 3500.0,
    this.lowSpeedThresholdMps = 0.4,
  });

  static double _tanh(double x) {
    final c = x.clamp(-12.0, 12.0);
    final ep = math.exp(c);
    final em = math.exp(-c);
    return (ep - em) / (ep + em);
  }

  /// Evaluates longitudinal and lateral contact-patch forces in wheel local frame.
  ///
  /// Positive [wheelSurfaceSpeedMps] exceeds [forwardSpeedMps] -> positive
  /// [TireForceSample.longitudinalForce] (forward acceleration along wheel forward axis).
  /// Positive [lateralSpeedMps] (drifting toward wheel `+right` axis) produces a
  /// negative [TireForceSample.lateralForce] opposing sideways motion.
  TireForceSample evaluate({
    required double normalForceN,
    required double forwardSpeedMps,
    required double lateralSpeedMps,
    required double wheelSurfaceSpeedMps,
    double surfaceGripScale = 1.0,
    bool handbrakeLocked = false,
  }) {
    if (normalForceN <= 0.0) {
      return const TireForceSample(
        longitudinalForce: 0.0,
        lateralForce: 0.0,
        slipRatio: 0.0,
        slipAngleRad: 0.0,
        frictionLimitN: 0.0,
      );
    }

    final loadFactor = math.max(
      0.65,
      1.0 - loadSensitivity * ((normalForceN - referenceLoadN) / math.max(1.0, referenceLoadN)),
    );
    final effectivePeakMu = peakMu * surfaceGripScale * loadFactor;
    final effectiveSlideMu = slidingMu * surfaceGripScale * loadFactor;

    final speedDenom = math.max(
      lowSpeedThresholdMps,
      math.max(forwardSpeedMps.abs(), wheelSurfaceSpeedMps.abs()),
    );
    final slipRatio = ((wheelSurfaceSpeedMps - forwardSpeedMps) / speedDenom).clamp(-2.5, 2.5);

    final latDenom = math.max(lowSpeedThresholdMps, forwardSpeedMps.abs());
    final slipAngle = math.atan2(lateralSpeedMps, latDenom);

    // Smooth blend from peakMu toward slidingMu at high combined slip magnitude.
    final combinedSlip = math.sqrt(slipRatio * slipRatio + slipAngle * slipAngle);
    final slipDecay = (combinedSlip / 0.65).clamp(0.0, 1.0);
    final mu = effectivePeakMu + (effectiveSlideMu - effectivePeakMu) * slipDecay;
    final maxForceN = mu * normalForceN;

    // Low-speed viscous stabilizer avoids wild oscillation when barely rolling.
    final lowSpeedBlend =
        (math.max(forwardSpeedMps.abs(), lateralSpeedMps.abs()) / lowSpeedThresholdMps).clamp(0.0, 1.0);

    var fLong = maxForceN * _tanh(longitudinalStiffness * slipRatio);
    final latGripScale = handbrakeLocked ? handbrakeLateralGripScale : 1.0;
    var fLat = -maxForceN * latGripScale * _tanh(lateralStiffness * slipAngle);

    if (lowSpeedBlend < 1.0) {
      // Preserve driven traction from standstill while damping passive lateral drift at v~0.
      fLat *= lowSpeedBlend;
    }

    // Combined friction ellipse saturation.
    final latCap = math.max(1e-6, maxForceN * latGripScale);
    final nx = fLong / math.max(1e-6, maxForceN);
    final ny = fLat / latCap;
    final ellipse = math.sqrt(nx * nx + ny * ny);
    if (ellipse > 1.0) {
      final inv = 1.0 / ellipse;
      fLong *= inv;
      fLat *= inv;
    }

    return TireForceSample(
      longitudinalForce: fLong,
      lateralForce: fLat,
      slipRatio: slipRatio,
      slipAngleRad: slipAngle,
      frictionLimitN: maxForceN,
    );
  }
}
