import 'dart:math' as math;

/// Drive train wheel delivery topology.
enum DriveLayout {
  fwd,
  rwd,
  awd,
}

/// Sample point on an engine's torque curve (`(rpm, torqueNm)`).
final class TorqueCurvePoint {
  final double rpm;
  final double torqueNm;

  const TorqueCurvePoint(this.rpm, this.torqueNm);
}

/// Transmission gear ratios, torque curve, RPM limits, and axle torque split.
final class DrivetrainSpec {
  final DriveLayout layout;
  final double idleRpm;
  final double redlineRpm;
  final double upshiftRpm;
  final double downshiftRpm;
  final double finalDriveRatio;
  final double reverseRatio;
  final List<double> gearRatios;
  final double transmissionEfficiency;
  final double maxBrakeTorqueNm;
  final double handbrakeTorqueNm;
  final double engineBrakingTorqueNm;
  final double awdFrontBias;
  bool autoShift;
  final List<TorqueCurvePoint> torqueCurve;

  DrivetrainSpec({
    this.layout = DriveLayout.rwd,
    this.idleRpm = 900.0,
    this.redlineRpm = 7200.0,
    this.upshiftRpm = 6100.0,
    this.downshiftRpm = 2400.0,
    this.finalDriveRatio = 3.73,
    this.reverseRatio = -3.20,
    this.gearRatios = const [3.45, 2.18, 1.52, 1.14, 0.90],
    this.transmissionEfficiency = 0.88,
    this.maxBrakeTorqueNm = 3400.0,
    this.handbrakeTorqueNm = 4200.0,
    this.engineBrakingTorqueNm = 48.0,
    this.awdFrontBias = 0.42,
    this.autoShift = true,
    List<TorqueCurvePoint>? torqueCurve,
  }) : torqueCurve = torqueCurve ?? const [
          TorqueCurvePoint(900.0, 195.0),
          TorqueCurvePoint(2200.0, 295.0),
          TorqueCurvePoint(4400.0, 365.0),
          TorqueCurvePoint(6000.0, 340.0),
          TorqueCurvePoint(7200.0, 260.0),
        ];

  factory DrivetrainSpec.sportsCarDefault() => DrivetrainSpec();

  factory DrivetrainSpec.offroad4x4Default() => DrivetrainSpec(
        layout: DriveLayout.awd,
        redlineRpm: 6200.0,
        upshiftRpm: 5300.0,
        downshiftRpm: 2000.0,
        finalDriveRatio: 4.10,
        awdFrontBias: 0.48,
        gearRatios: const [3.95, 2.42, 1.62, 1.18, 0.92],
        torqueCurve: const [
          TorqueCurvePoint(800.0, 280.0),
          TorqueCurvePoint(2000.0, 420.0),
          TorqueCurvePoint(3600.0, 460.0),
          TorqueCurvePoint(5200.0, 390.0),
          TorqueCurvePoint(6200.0, 290.0),
        ],
      );

  /// Interpolates gross engine torque in `Nm` at [rpm].
  double sampleTorqueNm(double rpm) {
    if (torqueCurve.isEmpty) return 250.0;
    if (rpm <= torqueCurve.first.rpm) return torqueCurve.first.torqueNm;
    for (var i = 0; i < torqueCurve.length - 1; i++) {
      final a = torqueCurve[i];
      final b = torqueCurve[i + 1];
      if (rpm <= b.rpm) {
        final span = math.max(1e-3, b.rpm - a.rpm);
        final t = ((rpm - a.rpm) / span).clamp(0.0, 1.0);
        return a.torqueNm + (b.torqueNm - a.torqueNm) * t;
      }
    }
    return torqueCurve.last.torqueNm;
  }

  /// Returns gear ratio for `-1` (reverse), `0` (neutral), or `1..gearRatios.length`.
  double gearRatioFor(int gear) {
    if (gear == -1) return reverseRatio;
    if (gear == 0 || gearRatios.isEmpty) return 0.0;
    final idx = (gear - 1).clamp(0, gearRatios.length - 1);
    return gearRatios[idx];
  }

  int get topGear => gearRatios.length;

  ({double frontShare, double rearShare}) axleDriveSplit() => switch (layout) {
        DriveLayout.fwd => (frontShare: 1.0, rearShare: 0.0),
        DriveLayout.rwd => (frontShare: 0.0, rearShare: 1.0),
        DriveLayout.awd => (
            frontShare: awdFrontBias.clamp(0.0, 1.0),
            rearShare: 1.0 - awdFrontBias.clamp(0.0, 1.0),
          ),
      };
}
