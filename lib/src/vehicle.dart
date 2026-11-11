import 'dart:math' as math;

import 'drivetrain.dart';
import 'math.dart';
import 'suspension.dart';
import 'tire.dart';

/// Driver control inputs for one vehicle physics step.
final class VehicleInput {
  final double throttle;
  final double brake;
  final double steer;
  final bool handbrake;

  const VehicleInput({
    this.throttle = 0.0,
    this.brake = 0.0,
    this.steer = 0.0,
    this.handbrake = false,
  });

  static const VehicleInput idle = VehicleInput();
}

/// Raycast intersection result from the host scene or custom ground collider.
final class WheelRaycastHit {
  final double distance;
  final WheelVec3 point;
  final WheelVec3 normal;

  const WheelRaycastHit({
    required this.distance,
    required this.point,
    this.normal = WheelVec3.unitY,
  });
}

/// Callback returning nearest ground hit along `[origin, origin + direction * maxDistance]`.
typedef WheelGroundQuery = WheelRaycastHit? Function(
  WheelVec3 origin,
  WheelVec3 direction,
  double maxDistance,
  int wheelIndex,
);

/// Full rigid-body & chassis configuration for [VehicleDynamicsSolver].
final class VehicleSpec {
  final double mass;
  final WheelVec3 inertiaTensor;
  final WheelVec3 centerOfMassOffset;
  final double maxSteerAngleRad;
  final double? wheelbaseOverride;
  final double? trackWidthOverride;
  final double dragCoefficient;
  final double frontalArea;
  final double downforceCoefficient;
  final double gravity;
  final double chassisRollTorqueScale;
  final TireBrushModel tireModel;
  final DrivetrainSpec drivetrain;
  final List<WheelConfig> wheels;
  final List<AntiRollPair> antiRollPairs;

  VehicleSpec({
    this.mass = 1350.0,
    this.inertiaTensor = const WheelVec3(1950.0, 2550.0, 820.0),
    this.centerOfMassOffset = const WheelVec3(0.0, -0.10, 0.0),
    this.maxSteerAngleRad = 0.56,
    this.wheelbaseOverride,
    this.trackWidthOverride,
    this.dragCoefficient = 0.31,
    this.frontalArea = 2.15,
    this.downforceCoefficient = 0.18,
    this.gravity = 9.81,
    this.chassisRollTorqueScale = 0.55,
    this.tireModel = const TireBrushModel(),
    DrivetrainSpec? drivetrain,
    List<WheelConfig>? wheels,
    List<AntiRollPair>? antiRollPairs,
  })  : drivetrain = drivetrain ?? DrivetrainSpec.sportsCarDefault(),
        wheels = wheels ?? _defaultFourWheels(DriveLayout.rwd),
        antiRollPairs = antiRollPairs ??
            const [
              AntiRollPair(leftWheelIndex: 0, rightWheelIndex: 1, stiffness: 9000.0),
              AntiRollPair(leftWheelIndex: 2, rightWheelIndex: 3, stiffness: 7500.0),
            ];

  static List<WheelConfig> _defaultFourWheels(DriveLayout layout) => const [
        WheelConfig(
          id: 'FL',
          localAnchor: WheelVec3(-0.78, 0.05, -1.28),
          axle: WheelAxle.front,
          sideSign: -1.0,
          steerable: true,
          braked: true,
          handbraked: false,
        ),
        WheelConfig(
          id: 'FR',
          localAnchor: WheelVec3(0.78, 0.05, -1.28),
          axle: WheelAxle.front,
          sideSign: 1.0,
          steerable: true,
          braked: true,
          handbraked: false,
        ),
        WheelConfig(
          id: 'RL',
          localAnchor: WheelVec3(-0.78, 0.05, 1.28),
          axle: WheelAxle.rear,
          sideSign: -1.0,
          steerable: false,
          braked: true,
          handbraked: true,
        ),
        WheelConfig(
          id: 'RR',
          localAnchor: WheelVec3(0.78, 0.05, 1.28),
          axle: WheelAxle.rear,
          sideSign: 1.0,
          steerable: false,
          braked: true,
          handbraked: true,
        ),
      ];

  factory VehicleSpec.sportsCar() => VehicleSpec();

  factory VehicleSpec.offroad4x4() => VehicleSpec(
        mass: 1850.0,
        inertiaTensor: const WheelVec3(2600.0, 3400.0, 1250.0),
        maxSteerAngleRad: 0.60,
        drivetrain: DrivetrainSpec.offroad4x4Default(),
        tireModel: const TireBrushModel(
          longitudinalStiffness: 10.5,
          lateralStiffness: 7.5,
          peakMu: 1.20,
          slidingMu: 0.94,
        ),
        wheels: const [
          WheelConfig(
            id: 'FL',
            localAnchor: WheelVec3(-0.85, 0.05, -1.35),
            axle: WheelAxle.front,
            sideSign: -1.0,
            restLength: 0.58,
            maxTravel: 0.30,
            wheelRadius: 0.42,
            springStiffness: 36000.0,
            steerable: true,
          ),
          WheelConfig(
            id: 'FR',
            localAnchor: WheelVec3(0.85, 0.05, -1.35),
            axle: WheelAxle.front,
            sideSign: 1.0,
            restLength: 0.58,
            maxTravel: 0.30,
            wheelRadius: 0.42,
            springStiffness: 36000.0,
            steerable: true,
          ),
          WheelConfig(
            id: 'RL',
            localAnchor: WheelVec3(-0.85, 0.05, 1.35),
            axle: WheelAxle.rear,
            sideSign: -1.0,
            restLength: 0.58,
            maxTravel: 0.30,
            wheelRadius: 0.42,
            springStiffness: 35000.0,
            handbraked: true,
          ),
          WheelConfig(
            id: 'RR',
            localAnchor: WheelVec3(0.85, 0.05, 1.35),
            axle: WheelAxle.rear,
            sideSign: 1.0,
            restLength: 0.58,
            maxTravel: 0.30,
            wheelRadius: 0.42,
            springStiffness: 35000.0,
            handbraked: true,
          ),
        ],
      );

  double resolveWheelbase() {
    if (wheelbaseOverride != null) return wheelbaseOverride!;
    var minZ = double.infinity;
    var maxZ = -double.infinity;
    for (final w in wheels) {
      if (w.localAnchor.z < minZ) minZ = w.localAnchor.z;
      if (w.localAnchor.z > maxZ) maxZ = w.localAnchor.z;
    }
    return (maxZ - minZ).clamp(0.5, 10.0);
  }

  double resolveTrackWidth() {
    if (trackWidthOverride != null) return trackWidthOverride!;
    var minX = double.infinity;
    var maxX = -double.infinity;
    for (final w in wheels) {
      if (w.localAnchor.x < minX) minX = w.localAnchor.x;
      if (w.localAnchor.x > maxX) maxX = w.localAnchor.x;
    }
    return (maxX - minX).clamp(0.4, 5.0);
  }
}

/// Live state of a simulated vehicle chassis.
final class VehicleState {
  WheelVec3 position;
  WheelQuat rotation;
  WheelVec3 linearVelocity;
  WheelVec3 angularVelocity;
  double rpm;
  int currentGear;
  double speedMps;
  double forwardSpeedMps;
  final List<WheelSuspensionState> wheelStates;

  VehicleState({
    this.position = const WheelVec3(0.0, 0.8, 0.0),
    this.rotation = WheelQuat.identity,
    this.linearVelocity = WheelVec3.zero,
    this.angularVelocity = WheelVec3.zero,
    this.rpm = 900.0,
    this.currentGear = 1,
    this.speedMps = 0.0,
    this.forwardSpeedMps = 0.0,
    required this.wheelStates,
  });

  double get speedKmh => speedMps * 3.6;

  int get groundedWheelCount {
    var count = 0;
    for (final w in wheelStates) {
      if (w.isGrounded) count++;
    }
    return count;
  }
}

/// Multi-substep raycast vehicle dynamics solver.
///
/// Local chassis frame matches engine conventions: `+X` right, `+Y` up, `-Z` forward.
final class VehicleDynamicsSolver {
  static const WheelVec3 localForward = WheelVec3(0.0, 0.0, -1.0);
  static const WheelVec3 localRight = WheelVec3(1.0, 0.0, 0.0);
  static const WheelVec3 localUp = WheelVec3(0.0, 1.0, 0.0);

  final VehicleSpec spec;
  final VehicleState state;

  VehicleDynamicsSolver({
    required this.spec,
    WheelVec3 initialPosition = const WheelVec3(0.0, 0.8, 0.0),
    WheelQuat initialRotation = WheelQuat.identity,
  }) : state = VehicleState(
          position: initialPosition,
          rotation: initialRotation.normalized,
          rpm: spec.drivetrain.idleRpm,
          currentGear: 1,
          wheelStates: [
            for (final w in spec.wheels)
              WheelSuspensionState(id: w.id, initialRestLength: w.restLength),
          ],
        );

  /// Convenience [WheelGroundQuery] against a horizontal plane at `y = groundY`.
  static WheelGroundQuery flatGroundQuery({double groundY = 0.0}) =>
      (WheelVec3 origin, WheelVec3 direction, double maxDistance, int wheelIndex) {
        if (direction.y >= -1e-6) return null;
        final t = (groundY - origin.y) / direction.y;
        if (t < 0.0 || t > maxDistance) return null;
        return WheelRaycastHit(
          distance: t,
          point: origin + direction * t,
          normal: WheelVec3.unitY,
        );
      };

  void shiftUp() {
    if (state.currentGear < spec.drivetrain.topGear) {
      state.currentGear = state.currentGear == -1 ? 1 : state.currentGear + 1;
    }
  }

  void shiftDown() {
    if (state.currentGear > 1) {
      state.currentGear--;
    } else if (state.currentGear == 1) {
      state.currentGear = 0;
    } else if (state.currentGear == 0) {
      state.currentGear = -1;
    }
  }

  void reset({
    WheelVec3 position = const WheelVec3(0.0, 0.8, 0.0),
    WheelQuat rotation = WheelQuat.identity,
  }) {
    state.position = position;
    state.rotation = rotation.normalized;
    state.linearVelocity = WheelVec3.zero;
    state.angularVelocity = WheelVec3.zero;
    state.rpm = spec.drivetrain.idleRpm;
    state.currentGear = 1;
    state.speedMps = 0.0;
    state.forwardSpeedMps = 0.0;
    for (var i = 0; i < state.wheelStates.length; i++) {
      final w = state.wheelStates[i];
      final cfg = spec.wheels[i];
      w
        ..isGrounded = false
        ..suspensionLength = cfg.restLength
        ..compression = 0.0
        ..compressionVelocity = 0.0
        ..compressionRatio = 0.0
        ..normalForce = 0.0
        ..antiRollForce = 0.0
        ..steerAngleRad = 0.0
        ..spinAngleRad = 0.0
        ..angularVelocityRadPerSec = 0.0
        ..slipRatio = 0.0
        ..slipAngleRad = 0.0
        ..longitudinalForce = 0.0
        ..lateralForce = 0.0;
    }
  }

  /// Advances the vehicle dynamics by [dt] seconds (split into stable internal substeps).
  void step(
    double dt,
    VehicleInput input, {
    WheelGroundQuery? groundQuery,
    int substeps = 2,
  }) {
    if (dt <= 0.0) return;
    final steps = math.max(1, substeps);
    final subDt = dt / steps;
    final query = groundQuery ?? flatGroundQuery();
    for (var s = 0; s < steps; s++) {
      _stepSingle(subDt, input, query);
    }
  }

  void _stepSingle(double dt, VehicleInput input, WheelGroundQuery groundQuery) {
    final throttle = input.throttle.clamp(0.0, 1.0);
    final brake = input.brake.clamp(0.0, 1.0);
    final steerInput = input.steer.clamp(-1.0, 1.0);
    final drivetrain = spec.drivetrain;

    final rot = state.rotation.normalized;
    final chassisFwd = rot.rotate(localForward).normalized;
    final chassisRight = rot.rotate(localRight).normalized;
    final chassisUp = rot.rotate(localUp).normalized;
    final chassisDown = -chassisUp;
    final worldCom = state.position + rot.rotate(spec.centerOfMassOffset);

    state.forwardSpeedMps = state.linearVelocity.dot(chassisFwd);
    state.speedMps = state.linearVelocity.length;

    // 1. Ackermann steering angles for steerable wheels.
    final centerSteerRad = steerInput * spec.maxSteerAngleRad;
    final ackermann = WheelSuspension.computeAckermannAngles(
      centerSteerAngleRad: centerSteerRad,
      wheelbase: spec.resolveWheelbase(),
      trackWidth: spec.resolveTrackWidth(),
    );

    // 2. Raycast every suspension strut & compute base spring-damper normal forces.
    final wheelHits = List<WheelRaycastHit?>.filled(spec.wheels.length, null);
    for (var i = 0; i < spec.wheels.length; i++) {
      final cfg = spec.wheels[i];
      final ws = state.wheelStates[i];

      if (cfg.steerable) {
        if (cfg.sideSign < 0) {
          ws.steerAngleRad = ackermann.leftAngleRad;
        } else if (cfg.sideSign > 0) {
          ws.steerAngleRad = ackermann.rightAngleRad;
        } else {
          ws.steerAngleRad = centerSteerRad;
        }
      } else {
        ws.steerAngleRad = 0.0;
      }

      final worldAnchor = state.position + rot.rotate(cfg.localAnchor);
      ws.worldAnchor = worldAnchor;

      final maxRay = cfg.restLength + cfg.wheelRadius;
      final hit = groundQuery(worldAnchor, chassisDown, maxRay, i);
      wheelHits[i] = hit;

      if (hit != null && hit.distance <= maxRay) {
        final wasGrounded = ws.isGrounded;
        final rawLen = hit.distance - cfg.wheelRadius;
        final clampedLen = rawLen.clamp(cfg.minLength, cfg.restLength);
        final newComp = (cfg.restLength - clampedLen).clamp(0.0, cfg.maxTravel);
        final compVel = wasGrounded ? (newComp - ws.compression) / dt : 0.0;

        ws
          ..isGrounded = true
          ..suspensionLength = clampedLen
          ..compression = newComp
          ..compressionVelocity = compVel
          ..compressionRatio =
              cfg.maxTravel > 1e-6 ? (newComp / cfg.maxTravel).clamp(0.0, 1.0) : 0.0
          ..worldContactPoint = hit.point
          ..worldContactNormal = hit.normal.normalized
          ..antiRollForce = 0.0
          ..normalForce = WheelSuspension.computeSpringDamperForce(
            config: cfg,
            compression: newComp,
            compressionVelocity: compVel,
          ) *
              hit.normal.normalized.dot(chassisUp).clamp(0.0, 1.0);
      } else {
        ws
          ..isGrounded = false
          ..suspensionLength = cfg.restLength
          ..compression = 0.0
          ..compressionVelocity = 0.0
          ..compressionRatio = 0.0
          ..normalForce = 0.0
          ..antiRollForce = 0.0
          ..slipRatio = 0.0
          ..slipAngleRad = 0.0
          ..longitudinalForce = 0.0
          ..lateralForce = 0.0
          ..worldContactPoint = worldAnchor + chassisDown * maxRay
          ..worldContactNormal = chassisUp;
      }
    }

    // 3. Apply anti-roll stabilizer bar transfer across linked left/right pairs.
    for (final pair in spec.antiRollPairs) {
      if (pair.leftWheelIndex < 0 ||
          pair.leftWheelIndex >= state.wheelStates.length ||
          pair.rightWheelIndex < 0 ||
          pair.rightWheelIndex >= state.wheelStates.length) {
        continue;
      }
      final leftWs = state.wheelStates[pair.leftWheelIndex];
      final rightWs = state.wheelStates[pair.rightWheelIndex];
      if (!leftWs.isGrounded && !rightWs.isGrounded) continue;
      final force = (leftWs.compression - rightWs.compression) * pair.stiffness;
      if (leftWs.isGrounded) {
        leftWs.antiRollForce += force;
        leftWs.normalForce = math.max(0.0, leftWs.normalForce + force);
      }
      if (rightWs.isGrounded) {
        rightWs.antiRollForce -= force;
        rightWs.normalForce = math.max(0.0, rightWs.normalForce - force);
      }
    }

    // 4. Drivetrain: compute driven wheel counts, automatic shifting & engine torque.
    final split = drivetrain.axleDriveSplit();
    var frontDrivenCount = 0;
    var rearDrivenCount = 0;
    for (final cfg in spec.wheels) {
      if (cfg.axle == WheelAxle.front && split.frontShare > 0.0) frontDrivenCount++;
      if (cfg.axle == WheelAxle.rear && split.rearShare > 0.0) rearDrivenCount++;
    }

    if (drivetrain.autoShift) {
      if (state.currentGear > 0 && state.forwardSpeedMps < 0.35 && brake > 0.25 && throttle <= 0.05) {
        state.currentGear = -1;
      } else if (state.currentGear == -1 && throttle > 0.15 && state.forwardSpeedMps > -0.5) {
        state.currentGear = 1;
      } else if (state.currentGear == 0 && throttle > 0.1) {
        state.currentGear = 1;
      }
    }

    final gearRatio = drivetrain.gearRatioFor(state.currentGear);
    final totalRatio = gearRatio * drivetrain.finalDriveRatio;

    // Sample average driven wheel angular speed for engine RPM coupling.
    var drivenWheelSpeedSum = 0.0;
    var drivenSampleCount = 0;
    for (var i = 0; i < spec.wheels.length; i++) {
      final cfg = spec.wheels[i];
      final isDriven = (cfg.axle == WheelAxle.front && split.frontShare > 0.0) ||
          (cfg.axle == WheelAxle.rear && split.rearShare > 0.0);
      if (isDriven) {
        drivenWheelSpeedSum += state.wheelStates[i].angularVelocityRadPerSec;
        drivenSampleCount++;
      }
    }
    final avgWheelOmega =
        drivenSampleCount > 0 ? (drivenWheelSpeedSum / drivenSampleCount) : 0.0;
    final coupledRpm = (avgWheelOmega / (2.0 * math.pi)) * totalRatio.abs() * 60.0;
    final launchSlipRpm = drivetrain.idleRpm + throttle * 2100.0;
    final targetRpm = state.currentGear == 0
        ? (drivetrain.idleRpm + throttle * (drivetrain.redlineRpm - drivetrain.idleRpm))
        : math.max(launchSlipRpm, coupledRpm).clamp(drivetrain.idleRpm, drivetrain.redlineRpm);
    state.rpm += (targetRpm - state.rpm) * math.min(1.0, dt * 10.0);

    if (drivetrain.autoShift && state.currentGear > 0) {
      if (state.rpm >= drivetrain.upshiftRpm &&
          state.currentGear < drivetrain.topGear &&
          throttle > 0.15) {
        state.currentGear++;
        final nextRatio =
            drivetrain.gearRatioFor(state.currentGear).abs() * drivetrain.finalDriveRatio;
        state.rpm = math.max(
          drivetrain.idleRpm + 800.0,
          (avgWheelOmega / (2.0 * math.pi)) * nextRatio * 60.0,
        );
      } else if (state.rpm <= drivetrain.downshiftRpm && state.currentGear > 1) {
        state.currentGear--;
      }
    }

    final effectiveRatio =
        drivetrain.gearRatioFor(state.currentGear) * drivetrain.finalDriveRatio;
    final grossEngineTorque = drivetrain.sampleTorqueNm(state.rpm);
    final totalDriveWheelTorque = throttle > 0.01 && effectiveRatio != 0.0
        ? grossEngineTorque * throttle * effectiveRatio * drivetrain.transmissionEfficiency
        : -drivetrain.engineBrakingTorqueNm *
            (state.forwardSpeedMps.abs() > 0.3 ? state.forwardSpeedMps.sign : 0.0) *
            (effectiveRatio != 0.0 ? 1.0 : 0.0);

    // 5. Accumulate forces & torques on chassis body.
    var totalForce = WheelVec3(0.0, -spec.mass * spec.gravity, 0.0);
    var totalTorque = WheelVec3.zero;

    for (var i = 0; i < spec.wheels.length; i++) {
      final cfg = spec.wheels[i];
      final ws = state.wheelStates[i];

      var wheelDriveTorque = 0.0;
      if (cfg.axle == WheelAxle.front && frontDrivenCount > 0) {
        wheelDriveTorque = totalDriveWheelTorque * split.frontShare / frontDrivenCount;
      } else if (cfg.axle == WheelAxle.rear && rearDrivenCount > 0) {
        wheelDriveTorque = totalDriveWheelTorque * split.rearShare / rearDrivenCount;
      }

      var wheelBrakeTorque = 0.0;
      if (cfg.braked && brake > 0.0) {
        // When in reverse gear (`gear == -1`), `brake` may drive reverse throttle or stop forward roll.
        if (state.currentGear >= 0 || state.forwardSpeedMps > 0.2) {
          wheelBrakeTorque += brake * drivetrain.maxBrakeTorqueNm * 0.25;
        }
      }
      if (cfg.handbraked && input.handbrake) {
        wheelBrakeTorque += drivetrain.handbrakeTorqueNm;
      }

      if (!ws.isGrounded) {
        // Free-spin airborne wheel slightly under engine torque.
        final freeAccel = wheelDriveTorque / cfg.wheelInertia;
        ws.angularVelocityRadPerSec =
            (ws.angularVelocityRadPerSec + freeAccel * dt) * math.max(0.0, 1.0 - 0.8 * dt);
        if (wheelBrakeTorque > 0.0) {
          final decel = (wheelBrakeTorque / cfg.wheelInertia) * dt;
          if (ws.angularVelocityRadPerSec.abs() <= decel) {
            ws.angularVelocityRadPerSec = 0.0;
          } else {
            ws.angularVelocityRadPerSec -= ws.angularVelocityRadPerSec.sign * decel;
          }
        }
        ws.spinAngleRad += ws.angularVelocityRadPerSec * dt;
        continue;
      }

      // Project wheel forward & right onto ground contact plane.
      final steerQuat = WheelQuat.fromAxisAngle(chassisUp, -ws.steerAngleRad);
      final wheelFwdWorld = steerQuat.rotate(chassisFwd).projectOntoPlane(ws.worldContactNormal).normalized;
      final wheelRightWorld = steerQuat.rotate(chassisRight).projectOntoPlane(ws.worldContactNormal).normalized;

      final r = ws.worldContactPoint - worldCom;
      final hubVel = state.linearVelocity + state.angularVelocity.cross(r);
      final vFwd = hubVel.dot(wheelFwdWorld);
      final vLat = hubVel.dot(wheelRightWorld);

      // Step candidate wheel spin with engine drive & brake torques.
      var omega = ws.angularVelocityRadPerSec;
      final omegaRoad = vFwd / cfg.wheelRadius;
      if (throttle <= 0.01 && wheelBrakeTorque <= 1e-3 && ws.isGrounded) {
        // Smoothly match free rolling tire speed when coasting.
        omega += (omegaRoad - omega) * math.min(1.0, dt * 18.0);
      } else {
        omega += (wheelDriveTorque / cfg.wheelInertia) * dt;
      }
      if (wheelBrakeTorque > 0.0) {
        final decel = (wheelBrakeTorque / cfg.wheelInertia) * dt;
        if (omega.abs() <= decel) {
          omega = 0.0;
        } else {
          omega -= omega.sign * decel;
        }
      }

      final handbrakeLocked = cfg.handbraked && input.handbrake;
      if (handbrakeLocked) omega = 0.0;

      final sample = spec.tireModel.evaluate(
        normalForceN: ws.normalForce,
        forwardSpeedMps: vFwd,
        lateralSpeedMps: vLat,
        wheelSurfaceSpeedMps: omega * cfg.wheelRadius,
        surfaceGripScale: cfg.peakGripMu / math.max(1e-6, spec.tireModel.peakMu),
        handbrakeLocked: handbrakeLocked,
      );

      // Tire road reaction torque stabilizes wheel spin toward ground traction.
      if (!handbrakeLocked) {
        final reactionAccel = (-sample.longitudinalForce * cfg.wheelRadius) / cfg.wheelInertia;
        omega += reactionAccel * dt * 0.35;
      }
      ws.angularVelocityRadPerSec = omega.clamp(-250.0, 250.0);
      ws.spinAngleRad += ws.angularVelocityRadPerSec * dt;
      ws.slipRatio = sample.slipRatio;
      ws.slipAngleRad = sample.slipAngleRad;
      ws.longitudinalForce = sample.longitudinalForce;
      ws.lateralForce = sample.lateralForce;

      // Apply suspension upward force at anchor, and tire traction on reduced roll-center lever arm.
      final fNormal = ws.worldContactNormal * ws.normalForce;
      final fTire = wheelFwdWorld * sample.longitudinalForce + wheelRightWorld * sample.lateralForce;

      totalForce += fNormal + fTire;

      final rAnchor = ws.worldAnchor - worldCom;
      final rTire = WheelVec3.lerp(rAnchor, r, spec.chassisRollTorqueScale.clamp(0.0, 1.0));
      totalTorque += rAnchor.cross(fNormal) + rTire.cross(fTire);
    }

    // 6. Aerodynamic drag & downforce.
    final speedSq = state.linearVelocity.lengthSquared;
    if (speedSq > 1e-6) {
      final airDensity = 1.225;
      final dragMag = 0.5 * airDensity * spec.dragCoefficient * spec.frontalArea * speedSq;
      final dragDir = -state.linearVelocity.normalized;
      final downMag = 0.5 * airDensity * spec.downforceCoefficient * spec.frontalArea * speedSq;
      totalForce += dragDir * dragMag - chassisUp * downMag;
    }

    // 7. Semi-implicit Euler integration in world space (transform inertia into local frame).
    final linAccel = totalForce / spec.mass;
    state.linearVelocity += linAccel * dt;

    final invRot = rot.conjugate;
    final localTorque = invRot.rotate(totalTorque);
    final localAngAccel = WheelVec3(
      localTorque.x / math.max(1.0, spec.inertiaTensor.x),
      localTorque.y / math.max(1.0, spec.inertiaTensor.y),
      localTorque.z / math.max(1.0, spec.inertiaTensor.z),
    );
    final worldAngAccel = rot.rotate(localAngAccel);
    state.angularVelocity =
        (state.angularVelocity + worldAngAccel * dt) * math.max(0.0, 1.0 - 0.35 * dt);

    state.position += state.linearVelocity * dt;
    state.rotation = rot.integrateAngularVelocity(state.angularVelocity, dt);

    state.forwardSpeedMps = state.linearVelocity.dot(state.rotation.rotate(localForward));
    state.speedMps = state.linearVelocity.length;
  }
}
