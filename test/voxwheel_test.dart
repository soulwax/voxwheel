import 'dart:math' as math;

import 'package:test/test.dart';
import 'package:voxwheel/voxwheel.dart';

void main() {
  group('Suspension & Ackermann Steering', () {
    test('settles smoothly onto flat ground and balances anti-roll transfer', () {
      final solver = VehicleDynamicsSolver(
        spec: VehicleSpec.sportsCar(),
        initialPosition: const WheelVec3(0.0, 0.72, 0.0),
      );
      for (var i = 0; i < 180; i++) {
        solver.step(1.0 / 60.0, VehicleInput.idle);
      }
      expect(solver.state.groundedWheelCount, 4);
      expect(solver.state.position.y, closeTo(0.64, 0.15));
      expect(solver.state.linearVelocity.y.abs(), lessThan(0.15));
      for (final w in solver.state.wheelStates) {
        expect(w.normalForce, greaterThan(2500.0));
        expect(w.compression, greaterThan(0.02));
      }

      final ackRight = WheelSuspension.computeAckermannAngles(
        centerSteerAngleRad: 0.35,
        wheelbase: 2.56,
        trackWidth: 1.56,
      );
      // Right turn -> inner (right) wheel turns sharper than outer (left) wheel.
      expect(ackRight.rightAngleRad, greaterThan(ackRight.leftAngleRad));
    });
  });

  group('Drivetrain & Acceleration', () {
    test('accelerates straight ahead and upshifts automatically through gears', () {
      final solver = VehicleDynamicsSolver(
        spec: VehicleSpec.sportsCar(),
        initialPosition: const WheelVec3(0.0, 0.65, 0.0),
      );

      for (var i = 0; i < 30; i++) {
        solver.step(1.0 / 60.0, VehicleInput.idle);
      }

      for (var i = 0; i < 320; i++) {
        solver.step(1.0 / 60.0, const VehicleInput(throttle: 1.0));
      }

      expect(solver.state.forwardSpeedMps, greaterThan(18.0));
      expect(solver.state.position.z, lessThan(-35.0)); // -Z is forward
      expect(solver.state.currentGear, greaterThanOrEqualTo(2));
      expect(solver.state.rpm, greaterThan(1500.0));
    });

    test('braking decelerates vehicle safely down toward standstill', () {
      final solver = VehicleDynamicsSolver(
        spec: VehicleSpec.sportsCar(),
        initialPosition: const WheelVec3(0.0, 0.65, 0.0),
      );
      for (var i = 0; i < 180; i++) {
        solver.step(1.0 / 60.0, const VehicleInput(throttle: 1.0));
      }
      final topSpeed = solver.state.forwardSpeedMps;
      expect(topSpeed, greaterThan(12.0));

      for (var i = 0; i < 160; i++) {
        solver.step(1.0 / 60.0, const VehicleInput(brake: 1.0));
      }
      expect(solver.state.forwardSpeedMps, lessThan(topSpeed * 0.25));
    });
  });

  group('Tire Brush Model & Cornering', () {
    test('turning right curves trajectory into +X and induces right yaw', () {
      final solver = VehicleDynamicsSolver(
        spec: VehicleSpec.sportsCar(),
        initialPosition: const WheelVec3(0.0, 0.65, 0.0),
      );

      for (var i = 0; i < 120; i++) {
        solver.step(1.0 / 60.0, const VehicleInput(throttle: 0.9));
      }
      final speedBeforeTurn = solver.state.forwardSpeedMps;
      expect(speedBeforeTurn, greaterThan(8.0));

      for (var i = 0; i < 90; i++) {
        solver.step(
          1.0 / 60.0,
          const VehicleInput(throttle: 0.5, steer: 0.65),
        );
      }

      // Forward (-Z) rotated right tilts toward +X.
      final fwd = solver.state.rotation.rotate(VehicleDynamicsSolver.localForward);
      expect(fwd.x, greaterThan(0.25));
      expect(solver.state.position.x, greaterThan(2.5));
      expect(math.sqrt(solver.state.linearVelocity.lengthSquared), greaterThan(5.0));
    });

    test('handbrake scales down rear lateral grip vs normal rolling tire', () {
      const tire = TireBrushModel();
      final gripNormal = tire.evaluate(
        normalForceN: 3400.0,
        forwardSpeedMps: 15.0,
        lateralSpeedMps: 3.5,
        wheelSurfaceSpeedMps: 15.0,
        handbrakeLocked: false,
      );
      final gripHandbrake = tire.evaluate(
        normalForceN: 3400.0,
        forwardSpeedMps: 15.0,
        lateralSpeedMps: 3.5,
        wheelSurfaceSpeedMps: 0.0,
        handbrakeLocked: true,
      );
      expect(gripHandbrake.lateralForce.abs(), lessThan(gripNormal.lateralForce.abs()));
    });
  });
}
