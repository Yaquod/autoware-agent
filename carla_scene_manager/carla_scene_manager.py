"""
CARLA Autonomous Taxi - Wine/macOS Compatible Version
Works without map data or Traffic Manager
"""

import carla
import grpc
import time
import random
import math
import vehicle_gateway_pb2
import vehicle_gateway_pb2_grpc

class AutonomousTaxi:
    def __init__(self, carla_host='host.docker.internal', carla_port=2000,
                 grpc_host='autoware_bridge', grpc_port=50051):
        self.carla_host = carla_host
        self.carla_port = carla_port
        self.grpc_host = grpc_host
        self.grpc_port = grpc_port

        self.client = None
        self.world = None
        self.vehicle = None
        self.destination = None

        self.channel = None
        self.trip_stub = None

        self.vin_number = "TESLA_MODEL_3_001"
        self.trip_id = f"TRIP_{int(time.time())}"

    def setup_carla(self):
        """Initialize CARLA - Wine compatible"""
        print(f" Connecting to CARLA...")
        self.client = carla.Client(self.carla_host, self.carla_port)
        self.client.set_timeout(20.0)

        # Just get current world, don't try to load maps
        self.world = self.client.get_world()

        # Enable synchronous mode
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        self.world.apply_settings(settings)

        print(f" CARLA connected (Wine/Mac mode)")

    def cleanup_world(self):
        """Remove old vehicles and debris"""
        print(" Cleaning up old actors...")
        try:
            actors = self.world.get_actors()

            # Destroy vehicles
            vehicles = actors.filter('*vehicle*')
            print(f"   Found {len(vehicles)} vehicles to remove")
            for actor in vehicles:
                actor.destroy()

            # Also clean up any debris/props that might block spawn
            for actor in actors.filter('*debris*'):
                actor.destroy()

            # Tick multiple times to ensure cleanup completes
            for _ in range(10):
                self.world.tick()
                time.sleep(0.05)

            print(" Cleanup complete")

        except Exception as e:
            print(f"  Cleanup warning: {e}")

    def spawn_vehicle(self, spawn_location=None):
        """Spawn vehicle with collision avoidance and ground detection"""
        blueprint_library = self.world.get_blueprint_library()

        # Try to get any vehicle
        vehicle_bps = blueprint_library.filter('vehicle.*')
        if not vehicle_bps:
            print(" No vehicle blueprints available")
            return False

        bp = random.choice(vehicle_bps)

        # Strategy 1: Try to use map spawn points (safest)
        print("🔍 Looking for safe spawn points...")
        try:
            carla_map = self.world.get_map()
            spawn_points = carla_map.get_spawn_points()

            if spawn_points and len(spawn_points) > 0:
                print(f"   Found {len(spawn_points)} map spawn points")

                # Shuffle to get random order
                random.shuffle(spawn_points)

                # Try first 30 spawn points
                for idx, sp in enumerate(spawn_points[:30]):
                    try:
                        self.vehicle = self.world.spawn_actor(bp, sp)
                        loc = sp.location
                        print(f" Vehicle spawned at map spawn point ({loc.x:.1f}, {loc.y:.1f}, {loc.z:.1f})")

                        # Tick to ensure stable spawn
                        for _ in range(10):
                            self.world.tick()
                            time.sleep(0.02)

                        return True

                    except RuntimeError as e:
                        if idx < 5:  # Only log first few failures
                            print(f"   Spawn point {idx} blocked, trying next...")
                        continue
        except Exception as e:
            print(f"  Could not get map spawn points: {e}")

        # Strategy 2: Try provided location with variations
        spawn_attempts = []

        if spawn_location is not None:
            print(" Trying provided location with variations...")
            for z_offset in [0, 1.0, 2.0, 5.0, 10.0]:
                for x_offset in [0, 5, -5, 10, -10]:
                    for y_offset in [0, 5, -5, 10, -10]:
                        spawn_attempts.append(carla.Transform(
                            carla.Location(
                                x=spawn_location.location.x + x_offset,
                                y=spawn_location.location.y + y_offset,
                                z=spawn_location.location.z + z_offset
                            ),
                            carla.Rotation(yaw=random.uniform(0, 360))
                        ))

        # Strategy 3: Random locations in open areas
        print(" Trying random open locations...")
        for _ in range(50):
            spawn_attempts.append(carla.Transform(
                carla.Location(
                    x=random.uniform(-200, 200),
                    y=random.uniform(-200, 200),
                    z=random.uniform(0.5, 3.0)  # Lower heights for ground level
                ),
                carla.Rotation(yaw=random.uniform(0, 360))
            ))

        # Try each spawn attempt
        for idx, spawn_transform in enumerate(spawn_attempts):
            try:
                self.vehicle = self.world.spawn_actor(bp, spawn_transform)
                loc = spawn_transform.location
                print(f" Vehicle spawned at ({loc.x:.1f}, {loc.y:.1f}, {loc.z:.1f})")

                # Tick to ensure stable
                for _ in range(10):
                    self.world.tick()
                    time.sleep(0.02)

                # Check if vehicle fell or is stuck
                time.sleep(0.1)
                final_loc = self.vehicle.get_location()
                if abs(final_loc.z - loc.z) > 5.0:
                    print("  Vehicle fell, trying another location...")
                    self.vehicle.destroy()
                    self.world.tick()
                    continue

                return True

            except RuntimeError as e:
                if "collision" in str(e).lower():
                    if idx % 10 == 0:  # Log every 10th attempt
                        print(f"   Tried {idx} locations so far...")
                    continue
                else:
                    print(f"  Spawn error: {e}")
                    continue

        print(" Failed to spawn vehicle after all attempts")
        print(" Tip: Make sure CARLA is running and the world is loaded")
        return False

    def setup_grpc(self):
        """Connect to gRPC"""
        print(f"🔌 Connecting to gRPC at {self.grpc_host}:{self.grpc_port}...")
        try:
            self.channel = grpc.insecure_channel(
                f"{self.grpc_host}:{self.grpc_port}",
                options=[
                    ('grpc.max_receive_message_length', 50 * 1024 * 1024),
                ]
            )

            grpc.channel_ready_future(self.channel).result(timeout=10)
            self.trip_stub = vehicle_gateway_pb2_grpc.TripServiceStub(self.channel)

            print(" gRPC connected")
            return True
        except Exception as e:
            print(f"  gRPC failed: {e}")
            return False

    def login_vehicle(self):
        """Login vehicle"""
        if self.trip_stub is None:
            return
        try:
            response = self.trip_stub.VehicleLogin(
                vehicle_gateway_pb2.LoginRequest(
                    vin_number=self.vin_number,
                    trip_id=self.trip_id
                )
            )
            print(f" Login: {response.message}")
        except Exception as e:
            print(f"  Login failed: {e}")

    def start_trip(self, start_loc, end_loc):
        """Start trip via gRPC"""
        if self.trip_stub is None:
            return
        try:
            request = vehicle_gateway_pb2.StartTripRequest(
                vin_number=self.vin_number,
                trip_id=self.trip_id,
                pickup_location=vehicle_gateway_pb2.Location(
                    latitude=start_loc.x,
                    longitude=start_loc.y,
                    altitude=start_loc.z
                ),
                destination_location=vehicle_gateway_pb2.Location(
                    latitude=end_loc.x,
                    longitude=end_loc.y,
                    altitude=end_loc.z
                )
            )
            response = self.trip_stub.StartTrip(request)
            print(f" Trip started: {response.message}")
        except Exception as e:
            print(f"  Start trip failed: {e}")

    def drive_simple(self, destination_location):
        """
        Simple point-to-point navigation
        No map data required - works on Wine/Mac
        """
        self.destination = destination_location

        print(" Starting simple navigation (no map required)...")
        print(f"   Target: {destination_location}")

        # Create simple waypoints
        start = self.vehicle.get_location()
        waypoints = self._interpolate_waypoints(start, destination_location, step=5.0)

        print(f" Generated {len(waypoints)} waypoints")

        current_wp_idx = 0
        target_speed = 25.0  # km/h - slower for safety

        try:
            while current_wp_idx < len(waypoints):
                self.world.tick()

                current_loc = self.vehicle.get_location()
                target_loc = waypoints[current_wp_idx]

                distance = current_loc.distance(target_loc)

                # Reached waypoint?
                if distance < 4.0:
                    current_wp_idx += 1
                    if current_wp_idx >= len(waypoints):
                        print(" Destination reached!")
                        self._stop_vehicle()
                        break

                    remaining = len(waypoints) - current_wp_idx
                    print(f"✓ Waypoint {current_wp_idx}/{len(waypoints)} | {remaining} remaining")

                # Calculate control
                control = self._calculate_simple_control(target_loc, target_speed)
                self.vehicle.apply_control(control)

                # Update camera
                self._update_camera()

                time.sleep(0.05)

        except KeyboardInterrupt:
            print("\n Interrupted")
            self._stop_vehicle()

        return True

    def _interpolate_waypoints(self, start, end, step=5.0):
        """Create waypoints between start and end"""
        distance = start.distance(end)
        num_points = max(5, int(distance / step))

        waypoints = []
        for i in range(num_points + 1):
            t = i / num_points
            wp = carla.Location(
                x=start.x + t * (end.x - start.x),
                y=start.y + t * (end.y - start.y),
                z=start.z + t * (end.z - start.z)
            )
            waypoints.append(wp)

        return waypoints

    def _calculate_simple_control(self, target, target_speed):
        """Simple steering and throttle control"""
        v_transform = self.vehicle.get_transform()
        v_location = v_transform.location
        v_velocity = self.vehicle.get_velocity()

        # Calculate steering angle
        forward = v_transform.get_forward_vector()
        to_target = target - v_location

        # Normalize
        def norm(v):
            length = math.sqrt(v.x**2 + v.y**2)
            if length > 0.001:
                return carla.Vector3D(v.x/length, v.y/length, 0)
            return carla.Vector3D(1, 0, 0)

        forward_n = norm(forward)
        target_n = norm(to_target)

        # Angle calculation
        cross = forward_n.x * target_n.y - forward_n.y * target_n.x
        dot = forward_n.x * target_n.x + forward_n.y * target_n.y

        angle = math.atan2(cross, dot)
        steer = max(-1.0, min(1.0, angle * 2.0))

        # Speed control
        current_speed_ms = math.sqrt(v_velocity.x**2 + v_velocity.y**2)
        current_speed_kmh = current_speed_ms * 3.6

        speed_diff = target_speed - current_speed_kmh

        if speed_diff > 2.0:
            throttle = min(0.7, speed_diff / 15.0)
            brake = 0.0
        elif speed_diff < -2.0:
            throttle = 0.0
            brake = min(0.6, abs(speed_diff) / 15.0)
        else:
            throttle = 0.3
            brake = 0.0

        # Reduce throttle when turning
        if abs(steer) > 0.3:
            throttle *= 0.5

        return carla.VehicleControl(
            throttle=throttle,
            steer=steer,
            brake=brake,
            hand_brake=False,
            reverse=False
        )

    def _stop_vehicle(self):
        """Stop the vehicle"""
        for _ in range(10):
            self.vehicle.apply_control(carla.VehicleControl(
                throttle=0.0,
                steer=0.0,
                brake=1.0,
                hand_brake=True
            ))
            self.world.tick()
            time.sleep(0.05)

    def _update_camera(self):
        """Follow vehicle with spectator camera"""
        try:
            v_transform = self.vehicle.get_transform()
            spectator = self.world.get_spectator()

            cam_location = v_transform.location + carla.Location(z=30, x=-30)
            cam_rotation = carla.Rotation(pitch=-40, yaw=v_transform.rotation.yaw)

            spectator.set_transform(carla.Transform(cam_location, cam_rotation))
        except:
            pass  # Camera update is non-critical

    def cleanup(self):
        """Cleanup"""
        print("\n Cleaning up...")

        if self.vehicle:
            try:
                self.vehicle.destroy()
            except:
                pass

        if self.world:
            try:
                settings = self.world.get_settings()
                settings.synchronous_mode = False
                self.world.apply_settings(settings)
            except:
                pass

        if self.channel:
            try:
                self.channel.close()
            except:
                pass

        print("✅ Cleanup complete")


def main():
    """Main entry point"""

    print("="*60)
    print(" CARLA Autonomous Taxi")
    print("="*60)

    taxi = AutonomousTaxi(
        carla_host='host.docker.internal',
        carla_port=2000,
        grpc_host='autoware_bridge',
        grpc_port=50051
    )

    try:
        # Setup
        taxi.setup_carla()

        taxi.cleanup_world()
        print(" Waiting for world to settle...")
        time.sleep(2)

        # Let spawn_vehicle find a good spot automatically
        print(" Spawning vehicle (will try safe locations)...")

        if not taxi.spawn_vehicle():
            print(" Failed to spawn vehicle")
            return

        # Get actual spawn location
        actual_spawn = taxi.vehicle.get_location()

        # Verify vehicle is stable (not falling)
        print(" Verifying vehicle stability...")
        time.sleep(0.5)
        for i in range(20):
            taxi.world.tick()
            time.sleep(0.05)

        current_loc = taxi.vehicle.get_location()
        height_diff = abs(current_loc.z - actual_spawn.z)

        if height_diff > 2.0:
            print(f"  Vehicle appears unstable (fell {height_diff:.1f}m)")
            print(" Restarting with new spawn location...")
            taxi.vehicle.destroy()
            taxi.world.tick()

            # Try again
            if not taxi.spawn_vehicle():
                print(" Failed to spawn stable vehicle")
                return

            actual_spawn = taxi.vehicle.get_location()
            time.sleep(0.5)
            for i in range(20):
                taxi.world.tick()
                time.sleep(0.05)

        print(" Vehicle is stable and ready")

        # Set destination relative to spawn (70 meters diagonal)
        destination_location = carla.Location(
            x=actual_spawn.x + 50,
            y=actual_spawn.y + 50,
            z=actual_spawn.z
        )

        # Setup gRPC (optional)
        taxi.setup_grpc()
        taxi.login_vehicle()
        taxi.start_trip(actual_spawn, destination_location)

        # Display trip info
        print("\n" + "="*60)
        print(" AUTONOMOUS TAXI SERVICE")
        print("="*60)
        print(f" Start:       ({actual_spawn.x:.1f}, {actual_spawn.y:.1f}, {actual_spawn.z:.1f})")
        print(f" Destination: ({destination_location.x:.1f}, {destination_location.y:.1f}, {destination_location.z:.1f})")
        distance = actual_spawn.distance(destination_location)
        print(f" Distance:    {distance:.1f} meters")
        print("="*60 + "\n")

        # Wait a moment before starting to drive
        print("⏳ Starting engine...")
        for _ in range(20):
            taxi.world.tick()
            time.sleep(0.05)

        # Drive using simple navigation
        taxi.drive_simple(destination_location)

        print("\n Trip completed!")

        # Hold for 3 seconds
        print(" Holding final position...")
        for _ in range(60):
            taxi.world.tick()
            taxi._update_camera()
            time.sleep(0.05)

    except Exception as e:
        print(f"\n Error: {e}")
        import traceback
        traceback.print_exc()

    finally:
        taxi.cleanup()


if __name__ == "__main__":
    main()