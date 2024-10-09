try:
    import framework32 as framework
except ImportError:
    import framework64 as framework

import math

# -------------------------------------------------------
#   Game Parameters
# -------------------------------------------------------

class Params:
    class Ship:
        LINEAR_SPEED = 0.5
        ANGULAR_SPEED = 0.5

    class Aircraft:
        LINEAR_SPEED = 2.0
        ANGULAR_SPEED = 2.5
        FLIGHT_DURATION = 10
        REFUEL_DURATION = 10
        ACCELERATION = 0.5
        MIN_SPEED = 0.3  # to prevent aircraft stop due to landing

# -------------------------------------------------------
#   Vector2 Class
# -------------------------------------------------------

class Vector2:
    def __init__(self, x=0.0, y=0.0):
        if isinstance(x, Vector2):
            self.x, self.y = x.x, x.y
        else:
            self.x, self.y = x, y

    def __add__(self, other):
        return Vector2(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Vector2(self.x - other.x, self.y - other.y)

    def __mul__(self, coef):
        return Vector2(self.x * coef, self.y * coef)

    __rmul__ = __mul__

    def length(self):
        return math.hypot(self.x, self.y)

    def __repr__(self):
        return "{},{}".format(self.x, self.y) #

# -------------------------------------------------------
#   Aircraft State and Orbit State Constants
# -------------------------------------------------------

class AircraftState:
    ON_GROUND = 0
    TAKING_OFF = 1
    IN_FLIGHT = 2
    LANDING = 3
    REFUELING = 4

class OrbitState:
    NONE = 0
    ENTERING = 1
    IN_ORBIT = 2
    EXITING = 3

# -------------------------------------------------------
#   Aircraft Class
# -------------------------------------------------------

class Aircraft:
    """Aircraft class representing each aircraft in the game."""

    def __init__(self, ship_position):
        self._model = None
        self._position = Vector2(ship_position)
        self._angle = 0.0
        self._state = AircraftState.ON_GROUND
        self._orbit_state = OrbitState.NONE
        self._flight_time = 0.0
        self._refuel_time = 0.0
        self._speed = 0.0
        self._max_speed = Params.Aircraft.LINEAR_SPEED
        self._max_angular_speed = Params.Aircraft.ANGULAR_SPEED
        self._acceleration = Params.Aircraft.ACCELERATION
        self._goal = None
        self._landing_approach_point = None

    def take_off(self, ship_position, ship_angle):
        """Initiate the takeoff sequence from the ship."""
        if self._state == AircraftState.ON_GROUND:
            print "Taking off"
            self._position = Vector2(ship_position)
            self._angle = ship_angle
            self._model = framework.createAircraftModel()
            self._state = AircraftState.TAKING_OFF
            self._flight_time = 0.0
            self._speed = 0.0
            framework.placeModel(self._model, self._position.x, self._position.y, self._angle)

    def update(self, dt, ship_position, ship_angle):
        """Update the aircraft's state."""
        if self._state == AircraftState.TAKING_OFF:
            self._update_taking_off(dt)
        elif self._state == AircraftState.IN_FLIGHT:
            self._update_in_flight(dt, ship_position, ship_angle)
        elif self._state == AircraftState.LANDING:
            self._update_landing(dt, ship_position, ship_angle)
        elif self._state == AircraftState.REFUELING:
            self._update_refueling(dt)

    def _update_taking_off(self, dt):
        """Handle the takeoff sequence."""
        self._speed += self._acceleration * dt
        if self._speed >= self._max_speed:
            self._speed = self._max_speed
            self._state = AircraftState.IN_FLIGHT
            self._flight_time = 0.0
        self._move(dt)

    def _update_in_flight(self, dt, ship_position, ship_angle):
        """Handle normal flight behavior."""
        self._flight_time += dt
        if self._flight_time >= Params.Aircraft.FLIGHT_DURATION:
            self._initiate_landing(ship_position, ship_angle)
        if self._goal:
            self._fly_towards_goal(dt)
        else:
            self._move(dt)

    def _update_landing(self, dt, ship_position, ship_angle):
        """Handle the landing procedure."""
        if self._landing_approach_point:
            self._approach_landing_point(dt)
        else:
            self._final_landing(dt, ship_position, ship_angle)

    def _update_refueling(self, dt):
        """Handle refueling."""
        self._refuel_time += dt
        if self._refuel_time >= Params.Aircraft.REFUEL_DURATION:
            self._state = AircraftState.ON_GROUND

    def _move(self, dt):
        """Move the aircraft forward."""
        direction = Vector2(math.cos(self._angle), math.sin(self._angle))
        move_vector = direction * self._speed * dt
        self._position += move_vector
        framework.placeModel(self._model, self._position.x, self._position.y, self._angle)

    def _initiate_landing(self, ship_position, ship_angle):
        """Initiate the landing sequence towards the ship's stern."""
        print "Initiating landing sequence"
        self._state = AircraftState.LANDING
        self._orbit_state = OrbitState.NONE
        self._goal = None
        landing_distance = 5.0
        back_direction = Vector2(-math.cos(ship_angle), -math.sin(ship_angle))
        self._landing_approach_point = ship_position + back_direction * landing_distance

    def _approach_landing_point(self, dt):
        """Approach the landing point behind the ship."""
        to_approach = self._landing_approach_point - self._position
        distance = to_approach.length()
        desired_angle = math.atan2(to_approach.y, to_approach.x)
        self._angle = self._adjust_angle_towards(self._angle, desired_angle, dt)
        self._speed = max(self._max_speed * 0.5, Params.Aircraft.MIN_SPEED)
        self._move(dt)
        if distance <= 1.0:
            self._landing_approach_point = None

    def _final_landing(self, dt, ship_position, ship_angle):
        """Perform the final landing approach onto the ship."""
        to_ship = ship_position - self._position
        distance_to_ship = to_ship.length()
        desired_angle_to_ship = math.atan2(to_ship.y, to_ship.x)
        blend_factor = 0.1
        desired_angle = self._interpolate_angles(desired_angle_to_ship, ship_angle, blend_factor)
        self._angle = self._adjust_angle_towards(self._angle, desired_angle, dt)
        self._speed = max(self._max_speed * 0.5, Params.Aircraft.MIN_SPEED)
        self._move(dt)
        if distance_to_ship <= 0.1:
            print "Landing complete"
            self._position = Vector2(ship_position)
            self._angle = ship_angle
            framework.placeModel(self._model, self._position.x, self._position.y, self._angle)
            self._state = AircraftState.REFUELING
            self._refuel_time = 0.0
            self.deinit()

    def _fly_towards_goal(self, dt):
        """Handle the flight behavior towards the goal, including orbiting."""
        orbit_radius = 0.5
        approach_radius = orbit_radius + 1.0
        to_goal = self._goal - self._position
        distance = to_goal.length()

        if self._orbit_state == OrbitState.EXITING:
            self._exit_orbit(dt, to_goal, distance)
        elif self._orbit_state == OrbitState.NONE and distance <= approach_radius:
            self._orbit_state = OrbitState.ENTERING
        if self._orbit_state == OrbitState.NONE:
            self._fly_straight(dt, to_goal)
        elif self._orbit_state == OrbitState.ENTERING:
            self._enter_orbit(dt, to_goal, orbit_radius)
        elif self._orbit_state == OrbitState.IN_ORBIT:
            self._orbit(dt, orbit_radius)

    def _fly_straight(self, dt, to_goal):
        """Fly straight towards the goal."""
        desired_angle = math.atan2(to_goal.y, to_goal.x)
        self._angle = self._adjust_angle_towards(self._angle, desired_angle, dt)
        self._move(dt)

    def _enter_orbit(self, dt, to_goal, orbit_radius):
        """Enter orbit around the goal."""
        angle_to_goal = math.atan2(to_goal.y, to_goal.x)
        orbit_entry_angle = angle_to_goal + math.pi / 2
        orbit_entry_point = self._goal + Vector2(math.cos(orbit_entry_angle), math.sin(orbit_entry_angle)) * orbit_radius
        to_entry_point = orbit_entry_point - self._position
        desired_angle = math.atan2(to_entry_point.y, to_entry_point.x)
        self._angle = self._adjust_angle_towards(self._angle, desired_angle, dt)
        self._move(dt)
        if to_entry_point.length() <= 0.5:
            self._orbit_state = OrbitState.IN_ORBIT

    def _orbit(self, dt, orbit_radius):
        """Orbit around the goal."""
        angle_to_aircraft = math.atan2(self._position.y - self._goal.y, self._position.x - self._goal.x)
        orbit_direction = angle_to_aircraft + math.pi / 2
        self._angle = self._adjust_angle_towards(self._angle, orbit_direction, dt)
        self._move(dt)
        to_aircraft = self._position - self._goal
        current_distance = to_aircraft.length()
        if abs(current_distance - orbit_radius) > 0.01:
            to_aircraft = to_aircraft * (orbit_radius / current_distance)
            self._position = self._goal + to_aircraft

    def _exit_orbit(self, dt, to_goal, distance):
        """Exit orbit and fly towards the goal."""
        desired_angle = math.atan2(to_goal.y, to_goal.x)
        self._angle = self._adjust_angle_towards(self._angle, desired_angle, dt)
        self._speed = max(self._max_speed, Params.Aircraft.MIN_SPEED)
        self._move(dt)
        if distance <= 0.5:
            self._orbit_state = OrbitState.NONE
            self._goal = None

    def _normalize_angle(self, angle):
        """Normalize angle to be within [-pi, pi]."""
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def _adjust_angle_towards(self, current_angle, target_angle, dt):
        """Adjust the current angle towards the target angle."""
        angle_diff = self._normalize_angle(target_angle - current_angle)
        max_angle_change = self._max_angular_speed * dt
        angle_change = max(-max_angle_change, min(max_angle_change, angle_diff))
        return self._normalize_angle(current_angle + angle_change)

    def _interpolate_angles(self, angle1, angle2, t):
        """Interpolate between two angles."""
        angle1 = self._normalize_angle(angle1)
        angle2 = self._normalize_angle(angle2)
        diff = self._normalize_angle(angle2 - angle1)
        return self._normalize_angle(angle1 + diff * t)

    def set_goal(self, goal):
        """Set a new goal for the aircraft."""
        self._goal = goal
        self._orbit_state = OrbitState.NONE

    def deinit(self):
        """Deinitialize the aircraft's model."""
        if self._model:
            framework.destroyModel(self._model)
            self._model = None

# -------------------------------------------------------
#   AircraftManager Class
# -------------------------------------------------------

class AircraftManager:
    """Manages multiple aircraft."""

    def __init__(self, ship):
        self._aircrafts = [Aircraft(ship._position) for _ in range(5)]
        self._next_aircraft_index = 0
        self._ship = ship
        self._current_goal = None

    def update(self, dt, ship_position, ship_angle):
        """Update all aircraft. Mediator parrern"""
        for aircraft in self._aircrafts:
            aircraft.update(dt, ship_position, ship_angle) # 

    def take_off_next_aircraft(self):
        """Initiate takeoff for the next available aircraft."""
        aircraft = self._aircrafts[self._next_aircraft_index]
        if aircraft._state == AircraftState.ON_GROUND:
            print "Aircraft take off from position", self._ship._position
            aircraft.take_off(self._ship._position, self._ship._angle)
            if self._current_goal is not None:
                aircraft.set_goal(self._current_goal)
            self._next_aircraft_index = (self._next_aircraft_index + 1) % len(self._aircrafts)

    def set_goal_for_airborne(self, goal):
        """Set a new goal for all airborne aircraft."""
        self._current_goal = goal
        for aircraft in self._aircrafts:
            if aircraft._state in [AircraftState.IN_FLIGHT, AircraftState.TAKING_OFF]:
                aircraft.set_goal(goal)
        framework.placeGoalModel(goal.x, goal.y)

# -------------------------------------------------------
#   Ship Class
# -------------------------------------------------------

class Ship:
    """Represents the player's ship."""

    def __init__(self):
        self._model = None
        self._position = Vector2()
        self._angle = 0.0
        self._input = {}
        self._aircraft_manager = None

    def init(self):
        """Initialize the ship and its components."""
        self._model = framework.createShipModel()
        self._position = Vector2()
        self._angle = 0.0
        self._aircraft_manager = AircraftManager(self)
        self._input = {
            framework.Keys.FORWARD: False,
            framework.Keys.BACKWARD: False,
            framework.Keys.LEFT: False,
            framework.Keys.RIGHT: False
        }

    def deinit(self):
        """Deinitialize the ship's model."""
        if self._model:
            framework.destroyModel(self._model)
            self._model = None

    def update(self, dt):
        """Update the ship's state and handle input."""
        linear_speed = 0.0
        angular_speed = 0.0

        if self._input.get(framework.Keys.FORWARD):
            linear_speed = Params.Ship.LINEAR_SPEED
        elif self._input.get(framework.Keys.BACKWARD):
            linear_speed = -Params.Ship.LINEAR_SPEED

        if self._input.get(framework.Keys.LEFT):
            angular_speed = Params.Ship.ANGULAR_SPEED
        elif self._input.get(framework.Keys.RIGHT):
            angular_speed = -Params.Ship.ANGULAR_SPEED

        self._angle += angular_speed * dt
        direction = Vector2(math.cos(self._angle), math.sin(self._angle))
        self._position += direction * linear_speed * dt
        framework.placeModel(self._model, self._position.x, self._position.y, self._angle)
        self._aircraft_manager.update(dt, self._position, self._angle)

    def keyPressed(self, key):
        """Handle key press events."""
        self._input[key] = True

    def keyReleased(self, key):
        """Handle key release events."""
        self._input[key] = False

    def mouseClicked(self, x, y, isLeftButton):
        """Handle mouse click events."""
        if isLeftButton:
            self._aircraft_manager.set_goal_for_airborne(Vector2(x, y))
        else:
            self._aircraft_manager.take_off_next_aircraft()

# -------------------------------------------------------
#   Game Class
# -------------------------------------------------------

class Game:
    """Main game class."""

    def __init__(self):
        self._ship = Ship()

    def init(self):
        """Initialize the game."""
        self._ship.init()

    def deinit(self):
        """Deinitialize the game."""
        self._ship.deinit()

    def update(self, dt):
        """Update the game state."""
        self._ship.update(dt)

    def keyPressed(self, key):
        """Handle key press events."""
        self._ship.keyPressed(key)

    def keyReleased(self, key):
        """Handle key release events."""
        self._ship.keyReleased(key)

    def mouseClicked(self, x, y, isLeftButton):
        """Handle mouse click events."""
        self._ship.mouseClicked(x, y, isLeftButton)

# -------------------------------------------------------
#   Run the Game
# -------------------------------------------------------

framework.runGame(Game())