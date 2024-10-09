try:
    import framework32 as framework
except:
    import framework64 as framework

import math

#-------------------------------------------------------
#   Game Parameters
#-------------------------------------------------------

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

#-------------------------------------------------------
#   Vector2 Class
#-------------------------------------------------------

class Vector2:
    def __init__(self, *args):
        if not args:
            self.x = self.y = 0.0
        elif len(args) == 1:
            self.x, self.y = args[0].x, args[0].y
        else:
            self.x, self.y = args

    def __add__(self, other):
        return Vector2(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Vector2(self.x - other.x, self.y - other.y)

    def __mul__(self, coef):
        return Vector2(self.x * coef, self.y * coef)

    def length(self):
        return math.sqrt(self.x ** 2 + self.y ** 2)

    def distance_to(self, other):
        return (self - other).length()

    def __repr__(self):
        return "{0},{1}".format(self.x, self.y)

#-------------------------------------------------------
#   Aircraft Class
#-------------------------------------------------------

class Aircraft:
    """Aircraft class representing each aircraft in the game."""
    def __init__(self, ship_position):
        self._model = None
        self._position = Vector2(ship_position.x, ship_position.y)
        self._angle = 0.0
        self._is_airborne = False
        self._is_taking_off = False  # Takeoff flag
        self._flight_time = 0.0
        self._refuel_time = 0.0
        self._speed = 0.0
        self._max_speed = Params.Aircraft.LINEAR_SPEED
        self._max_angular_speed = Params.Aircraft.ANGULAR_SPEED
        self._acceleration = Params.Aircraft.ACCELERATION
        self._goal = None
        self._is_refueling = False
        self._is_entering_orbit = False  # State for entering orbit
        self._is_in_orbit = False        # State when in orbit
        self._is_exiting_orbit = False   # State for exiting orbit

    def take_off(self, ship_position, ship_angle):
        """Initiate the takeoff sequence from the ship."""
        print("Taking off")
        if not self._is_airborne and not self._is_refueling:
            self._position = Vector2(ship_position.x, ship_position.y)
            self._angle = ship_angle  # Set aircraft's angle to ship's angle
            self._model = framework.createAircraftModel()
            self._is_airborne = True
            self._is_taking_off = True  # Aircraft starts taking off
            self._flight_time = 0.0
            self._speed = 0.0
            framework.placeModel(self._model, self._position.x, self._position.y, self._angle)

    def update(self, dt, ship_position):
        """Update the aircraft's state."""
        if self._is_airborne:
            if self._is_taking_off:
                # Accelerate along the deck
                self._speed += self._acceleration * dt
                if self._speed >= self._max_speed:
                    self._speed = self._max_speed
                    self._is_taking_off = False  # Takeoff is complete
                    self._flight_time = 0.0  # Start counting flight time
                # Move in the direction of the ship's angle
                direction = Vector2(math.cos(self._angle), math.sin(self._angle))
                move_vector = direction * self._speed * dt
                self._position += move_vector
                framework.placeModel(self._model, self._position.x, self._position.y, self._angle)
            else:
                # Normal flight behavior
                self._flight_time += dt
                if self._flight_time >= Params.Aircraft.FLIGHT_DURATION:
                    # Flight duration is over, start landing sequence
                    self._is_exiting_orbit = True
                    self._goal = ship_position  # Set goal to ship for landing
                    self._is_entering_orbit = False
                    self._is_in_orbit = False
                if self._goal:
                    self._fly_towards_goal(dt)
                else:
                    # Continue moving forward
                    direction = Vector2(math.cos(self._angle), math.sin(self._angle))
                    move_vector = direction * self._speed * dt
                    self._position += move_vector
                    framework.placeModel(self._model, self._position.x, self._position.y, self._angle)
        if self._is_refueling:
            self._refuel_time += dt
            if self._refuel_time >= Params.Aircraft.REFUEL_DURATION:
                self._is_refueling = False

    def _land(self, ship_position, dt):
        """Handle the landing sequence back to the ship."""
        print("Landing")
        self._goal = ship_position
        self._is_entering_orbit = False
        self._is_in_orbit = False
        self._is_exiting_orbit = True

    def _normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def _adjust_angle_towards(self, current_angle, target_angle, max_angular_speed, dt):
        """Adjust the current angle towards the target angle, considering max angular speed."""
        angle_diff = self._normalize_angle(target_angle - current_angle)
        max_angle_change = max_angular_speed * dt
        if abs(angle_diff) < max_angle_change:
            return target_angle
        else:
            return current_angle + max_angle_change * (1 if angle_diff > 0 else -1)

    def _fly_towards_goal(self, dt):
        """Handle the flight behavior towards the goal, including orbiting."""
        if self._goal:
            orbit_radius = 0.5
            approach_radius = orbit_radius + 1.0  # Distance to start entering orbit

            to_goal = self._goal - self._position
            distance = to_goal.length()

            if self._is_exiting_orbit:
                # Smoothly exit orbit and approach the goal
                desired_angle = math.atan2(to_goal.y, to_goal.x)
                self._angle = self._adjust_angle_towards(self._angle, desired_angle, self._max_angular_speed, dt)

                # Move in the direction of the current angle
                direction = Vector2(math.cos(self._angle), math.sin(self._angle))
                move_vector = direction * self._max_speed * dt
                self._position += move_vector

                # Check if the aircraft has reached the goal (ship)
                if distance <= 0.1:
                    self._is_airborne = False
                    self._is_refueling = True
                    self._refuel_time = 0.0

                    framework.placeModel(self._model, self._position.x, self._position.y, self._angle)
                    self.deinit()
                else:
                    framework.placeModel(self._model, self._position.x, self._position.y, self._angle)
            else:
                if not self._is_entering_orbit and not self._is_in_orbit and distance <= approach_radius:
                    self._is_entering_orbit = True  # Start entering orbit

                if not self._is_entering_orbit and not self._is_in_orbit:
                    # Fly straight towards the goal
                    desired_angle = math.atan2(to_goal.y, to_goal.x)
                    self._angle = self._adjust_angle_towards(self._angle, desired_angle, self._max_angular_speed, dt)

                    direction = Vector2(math.cos(self._angle), math.sin(self._angle))
                    move_vector = direction * self._max_speed * dt
                    self._position += move_vector
                elif self._is_entering_orbit:
                    # Smoothly enter orbit
                    angle_to_goal = math.atan2(to_goal.y, to_goal.x)
                    orbit_entry_angle = angle_to_goal + math.pi / 2  # Adjust for tangential entry
                    orbit_entry_point = self._goal + Vector2(math.cos(orbit_entry_angle), math.sin(orbit_entry_angle)) * orbit_radius

                    to_entry_point = orbit_entry_point - self._position
                    desired_angle = math.atan2(to_entry_point.y, to_entry_point.x)
                    self._angle = self._adjust_angle_towards(self._angle, desired_angle, self._max_angular_speed, dt)

                    direction = Vector2(math.cos(self._angle), math.sin(self._angle))
                    move_vector = direction * self._max_speed * dt
                    self._position += move_vector

                    # Check if the aircraft has reached the orbit entry point
                    if to_entry_point.length() <= 0.1:
                        self._is_entering_orbit = False
                        self._is_in_orbit = True
                elif self._is_in_orbit:
                    # Orbit around the goal
                    angle_to_aircraft = math.atan2(self._position.y - self._goal.y, self._position.x - self._goal.x)

                    # Tangential direction for orbit
                    orbit_direction = angle_to_aircraft + math.pi / 2  # Use -math.pi / 2 for opposite direction

                    self._angle = self._adjust_angle_towards(self._angle, orbit_direction, self._max_angular_speed, dt)

                    # Move in the direction of the current angle
                    direction = Vector2(math.cos(self._angle), math.sin(self._angle))
                    move_vector = direction * self._max_speed * dt
                    self._position += move_vector

                    # Maintain constant orbit radius
                    to_aircraft = self._position - self._goal
                    current_distance = to_aircraft.length()
                    if abs(current_distance - orbit_radius) > 0.01:
                        to_aircraft = to_aircraft * (orbit_radius / current_distance)
                        self._position = self._goal + to_aircraft

                framework.placeModel(self._model, self._position.x, self._position.y, self._angle)

    def set_goal(self, goal):
        """Set a new goal for the aircraft."""
        self._goal = goal
        self._is_entering_orbit = False
        self._is_in_orbit = False
        self._is_exiting_orbit = False  # Reset exiting orbit state

    def deinit(self):
        """Deinitialize the aircraft's model."""
        assert self._model
        framework.destroyModel(self._model)
        self._model = None

#-------------------------------------------------------
#   AircraftManager Class
#-------------------------------------------------------

class AircraftManager:
    """Manages multiple aircraft."""
    def __init__(self, ship):
        self._aircrafts = [Aircraft(ship._position) for _ in range(5)]
        self._next_aircraft_index = 0
        self._ship = ship

    def update(self, dt):
        """Update all aircraft."""
        current_ship_position = self._ship._position
        for aircraft in self._aircrafts:
            aircraft.update(dt, current_ship_position)

    def take_off_next_aircraft(self):
        """Initiate takeoff for the next available aircraft."""
        aircraft = self._aircrafts[self._next_aircraft_index]
        if not aircraft._is_airborne:
            print("Aircraft taking off from position", self._ship._position)
            aircraft.take_off(self._ship._position, self._ship._angle)  # Pass ship's angle
            self._next_aircraft_index = (self._next_aircraft_index + 1) % len(self._aircrafts)

    def set_goal_for_airborne(self, goal):
        """Set a new goal for all aircraft."""
        for aircraft in self._aircrafts:
            aircraft.set_goal(goal)

#-------------------------------------------------------
#   Ship Class
#-------------------------------------------------------

class Ship:
    """Represents the player's ship."""
    def __init__(self):
        self._model = None
        self._position = None
        self._angle = 0.0
        self._input = None
        self._aircraft_manager = None

    def init(self):
        """Initialize the ship and its components."""
        assert not self._model
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
        assert self._model
        framework.destroyModel(self._model)
        self._model = None

    def update(self, dt):
        """Update the ship's state and handle input."""
        linearSpeed = 0.0
        angularSpeed = 0.0

        if self._input[framework.Keys.FORWARD]:
            linearSpeed = Params.Ship.LINEAR_SPEED
        elif self._input[framework.Keys.BACKWARD]:
            linearSpeed = -Params.Ship.LINEAR_SPEED

        if self._input[framework.Keys.LEFT] and linearSpeed != 0.0:
            angularSpeed = Params.Ship.ANGULAR_SPEED
        elif self._input[framework.Keys.RIGHT] and linearSpeed != 0.0:
            angularSpeed = -Params.Ship.ANGULAR_SPEED

        # Update position and angle
        self._angle = self._angle + angularSpeed * dt
        self._position = self._position + Vector2(math.cos(self._angle), math.sin(self._angle)) * linearSpeed * dt
        framework.placeModel(self._model, self._position.x, self._position.y, self._angle)

        # Update aircraft manager
        self._aircraft_manager.update(dt)

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
            framework.placeGoalModel(x, y)
        else:
            self._aircraft_manager.take_off_next_aircraft()

#-------------------------------------------------------
#   Game Class
#-------------------------------------------------------

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

#-------------------------------------------------------
#   Run the Game
#-------------------------------------------------------

framework.runGame(Game())