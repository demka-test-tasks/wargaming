try:
	import framework32 as framework
except:
	import framework64 as framework

import math


class Aircraft:
	"""Aircraft"""
	def __init__(self, ship_position):
		self._model = None
		self._position = Vector2(ship_position.x, ship_position.y)
		self._angle = 0.0
		self._is_airborne = False
		self._flight_time = 0.0
		self._refuel_time = 0.0
		self._speed = 0.0
		self._max_speed = Params.Aircraft.LINEAR_SPEED
		self._max_angular_speed = Params.Aircraft.ANGULAR_SPEED
		self._acceleration = Params.Aircraft.ACCELERATION
		self._goal = None
		self._is_refueling = False


	def take_off(self, ship_position):
		print(u"Taking off")
		if not self._is_airborne and not self._is_refueling:
			self._position = Vector2(ship_position.x, ship_position.y)
			self._angle = math.atan2(math.sin(self._angle), math.cos(self._angle))
			self._model = framework.createAircraftModel()
			self._is_airborne = True
			self._flight_time = 0.0
			self._speed = 0.0
			framework.placeModel(self._model, self._position.x, self._position.y, self._angle)

	def update(self, dt, ship_position):
		if self._is_airborne:
			self._flight_time += dt
			if self._flight_time >= Params.Aircraft.FLIGHT_DURATION:
				self._land(ship_position, dt)
			else:
				if self._speed < self._max_speed:
					self._speed += self._acceleration * dt
					self._speed = min(self._speed, self._max_speed)

				take_off_vector = Vector2(math.cos(self._angle), math.sin(self._angle))
				move_vector = take_off_vector * self._speed * dt
				self._position += move_vector

				framework.placeModel(self._model, self._position.x, self._position.y, self._angle)
				self._fly_towards_goal(dt)

		if self._is_refueling:
			self._refuel_time += dt
			if self._refuel_time >= Params.Aircraft.REFUEL_DURATION:
				self._is_refueling = False

	def _land(self, ship_position, dt):
		print("LANDING IS WORKING")
		self._goal = ship_position

		distance_to_ship = self._position.distance_to(ship_position)

		if distance_to_ship >= 0.5:
			direction = Vector2(ship_position.x - self._position.x, ship_position.y - self._position.y)
			distance = direction.length()

			direction = direction * (1.0 / distance)
			self._speed = max(0.1, self._speed - self._acceleration * dt)
			move_vector = direction * self._speed * dt
			self._position += move_vector
			self._angle = math.atan2(direction.y, direction.x)

			framework.placeModel(self._model, self._position.x, self._position.y, self._angle)
		else:
			self._is_airborne = False
			self._is_refueling = True
			self._refuel_time = 0.0

			framework.placeModel(self._model, self._position.x, self._position.y, self._angle)
			self.deinit()

	def _fly_towards_goal(self, dt):
		if self._goal:
			orbit_radius = 0.5

			direction = Vector2(self._goal.x - self._position.x, self._goal.y - self._position.y)
			distance = direction.length()

			if distance > orbit_radius:
				direction = direction * (1.0 / distance)
				move_vector = direction * self._max_speed * dt
				self._position += move_vector
				self._angle = math.atan2(direction.y, direction.x)
			else:
				if not hasattr(self, '_orbit_angle'):
					self._orbit_angle = math.atan2(direction.y, direction.x)

				self._orbit_angle += self._max_angular_speed * dt
				self._position.x = self._goal.x + orbit_radius * math.cos(self._orbit_angle)
				self._position.y = self._goal.y + orbit_radius * math.sin(self._orbit_angle)

			framework.placeModel(self._model, self._position.x, self._position.y, self._angle)

	def set_goal(self, goal):
		if self._is_airborne:
			self._goal = goal
			

	def deinit( self ):
		assert self._model
		framework.destroyModel( self._model )
		self._model = None

class AircraftManager:
	def __init__(self, ship):
		self._aircrafts = [Aircraft(ship._position) for _ in range(5)]
		self._next_aircraft_index = 0
		self._ship = ship

	def update(self, dt):
		current_ship_position = self._ship._position
		for aircraft in self._aircrafts:
			aircraft.update(dt, current_ship_position)

	def take_off_next_aircraft(self):
		'''
		Item to take off next aircraft
		'''
		aircraft = self._aircrafts[self._next_aircraft_index]
		if not aircraft._is_airborne:
			print("TAKE OFF", self._ship._position)
			aircraft.take_off(self._ship._position)
			self._next_aircraft_index = (self._next_aircraft_index + 1) % len(self._aircrafts)
			

	def set_goal_for_airborne(self, goal):
		for aircraft in self._aircrafts:
			aircraft.set_goal(goal)



#-------------------------------------------------------
#	game parameters
#-------------------------------------------------------

class Params:
	class Ship:
		LINEAR_SPEED = 0.5
		ANGULAR_SPEED = 0.5
	
	class Aircraft:
		LINEAR_SPEED = 2.0
		ANGULAR_SPEED = 2.5
		FLIGHT_DURATION = 10
		REFUEL_DURATION = 30
		ACCELERATION = 0.5


#-------------------------------------------------------
#	Basic Vector2 class
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
#	Simple ship logic
#-------------------------------------------------------

class Ship:
	
	def __init__( self ):
		self._model = None
		self._position = None
		self._angle = 0.0
		self._input = None
		# Added aircraft manager 
		self._aircraft_manager = None
	
	
	def init( self ):
		assert not self._model
		self._model = framework.createShipModel()
		self._position = Vector2()
		self._angle = 0.0
		self._aircraft_manager = AircraftManager(self)
		self._input = {
			framework.Keys.FORWARD : False,
			framework.Keys.BACKWARD : False,
			framework.Keys.LEFT : False,
			framework.Keys.RIGHT : False
		}
	
	def deinit( self ):
		assert self._model
		framework.destroyModel( self._model )
		self._model = None
	
	def update( self, dt ):
		linearSpeed = 0.0
		angularSpeed = 0.0
		
		if self._input[ framework.Keys.FORWARD ]:
			linearSpeed = Params.Ship.LINEAR_SPEED;
		elif self._input[ framework.Keys.BACKWARD ]:
			linearSpeed = -Params.Ship.LINEAR_SPEED;
		
		if self._input[ framework.Keys.LEFT ] and linearSpeed != 0.0:
			angularSpeed = Params.Ship.ANGULAR_SPEED;
		elif self._input[ framework.Keys.RIGHT ] and linearSpeed != 0.0:
			angularSpeed = -Params.Ship.ANGULAR_SPEED;
		
		self._angle = self._angle + angularSpeed * dt
		self._position = self._position + Vector2( math.cos( self._angle ), math.sin( self._angle ) ) * linearSpeed * dt
		framework.placeModel( self._model, self._position.x, self._position.y, self._angle )
		self._aircraft_manager.update(dt)
	
	def keyPressed( self, key ):
		self._input[ key ] = True
	
	def keyReleased( self, key ):
		self._input[ key ] = False
	
	def mouseClicked(self, x, y, isLeftButton):
		if isLeftButton:
			self._aircraft_manager.set_goal_for_airborne(Vector2(x, y))
			framework.placeGoalModel( x, y )
		else:
			self._aircraft_manager.take_off_next_aircraft()


#-------------------------------------------------------
#	game public interface
#-------------------------------------------------------

class Game:
	
	def __init__( self ):
		self._ship = Ship()
	
	def init( self ):
		self._ship.init()
	
	def deinit( self ):
		self._ship.deinit()
	
	def update( self, dt ):
		self._ship.update( dt )
	
	def keyPressed( self, key ):
		self._ship.keyPressed( key )
	
	def keyReleased( self, key ):
		self._ship.keyReleased( key )
	
	def mouseClicked( self, x, y, isLeftButton ):
		self._ship.mouseClicked( x, y, isLeftButton )


#-------------------------------------------------------
#	finally we can run our game!
#-------------------------------------------------------

framework.runGame( Game() )