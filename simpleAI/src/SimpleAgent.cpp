//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//


#include "SteerLib.h"
#include "SimpleAgent.h"
#include "SimpleAIModule.h"

/// @file SimpleAgent.cpp
/// @brief Implements the SimpleAgent class.

#define MAX_FORCE_MAGNITUDE 10.0f
#define MAX_SPEED 1.3f
#define AGENT_MASS 1.0f

SimpleAgent::SimpleAgent()
{
	_enabled = false;
}

SimpleAgent::~SimpleAgent()
{
	if (_enabled) {
		Util::AxisAlignedBox bounds(_position.x-_radius, _position.x+_radius, 0.0f, 0.0f, _position.z-_radius, _position.z+_radius);
		gSpatialDatabase->removeObject( this, bounds);
	}
}

void SimpleAgent::disable()
{
	Util::AxisAlignedBox bounds(_position.x-_radius, _position.x+_radius, 0.0f, 0.0f, _position.z-_radius, _position.z+_radius);
	gSpatialDatabase->removeObject( this, bounds);
	_enabled = false;
}

void SimpleAgent::reset(const SteerLib::AgentInitialConditions & initialConditions, SteerLib::EngineInterface * engineInfo)
{
	// compute the "old" bounding box of the agent before it is reset.  its OK that it will be invalid if the agent was previously disabled
	// because the value is not used in that case.
	Util::AxisAlignedBox oldBounds(_position.x-_radius, _position.x+_radius, 0.0f, 0.0f, _position.z-_radius, _position.z+_radius);

	// initialize the agent based on the initial conditions
	_position = initialConditions.position;
	_forward = initialConditions.direction;
	_radius = initialConditions.radius;
	_velocity = initialConditions.speed * Util::normalize(initialConditions.direction);

	// compute the "new" bounding box of the agent
	Util::AxisAlignedBox newBounds(_position.x-_radius, _position.x+_radius, 0.0f, 0.0f, _position.z-_radius, _position.z+_radius);

	if (!_enabled) {
		// if the agent was not enabled, then it does not already exist in the database, so add it.
		gSpatialDatabase->addObject( this, newBounds);
	}
	else {
		// if the agent was enabled, then the agent already existed in the database, so update it instead of adding it.
		gSpatialDatabase->updateObject( this, oldBounds, newBounds);
	}

	_enabled = true;

	if (initialConditions.goals.size() == 0) {
		throw Util::GenericException("No goals were specified!\n");
	}

	// iterate over the sequence of goals specified by the initial conditions.
	for (unsigned int i=0; i<initialConditions.goals.size(); i++) {
		if (initialConditions.goals[i].goalType == SteerLib::GOAL_TYPE_SEEK_STATIC_TARGET) {
			_goalQueue.push(initialConditions.goals[i]);
			if (initialConditions.goals[i].targetIsRandom) {
				// if the goal is random, we must randomly generate the goal.
				SteerLib::AgentGoalInfo _goal;
				_goal.targetLocation = gSpatialDatabase->randomPositionWithoutCollisions(1.0f, true);
				_goalQueue.push(_goal);
			}
		}
		else {
			throw Util::GenericException("Unsupported goal type; SimpleAgent only supports GOAL_TYPE_SEEK_STATIC_TARGET.");
		}
	}

	assert(_forward.length()!=0.0f);
	assert(_goalQueue.size() != 0);
	assert(_radius != 0.0f);
}

void SimpleAgent::updateAI(float timeStamp, float dt, unsigned int frameNumber)
{

	// for this function, we assume that all goals are of type GOAL_TYPE_SEEK_STATIC_TARGET.
	// the error check for this was performed in reset().
	Util::AutomaticFunctionProfiler profileThisFunction( &SimpleAIGlobals::gPhaseProfilers->aiProfiler );

	Util::Vector vectorToGoal = _goalQueue.front().targetLocation - _position;

	// it is up to the agent to decide what it means to have "accomplished" or "completed" a goal.
	// for the simple AI, if the agent's distance to its goal is less than its radius, then the agent has reached the goal.
	if (vectorToGoal.lengthSquared() < _radius * _radius) {
		_goalQueue.pop();
		if (_goalQueue.size() != 0) {
			// in this case, there are still more goals, so start steering to the next goal.
			vectorToGoal = _goalQueue.front().targetLocation - _position;
		}
		else {
			// in this case, there are no more goals, so disable the agent and remove it from the spatial database.
			disable();
			return;
		}
	}

	//---social force---
	static Util::Vector f_to_1;
	static float vision( 5.f );

	static float w_at( 1.f );
	static float w_wa( 1.f );
	static float w_ob( 1.f );
	static float w_ot( 1.f );
	Util::Vector f_at;
	Util::Vector f_wa;
	Util::Vector f_ob;
	Util::Vector f_ot;

	//---force towards attractor---
	f_at = vectorToGoal;

	//---force to avoid wall, obstacle, other agents---
	std::set< SteerLib::SpatialDatabaseItemPtr > neighbors;

	neighbors.clear();
	getSimulationEngine()->getSpatialDatabase()->getItemsInRange( neighbors,
		_position.x - ( this->_radius + vision ),
		_position.x + ( this->_radius + vision ),
		_position.z - ( this->_radius + vision ),
		_position.z + ( this->_radius + vision ),
		dynamic_cast< SteerLib::SpatialDatabaseItemPtr >( this ) );

	for( auto neighbor : neighbors ){
		
		if( neighbor->isAgent() ){

			//---other agent---
			SteerLib::AgentInterface *other = dynamic_cast< SteerLib::AgentInterface * >( neighbor );
		//	Util::Vector d_ji = position() - other->position();
		//	Util::Vector v_i = velocity();
		//	Util::Vector t_j = Util::normalize( Util::cross( Util::cross( d_ji, t_j ), d_ji ) );
		//	float w_d_i = d_ji.lengthSquared() - vision; w_d_i *= w_d_i;
		//	float w_o_i = velocity() * other->velocity() > 0 ? 1.2f : 2.4f;
		//	
		//	f_ot += t_j * w_d_i * w_o_i;
			
		//}else{
		//	
		//	//---obstacle---
		//	SteerLib::ObstacleInterface *other = dynamic_cast< SteerLib::ObstacleInterface * >( neighbor );
		//	SteerLib::CircleObstacle *other_cir = dynamic_cast< SteerLib::CircleObstacle * >( other );

		//	if( other_cir != NULL ){
		//		
		//		//---circular obstacle---
		//		Util::Vector d_ki = position() - other_cir->position();
		//		Util::Vector v_i = velocity();
		//		Util::Vector f_ob_ki = Util::normalize( Util::cross( Util::cross( d_ki, v_i ), d_ki ) );

		//		f_ob += f_ob_ki * w_ob;
		//	}else{
		//		
		//		//---wall---
		//		Util::Vector n_w = calcWallNormal( other );
		//		Util::Vector v_i = velocity();
		//		Util::Vector f_wa_ki = Util::normalize( Util::cross( Util::cross( n_w, v_i ), n_w ) );

		//		f_wa += f_wa_ki * w_wa;
		//	}
		}
	}

	//---net force---
	Util::Vector f_to;
	f_to = Util::Vector( 0.f, 0.f, 0.f )
		//+ f_to_1 
		+ f_at * w_at 
		//+ f_wa * w_wa 
		//+ f_ob * w_ob 
		+ f_ot * w_ot
		;

	// use the vectorToGoal as a force for the agent to steer towards its goal.
	// the euler integration step will clamp this vector to a reasonable value, if needed.
	// also, the Euler step updates the agent's position in the spatial database.
	_doEulerStep( f_to, dt);
	f_to_1 = vectorToGoal;
}

SteerLib::EngineInterface * SimpleAgent::getSimulationEngine()
{
	return gEngine;
}

void SimpleAgent::draw()
{
#ifdef ENABLE_GUI
	// if the agent is selected, do some annotations just for demonstration
	if (gEngine->isAgentSelected(this)) {
		Util::Ray ray;
		ray.initWithUnitInterval(_position, _forward);
		float t = 0.0f;
		SteerLib::SpatialDatabaseItem * objectFound;
		Util::DrawLib::drawLine(ray.pos, ray.eval(1.0f));
		if (gSpatialDatabase->trace(ray, t, objectFound, this, false)) {
			Util::DrawLib::drawAgentDisc(_position, _forward, _radius, Util::gBlue);
		}
		else {
			Util::DrawLib::drawAgentDisc(_position, _forward, _radius);
		}
	}
	else {
		Util::DrawLib::drawAgentDisc(_position, _forward, _radius, Util::gGray40);
	}
	if (_goalQueue.front().goalType == SteerLib::GOAL_TYPE_SEEK_STATIC_TARGET) {
		Util::DrawLib::drawFlag(_goalQueue.front().targetLocation);
	}
#endif
}


void SimpleAgent::_doEulerStep(const Util::Vector & steeringDecisionForce, float dt)
{
	// compute acceleration, _velocity, and newPosition by a simple Euler step
	const Util::Vector clippedForce = Util::clamp(steeringDecisionForce, MAX_FORCE_MAGNITUDE);
	Util::Vector acceleration = (clippedForce / AGENT_MASS);
	_velocity = _velocity + (dt*acceleration);
	_velocity = clamp(_velocity, MAX_SPEED);  // clamp _velocity to the max speed
	const Util::Point newPosition = _position + (dt*_velocity);

	// For this simple agent, we just make the orientation point along the agent's current velocity.
	if (_velocity.lengthSquared() != 0.0f) {
		_forward = normalize(_velocity);
	}

	// update the database with the new agent's setup
	Util::AxisAlignedBox oldBounds(_position.x - _radius, _position.x + _radius, 0.0f, 0.0f, _position.z - _radius, _position.z + _radius);
	Util::AxisAlignedBox newBounds(newPosition.x - _radius, newPosition.x + _radius, 0.0f, 0.0f, newPosition.z - _radius, newPosition.z + _radius);
	gSpatialDatabase->updateObject( this, oldBounds, newBounds);

	_position = newPosition;
}

Util::Vector SimpleAgent::calcWallNormal(SteerLib::ObstacleInterface* obs)
{
	Util::AxisAlignedBox box = obs->getBounds();
	if ( position().x > box.xmax )
	{
		if ( position().z > box.zmax)
		{
			if (fabs(position().z - box.zmax) >
				fabs(position().x - box.xmax))
			{
				return Util::Vector(0, 0, 1);
			}
			else
			{
				return Util::Vector(1, 0, 0);
			}

		}
		else if ( position().z < box.zmin )
		{
			if (fabs(position().z - box.zmin) >
				fabs(position().x - box.xmax))
			{
				return Util::Vector(0, 0, -1);
			}
			else
			{
				return Util::Vector(1, 0, 0);
			}

		}
		else
		{ // in between zmin and zmax
			return Util::Vector(1, 0, 0);
		}

	}
	else if ( position().x < box.xmin )
	{
		if ( position().z > box.zmax )
		{
			if (fabs(position().z - box.zmax) >
				fabs(position().x - box.xmin))
			{
				return Util::Vector(0, 0, 1);
			}
			else
			{
				return Util::Vector(-1, 0, 0);
			}

		}
		else if ( position().z < box.zmin )
		{
			if (fabs(position().z - box.zmin) >
				fabs(position().x - box.xmin))
			{
				return Util::Vector(0, 0, -1);
			}
			else
			{
				return Util::Vector(-1, 0, 0);
			}

		}
		else
		{ // in between zmin and zmax
			return Util::Vector(-1, 0, 0);
		}
	}
	else // between xmin and xmax
	{
		if ( position().z > box.zmax )
		{
			return Util::Vector(0, 0, 1);
		}
		else if ( position().z < box.zmin)
		{
			return Util::Vector(0, 0, -1);
		}
		else
		{ // What do we do if the agent is inside the wall?? Lazy Normal
			return calcObsNormal( obs );
		}
	}

}

/**
* Treats Obstacles as a circle and calculates normal
*/
Util::Vector SimpleAgent::calcObsNormal(SteerLib::ObstacleInterface* obs)
{
	Util::AxisAlignedBox box = obs->getBounds();
	Util::Point obs_centre = Util::Point((box.xmax+box.xmin)/2, (box.ymax+box.ymin)/2,
		(box.zmax+box.zmin)/2);
	return normalize(position() - obs_centre);
}