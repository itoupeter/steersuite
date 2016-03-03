//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//


#include <random>
using std::default_random_engine;
using std::uniform_int_distribution;

#include "SteerLib.h"
#include "SimpleAgent.h"
#include "SimpleAIModule.h"

#include <iostream>
using std::endl;

#include <fstream>
using std::fstream;

#include <ios>
using std::ios;

#define ASSERT( x, y ) if( !x ) log << "\nassert failed " << y << endl;

/// @file SimpleAgent.cpp
/// @brief Implements the SimpleAgent class.

#define MAX_FORCE_MAGNITUDE 10000.0f
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
	if (vectorToGoal.lengthSquared() < _radius * _radius * 4.f ) {
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
	
	//---Pelechano's social force model (HiDAC)---
	//---[PAB07]Controlling Individual Agents in High-Density Crowd Simulation---

	//---log---
	static fstream log( "_log.txt", ios::out );

	//---social force---
	static Util::Vector f_to_1;
	static bool isStopping = false;
	static bool isWaiting = true;
	static int waitCountdown = 0;
	static float rect_width = 2.f;
	static float rect_height = 4.f;
	static float vision( 10.f );
	static float w_at( 10.f );
	static float w_wa( 1.f );
	static float w_ob( 1.f );
	static float w_ot( 5.f );
	static float lamda( 1.f );
	static float w_r_wa( 1000.f );
	static float w_r_ot( 1000.f );
	static default_random_engine generator;
	static uniform_int_distribution< int > distribution( 0, 100 );
	Util::Vector f_to( 0.f, 0.f, 0.f );
	Util::Vector f_at( 0.f, 0.f, 0.f );
	Util::Vector f_wa( 0.f, 0.f, 0.f );
	Util::Vector f_ob( 0.f, 0.f, 0.f );
	Util::Vector f_ot( 0.f, 0.f, 0.f );
	Util::Vector f_r( 0.f, 0.f, 0.f );
	Util::Vector f_r_ot( 0.f, 0.f, 0.f );
	Util::Vector f_r_ob( 0.f, 0.f, 0.f );
	Util::Vector f_r_wa( 0.f, 0.f, 0.f );

	//---force towards attractor---
	f_at = Util::normalize( vectorToGoal );

	//---computer forces---
	std::set< SteerLib::SpatialDatabaseItemPtr > neighbors;

	neighbors.clear();
	getSimulationEngine()->getSpatialDatabase()->getItemsInRange( neighbors,
		_position.x - vision, _position.x + vision,
		_position.z - vision, _position.z + vision,
		dynamic_cast< SteerLib::SpatialDatabaseItemPtr >( this ) );

	visionRectFilter( rect_width, rect_height, _position, _forward, neighbors );

	if( waitCountdown > 0 ){
		--waitCountdown;
		return;
	}

	for( auto neighbor : neighbors ){

		if( neighbor->isAgent() ){

			//---agent avoidance---
			SteerLib::AgentInterface *other = dynamic_cast< SteerLib::AgentInterface * >( neighbor );
			Util::Vector d_ji = position() - other->position();
			Util::Vector f_ot_ij;
			Util::Vector tmp = Util::cross( _forward, other->forward() );
			float r_ij = radius() + other->radius() - d_ji.length();
			
			if( tmp.y > 0 ){
				f_ot_ij = Util::cross( Util::Vector( 0, -1, 0 ), _forward );
			}else{
				f_ot_ij = Util::cross( Util::Vector( 0, 1, 0 ), _forward );
			}
			//Util::Vector v_i = velocity();
			//Util::Vector t_j = Util::normalize( Util::cross( Util::cross( d_ji, v_i ), d_ji ) );
			//float w_d_i = d_ji.length() - vision; w_d_i *= w_d_i;
			//float w_o_i = velocity() * other->velocity() >= 0.f ? 1.f : 5.f;
			//
			//if( w_o_i != 0.f ) f_ot += t_j * w_d_i * w_o_i;
			//f_ot += d_ji / d_ji.length() * 5.f * exp( r_ij / .1f ) * w_o_i;
			f_ot += f_ot_ij;
			log << "f_ot: " << f_ot << endl;

			//---agent repulsion---
			if( r_ij < 1e-6 ) continue;
			
			//---personal pushing ability epsilon should be added here---
			f_r_ot = Util::normalize( d_ji / d_ji.length() ) * r_ij;
			log << "f_r_ot: " << f_r_ot << endl;
						
		}else{

			//---obstacle avoidance---
			SteerLib::ObstacleInterface *other_obs = dynamic_cast< SteerLib::ObstacleInterface * >( neighbor );
			SteerLib::CircleObstacle *other_cir = dynamic_cast< SteerLib::CircleObstacle * >( other_obs );

			if( other_cir != NULL ){

				//---circular obstacle---
				Util::Vector d_ki = position() - other_cir->position();
				Util::Vector v_i = velocity();
				Util::Vector f_ob_ki = Util::normalize( Util::cross( Util::cross( d_ki, v_i ), d_ki ) );

				f_ob += f_ob_ki;
				log << "f_ob: " << f_ob << endl;

				//---obstacle repulsion---
				if( other_cir->computePenetration( position(), radius() ) < 1e-6 ) continue;
				//---personal pushing ability epsilon should be added here---
				f_r_ob = d_ki * ( radius() + other_cir->radius() - d_ki.length() ) / d_ki.length();
				log << "f_r_ob: " << f_r_ob << endl;

			}else{

				//---wall avoidance---
				Util::Vector n_w = calcWallNormal( other_obs );
				Util::Vector v_i = velocity();
				
				//---agent not moving---
				if( v_i.lengthSquared() < 1e-5 ) continue;

				//---leaving a wall, ignore---
				if( Util::dot( n_w, v_i ) >= 0.f ) continue;

				//---approaching a wall, steer away---
				Util::Vector f_wa_ki;
				if( Util::dot( n_w, v_i ) <= 0.f )
					f_wa_ki = Util::normalize( Util::cross( Util::Vector( 0, 1, 0 ), n_w ) );
				else
					f_wa_ki = Util::normalize( Util::cross( Util::Vector( 0, -1, 0 ), n_w ) );
				f_wa += f_wa_ki;
				log << "f_wa: " << f_wa << endl;

				//---wall repulsion---
				//---no contact with wall---
				if( other_obs->computePenetration( position(), radius() ) < 1e-6 ) continue;

				//---personal pushing ability epsilon should be added here---
				auto edge = calcWallPointsFromNormal( other_obs, n_w );
				auto min_dist = minimum_distance( edge.first, edge.second, position() );
				f_r_wa += Util::normalize( n_w ) * ( radius() - min_dist.first ) / min_dist.first;
				log << "f_r_wa: " << f_r_wa << endl;
			}
		}
	}

	//---net repulsion---
	f_r = f_r_ob + f_r_wa + f_r_ot * lamda;
	log << "f_r: " << f_r << endl;

	//---net avoidance---
	f_to = Util::Vector( 0.f, 0.f, 0.f )
		+ f_to_1
		+ f_at * w_at 
		+ f_wa * w_wa 
		+ f_r_wa * w_r_wa
		//+ f_ob * w_ob 
		+ f_ot * w_ot 
		+ f_r_ot * w_r_ot
		;
	log << "f_to: " << f_to << endl;

	//---save for next step---
	f_to_1 = f_to = f_to * .5f;

	//---calculate new position---
	//calNewPosition( f_to, f_r, dt );

	_doEulerStep( f_to, dt );
}

void SimpleAgent::visionRectFilter( float width, float height, Util::Point position, Util::Vector orientation, 
	std::set< SteerLib::SpatialDatabaseItemPtr > &neighbors ){
	
	//---agent not moving---
	if( orientation.lengthSquared() < 1e-5 ) return;

	Util::Vector v = Util::normalize( orientation );
	Util::Vector h = Util::cross( v, Util::Vector( 0, 1, 0 ) );

	for( auto neighbor = neighbors.begin(); neighbor != neighbors.end();  ){

		SteerLib::AgentInterface *other = dynamic_cast< SteerLib::AgentInterface * >( *neighbor );

		//---not agent---
		if( other == NULL ){
			++neighbor;
			continue;
		}

		Util::Vector error = other->position() - position;

		//---exclude agents behind---
		if( Util::dot( error, v ) <= 0.f ){
			neighbors.erase( neighbor++ );
			continue;
		}

		//---exclude agents out of rect---
		if( abs( Util::dot( error, v ) ) > height ) {
			neighbors.erase( neighbor++ );
			continue;
		}
		if( abs( Util::dot( error, h ) ) > width * .5f ){
			neighbors.erase( neighbor++ );
			continue;
		}

		++neighbor;
	}
}

void SimpleAgent::calNewPosition( Util::Vector &f_to, Util::Vector &f_r, float dt ){

	static float acceleration = 1.f;

	float v = _velocity.length() + acceleration * dt;
	if( v > MAX_SPEED ) v = MAX_SPEED;

	float a = f_r.length() > 0 ? 0 : 10.f;

	Util::Vector displacement = a * v * f_to * dt + f_r;

	_position = _position + displacement;
	_velocity = displacement;
	if( _velocity.lengthSquared() > 0 )
		_forward = Util::normalize( _velocity );
}

SteerLib::EngineInterface * SimpleAgent::getSimulationEngine()
{
	return gEngine;
}

void SimpleAgent::draw()
{
#ifdef ENABLE_GUI
	// if the agent is selected, do some annotations just for demonstration
	if (true || gEngine->isAgentSelected(this)) {
		Util::Ray ray;
		ray.initWithUnitInterval(_position, _forward);
		float t = 0.f;
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

void SimpleAgent::_doEulerStep(Util::Vector & steeringDecisionForce, float dt)
{
	// compute acceleration, _velocity, and newPosition by a simple Euler step
	const Util::Vector clippedForce = Util::clamp(steeringDecisionForce, MAX_FORCE_MAGNITUDE);
	steeringDecisionForce = clippedForce;
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

std::pair<Util::Point, Util::Point> SimpleAgent::calcWallPointsFromNormal(SteerLib::ObstacleInterface* obs, Util::Vector normal)
{
	Util::AxisAlignedBox box = obs->getBounds();
	if ( normal.z == 1)
	{
		return std::make_pair(Util::Point(box.xmin,0,box.zmax), Util::Point(box.xmax,0,box.zmax));
		// Ended here;
	}
	else if ( normal.z == -1 )
	{
		return std::make_pair(Util::Point(box.xmin,0,box.zmin), Util::Point(box.xmax,0,box.zmin));
	}
	else if ( normal.x == 1)
	{
		return std::make_pair(Util::Point(box.xmax,0,box.zmin), Util::Point(box.xmax,0,box.zmax));
	}
	else // normal.x == -1
	{
		return std::make_pair(Util::Point(box.xmin,0,box.zmin), Util::Point(box.xmin,0,box.zmax));
	}
}

std::pair<float, Util::Point> SimpleAgent::minimum_distance(Util::Point l1, Util::Point l2, Util::Point p)
{
	// Return minimum distance between line segment vw and point p
	float lSq = (l1 - l2).lengthSquared();  // i.e. |l2-l1|^2 -  avoid a sqrt
	if (lSq == 0.0)
		return std::make_pair((p - l2).length(),l1 );   // l1 == l2 case
														// Consider the line extending the segment, parameterized as l1 + t (l2 - l1).
														// We find projection of point p onto the line.
														// It falls where t = [(p-l1) . (l2-l1)] / |l2-l1|^2
	const float t = dot(p - l1, l2 - l1) / lSq;
	if (t < 0.0)
	{
		return std::make_pair((p - l1).length(), l1);       // Beyond the 'l1' end of the segment
	}
	else if (t > 1.0)
	{
		return std::make_pair((p - l2).length(), l2);  // Beyond the 'l2' end of the segment
	}
	const Util::Point projection = l1 + t * (l2 - l1);  // Projection falls on the segment
	return std::make_pair((p - projection).length(), projection) ;
}
