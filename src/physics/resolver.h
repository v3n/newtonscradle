/* 
 * Copyright (c) 2015 Jonathan Howard
 * License: https://github.com/v3n/altertum/blob/master/LICENSE
 */

#pragma once

#include <vector>

#include "math/math_types.h"

using namespace altertum;

static const float g_restingThreshold = 4.0f; 
static const float g_positionWarming = 0.8f;
static const float g_positionDampen = 0.9f;
static const float g_frictionNormalMult = 5.0f;
static const size_t g_positionIterations = 6;
static const size_t g_velocityIterations = 4;

struct Collision
{
    float penetration;
    Vector3 normal;
    Vector3 tangent;
};

struct CollisionPair
{
    PhysicsBody * bodyA;
    PhysicsBody * bodyB;
    Collision collision;
    
    float seperation;
    Vector3 normal;

    static constexpr float slop = 0.05f;
};

inline void presolve_positions(std::vector<CollisionPair>& pairs)
{
    for ( size_t i = 0; i < pairs.size(); i++ )
    {
        PhysicsBody * BodyA = pairs[i].bodyA;
        PhysicsBody * BodyB = pairs[i].bodyB;

        BodyA->total_contacts++;
        BodyB->total_contacts++;
    }
}

inline void solve_positions(std::vector<CollisionPair>& pairs)
{
    Vector3 tempA, tempB, tempC, tempD;

    for ( size_t i = 0; i < pairs.size(); i++ )
    {
        CollisionPair * pair = &(pairs[i]);

        tempA = pair->bodyB->positionImpulse + pair->bodyB->position;
        tempB = pair->bodyB->position - pair->collision.penetration;
        tempC = pair->bodyA->positionImpulse + tempB;
        tempD = tempA - tempC;

        pair->seperation = vector3::dot(pair->collision.normal, tempD);
    }

    for ( size_t i = 0; i < pairs.size(); i++ )
    {
        CollisionPair pair = pairs[i];
        Collision collision = pair.collision;
        PhysicsBody * BodyA = pair.bodyA;
        PhysicsBody * BodyB = pair.bodyB;
        Vector3 normal = collision.normal;

        float positionImpulse = (pair.seperation - CollisionPair::slop);

        float cA = 0.04 / BodyA->total_contacts;
        float cB = 0.04 / BodyB->total_contacts;

        BodyA->positionImpulse += normal * BodyA->positionImpulse * cA;
        BodyB->positionImpulse += normal * BodyB->positionImpulse * cB;
    } 
}

inline void postsolve_positions(std::vector<PhysicsBody>& bodies)
{
    for ( size_t i = 0; i < bodies.size(); i++ )
    {
        PhysicsBody & body = bodies[i];
        
        if ( vector3::distance(body.impulse) > 0 )
        {
            body.lastPosition += body.positionImpulse;
        
            if ( vector3::dot(body.positionImpulse, body.velocity) < 0 )
            {
                body.positionImpulse.x = 0;
                body.positionImpulse.y = 0;
                body.positionImpulse.z = 0;
            }
            else
            {
                body.positionImpulse *= g_positionWarming;
            }
        }
    }
}

inline void presolve_velocities(std::vector<CollisionPair>& pairs)
{
    for ( size_t i = 0; i < pairs.size(); i++ )
    {
        CollisionPair & pair = pairs[i];
        Collision & collision = pair.collision;
        PhysicsBody * BodyA = pair.bodyA;
        PhysicsBody * BodyB = pair.bodyB;
    }
}

inline void solve_velocities(std::vector<CollisionPair>& pairs)
{
    for ( size_t i = 0; i < pairs.size(); i++ )
    {
        PhysicsBody * bodyA = pairs[i].bodyA;
        PhysicsBody * bodyB = pairs[i].bodyB;

        bodyA->velocity = bodyA->position - bodyA->lastPosition;
        bodyB->velocity = bodyB->position - bodyB->lastPosition;

        bodyA->angularVelocity = bodyA->angle - bodyA->lastAngle;
        bodyB->angularVelocity = bodyB->angle - bodyB->lastAngle;

        // for ( size_t j = 0; j < pair)
    }
}

