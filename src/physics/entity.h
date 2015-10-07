/* 
 * Copyright (c) 2015 Jonathan Howard
 * License: https://github.com/v3n/altertum/blob/master/LICENSE
 */

#pragma once

#include "math/math_types.h"
#include "math/vector3.h"

using namespace altertum;

struct BoundingSphere
{
    Vector3 origin;
    float radius;

    inline bool check_collision(BoundingSphere& s2)
    {
        BoundingSphere s1 = *this;

        Vector3 pos = s1.origin - s2.origin;
        float distance_sq = pos.x * pos.x + pos.y * pos.y;
        float min_dist_sq = s1.radius + s2.radius;

        return distance_sq <= (min_dist_sq * min_dist_sq);
    }

    inline void update(Vector3& vel)
    {
        origin += vel; 
    }
};

struct PhysicsBody
{
    /** collision body radius */
    BoundingSphere collision;

    /** 2D space */
    float angle;
    
    float lastAngle;
    float torque;
    float speed;
    float mass;

    float angularVelocity;
    float angularSpeed;

    /** 3D space */
    Vector3 position;
    Vector3 lastPosition;
    Vector3 force;
    Vector3 velocity;
    Vector3 acceleration;
    Vector3 impulse;
    Vector3 positionImpulse;

    Vector3 constraintLoc;
    float   constraintLen;

    size_t total_contacts;

    inline void init_body(   Vector3& pos,
                                    float _mass, 
                                    float angle, 
                                    float radius,
                                    float length
                                )
    {
        position     = pos;
        lastPosition = pos;

        constraintLoc = pos;
        constraintLen = length;

        collision.origin = pos;
        collision.origin.y -= length;
        collision.radius = radius;

        position     = 
        lastPosition = collision.origin;

        mass         = _mass;

        angle        = angle;
        lastAngle    = angle;

        speed        = 0.0f;

        force        = vector3::vector3( 0.0f, 0.0f, 0.0f );
        velocity     = vector3::vector3( 0.0f, 0.0f, 0.0f );
        acceleration = vector3::vector3( 0.0f, 0.0f, 0.0f );
        impulse      = vector3::vector3( 0.0f, 0.0f, 0.0f );
        positionImpulse = vector3::vector3( 0.0f, 0.0f, 0.0f );

        total_contacts = 0;
    }

    inline void applyGravity()
    {
        force.y += mass * 0.001f;
    }

    /**
     * update body
     * @param deltaTime  time difference
     * @param correction deltaTime / lastDeltaTime
     */
    inline void update(float deltaTime, float correction)
    {
        float deltaTimeSq = pow(deltaTime, 2);

        /* compute last step values */
        float frictionAir = 1 - frictionAir;
        Vector3 lastVelocity = position - lastPosition;

        /* Verlet integration for velocity */
        velocity = (lastVelocity * frictionAir * correction) + (force / mass) * deltaTimeSq;
        lastPosition = position;
        position += velocity;

        /* Verlet integration for angular velocity */
        angularVelocity = ((angle - lastAngle) * frictionAir * correction);
        lastAngle = angle;
        angle += angularVelocity;

        /* track speed and acceleration */
        speed = vector3::distance(velocity);
        angularSpeed = abs(angularVelocity);

        /* update bounds */
        collision.update(velocity);
    }
};
