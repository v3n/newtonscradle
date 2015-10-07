/* 
 * Copyright (c) 2015 Jonathan Howard
 * License: https://github.com/v3n/altertum/blob/master/LICENSE
 */

#pragma once

#include "math/math_types.h"
#include "math/vector3.h"

using namespace altertum;

inline float clamp(float x, float a, float b)
{
    return x < a ? a : (x > b ? b : x);
}

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
    float angularVelocity;
    float angularSpeed;
    
    float torque;
    float speed;
    float mass;

    const float inertia = 99999.0f;
    const float restitution = 1.0f;
    const float friction = 0.0f;
    const float frictionAir = 0.001f;
    const float slop = 0.01f;

    /** 3D space */
    Vector3 position;
    Vector3 lastPosition;
    Vector3 force;
    Vector3 velocity;
    Vector3 acceleration;

    Vector3 impulse;
    Vector3 positionImpulse;
    Vector3 constraintImpulse;

    Vector3 constraintLoc;
    float   constraintLen;
    float   constraintAngle;
    float   constraintImpulse_angle;

    size_t total_contacts;

    inline void init_body(   Vector3& pos,
                                    float _mass, 
                                    float _angle, 
                                    float radius,
                                    float length
                                )
    {
        position     = pos;
        lastPosition = pos;

        constraintAngle = _angle;
        constraintImpulse_angle = 0.0f;
        constraintLoc = pos;
        constraintLen = length;

        collision.origin = pos;
        collision.origin.y -= length;
        collision.radius = radius;

        position     =  lastPosition = collision.origin;

        mass         = _mass;

        angle        = _angle;
        lastAngle    = _angle;

        angularVelocity = 0.0f;
        angularSpeed = 0.0f;

        speed        = 0.0f;
        torque       = 0.0f;
        speed        = 0.0f;

        force        = vector3::vector3( 0.0f, 0.0f, 0.0f );
        velocity     = vector3::vector3( 0.0f, 0.0f, 0.0f );
        acceleration = vector3::vector3( 0.0f, 0.0f, 0.0f );
        impulse      = vector3::vector3( 0.0f, 0.0f, 0.0f );
        positionImpulse = vector3::vector3( 0.0f, 0.0f, 0.0f );

        total_contacts = 0;
    }

    inline void solve_constraint()
    {
        float rot_angle = angle - constraintAngle;
        Vector3 point_a = vector3::vector3(  position.x * cos(rot_angle) - position.y * sin(rot_angle),
                                    position.x * sin(rot_angle) + position.y * cos(rot_angle),
                                    0.0f 
                                );
        Vector3 point_b = constraintLoc;

        float angleA = angle;
        float angleB = constraintAngle;

        Vector3 point_a_world = position + point_a;  
        Vector3 point_b_world = point_b;

        point_a_world = point_a_world + position;

        Vector3 delta = point_a_world - point_b_world;
        float current_length = vector3::distance(delta);

        /* Gayss-Siedel method */
        float difference = (current_length - constraintLen) / current_length;
        Vector3 normal   = delta / current_length;
        Vector3 force    = delta * (difference * 0.5);

        /* point body offset */
        Vector3 offset_a = point_a_world - position + force;

        /* update velocity */
        velocity = position - lastPosition;
        angularVelocity = angle - lastAngle;

        /* velocity for moving point */
        Vector3 velocity_point_a = velocity + (vector3::vector3(-offset_a.y, offset_a.x, 0.0f) * angularVelocity);
        float oAn = vector3::dot(offset_a, normal);
        float body_a_denom = (1 / mass) + (1 / inertia) * oAn * oAn;

        /* velocity for fixed point */
        Vector3 velocity_point_b = vector3::vector3( 0.0f, 0.0f, 0.0f );
        float body_b_denom = 0.0f; 

        Vector3 relative_velocity = velocity_point_b - velocity_point_a;
        float normal_impulse = vector3::dot(normal, relative_velocity);

        if ( normal_impulse > 0 ) normal_impulse = 0;
        Vector3 normal_velocity = normal * normal_impulse;

        /* torque */
        float torque = ((offset_a.x * normal_velocity.y) - (offset_a.y * normal_velocity.x)) * (1 / inertia);

        /* clamp torque to fix instability */
        torque = clamp(torque, -0.01, 0.01);

        /* track impulses for post-resolution */
        constraintImpulse -= force;
        constraintImpulse_angle += torque;

        /* apply forces */
        position -= force;
        angle += torque;
    }

    inline void postsolve_constraint()
    {
        impulse = constraintImpulse;

        impulse.x = 0.0f;
        impulse.y = 0.0f;
    }

    inline void applyGravity()
    {
        force.y += mass * 1.0f;
    }

    /**
     * update body
     * @param deltaTime  time difference
     * @param correction deltaTime / lastDeltaTime
     */
    inline void update(float deltaTime, float correction)
    {
        float deltaTimeSq = deltaTime * deltaTime;

        /* compute last step values */
        float frictionAir = 1 - this->frictionAir;
        Vector3 lastVelocity = position - lastPosition;

        /* Verlet integration for velocity */
        velocity = (lastVelocity * frictionAir * correction) + (force / mass) * deltaTimeSq;
        lastPosition = position;
        position += velocity;

        /* Verlet integration for angular velocity */
        angularVelocity = ((angle - lastAngle) * frictionAir * correction) + (torque / inertia) * deltaTimeSq;
        lastAngle = angle;
        angle += angularVelocity;

        /* track speed and acceleration */
        speed = vector3::distance(velocity);
        angularSpeed = abs(angularVelocity);

        /* update bounds */
        collision.update(velocity);
    }

    inline void clearForces()
    {
        force = vector3::vector3( 0.0f, 0.0f, 0.0f );
        torque = 0.0f;
        position.z = 0.0f;
    }
};
