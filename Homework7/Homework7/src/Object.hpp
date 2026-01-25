//
// Created by LEI XU on 5/13/19.
//
#pragma once
#ifndef RAYTRACING_OBJECT_H
#define RAYTRACING_OBJECT_H

#include "Vector.hpp"
#include "global.hpp"
#include "Bounds3.hpp"
#include "Ray.hpp"
#include "Intersection.hpp"
// The base class of an object
class Object
{
public:
    Object() {}
    virtual ~Object() {} // For the polymorphism
    // Determine whether the ray intersects with the object
    virtual bool intersect(const Ray& ray) = 0;
    // Determine whether the ray intersects with the object, and return the nearest intersection distance tnear and the index of the intersected triangle
    virtual bool intersect(const Ray& ray, float& tnear, uint32_t& index) const = 0;
    // Cast the ray, and get the detailed intersection information
    virtual Intersection getIntersection(Ray _ray) = 0;
    // Get the surface properties of the intersection point
    virtual void getSurfaceProperties(const Vector3f &, const Vector3f &, const uint32_t &, const Vector2f &, Vector3f &, Vector2f &) const = 0;
    // Evaluate the diffuse color at the intersection point
    virtual Vector3f evalDiffuseColor(const Vector2f &) const =0;
    // Get the bounding box of the object
    virtual Bounds3 getBounds()=0;
    // Get the surface area of the object
    virtual float getArea()=0;
    // Sample a point on the surface of the object, return the position and normal at that point, as well as the PDF value
    virtual void Sample(Intersection &pos, float &pdf)=0;
    // Determine whether the object is emissive
    virtual bool hasEmit()=0;
};



#endif //RAYTRACING_OBJECT_H
