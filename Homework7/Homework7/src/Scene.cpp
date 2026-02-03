//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}
// Sample a light source in the scene, return the intersection information and the PDF value
void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    // Get a random number in [0, emit_area_sum)
    float shadePoint = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            // When the random number is less than the sum area, the corresponding light source is selected
            if (shadePoint <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

//Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    // Get the shade point
    Vector3f hitColor = this->backgroundColor;
    // Determine whether the ray coming out from the pixel hit any object to obtain the shading point
    Intersection shadePoint = intersect(ray);
    if (shadePoint.happened)
    {
        Vector3f p = shadePoint.coords;// Position of shading point
        Material* pm = shadePoint.m;
        Vector3f wo = normalize(-ray.direction);// View direction
        Vector3f N = normalize(shadePoint.normal);// Normal vector of shading point
        Vector3f lightDir(0), lightIndir(0);
        
        // Sample direct illumination
        Intersection lightInsct;
        float pdfLight;
        sampleLight(lightInsct, pdfLight);
        Vector3f x = lightInsct.coords;// The coordinates of the sampling point on the light source surface
        Vector3f ws = normalize(x - p);// The direction from shading point to light sampling point
        Vector3f NN = normalize(lightInsct.normal);// The normal vector of the light sampling point
        Vector3f emit = lightInsct.emit;
        float distancePtoX = (x-p).norm();// The distance between shading point and light sampling point
        // Avoid self-intersection between the shadow ray and the object itself due to floating-point number precision issue
        // Raise the coordinates a little towards/backwards the normal direction
        Vector3f pOffset;
        if (dotProduct(wo, N) > 0)
            pOffset= p + EPSILON * N;
        else
            pOffset= p - EPSILON * N;
        // Determine whether there is any block between two points
        Intersection blockInsct = intersect(Ray(pOffset, ws));
        float cosine = dotProduct(ws, N);
        // cosine >EPSILON, avoid back direct lighting
        if (abs(distancePtoX - blockInsct.distance) < 0.01f && cosine >EPSILON)
        {
            lightDir = emit * pm->eval(ws, wo, N) * cosine * dotProduct(-ws, NN) / (distancePtoX * distancePtoX * pdfLight);
        }
        
        // Sample indirect illumination
        float ksi = get_random_float();
        if (ksi < RussianRoulette)
        {
            Vector3f wi = normalize(pm->sample(wo,N));
            // Determine whether the hit object is an emitting object
            Ray bounceRay = Ray(pOffset, wi);
            Intersection bounceInsct = intersect(bounceRay);
            if (bounceInsct.happened && !bounceInsct.obj->hasEmit())
            {
                float pdf = pm->pdf(wi, wo, N);
                if (pdf>EPSILON)
					lightIndir = castRay(bounceRay, depth + 1) * pm->eval(wi, wo, N) * dotProduct(wi, N) / (pm->pdf(wi,wo,N) * RussianRoulette);
            }
        }
        hitColor = pm->getEmission() + lightDir + lightIndir;
    }
    return hitColor;
}
