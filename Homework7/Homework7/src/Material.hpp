//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_MATERIAL_H
#define RAYTRACING_MATERIAL_H

#include "Vector.hpp"

enum MaterialType { DIFFUSE, MICROFACET };

class Material
{
private:
    // Compute reflection direction
    Vector3f reflect(const Vector3f& I, const Vector3f& N) const
    {
        return I - 2 * dotProduct(I, N) * N;
    }

    // Compute refraction direction using Snell's law
    //
    // We need to handle with care the two possible situations:
    //
    //    - When the ray is inside the object
    //
    //    - When the ray is outside.
    //
    // If the ray is outside, you need to make cosi positive cosi = -N.I
    //
    // If the ray is inside, you need to invert the refractive indices and negate the normal N
    Vector3f refract(const Vector3f& I, const Vector3f& N, const float& ior) const
    {
        float cosi = clamp(-1, 1, dotProduct(I, N));
        float etai = 1, etat = ior;
        Vector3f n = N;
        if (cosi < 0) { cosi = -cosi; }
        else
        {
            std::swap(etai, etat);
            n = -N;
        }
        float eta = etai / etat;
        float k = 1 - eta * eta * (1 - cosi * cosi);
        return k < 0 ? 0 : eta * I + (eta * cosi - sqrtf(k)) * n;
    }

    // Compute Fresnel equation
    //
    // \param I is the incident view direction
    // Attention! The direction of incidence is from the light source to the shading point, not outward!
    //
    // \param N is the normal at the intersection point
    //
    // \param ior is the material refractive index
    //
    // \param[out] kr is the amount of light reflected
    void fresnel(const Vector3f& I, const Vector3f& N, const float& ior, float& kr) const
    {
        float cosi = clamp(-1, 1, dotProduct(I, N));
        float etai = 1, etat = ior;
        if (cosi > 0) { std::swap(etai, etat); }
        // Compute sini using Snell's law
        float sint = etai / etat * sqrtf(std::max(0.f, 1 - cosi * cosi));
        // Total internal reflection
        if (sint >= 1)
        {
            kr = 1;
        }
        else
        {
            float cost = sqrtf(std::max(0.f, 1 - sint * sint));
            cosi = fabsf(cosi);
            float Rs = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
            float Rp = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
            kr = (Rs * Rs + Rp * Rp) / 2;
        }
        // As a consequence of the conservation of energy, transmittance is given by:
        // kt = 1 - kr;
    }
    
    /* Convert the local direction to world space
     * This method is a transformation under the condition of no explict tangent direction definition.
     * The different choices of T and B will indeed lead to different world space direction results.
     * But in the scenarios such as isotropic sampling, the final rendering result will not be affected.
     */
    Vector3f toWorld(const Vector3f& a, const Vector3f& N)
    {
        Vector3f T, B;
        if (std::fabs(N.x) > std::fabs(N.y))
        {
            float invLen = 1.0f / std::sqrt(N.x * N.x + N.z * N.z);
            B = Vector3f(N.z * invLen, 0.0f, -N.x * invLen);
        }
        else
        {
            float invLen = 1.0f / std::sqrt(N.y * N.y + N.z * N.z);
            B = Vector3f(0.0f, N.z * invLen, -N.y * invLen);
        }
        T = crossProduct(B, N);
        return a.x * T + a.y * B + a.z * N;
    }

public:
    MaterialType m_type;
    //Vector3f m_color;
    Vector3f m_emission;
    // Refractive index
    float ior;
    // Material parameters for Phong model
    Vector3f Kd, Ks;
    float specularExponent;
    //Texture tex;
    // roughness for microsurface model
    float roughness = 1.0f;
    inline Material(MaterialType t = DIFFUSE, Vector3f e = Vector3f(0, 0, 0));
    inline MaterialType getType();
    //inline Vector3f getColor();
    inline Vector3f getColorAt(double u, double v);
    inline Vector3f getEmission();
    inline bool hasEmission();

    // sample a ray by Material properties in hemisphere
    inline Vector3f sample(const Vector3f& wi, const Vector3f& N);
    // given a ray, calculate the PdF of this ray
    inline float pdf(const Vector3f& wi, const Vector3f& wo, const Vector3f& N);
    // BRDF, given a ray, calculate the contribution of this ray
    inline Vector3f eval(const Vector3f& wi, const Vector3f& wo, const Vector3f& N);
};

Material::Material(MaterialType t, Vector3f e)
{
    m_type = t;
    //m_color = c;
    m_emission = e;
}

MaterialType Material::getType() { return m_type; }
///Vector3f Material::getColor(){return m_color;}
Vector3f Material::getEmission() { return m_emission; }

bool Material::hasEmission()
{
    if (m_emission.norm() > EPSILON) return true;
    else return false;
}

Vector3f Material::getColorAt(double u, double v)
{
    return Vector3f();
}

// sample a ray randomly by Material properties in hemisphere
Vector3f Material::sample(const Vector3f& wi, const Vector3f& N)
{
    switch (m_type)
    {
    case DIFFUSE:
    case MICROFACET:
        {
            // Uniform sample on the hemisphere
            // x_1 controls the height (z_axis), x_2 controls the azimuth phi (rotation around the z-axis)
            float x_1 = get_random_float(), x_2 = get_random_float();
            // map x_1 from [0,1] to [-1,1] on the entire sphere, then take the absolute value to obtain z on the upper hemisphere
            float z = std::fabs(1.0f - 2.0f * x_1);
            // Calculate the projection radius (r) on the XY plane and the azimuth angle (phi)
            float r = std::sqrt(1.0f - z * z), phi = 2 * M_PI * x_2;
            // Generate the local ray direction
            Vector3f localRay(r * std::cos(phi), r * std::sin(phi), z);
            // Convert the local ray direction to world space
            return toWorld(localRay, N);

            break;
        }
    }
}

float Material::pdf(const Vector3f& wi, const Vector3f& wo, const Vector3f& N)
{
    switch (m_type)
    {
    case DIFFUSE:
    case MICROFACET:
        {
            // uniform sample probability 1 / (2 * PI)
            if (dotProduct(wo, N) > 0.0f)
                return 0.5f / M_PI;
            else
                return 0.0f;
            break;
        }
    }
}

Vector3f Material::eval(const Vector3f& wi, const Vector3f& wo, const Vector3f& N)
{
    switch (m_type)
    {
    case DIFFUSE:
        {
            // calculate the contribution of diffuse model
            // If the reflection direction is in the positive hemisphere, then return diffuse brdf = albedo(kd)/ PI
            // albedo(kd) is the percentage of incoming light that is reflected by the surface, which can be understood as the color of the object
            float cosalpha = dotProduct(N, wo);
            if (cosalpha > 0.0f)
            {
                Vector3f diffuse = Kd / M_PI;
                return diffuse;
            }
            else
            {
                return Vector3f(0.0f);
            }
            break;
        }
        case MICROFACET:
        {
            // calculate the contribution of microsurface model (Cook-Torrance)
            // Make sure the eye ray can reach the plane
            if (dotProduct(wo,N) > 0.0f)
            {
                float F,G,D;
                // Calculate the fresnel term, that is, what proportion of power was reflected
                fresnel(wi,N,ior,F);
                // Calculate the geometry masking/shadowing term
                auto G_Function = [&](const float& roughness, const Vector3f& wi, const Vector3f& wo, const Vector3f& N)
                {
                    float A_wi, A_wo;
                    float tan_wi = crossProduct(wi, N).norm() / dotProduct(wi, N);
                    A_wi = (-1 + sqrt(1+ roughness* roughness * pow(tan_wi,2))) * 0.5f;
                    
                    float tan_wo = crossProduct(wo, N).norm() / dotProduct(wo, N);
                    A_wo = (-1 + sqrt(1+ roughness* roughness * pow(tan_wo,2))) * 0.5f;
                    float divisor = 1 + A_wi + A_wo;
                    if (divisor < EPSILON)
                        return 1.f;
                    else
                        return 1.f / divisor;
                };
                G = G_Function(roughness, wi, wo, N);
                
                // Calculate the NDF(Normal Distribution Function)
                auto D_function = [&](const float& roughness, const Vector3f& h, const Vector3f& N)
                {
                    float cos_sita = dotProduct(h, N);
                    // 这里使用了一个变换，sin^2 = 1 - cos^2
                    float divisor = (M_PI * pow(1.0 + cos_sita * cos_sita * (roughness * roughness - 1), 2));
                    if (divisor < 0.001)
                        return 1.f;
                    else 
                        return (roughness * roughness) / divisor;
                };
                Vector3f h = normalize(-wi + wo);
                D = D_function(roughness, h, N);
                
                // Calculate Cook-Torrance
                Vector3f brdf_diffuse = Kd / M_PI;
                Vector3f diffuse = (Vector3f(1.f - F)) * brdf_diffuse;
                
                Vector3f specular;
                float divisor = 4.f*dotProduct(N, -wi) * dotProduct(N, wo);
                if (divisor < 0.001)
                    specular = Vector3f(1.f);
                else 
                    specular = F * G * D / divisor;
                return diffuse + specular;
            }
            else
            {
                return Vector3f(0.0f);
            }
            break;
        }
    }
}

#endif //RAYTRACING_MATERIAL_H
