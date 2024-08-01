/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/

#include <nori/warp.h>
#include <nori/vector.h>
#include <nori/frame.h>

NORI_NAMESPACE_BEGIN

Point2f Warp::squareToUniformSquare(const Point2f &sample) {
    return sample;
}

float Warp::squareToUniformSquarePdf(const Point2f &sample) {
    return ((sample.array() >= 0).all() && (sample.array() <= 1).all()) ? 1.0f : 0.0f;
}

float tentPdf(float t)
{
    return t >= -1 && t <= 1 ? 1 - std::abs(t) : 0;    
}

float uniform2Tent(float t)
{
    return t < 0.5f ? std::sqrt(2.0f * t) - 1 : 1 - sqrt(2.0f - 2.0f * t);
}

Point2f Warp::squareToTent(const Point2f &sample)
{
    return Point2f(uniform2Tent(sample.x()), uniform2Tent(sample.y()));
}

float Warp::squareToTentPdf(const Point2f &p)
{
    return tentPdf(p.x()) * tentPdf(p.y());
}

Point2f Warp::squareToUniformDisk(const Point2f &sample)
{
    float r = std::sqrt(sample.x());
    float theta = sample.y() * (float)M_PI * 2;
    return Point2f(r * cos(theta), r * sin(theta));
}

float Warp::squareToUniformDiskPdf(const Point2f &p)
{
    return std::sqrt(p.x() * p.x() + p.y() * p.y()) <= 1.0f ? INV_PI : 0.0f;
}

Vector3f Warp::squareToUniformSphere(const Point2f &sample)
{
    float z = 1 - 2.0f * sample.x();
    float sintheta = std::sqrt(1 - z * z);
    float phi = 2.0f * M_PI * sample.y();
    return Vector3f(sintheta * std::cos(phi), sintheta * std::sin(phi), z);
}

float Warp::squareToUniformSpherePdf(const Vector3f &v)
{
    return 0.25f * INV_PI;
}

Vector3f Warp::squareToUniformHemisphere(const Point2f &sample)
{
    float z = sample.x();
    float sintheta = std::sqrt(1 - z * z);
    float phi = 2.0f * M_PI * sample.y();
    return Vector3f(sintheta * std::cos(phi), sintheta * std::sin(phi), z);
}

float Warp::squareToUniformHemispherePdf(const Vector3f &v)
{
    return 0.5f * INV_PI;
}

Vector3f Warp::squareToCosineHemisphere(const Point2f &sample)
{
    Point2f bottom = squareToUniformDisk(sample);
    float x = bottom.x();
    float y = bottom.y();
    return Vector3f(x, y, std::sqrt(1 - x * x - y * y));
}

float Warp::squareToCosineHemispherePdf(const Vector3f &v)
{
    return v.z() < 0 ? 0 : v.z() * INV_PI;
}

Vector3f Warp::squareToBeckmann(const Point2f &sample, float alpha) {
    throw NoriException("Warp::squareToBeckmann() is not yet implemented!");
}

float Warp::squareToBeckmannPdf(const Vector3f &m, float alpha) {
    throw NoriException("Warp::squareToBeckmannPdf() is not yet implemented!");
}

NORI_NAMESPACE_END
