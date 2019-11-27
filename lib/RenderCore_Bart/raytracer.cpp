#include "core_settings.h"

uint ScaleColor(uint c, int scale)
{
	unsigned int rb = (((c & 0xff00ff) * scale) >> 8) & 0xff00ff;
	unsigned int g = (((c & 0xff00) * scale) >> 8) & 0xff00;
	return rb + g;
}

bool Ray::IntersectsTriangle(const CoreTri& triangle, float& t) {
	float3 vertex0 = triangle.vertex0;
	float3 vertex1 = triangle.vertex1;
	float3 vertex2 = triangle.vertex2;
	float3 edge1, edge2, h, s, q;
	float a, f, u, v;
	edge1 = vertex1 - vertex0;
	edge2 = vertex2 - vertex0;
	h = cross(D, edge2);
	a = dot(edge1, h);
	if (a > -EPSILON && a < EPSILON)
		return false;    // This ray is parallel to this triangle.
	f = 1.0 / a;
	s = O - vertex0;
	u = f * dot(s, h);
	if (u < 0.0 || u > 1.0)
		return false;
	q = cross(s, edge1);
	v = f * dot(D, q);
	if (v < 0.0 || u + v > 1.0)
		return false;
	// At this stage we can compute t to find out where the intersection point is on the line.
	float tt = f * dot(edge2, q);
	if (tt > EPSILON && tt < 1 / EPSILON) // ray intersection
	{
		t = tt;
		return true;
	}
	else // This means that there is a line intersection but not a ray intersection.
		return false;
}

Scene::~Scene()
{
	for (auto mat : matList) delete mat;
}

// -----------------------------------------------------------
// static data for the ray tracer
// -----------------------------------------------------------
Scene RayTracer::scene;

