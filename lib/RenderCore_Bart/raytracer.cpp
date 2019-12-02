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

float3 RayTracer::Color(float3 O, float3 D, uint depth) {
	uint d = depth - 1;
	Ray ray = Ray(O, D);

	float smallest_t = FLT_MAX;
	float3 color = make_float3(0, 0, 0);

	for (Mesh& mesh : scene.meshes) for (int i = 0; i < mesh.vcount / 3; i++)
	{
		float t;
		if (ray.IntersectsTriangle(mesh.triangles[i], t) && t < smallest_t) {
			smallest_t = t;
			color = scene.matList[mesh.triangles[i].material]->diffuse;
		}
	}

	// Reached maximum depth, do lighting
	if (d <= 0) { color = Illumination(color, ray.point(smallest_t)); }

	// The ray hit the skydome
	if (smallest_t == FLT_MAX) {
		float3 Dn = normalize(D);
		float u = 1 + atan2(Dn.x, -Dn.z) / PI;
		float v = acos(Dn.y) / PI;

		unsigned long long width = round(u * scene.skyHeight);
		unsigned long long height = round(v * scene.skyHeight);

		return scene.skyDome[min(height * scene.skyHeight * 2 + width, scene.skyDome.size() - 1)];
	}

	return color;
}

float3 RayTracer::Illumination(float3 color, float3 O) {
	// Point lights
	for (int i = 0; i < scene.pointLights.size(); i++) {
		float3 direction = scene.pointLights[i]->position - O;
		Ray shadow_ray = Ray(O, direction);

		float t_light = length(fabs(direction));
		float t;
		for (Mesh& mesh : scene.meshes) for (int i = 0; i < mesh.vcount / 3; i++) {
			if (shadow_ray.IntersectsTriangle(mesh.triangles[i], t) && t < t_light) {
				return make_float3(0, 0, 0);
			}
		}
	}

	return color;
}

Scene::~Scene()
{
	for (auto mat : matList) delete mat;
}

// -----------------------------------------------------------
// static data for the ray tracer
// -----------------------------------------------------------
Scene RayTracer::scene;

