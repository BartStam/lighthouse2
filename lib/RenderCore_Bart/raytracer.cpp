#include "core_settings.h"
#include <iostream>

uint ScaleColor(uint c, int scale)
{
	unsigned int rb = (((c & 0xff00ff) * scale) >> 8) & 0xff00ff;
	unsigned int g = (((c & 0xff00) * scale) >> 8) & 0xff00;
	return rb + g;
}

void Accumulator::Rebuild(int width, int height) {
	frame.clear();
	frame.resize(width * height, make_float3(0, 0, 0));
	frame_count = 0;
	w = width;
}

float3 Triangle::RandomPoint() {
	float a = Rand(1.0f), b = Rand(1.0f);
	if (a + b > 1.0f) { a = 1 - a; b = 1 - b; }
	return vertex0 + a * (vertex1 - vertex0) + b * (vertex2 - vertex0);
}

float3 RayTracer::Color(float3 O, float3 D, uint depth, bool outside) {
	float3 color = make_float3(0, 0, 0);
	if (depth <= 0) { return color; }

	Ray ray = Ray(O, D);
	float t = FLT_MAX;
	CoreTri triangle;

	// The ray hit the skydome, we don't do lighting
	if (!top_level_bvh.Traverse(ray, triangle, t)) {
		float u = 1 + atan2(ray.D.x, -ray.D.z) * INVPI;
		float v = acos(ray.D.y) * INVPI;

		unsigned long long width = round(u * scene.skyHeight);
		unsigned long long height = round(v * scene.skyHeight);

		return clamp(scene.skyDome[min(height * scene.skyHeight * 2 + width, scene.skyDome.size() - 1)], 0.0f, 1.0f);
	}

	float specularity = scene.matList[triangle.material]->specularity;
	float transmission = scene.matList[triangle.material]->transmission;

	// Normalize specularity and transmission if they sum > 1
	if (specularity + transmission > 1) {
		specularity /= specularity + transmission;
		transmission /= specularity + transmission;
	}

	float diffusion = 1.0f - specularity - transmission;
	float3 N = make_float3(triangle.Nx, triangle.Ny, triangle.Nz); // Normalized

	// Transmission
	if (transmission > 0.01f) {
		float H1 = 1.0f, H2 = scene.matList[triangle.material]->IOR;

		if (!outside) {		// If the ray is exiting a transmissive material
			swap(H1, H2);	// Swap the IOR
			N = -N;			// Invert the normal
		}

		float cosTi = dot(-D, N);
		float sin2Tt = (H1 / H2) * (H1 / H2) * (1.0f - (cosTi * cosTi));

		// Calculate using Schlick's approximation
		float fresnel;
		float R0 = ((H1 - H2) / (H1 + H2)) * ((H1 - H2) / (H1 + H2));
		if (H1 > H2 && sin2Tt > 1.0f) { // If there is TIR
			fresnel = 1.0f;

			// Testing purposes, colors area of TIR green
			// return make_float3(0, 1, 0);
		}
		else {
			float x = 1.0f - cosTi;
			fresnel = R0 + (1.0f - R0) * x * x * x * x * x;
		}

		specularity = specularity + transmission * fresnel;
		transmission = transmission * (1.0f - fresnel);

		float H = H1 / H2;
		float S = sqrt(1 - sin2Tt);

		color += transmission * Color(ray.Point(t) - N * 2 * EPSILON, H * D + (H * cosTi - S) * N, depth - 1, !outside);
	}

	if (specularity > 0.01f) { color += specularity * Color(ray.Point(t), D - 2 * (D * N) * N, depth - 1, outside); }
	if (diffusion > 0.01f) { color += diffusion * Illumination(scene.matList[triangle.material]->diffuse, ray.Point(t)); }

	return color;
}

float3 RayTracer::ColorDebugBVH(float3 O, float3 D) {
	float3 color = make_float3(0, 1.0f, 0);

	Ray ray = Ray(O, D);
	CoreTri triangle;
	float t = FLT_MAX;
	int c = -1;

	if (top_level_bvh.Traverse(ray, triangle, t, 0, &c)) {
		// return make_float3(1, 1, 1);
	}

	if (c == 0) { return make_float3(0, 0, 0); } // No BVH intersection, black

	float delta = 0.006f;
	color += make_float3(c * delta, c * -delta, 0);

	return clamp(color, 0, 1);
}

float3 RayTracer::Illumination(float3 color, float3 O) {
	float3 light_color = make_float3(0, 0, 0);

	// Point lights
	for (int i = 0; i < scene.pointLights.size(); i++) {
		float3 direction = scene.pointLights[i]->position - O;
		Ray shadow_ray = Ray(O, direction);

		float t_light = length(fabs(direction));
		float t = FLT_MAX;
		CoreTri triangle;

		top_level_bvh.Traverse(shadow_ray, triangle, t);
		if (t_light <= t) {
			light_color += scene.pointLights[i]->radiance / (t_light * t_light);
		}
	}

	// Monte Carlo area lighting
	int N = 1;
	for (int n = 0; n < N; n++) {
		uint i = RandomUInt() % scene.areaLights.size();

		float p = 1.0f / scene.areaLights.size();
		float3 direction = scene.areaLights[i]->RandomPoint() - O;
		float distance = length(fabs(direction));

		float3 L = scene.areaLights[i]->radiance / (distance * distance);
		
		Ray shadow_ray = Ray(O, direction);
		float t = FLT_MAX;
		CoreTri triangle;

		top_level_bvh.Traverse(shadow_ray, triangle, t);

		if (distance <= t) { // If the shadow ray hit the light
			light_color += L / (p * N);
		}
	}

	return clamp(color * light_color, 0, 1);
}

Scene::~Scene() {
	for (auto mat : matList) delete mat;
	for (auto pointLight : pointLights) delete pointLight;
	for (auto areaLight : areaLights) delete areaLight;
	for (auto instance : instances) delete instance;
	for (auto mesh : meshes) delete mesh;
}

RayTracer::~RayTracer() {
	
}

// -----------------------------------------------------------
// static data for the ray tracer
// -----------------------------------------------------------
Scene RayTracer::scene;

