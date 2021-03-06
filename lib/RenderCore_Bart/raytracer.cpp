#include "core_settings.h"

void Accumulator::Rebuild(int width, int height) {
	frame.clear();
	frame.resize(width * height, make_float4(0));
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
	int material;
	float3 N;
	float t = FLT_MAX;

	// The ray hit the skydome, we don't do lighting
	if (!top_level_bvh.Traverse(ray, material, N, t)) {
		float u = 1 + atan2(ray.D.x, -ray.D.z) * INVPI;
		float v = acos(ray.D.y) * INVPI;

		unsigned long long width = round(u * scene.skyHeight);
		unsigned long long height = round(v * scene.skyHeight);

		return clamp(scene.skyDome[min(height * scene.skyHeight * 2 + width, scene.skyDome.size() - 1)], 0.0f, 1.0f);
	}

	// If we hit a light
	if (scene.matList[material]->IsLight()) {
		// If we hit the front of the light, return its scaled color
		if (dot(N, ray.D) < 0) {
			float3 c = scene.matList[material]->diffuse;
			float scale = 1.0f / max(c.x, max(c.y, c.z));
			return scale * c;
		}
		
		return make_float3(0); // If we hit the back of the light, return black
	}

	float specularity = scene.matList[material]->specularity;
	float transmission = scene.matList[material]->transmission;

	// Normalize specularity and transmission if they sum > 1
	if (specularity + transmission > 1) {
		specularity /= specularity + transmission;
		transmission /= specularity + transmission;
	}

	float diffusion = 1.0f - specularity - transmission;
	
	// Transmission
	if (transmission > 0.01f) {
		float H1 = 1.0f, H2 = scene.matList[material]->IOR;

		if (!outside) {		// If the ray is exiting a transmissive material
			swap(H1, H2);	// Swap the IOR
			N = -N;			// Invert the normal
		}

		float cosTi = dot(-ray.D, N);

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

		color += transmission * Color(ray.Point(t) - N * 2 * EPSILON, H * ray.D + (H * cosTi - S) * N, depth - 1, !outside);
	}

	if (specularity > 0.01f) { color += specularity * Color(ray.Point(t) + N * 2 * EPSILON, ray.D - 2 * (ray.D * N) * N, depth - 1, outside); }
	if (diffusion > 0.01f) { color += diffusion * Illumination(scene.matList[material]->diffuse, ray.Point(t) + N * 2 * EPSILON, N, 4); }

	return color;
}

float3 RayTracer::ColorDebugBVH(float3 O, float3 D, float delta) {
	float3 color = make_float3(0, 1.0f, 0);

	Ray ray = Ray(O, D);
	int material;
	float3 N;
	float t = FLT_MAX;
	int c = -1;

	if (top_level_bvh.Traverse(ray, material, N, t, 0, &c)) {
		// return make_float3(1, 1, 1);
	}

	if (c == 0) { return make_float3(0, 0, 0); } // No BVH intersection, black

	color += make_float3(c * delta, c * -delta, 0);

	return clamp(color, 0, 1);
}

float3 RayTracer::Illumination(float3 color, float3 O, float3 N, int samples) {
	float3 light_color = make_float3(0, 0, 0);

	// Point lights
	for (int i = 0; i < scene.pointLights.size(); i++) {
		float3 D = scene.pointLights[i]->position - O;
		Ray shadow_ray = Ray(O, D);

		float t_light = length(fabs(D));
		int material;
		float3 N;
		float t = FLT_MAX;

		top_level_bvh.Traverse(shadow_ray, material, N, t);
		
		if (t_light <= t) {
			light_color += scene.pointLights[i]->radiance / (t_light * t_light);
		}
	}

	// Monte Carlo area lighting
	float c = (float)scene.areaLights.size() / samples;
	for (int n = 0; n < samples; n++) {
		uint i = RandomUInt() % scene.areaLights.size();

		float3 D = scene.areaLights[i]->RandomPoint() - O;
		float r = length(fabs(D)) - EPSILON;

		Ray shadow_ray = Ray(O, D);
		int material;
		float3 triN;
		float t = FLT_MAX;

		top_level_bvh.Traverse(shadow_ray, material, triN, t);

		D = shadow_ray.D; // D is normalized in Ray constructor, no reason to do the work twice
		float angle = dot(D, N);
		if (r <= t && angle > 0) { // If we hit the front of the light
			float A = scene.areaLights[i]->area;
			float sr = A / (4 * PI * r * r);
			float3 L = scene.areaLights[i]->radiance * sr;
			float3 E = L * angle;

			//return make_float3(clamp(theta, 0.0f, 1.0f));

			light_color += E * c;
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

