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

float3 RayTracer::Color(float3 O, float3 D, uint depth, bool outside) {
	float3 color = make_float3(0, 0, 0);
	if (depth <= 0) { return color; }
	D = normalize(D);

	Ray ray = Ray(O, D);
	CoreTri* triangle;
	float smallest_t = FLT_MAX;

	for (Mesh& mesh : scene.meshes) for (int i = 0; i < mesh.vcount / 3; i++)
	{
		float t;
		if (ray.IntersectsTriangle(mesh.triangles[i], t) && t < smallest_t) {
			smallest_t = t;
			triangle = &mesh.triangles[i];
		}
	}

	// The ray hit the skydome, we don't do lighting
	if (smallest_t == FLT_MAX) {
		float3 Dn = normalize(D);
		float u = 1 + atan2(Dn.x, -Dn.z) / PI;
		float v = acos(Dn.y) / PI;

		unsigned long long width = round(u * scene.skyHeight);
		unsigned long long height = round(v * scene.skyHeight);

		return clamp(scene.skyDome[min(height * scene.skyHeight * 2 + width, scene.skyDome.size() - 1)], 0.0f, 1.0f);
	}

	float specularity = scene.matList[triangle->material]->specularity;
	float transmission = scene.matList[triangle->material]->transmission;
	if (specularity + transmission > 1) { specularity /= specularity + transmission; transmission /= specularity + transmission; }
	float diffusion = 1.0f - specularity - transmission;
	
	float3 N = make_float3(triangle->Nx, triangle->Ny, triangle->Nz); // Normalized

	// Transmission
	float H1 = 1.0f, H2 = scene.matList[triangle->material]->IOR;

	if (!outside) {		// If the ray is exiting a transmissive material
		swap(H1, H2);	// Swap the IOR
		N = -N;			// Invert the normal
	}

	float cosTi = dot(-D, N);
	float sin2Tt = (H1 / H2) * (H1 / H2) * (1.0f - (cosTi * cosTi));

	// Calculate using Schlick's approximation
	float fresnel;
	float R0 = ((H1 - H2) / (H1 + H2)) * ((H1 - H2) / (H1 + H2));
	if (sin2Tt > 1.0f) { // If there is TIR
		fresnel = 1.0f;

		// Testing purposes
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

	if (transmission > 0.01f) { color += transmission * Color(ray.point(smallest_t) - N * 2 * EPSILON, H * D + (H * cosTi - S) * N, depth - 1, !outside); }
	if (specularity > 0.01f) { color += specularity * Color(ray.point(smallest_t), D - 2 * (D * N) * N, depth - 1, outside); }
	if (diffusion > 0.01f) { color += diffusion * Illumination(scene.matList[triangle->material]->diffuse, ray.point(smallest_t)); }

	

	


	return color;
}

float3 RayTracer::Illumination(float3 color, float3 O) {
	float3 light_color = make_float3(0, 0, 0);

	// Point lights
	for (int i = 0; i < scene.pointLights.size(); i++) {
		float3 direction = scene.pointLights[i]->position - O;
		Ray shadow_ray = Ray(O, direction);

		float t_light = length(fabs(direction));
		float t = FLT_MAX;
		for (Mesh& mesh : scene.meshes) for (int i = 0; i < mesh.vcount / 3; i++) {
			if (shadow_ray.IntersectsTriangle(mesh.triangles[i], t) && t < t_light) {
				break;
			}
		}

		// If the triangle is illuminated by this light
		if (t_light <= t) {
			light_color += scene.pointLights[i]->radiance / (t_light * t_light);
		}
	}

	// Monte Carlo area lighting
	float total_area = 0;
	for (int i = 0; i < scene.areaLights.size(); i++) {
		total_area += scene.areaLights[i]->area;
	}

	int N = 8;
	for (int n = 0; n < N; n++) {
		uint i = RandomUInt() % scene.areaLights.size();
		float p = scene.areaLights[i]->area / total_area; // Probability of hitting this light

		float3 direction = scene.areaLights[i]->RandomPoint() - O;
		Ray shadow_ray = Ray(O, direction);

		float t_light = length(fabs(direction)), t = FLT_MAX;
		for (Mesh& mesh : scene.meshes) for (int m = 0; m < mesh.vcount / 3; m++) {
			if (shadow_ray.IntersectsTriangle(mesh.triangles[m], t) && t < t_light) {
				break;
			}
		}

		// If the triangle is illuminated by this light (V is 1 in rendering equation)
		if (t_light <= t) {
			light_color += ((scene.areaLights[i]->radiance / (t_light * t_light)) / p) / N;
		}
	}

	return clamp(color * light_color, 0, 1);
}

void Accumulator::Rebuild(int width, int height) {
	frame.clear();
	frame.resize(width * height, make_float3(0, 0, 0));
	frame_count = 0;
	w = width;
}

float3 AreaLight::RandomPoint() {
	float a = Rand(1.0f), b = Rand(1.0f);
	if (a + b > 1.0f) { a = 1 - a; b = 1 - b; }
	return vertex0 + a * (vertex1 - vertex0) + b * (vertex2 - vertex0);
}

Scene::~Scene()
{
	for (auto mat : matList) delete mat;
}

// -----------------------------------------------------------
// static data for the ray tracer
// -----------------------------------------------------------
Scene RayTracer::scene;

