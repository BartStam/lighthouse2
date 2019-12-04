#pragma once

namespace lh2core
{

class Mesh {
public:
	float4* vertices = 0;
	int vcount = 0;
	CoreTri* triangles = 0;
};

class Material {
public:
	Material() = default;
	float3 diffuse = make_float3(1, 1, 1);
	float specularity = 0;
	float transmission = 0;
	float IOR = 1.52; // TODO: actually set the IOR correctly per material
};

class PointLight {
public:
	PointLight(const float3 pos, const float3 rad) {
		position = pos;
		radiance = rad;
	}
	float3 position;
	float3 radiance;
};

class AreaLight {
public:
	AreaLight(const float3 v0, const float3 v1, const float3 v2, float3 N, float3 c, float A, float3 rad) {
		vertex0 = v0;
		vertex1 = v1;
		vertex2 = v2;
		normal = N;
		center = c;
		area = A;
		radiance = rad;
	}
	float3 vertex0;
	float3 vertex1;
	float3 vertex2;
	float3 normal;
	float3 center;
	float area;
	float3 radiance;

	float3 RandomPoint(); // Generate a random point uniformly distributed over the triangle's surface
};

class Scene {
public:
	Scene() = default;
	~Scene();

	vector<float3> skyDome;
	uint skyWidth;
	uint skyHeight;

	vector<Material*> matList;
	vector<AreaLight*> areaLights;
	vector<PointLight*> pointLights;
	vector<Mesh> meshes;
};

class Ray {
public:
	Ray(const float3& o, const float3 d, float ior = 1.0f) { O = o; D = normalize(d); IOR = ior; }
	const float3 origin() { return O; }
	const float3 direction() { return D; }
	const float3 point(float t) { return O + t * D; }

	bool IntersectsTriangle(const CoreTri& triangle, float& t);

private:
	float3 O;
	float3 D;
	float IOR;
};

class Accumulator {
public:
	Accumulator() = default;
	void Rebuild(int width, int height);
	float3 Pixel(int x, int y) { return frame[y * w + x] / frame_count; }
	void addPixel(int x, int y, float3 pixel) { frame[y * w + x] += pixel; }
	void Increment() { frame_count++; }
private:
	int w; // screen width
	int frame_count;
	vector<float3> frame;
};

class RayTracer
{
public:
	RayTracer() = default;
	float3 Color(float3 O, float3 D, uint depth, bool outside=true);
	float3 Illumination(float3 color, float3 O);
	static Scene scene;
	Accumulator accumulator;
};

} // namespace lh2core
