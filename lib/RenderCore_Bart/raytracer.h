#pragma once

namespace lh2core
{

class Mesh
{
public:
	float4* vertices = 0;
	int vcount = 0;
	CoreTri* triangles = 0;
};

class Material
{
public:
	Material() = default;
	float3 diffuse = make_float3(1, 1, 1);
	float specularity = 0;
	float transmission = 0;
	float IOR = 1.52; // TODO: actually set the IOR correctly per material
};

class PointLight
{
public:
	PointLight(const float3 pos, const float3 rad) {
		radiance = rad;
		position = pos;
	}
	float3 radiance;
	float3 position;
};

class Scene
{
public:
	Scene() = default;
	~Scene();

	vector<float3> skyDome;
	uint skyWidth;
	uint skyHeight;

	vector<Material*> matList;
	vector<PointLight*> pointLights;
	vector<Mesh> meshes;
};

class Ray
{
public:
	Ray(const float3& o, const float3 d) { O = o; D = normalize(d); }
	const float3 origin() { return O; }
	const float3 direction() { return D; }
	const float3 point(float t) { return O + t * D; }

	bool IntersectsTriangle(const CoreTri& triangle, float& t);

private:
	float3 O;
	float3 D;
};

class RayTracer
{
public:
	RayTracer() = default;
	float3 Color(float3 O, float3 D, uint depth);
	float3 Illumination(float3 color, float3 O);
	static Scene scene;
};

} // namespace lh2core
