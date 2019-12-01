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

	uint diffuse = 0xffffffff;
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
	uint Color(float3 O, float3 D, uint depth);
	static Scene scene;
};

} // namespace lh2core
