#include "core_settings.h"
#pragma once

namespace lh2core
{
class BVH; // Forward declaration for use in Mesh
class SBVH; // Forward declaration for use in Mesh

struct Ray {
	Ray(const float3& o, const float3 d, float ior = 1.0f)
		: O(o), D(normalize(d)), IOR(ior) {
	}

	float3 O;
	float3 D;
	float IOR;

	const float4 O4() {	return make_float4(O, 1.0f); }
	const float4 D4() { return make_float4(D, 0.0f); }
	const float3 Point(float t) { return O + t * D; }

	bool IntersectTriangle(const CoreTri& triangle, float& t);
};

struct Mesh {
	Mesh(int vertex_count, int triangle_count) : vcount(vertex_count) {
		triangles = new CoreTri[triangle_count];
		tri_min_bounds = new float3[triangle_count];
		tri_max_bounds = new float3[triangle_count];
		tri_centers = new float3[triangle_count];
	}

	~Mesh();

	int vcount = 0;
	CoreTri* triangles = nullptr;
	float3* tri_min_bounds;
	float3* tri_max_bounds;
	float3* tri_centers;

	float3 aabb_center; // Used for BVH construction
	SBVH* bvh = nullptr;
};

struct Instance {
	Instance(Mesh* m, mat4 t) : mesh(m), transform(t) {};

	Mesh* mesh = nullptr;
	mat4 transform;
};

struct Triangle {
	float3 vertex0;
	float3 vertex1;
	float3 vertex2;
	float3 normal;
	float3 center;
	float area;

	float3 RandomPoint(); // Generate a random point uniformly distributed over the triangle's surface
};

struct Material {
	Material() = default;
	float3 diffuse = make_float3(1, 1, 1);
	float specularity = 0;
	float transmission = 0;
	float IOR = 1.0f;

	bool IsLight() { return diffuse.x > 1 || diffuse.y > 1 || diffuse.z > 1; }
};

struct PointLight {
	float3 position;
	float3 radiance;
};

struct AreaLight : public Triangle {
	float3 radiance;
};
}
