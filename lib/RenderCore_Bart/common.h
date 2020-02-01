#include "core_settings.h"
#pragma once

namespace lh2core
{
class BVH; // Forward declaration for use in Mesh

struct Ray {
	Ray(const float3& o, const float3 d, float ior = 1.0f)
		: O(o), D(normalize(d)), IOR(ior) {
	}

	float3 O;
	float3 D;
	float IOR;

	const float3 Point(float t) { return O + t * D; }

	bool IntersectTriangle(const CoreTri& triangle, float& t) {
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
};

struct Mesh {
	Mesh(int vertex_count, int triangle_count) : vcount(vertex_count) {
		triangles = new CoreTri[triangle_count];
		tri_centers = new float3[triangle_count];
		tri_min_bounds = new float3[triangle_count];
		tri_max_bounds = new float3[triangle_count];
	}

	~Mesh() {
		delete[] triangles;
		delete[] tri_centers;
		delete[] tri_min_bounds;
		delete[] tri_max_bounds;
		delete bvh;
	}

	int vcount = 0;
	CoreTri* triangles = nullptr;
	float3* tri_centers = nullptr;
	float3* tri_min_bounds = nullptr;
	float3* tri_max_bounds = nullptr;

	float3 aabb_min_bound;
	float3 aabb_max_bound;
	float3 aabb_center;	// Used for BVH construction
	BVH* bvh = nullptr;
};

struct Instance {
	Instance(Mesh* m, mat4 t) : mesh(m), transform(t), inv_transform(t.Inverted()) {};

	Mesh* mesh = nullptr;
	mat4 transform;
	mat4 inv_transform; // Cached for efficiency
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
