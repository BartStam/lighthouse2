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
		vertices = new float4[vertex_count];
		triangles = new CoreTri[triangle_count];
	}

	~Mesh() {
		delete[] vertices;
		delete[] triangles;
		delete bvh;
	}

	float4* vertices = nullptr;
	int vcount = 0;
	CoreTri* triangles = nullptr;
	BVH* bvh = nullptr;
};

struct Instance {
	Instance(Mesh* m, mat4 t) : mesh(m), transform(t) {};

	Mesh* mesh = nullptr;
	mat4 transform;

	void SetPosition(float3& pos) {	transform[3] = pos.x, transform[7] = pos.y, transform[11] = pos.z; }
	float3 GetPosition() { return make_float3(transform[3], transform[7], transform[11]); }
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
