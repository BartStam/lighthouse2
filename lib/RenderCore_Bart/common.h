#include "core_settings.h"
#pragma once

namespace lh2core
{
	class Ray {
	public:
		Ray(const float3& o, const float3 d, float ior = 1.0f)
			: O(o), D(normalize(d)), IOR(ior) {
		}

		float3 O;
		float3 D;
		float IOR;

		const float3 Point(float t) { return O + t * D; }
		bool IntersectTriangle(const CoreTri& triangle, float& t);
	};

	class Mesh {
	public:
		float4* vertices = 0;
		int vcount = 0;
		CoreTri* triangles = 0;
	};

	class Triangle {
	public:
		Triangle(const float3 v0, const float3 v1, const float3 v2, float3 N, float3 c, float A)
			: vertex0(v0), vertex1(v1), vertex2(v2), normal(N), center(c), area(A) {
		}

		float3 vertex0;
		float3 vertex1;
		float3 vertex2;
		float3 normal;
		float3 center;
		float area;

		float3 RandomPoint(); // Generate a random point uniformly distributed over the triangle's surface
	};

	class Material {
	public:
		Material() = default;
		float3 diffuse = make_float3(1, 1, 1);
		float specularity = 0;
		float transmission = 0;
		float IOR = 1.0f;
	};

	class PointLight {
	public:
		PointLight(const float3 pos, const float3 rad)
			: position(pos), radiance(rad) {
		}

		float3 position;
		float3 radiance;
	};

	class AreaLight : public Triangle {
	public:
		AreaLight(const float3 v0, const float3 v1, const float3 v2, float3 N, float3 c, float A, float3 rad)
			: Triangle(v0, v1, v2, N, c, A), radiance(rad) {
		}

		float3 radiance;
	};
}
