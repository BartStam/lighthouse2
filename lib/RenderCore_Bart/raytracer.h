#pragma once

namespace lh2core
{

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

class BVH {
public:
	BVH() = default;
	~BVH();
	void Split();
	void UpdateBounds();
	void RecursiveDelete();

	float3 pos1 = make_float3(0, 0, 0);
	float3 pos2 = make_float3(0, 0, 0); // AABB positions
	vector<BVH*> children;
	vector<CoreTri*> leaves;
	bool isLeaf = true;
};

class Ray {
public:
	Ray(const float3& o, const float3 d, float ior = 1.0f)
		: O(o), D(normalize(d)), IOR(ior) {

	}
	const float3 origin() { return O; }
	const float3 direction() { return D; }
	const float3 point(float t) { return O + t * D; }

	bool IntersectsTriangle(const CoreTri& triangle, float& t); // If the ray intersects a triangle
	bool IntersectsBVH(const BVH& bvh, float& t);				// If the ray intersects the AABB of a BVH
	CoreTri* RecursiveIntersection(const BVH& bvh, float& t);	// Finds the closest triangle intersection in a BVH

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

class RayTracer
{
public:
	void ConstructBVH(); // Constructs a BVH from the meshes in the scene

	RayTracer() = default;
	float3 Color(float3 O, float3 D, uint depth, bool outside=true);
	float3 Illumination(float3 color, float3 O);
	static Scene scene;
	Accumulator accumulator;
	BVH BVH;

	int frameCount = 0;
};

} // namespace lh2core
