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
	void UpdateBounds();
	void RecursiveSplit();
	void RecursivePrint();

	float3 min_bound = make_float3(0, 0, 0);
	float3 max_bound = make_float3(0, 0, 0); // AABB positions
	vector<BVH*> children;
	vector<CoreTri*> leaves;

	BVH* left;
	BVH* right;
	int first, count;

	bool isLeaf = true;

private:
	bool Split();
	float SplitCost(vector<CoreTri*> leaves);
	void RecursiveDelete();
};

class Ray {
public:
	Ray(const float3& o, const float3 d, float ior = 1.0f)
		: O(o), D(normalize(d)), IOR(ior) {

	}
	const float3 point(float t) { return O + t * D; }

	bool IntersectsTriangle(const CoreTri& triangle, float& t);							// If the ray intersects a triangle
	bool IntersectsBVH(const BVH& bvh, float& t);										// If the ray intersects the AABB of a BVH
	bool RecursiveIntersection(const BVH& bvh, CoreTri& tri, float& t);					// Finds the closest triangle intersection in a BVH

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
	RayTracer() = default;
	~RayTracer();
	float3 Color(float3 O, float3 D, uint depth, bool outside=true);					// Trace a ray and return its color
	float3 Illumination(float3 color, float3 O);										// Given a color at a location, scale it based on visible lighting

	Accumulator accumulator;
	BVH root_bvh;
	int frameCount = 0;

	// BVH
	int poolPtr = 0;																	// Pointer to a BVH in pool array
	int N;																				// Total amount of primitives encompassed by the BVH
	CoreTri* triangle_pointers;															// Pointers to primitives, referenced inside BVH struct
	BVH* pool;																			// Pool of BVHs, neighbours are next to each other in the pool
	void ConstructBVH();																// Constructs a BVH from the meshes in the scene and recursively splits it
	float SplitCost(CoreTri* primitives, int first, int count);							// Defines the cost of a given split based on number of primites * surface area
	bool SplitBVH(BVH& bvh);															// Splits a BVH in two based on the SplitCost() implementation
	void RecursiveSplitBVH(BVH& bvh);													// Recursively splits a BVH (2 children) until splitting is no longer worth it
	void UpdateBounds(BVH& bvh);														// Updates the AABB of a BVH

	// Intersection
	bool IntersectTriangle(const Ray& ray, const CoreTri& triangle, float& t);			// Returns whether a ray intersects a triangle, and reports the distance as t
	bool IntersectBVH(const Ray& ray, const BVH& bvh, float& t);						// Returns whether a ray intersects a BVH, and reports the distance as t
	bool RecursiveIntersectBVH(const Ray& ray, const BVH& bvh, CoreTri& tri, float& t); // Recursively intersect a BVH, return the first hit as tri and distance as t

	static Scene scene;
};

} // namespace lh2core
