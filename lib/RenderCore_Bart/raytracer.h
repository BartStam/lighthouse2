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

struct BVH {
	float3 min_bound;	// 12
	int first;			// 4
	float3 max_bound;	// 12
	int count = -1;		// 4
};

struct TopBVH {
	vector<int> root_nodes;
	float3 min_bound;
	float3 max_bound;
};

class Ray {
public:
	Ray(const float3& o, const float3 d, float ior = 1.0f)
		: O(o), D(normalize(d)), IOR(ior) {

	}
	const float3 point(float t) { return O + t * D; }

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

	Accumulator accumulator;
	int frameCount = 0;

	float3 Color(float3 O, float3 D, uint depth, bool outside=true);					// Trace a ray and return its color
	float3 ColorDebugBVH(float3 O, float3 D);											// Trace primary rays and color based on how many BVHs they intersected
	float3 Illumination(float3 color, float3 O);										// Given a color at a location, scale it based on visible lighting

	// BVH
	int N = 0;																			// Total amount of primitives in the scene
	int pool_ptr = 0;																	// Pointer to a BVH in pool array
	int geometry_ptr = 0;																// Pointer to a triangles in triangle_pointers, for top-level BVH construction
	CoreTri* triangle_pointers;															// Pointers to primitives, referenced inside BVH struct
	BVH* alignas(128) pool;																// Pool of BVHs, neighbours are next to each other in the pool
	TopBVH top_bvh;																		// Top level BVH, holds a child BVH for every mesh in the scene
	void ConstructBVH();																// Constructs a BVH from the meshes in the scene and recursively splits it
	float SplitCost(CoreTri* primitives, int first, int count);							// Defines the cost of a given split based on number of primites * surface area
	bool SplitBVH(BVH& bvh);															// Splits a BVH in two based on the SplitCost() implementation
	void RecursiveSplitBVH(BVH& bvh);													// Recursively splits a BVH (2 children) until splitting is no longer worth it
	bool PartitionSAH(BVH& bvh, CoreTri* left, CoreTri* right, int& left_count);		// Partitions a BVH based on the Surface Area Heuristic
	bool PartitionBinningSAH(BVH& bvh, CoreTri* left, CoreTri* right, int& left_count); // Partitions a BVH based on the Surface Area Heuristic and binning (8 bins)
	void UpdateBounds();																// Updates the AABB of all BVH nodes (called after constructing full BVH)
	void PrintBVH(const BVH& bvh);														// Print a text-based representation of a BVH to cout (for debug purposes)

	// Intersection
	bool IntersectTriangle(const Ray& ray, const CoreTri& triangle, float& t);			// Returns whether a ray intersects a triangle, and reports the distance as t
	bool IntersectBVH(const Ray& ray, const BVH& bvh, float& t);						// Returns whether a ray intersects a BVH, and reports the distance as t
	bool RecursiveIntersectBVH(const Ray& ray, const BVH& bvh, CoreTri& tri,			// Recursively intersect a BVH, return the first hit as tri and distance as t
		float& t, int* c = nullptr);
	bool IntersectTopBVH(const Ray& ray, CoreTri&tri, float&t, int* c = nullptr);

	// Lights
	float total_light_area = 0;

	static Scene scene;
};

} // namespace lh2core
