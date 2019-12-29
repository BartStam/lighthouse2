#pragma once

namespace lh2core
{

struct IBVH {
	float3 min_bound;	// 12
	int first;			// 4
	float3 max_bound;	// 12
	int count = -1;		// 4
};

struct TopBVH {
	float3 min_bound;
	int first;
	float3 max_bound;
	int count = -1;
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

class RayTracer {
public:
	RayTracer() = default;
	~RayTracer();

	Accumulator accumulator;
	int frameCount = 0;

	float3 Color(float3 O, float3 D, uint depth, bool outside=true);						// Trace a ray and return its color
	float3 ColorDebugBVH(float3 O, float3 D);												// Trace primary rays and color based on how many BVHs they intersected
	float3 Illumination(float3 color, float3 O);											// Given a color at a location, scale it based on visible lighting

	TopLevelBVH top_level_bvh;

	// Top-level BVH
	TopBVH top_bvh;																			// Top level BVH, holds a child BVH for every mesh in the scene
	int* bvh_root_nodes;
	int geometry_ptr = 0;																	// Pointer to a triangle in triangle_pointers, for top-level BVH construction
	void ConstructTopBVH();

	// BVH
	int N = 0;																				// Total amount of primitives in the scene
	int pool_ptr = 0;																		// Pointer to a BVH in pool array
	CoreTri* triangle_pointers;																// Pointers to primitives, referenced inside BVH struct
	IBVH* alignas(128) pool;																// Pool of BVHs, neighbours are next to each other in the pool
	void ConstructBVH();																	// Constructs a BVH from the meshes in the scene and recursively splits it
	float SplitCost(CoreTri* primitives, int first, int count);								// Defines the cost of a given split based on number of primites * surface area
	bool SplitBVH(IBVH& bvh);																// Splits a BVH in two based on the SplitCost() implementation
	void RecursiveSplitBVH(IBVH& bvh);														// Recursively splits a BVH (2 children) until splitting is no longer worth it
	bool PartitionSAH(IBVH& bvh, CoreTri* left, CoreTri* right, int& left_count);			// Partitions a BVH based on the Surface Area Heuristic
	bool PartitionBinningSAH(IBVH& bvh, CoreTri* left, CoreTri* right, int& left_count);	// Partitions a BVH based on the Surface Area Heuristic and binning (8 bins)
	void UpdateBounds();																	// Updates the AABB of all BVH nodes (called after constructing full BVH)
	void PrintBVH(const IBVH& bvh);															// Print a text-based representation of a BVH to cout (for debug purposes)

	// Intersection
	bool IntersectTriangle(const Ray& ray, const CoreTri& triangle, float& t);				// Returns whether a ray intersects a triangle, and reports the distance as t
	bool IntersectBVH(const Ray& ray, const IBVH& bvh, float& t);							// Returns whether a ray intersects a BVH, and reports the distance as t
	bool RecursiveIntersectBVH(const Ray& ray, const IBVH& bvh, CoreTri& tri,				// Recursively intersect a BVH, return the first hit as tri and distance as t
		float& t, int* c = nullptr);
	bool IntersectTopBVH(const Ray& ray, CoreTri& tri, float& t, int* c = nullptr);

	static Scene scene;
};

} // namespace lh2core
