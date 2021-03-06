#pragma once

namespace lh2core
{

/*
	BVHNode
*/

struct BVHNode {		// 32 bytes
	float3 min_bound;	//  8 bytes
	int first;			//  4 bytes
	float3 max_bound;	//  8 bytes
	int count = -1;		//  4 bytes
};

/*
	BVH
*/

class BVH {
public:
	BVH() = default;
	virtual ~BVH() {};

	virtual void Rebuild() = 0;
	virtual bool Traverse(Ray& ray, int& material, float3& N, float& t, int pool_index = 0, int* c = nullptr) = 0;
	virtual void Print(BVHNode& bvh) = 0;
	const BVHNode& Root() { return pool[0]; }

protected:
	int N;																	// Total amount of primitives in this BVH
	int pool_pointer = -1;													// Pointer to the "current" BVHNode, used for construction. -1 indicates the BVH is not yet built.
	alignas(128) BVHNode* pool = nullptr;									// Pool of cache aligned BVHNodes, each 32 bytes with neighbours residing next to each other
	int* tri_indices;														// Indices of the primitive array, triangles for instance-level BVHs and BVHs for top-level BVH
	
	bool IntersectAABB(const Ray& ray, const BVHNode& bvh, float& t);		// Returns whether a ray intersects a BVHNode AABB, and at what distance along the ray (t)

	virtual float SplitCost(const int* indices, int first, int count) = 0;	// Defines the "cost" of a set of primitives as [aabb surface area * number of primitives]
	virtual bool Partition(BVHNode& bvh, int* indices, int* counts, int bins = 8) = 0;	// Partitions a set of primitives into two groups, based on the binning SAH heuristic
	virtual bool Subdivide(BVHNode& bvh) = 0;								// Subdivides a single BVH into two, based on a greedy binning SAH heuristic
	virtual void SubdivideRecursively(BVHNode& bvh) = 0;					// Splits a BVH recursively until there is no value in splitting anymore
	virtual void UpdateBounds() = 0;										// Updates the AABB bounds of every BVHNode in the pool (back to front)
};

class BVH2 : public BVH {
public:
	BVH2(Mesh* mesh);
	~BVH2();

	void Rebuild();
	bool Traverse(Ray& ray, int& material, float3& N, float& t, int pool_index = 0, int* c = nullptr);
	void Print(BVHNode& bvh);

private:
	Mesh* mesh = nullptr; // Meshes are owned by the rendercore

	float SplitCost(const int* indices, int first, int count);
	bool Partition(BVHNode& bvh, int* indices, int* counts, int bins = 8);
	bool Subdivide(BVHNode& bvh);
	void SubdivideRecursively(BVHNode& bvh);
	void UpdateBounds();
};

class BVH4 : public BVH {
public:
	BVH4(Mesh* mesh);
	~BVH4();

	void Rebuild();
	bool Traverse(Ray& ray, int& material, float3& N, float& t, int pool_index = 0, int* c = nullptr);
	void Print(BVHNode& bvh);

private:
	Mesh* mesh = nullptr; // Meshes are owned by the rendercore

	float SplitCost(const int* indices, int first, int count);
	bool Partition(BVHNode& bvh, int* indices, int* counts, int bins = 8);
	bool Subdivide(BVHNode& bvh);
	void SubdivideRecursively(BVHNode& bvh);
	void UpdateBounds();
};

class TopLevelBVH : public BVH {
public:
	TopLevelBVH();
	~TopLevelBVH();

	void AddInstance(Instance* instance);
	void Rebuild();
	bool Traverse(Ray& ray, int& material, float3& N, float& t, int pool_index = 0, int* c = nullptr);
	void Print(BVHNode& bvh);

private:
	vector<Instance*> instance_vector;
	
	float SplitCost(const int* indices, int first, int count);
	bool Partition(BVHNode& bvh, int* indices, int* counts, int bins = 8);
	bool Subdivide(BVHNode& bvh);
	void SubdivideRecursively(BVHNode& bvh);
	void UpdateBounds();
};

} // namespace lh2core
