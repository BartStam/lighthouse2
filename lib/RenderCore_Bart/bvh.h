#pragma once

namespace lh2core
{

/*
	BVHNode
*/

struct BVHNode {		// 32 bytes
	float3 min_bound;	// 12 bytes
	int first;			//  4 bytes
	float3 max_bound;	// 12 bytes
	int count = -1;		//  4 bytes
};

/*
	BVH
*/

class BVH {
public:
	BVH() = default;
	~BVH();

	int N;						// Total amount of primitives in this BVH
	int pool_pointer = 2;		// Pointer to the "current" BVHNode, used for construction
	BVHNode* alignas(128) pool;	// Pool of cache aligned BVHNodes, each 32 bytes with neighbours residing next to each other
	int* indices;				// Indices of the primitive array, triangles for mesh-level BVHs and BVHs for top-level BVH
	
	float SplitCost(const CoreTri* triangles, int first, int count);
	bool IntersectAABB(const Ray& ray, const BVHNode& bvh, float& t);
	
	virtual BVHNode& Root() = 0;
	virtual bool Partition(BVHNode& bvh, CoreTri* triangles, int* counts) = 0;
	virtual bool Subdivide(BVHNode& bvh) = 0;
	virtual void SubdivideRecursively(BVHNode& bvh) = 0;
	virtual void UpdateBounds() = 0;
	virtual bool Traverse(Ray& ray, const BVHNode& bvh, CoreTri& tri, float& t, int* c = nullptr) = 0;
	virtual void Print(BVHNode& bvh) = 0;
};

class BVH2 : public BVH {
public:
	BVH2(Mesh* mesh);
	~BVH2();

	CoreTri* triangle_pointers;
	Mesh* mesh;

	BVHNode& Root();
	bool Partition(BVHNode& bvh, CoreTri* triangles, int* counts);
	bool Subdivide(BVHNode& bvh);
	void SubdivideRecursively(BVHNode& bvh);
	void UpdateBounds();
	bool Traverse(Ray& ray, const BVHNode& bvh, CoreTri& tri, float& t, int* c = nullptr);
	void Print(BVHNode& bvh);
};

class BVH4 : public BVH {
public:
	BVH4(Mesh* mesh);
	~BVH4();

	CoreTri* triangle_pointers;
	Mesh* mesh;

	BVHNode& Root();
	bool Partition(BVHNode& bvh, CoreTri* triangles, int* counts);
	bool Subdivide(BVHNode& bvh);
	void SubdivideRecursively(BVHNode& bvh);
	void UpdateBounds();
	bool Traverse(Ray& ray, const BVHNode& bvh, CoreTri& tri, float& t, int* c = nullptr);
	void Print(BVHNode& bvh);
};

class TopLevelBVH : public BVH {
public:
	TopLevelBVH();
	~TopLevelBVH();

	vector<BVH*> bvh_vector;	// Stores pointers to all mesh-level BVHs in no particular order
	
	BVHNode& Root();
	bool Partition(BVHNode& bvh, CoreTri* triangles, int* counts);
	bool Subdivide(BVHNode& bvh);
	void SubdivideRecursively(BVHNode& bvh);
	void UpdateBounds();
	bool Traverse(Ray& ray, const BVHNode& bvh, CoreTri& tri, float& t, int* c = nullptr);
	void Print(BVHNode& bvh);
	void AddBVH(BVH* bvh, bool rebuild = false);
	void Rebuild();
};

} // namespace lh2core
