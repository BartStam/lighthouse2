#pragma once

namespace lh2core
{

/*
	BVHNode
*/

struct BVHNode {			// 32 bytes
	float3 min_bound;	// 12 bytes
	int first;			//  4 bytes
	float3 max_bound;	// 12 bytes
	int count = -1;		//  4 bytes
};

struct BVH2Node : BVHNode { 
	
};

struct BVH4Node : BVHNode {

};

struct TopBVHNode : BVH2Node {
	vector<BVH*> bvh_pointers;
};

/*
	BVH
*/

class BVH {
public:
	BVH() = default;
	~BVH();

	int N;
	int pool_pointer = 2;
	
	float SplitCost(const CoreTri* triangles, int first, int count);
	bool IntersectAABB(const Ray& ray, const BVHNode& bvh, float& t);

	virtual const BVHNode& Root() = 0;
	virtual bool Partition(BVHNode& bvh, CoreTri* triangles, int* counts) = 0;
	virtual bool Subdivide(BVHNode& bvh) = 0;
	virtual void SubdivideRecursively(BVHNode& bvh) = 0;
	virtual void UpdateBounds() = 0;
	virtual bool Traverse(Ray& ray, const BVHNode& bvh, CoreTri& tri, float& t, int* c = nullptr) = 0;
};

class BVH2 : public BVH {
public:
	BVH2(Mesh mesh);
	~BVH2();

	BVH2Node* alignas(128) pool;
	CoreTri* triangle_pointers;

	const BVHNode& Root();
	bool Partition(BVHNode& bvh, CoreTri* triangles, int* counts);
	bool Subdivide(BVHNode& bvh);
	void SubdivideRecursively(BVHNode& bvh);
	void UpdateBounds();
	bool Traverse(Ray& ray, const BVHNode& bvh, CoreTri& tri, float& t, int* c = nullptr);
};

class BVH4 : public BVH {
public:
	BVH4(Mesh mesh);
	~BVH4();

	BVH4Node* alignas(128) pool;
	CoreTri* triangle_pointers;

	const BVHNode& Root();
	bool Partition(BVHNode& bvh, CoreTri* triangles, int* counts);
	bool Subdivide(BVHNode& bvh);
	void SubdivideRecursively(BVHNode& bvh);
	void UpdateBounds();
	bool Traverse(Ray& ray, const BVHNode& bvh, CoreTri& tri, float& t, int* c = nullptr);
};

class TopLevelBVH : public BVH {
public:
	TopLevelBVH();
	~TopLevelBVH();

	vector<BVH*> bvh_vector;		// Stores pointers to all mesh-level BVHs in no particular order
	int* bvh_indices;				// Index of a BVH in bvh_vector
	TopBVHNode* alignas(128) pool;
	
	
	const BVHNode& Root();
	bool Partition(BVHNode& bvh, CoreTri* triangles, int* counts);
	bool Subdivide(BVHNode& bvh);
	void SubdivideRecursively(BVHNode& bvh);
	void UpdateBounds();
	bool Traverse(Ray& ray, const BVHNode& bvh, CoreTri& tri, float& t, int* c = nullptr);
	void AddBVH(BVH* bvh, bool rebuild = false);
	void Rebuild();
};

} // namespace lh2core
