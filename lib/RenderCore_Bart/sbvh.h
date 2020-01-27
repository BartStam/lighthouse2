#pragma once

namespace lh2core
{

/*
	SBVH (abstract)
*/

class SBVH {
public:
	SBVH() = default;
	virtual ~SBVH() {};

	float3 min_bound = make_float3(FLT_MAX);
	float3 max_bound = make_float3(-FLT_MAX);

	bool IntersectAABB(Ray& ray, float& t);
	virtual bool IsLeaf() { return indices.size() > 0; }
	virtual void Rebuild() = 0;
	virtual bool Traverse(Ray& ray, int& material, float3& N, float& t, int* c = nullptr) = 0;
	virtual void SubdivideRecursively() = 0;
	virtual void Print() = 0;

protected:
	vector<int> indices;							// Indices referencing objects at the leaves (typically triangles or mesh-level SBVHS)

	virtual float SplitCost(vector<int> idxs) = 0;	// Calculates the cost of a split according to SAH (assumes AABB is tightly fit around geometry)
	virtual bool Subdivide() = 0;
};

/*
	MeshLevelSBVH
*/

class MeshLevelSBVH : public SBVH {
public:
	MeshLevelSBVH(Mesh* m);						// Used only for creating a root node (that carries ALL primitives)
	MeshLevelSBVH(Mesh* m, vector<int> idxs);	// Used for any children of other nodes
	~MeshLevelSBVH();

	MeshLevelSBVH* left = nullptr;
	MeshLevelSBVH* right = nullptr;

	void Rebuild();
	bool Traverse(Ray& ray, int& material, float3& N, float& t, int* c = nullptr);
	void SubdivideRecursively();
	void Print();

private:
	Mesh* mesh;
	
	float SplitCost(vector<int> idxs);
	bool Subdivide();

	// Partitioning algorithms. Returns the "gain" of the split according to the SAH heuristic. Negative (or 0) values mean there is no gain.
	// Each algorithm takes reference parameters for indices and AABBs.
	float PartitionBinningSAH(vector<int>& l, float3& l_min, float3& l_max, vector<int>& r, float3& r_min, float3& r_max, int bin_count = 8);
	float PartitionBinningKD(vector<int>& l, float3& l_min, float3& l_max, vector<int>& r, float3& r_min, float3& r_max, int bin_count = 8);
};

/*
	TopLevelSBVH
*/

class TopLevelSBVH : public SBVH {
public:
	TopLevelSBVH() = default;										// Used only for creating a root node (that carries ALL primitives)
	TopLevelSBVH(vector<Instance*> instances, vector<int> idxs);	// Used for any children of other nodes
	~TopLevelSBVH();

	TopLevelSBVH* left = nullptr;
	TopLevelSBVH* right = nullptr;

	void AddInstance(Instance* instance);
	void Rebuild();
	bool Traverse(Ray& ray, int& material, float3& N, float& t, int* c = nullptr);
	void SubdivideRecursively();
	void Print();

private:	
	vector<Instance*> instance_vector; // Vector is guaranteed to not move in memory between assignment and usage
	
	float SplitCost(vector<int> idxs);
	bool Subdivide();
	void UpdateBounds();
	float PartitionBinningSAH(vector<int>& l, vector<int>& r, int bin_count = 8);
};

}