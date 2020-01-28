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

	float cost = 0;
	bool is_leaf = true;
	float3 min_bound = make_float3(FLT_MAX);
	float3 max_bound = make_float3(-FLT_MAX);

	bool IntersectAABB(Ray& ray, float& t);
	bool IntersectAABB(Ray& ray, float3 min_b, float3 max_b, float& t);
	virtual void Rebuild() = 0;
	virtual bool Traverse(Ray& ray, int& material, float3& N, float& t, int* c = nullptr) = 0;
	virtual void SubdivideRecursively() = 0;
	virtual float Cost() = 0;							// Recursively calculates the complete cost of this BVH
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
	float Cost();
	void Print();

private:
	Mesh* mesh;
	
	float SplitCost(vector<int> idxs);																	// Used for object splits
	float SplitCost(int count, float3 min_b, float3 max_b, int split_plane, float split_pos);			// Used for spatial splits
	void ClipAABB(vector<int> idxs, float3& min_b, float3& max_b, int split_plane, float split_pos);	// Clips geometry in an AABB to minimize AABB surface area
	bool Subdivide();

	float ALPHA = .2; // Parameter for restricting spatial split attempts (see PartitionBinningKDSAH_R())

	// Partitioning algorithms. Returns the "gain" of the split according to the SAH heuristic. Negative (or 0) values mean there is no gain.
	// Each algorithm takes reference parameters for indices and AABBs.
	float PartitionBinningSAH(vector<int>& l, float3& l_min, float3& l_max, float& l_cost,
		vector<int>& r, float3& r_min, float3& r_max, float& r_cost, int bin_count = 8);				// Object-split partitioning with binning
	float PartitionBinningKD(vector<int>& l, float3& l_min, float3& l_max, float& l_cost,
		vector<int>& r, float3& r_min, float3& r_max, float& r_cost, int bin_count = 8);				// Spatial-split partitioning with binning
	float PartitionBinningKD_Clip(vector<int>& l, float3& l_min, float3& l_max, float& l_cost,
		vector<int>& r, float3& r_min, float3& r_max, float& r_cost, int bin_count = 8);				// Spatial-split partitioning with AABB clipping (not working correctly unfortunately)
	float PartitionBinningKDSAH(vector<int>& l, float3& l_min, float3& l_max, float& l_cost,
		vector<int>& r, float3& r_min, float3& r_max, float& r_cost, int bin_count = 8);				// Decides between object and spatial-split partitioning at every split based on SAH heuristic
	float PartitionBinningKDSAH_R(vector<int>& l, float3& l_min, float3& l_max, float& l_cost,
		vector<int>& r, float3& r_min, float3& r_max, float& r_cost, int bin_count = 8);				// Described in section 4.5 of https://www.nvidia.com/docs/IO/77714/sbvh.pdf
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
	float Cost();
	void Print();

private:	
	vector<Instance*> instance_vector; // Vector is guaranteed to not move in memory between assignment and usage
	
	float SplitCost(vector<int> idxs);
	bool Subdivide();
	void UpdateBounds();
	float PartitionBinningSAH(vector<int>& l, vector<int>& r, int bin_count = 8);
};

}