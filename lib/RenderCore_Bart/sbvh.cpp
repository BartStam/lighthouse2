#include "core_settings.h"

/*
	SBVH (abstract)
*/

bool SBVH::IntersectAABB(Ray& ray, float& t) {
	// Taken from: https://gamedev.stackexchange.com/a/18459

	float3 dir_frac = make_float3(0, 0, 0);

	// D is the unit direction of the ray
	dir_frac.x = 1.0f / ray.D.x;
	dir_frac.y = 1.0f / ray.D.y;
	dir_frac.z = 1.0f / ray.D.z;

	// O is the origin of ray
	float t1 = (min_bound.x - ray.O.x) * dir_frac.x;
	float t2 = (max_bound.x - ray.O.x) * dir_frac.x;
	float t3 = (min_bound.y - ray.O.y) * dir_frac.y;
	float t4 = (max_bound.y - ray.O.y) * dir_frac.y;
	float t5 = (min_bound.z - ray.O.z) * dir_frac.z;
	float t6 = (max_bound.z - ray.O.z) * dir_frac.z;

	float t_min = max(max(min(t1, t2), min(t3, t4)), min(t5, t6));
	float t_max = min(min(max(t1, t2), max(t3, t4)), max(t5, t6));

	// If tmax < 0, ray (line) is intersecting AABB, but the whole AABB is behind us
	if (t_max < 0) {
		t = t_max;
		return false;
	}

	// If tmin > tmax, ray doesn't intersect AABB
	if (t_min > t_max) {
		t = t_max;
		return false;
	}

	t = t_min;
	return true;
}

/*
	MeshLevelSBVH
*/

MeshLevelSBVH::MeshLevelSBVH(Mesh* m) : mesh(m) {
	indices.resize(mesh->vcount / 3);
}

MeshLevelSBVH::MeshLevelSBVH(Mesh* m, vector<int> idxs) : mesh(m) {
	indices = idxs;
}

MeshLevelSBVH::~MeshLevelSBVH() {
	delete left;
	delete right;
}

void MeshLevelSBVH::Rebuild() {
	// Recursively delete children if we are not a leaf
	if (!IsLeaf()) {
		delete left;
		delete right;
	}
	
	// Start with all primitives in the root node
	indices.resize(mesh->vcount / 3);
	for (int i = 0; i < indices.size(); i++) {
		indices[i] = i;
	}

	SubdivideRecursively();
}

bool MeshLevelSBVH::Traverse(Ray& ray, int& material, float3& N, float& t, int* c) {
	if (c) { (*c)++; } // If we are doing BVH debugging, increment the count of BVH intersections

	float initial_t = t; // Reference distance
	float found_t = t;

	if (IsLeaf()) {
		for (int i = 0; i < indices.size(); i++) {
			if (ray.IntersectTriangle((*mesh).triangles[indices[i]], found_t) && found_t < t) {
				t = found_t;
				CoreTri& tri = mesh->triangles[indices[i]];
				material = tri.material;
				N = make_float3(tri.Nx, tri.Ny, tri.Nz);
			}
		}
	}
	else {
		float t_left = FLT_MAX;		// Left child AABB intersection distance
		float t_right = FLT_MAX;	// Right child AABB intersection distance

		if (left->IntersectAABB(ray, t_left)) {
			if (right->IntersectAABB(ray, t_right)) {
				// Both children are intersected, traverse the closest child first
				if (t_left < t_right) {
					left->Traverse(ray, material, N, t, c);
					right->Traverse(ray, material, N, t, c);
				}
				else {
					right->Traverse(ray, material, N, t, c);
					left->Traverse(ray, material, N, t, c);
				}
			}
			else {
				// Only the left child is intersected
				left->Traverse(ray, material, N, t, c);
			}
		}
		else if (right->IntersectAABB(ray, t_right)) {
			// Only the right child is intersected
			right->Traverse(ray, material, N, t, c);
		}
	}

	return t < initial_t;
}

void MeshLevelSBVH::Print() {
	if (IsLeaf()) {
		cout << "[" << indices.size() << "]";
	}
	else {
		cout << "[";
		left->Print();
		right->Print();
		cout << "]";
	}
}

float MeshLevelSBVH::SplitCost(vector<int> idxs) {
	float3 min_b = make_float3(FLT_MAX);
	float3 max_b = -min_b;

	// Calculate AABB bounds
	for (int i = 0; i < idxs.size(); i++) {
		min_b = fminf(min_b, mesh->tri_min_bounds[indices[i]]);
		max_b = fmaxf(max_b, mesh->tri_max_bounds[indices[i]]);
	}

	float3 dims = max_b - min_b; // Dimensions of the AABB
	float area = 2 * dims.x * dims.y + 2 * dims.x * dims.z + 2 * dims.y * dims.z; // AABB surface area

	return idxs.size() * area; // SAH heuristic
}

float MeshLevelSBVH::PartitionBinningSAH(vector<int>& l, float3& l_min, float3& l_max, vector<int>& r, float3& r_min, float3& r_max, int bin_count) {
	// Calculate the centroid AABB (used as a reference for primitive positions)
	float3 c_min_bound = make_float3(FLT_MAX);
	float3 c_max_bound = -c_min_bound;

	for (int i = 0; i < indices.size(); i++) {
		c_min_bound = fminf(c_min_bound, mesh->tri_centers[indices[i]]);
		c_max_bound = fmaxf(c_max_bound, mesh->tri_centers[indices[i]]);
	}

	float base_cost = SplitCost(indices);	// The "cost" of this node before splitting
	float best_split_cost = FLT_MAX;		// The best split we've found thus far (no split by default)
	int best_split_plane;					// The plane to split over (0 = X; 1 = Y; 2 = Z)
	float best_split_pos;					// The position of the split plane

	// Find the best split using binning SAH
	for (int i = 0; i < 3; i++) { // X, Y, Z
		if (c_min_bound.at(i) == c_max_bound.at(i)) { continue; }				// If all primitives have the same position on this plane, don't split
		float interval = (c_max_bound.at(i) - c_min_bound.at(i)) / bin_count; // Interval between bins

		for (int j = 0; j < bin_count; j++) {
			float pos = c_min_bound.at(i) + j * interval; // The position of the bin

			vector<int> left_indices;
			vector<int> right_indices;
			
			// Perform the split for this bin
			for (int k = 0; k < indices.size(); k++) {
				float3 tri_center = mesh->tri_centers[indices[k]];

				if (tri_center.at(i) <= pos) {
					left_indices.push_back(indices[k]);
				}
				else {
					right_indices.push_back(indices[k]);
				}
			}

			float split_cost = SplitCost(left_indices) + SplitCost(right_indices); // Calculate the combined cost of these two splits

			// If it's less than the best split cost we've thus far found, we save this split
			if (split_cost + EPSILON < best_split_cost) {
				best_split_cost = split_cost;
				best_split_plane = i;
				best_split_pos = pos;
			}
		}
	}

	// If there is no value in splitting, don't split and exit early
	if (base_cost <= best_split_cost) { return base_cost - best_split_cost; }

	// Perform the best split that we found
	for (int i = 0; i < indices.size(); i++) {
		float3 tri_center = mesh->tri_centers[indices[i]];

		if (tri_center.at(best_split_plane) <= best_split_pos) {
			l.push_back(indices[i]);
		}
		else {
			r.push_back(indices[i]);
		}
	}

	for (int i = 0; i < l.size(); i++) {
		l_min = fminf(l_min, mesh->tri_min_bounds[l[i]]);
		l_max = fmaxf(l_max, mesh->tri_max_bounds[l[i]]);
	}

	for (int i = 0; i < r.size(); i++) {
		r_min = fminf(r_min, mesh->tri_min_bounds[r[i]]);
		r_max = fmaxf(r_max, mesh->tri_max_bounds[r[i]]);
	}

	return base_cost - best_split_cost; // Indicate that there was value in splitting
}

float MeshLevelSBVH::PartitionBinningKD(vector<int>& l, float3& l_min, float3& l_max, vector<int>& r, float3& r_min, float3& r_max, int bin_count) {
	// Calculate the base cost of this node
	float3 dims = max_bound - min_bound; // AABB dimensions of parent node
	float area = 2 * dims.x * dims.y + 2 * dims.x * dims.z + 2 * dims.y * dims.z;
	float base_cost = indices.size() * area;

	float best_split_cost = FLT_MAX;	// The best split we've found thus far (no split by default)
	int best_split_plane;				// The plane to split over (0 = X; 1 = Y; 2 = Z)
	float best_split_pos;				// The position of the split plane

	// Find the best split using binning SAH
	for (int i = 0; i < 3; i++) { // X, Y, Z
		float interval = dims.at(i) / bin_count; // Interval between bins

		for (int j = 0; j < bin_count; j++) {
			float pos = min_bound.at(i) + j * interval; // The position of the bin

			vector<int> left_indices;
			vector<int> right_indices;

			// Perform the split for this bin
			for (int k = 0; k < indices.size(); k++) {
				float3 tri_min_bound = mesh->tri_min_bounds[indices[k]];
				float3 tri_max_bound = mesh->tri_max_bounds[indices[k]];

				if (tri_min_bound.at(i) <= pos) {
					left_indices.push_back(indices[k]);

					if (tri_max_bound.at(i) > pos) {
						right_indices.push_back(indices[k]);
					}
				}
				else {
					right_indices.push_back(indices[k]);
				}
			}

			// Calculate split costs
			float3 left_min_bound = min_bound;
			float3 left_max_bound = max_bound;
			left_max_bound.at(i) = pos;

			float3 right_min_bound = min_bound;
			right_min_bound.at(i) = pos;
			float3 right_max_bound = max_bound;

			float3 left_dims = left_max_bound - left_min_bound;
			float3 right_dims = right_max_bound - right_min_bound;

			float left_area = 2 * left_dims.x * left_dims.y + 2 * left_dims.x * left_dims.z + 2 * left_dims.y * left_dims.z;
			float right_area = 2 * right_dims.x * right_dims.y + 2 * right_dims.x * right_dims.z + 2 * right_dims.y * right_dims.z;

			float split_cost = left_indices.size() * left_area + right_indices.size() * right_area;

			// If it's less than the best split cost we've thus far found, we save this split
			if (split_cost + EPSILON < best_split_cost) {
				best_split_cost = split_cost;
				best_split_plane = i;
				best_split_pos = pos;
			}
		}
	}

	// If there is no value in splitting, don't split and exit early
	if (base_cost <= best_split_cost) { return base_cost - best_split_cost; }

	// Perform the best split that we found
	for (int i = 0; i < indices.size(); i++) {
		float3 tri_min_bound = mesh->tri_min_bounds[indices[i]];
		float3 tri_max_bound = mesh->tri_max_bounds[indices[i]];

		if (tri_min_bound.at(best_split_plane) <= best_split_pos) {
			l.push_back(indices[i]);

			if (tri_max_bound.at(best_split_plane) > best_split_pos) {
				r.push_back(indices[i]);
			}
		}
		else {
			r.push_back(indices[i]);
		}
	}

	l_min = min_bound;
	l_max = max_bound;
	l_max.at(best_split_plane) = best_split_pos;

	r_min = min_bound;
	r_min.at(best_split_plane) = best_split_pos;
	r_max = max_bound;

	//cout << endl;
	//cout << "Best split plane: " << best_split_plane << endl;
	//cout << "Best split pos: " << best_split_pos << endl;
	//cout << "  Parent min: " << min_bound.x << ", " << min_bound.y << ", " << min_bound.z << endl;
	//cout << "  Parent max: " << max_bound.x << ", " << max_bound.y << ", " << max_bound.z << endl;
	//cout << "  Parent dims: " << max_bound.x - min_bound.x << ", " << max_bound.y - min_bound.y << ", " << max_bound.z - min_bound.z << endl;
	//cout << "    Left min: " << l_min.x << ", " << l_min.y << ", " << l_min.z << endl;
	//cout << "    Left max: " << l_max.x << ", " << l_max.y << ", " << l_max.z << endl;
	//cout << "    Left dims: " << l_max.x - l_min.x << ", " << l_max.y - l_min.y << ", " << l_max.z - l_min.z << endl;
	//cout << "      Right min: " << r_min.x << ", " << r_min.y << ", " << r_min.z << endl;
	//cout << "      Right max: " << r_max.x << ", " << r_max.y << ", " << r_max.z << endl;
	//cout << "      Right dims: " << r_max.x - r_min.x << ", " << r_max.y - r_min.y << ", " << r_max.z - r_min.z << endl;

	return base_cost - best_split_cost; // Indicate that there was value in splitting
}

bool MeshLevelSBVH::Subdivide() {
	if (indices.size() < 16) { return false; } // Leaves have at least 2 primitives in them

	// These are passed to Partition() as references
	vector<int> left_indices;
	float3 l_min = make_float3(FLT_MAX);
	float3 l_max = make_float3(-FLT_MAX);

	vector<int> right_indices;
	float3 r_min = make_float3(FLT_MAX);
	float3 r_max = make_float3(-FLT_MAX);

	// If there is value in splitting according to Partition() implementation
	if (PartitionBinningKD(left_indices, l_min, l_max, right_indices, r_min, r_max) > 0.0f) {
		// Turn this SBVH from a leaf into a node with children
		left = new MeshLevelSBVH(mesh, left_indices);
		left->min_bound = l_min;
		left->max_bound = l_max;

		right = new MeshLevelSBVH(mesh, right_indices);
		right->min_bound = r_min;
		right->max_bound = r_max;

		indices.clear();

		return true;
	}

	return false;
}

void MeshLevelSBVH::SubdivideRecursively() {
	// If there is value in subdividing this SBVH, do so and attempt to subdivide its children
	if (Subdivide()) {
		left->SubdivideRecursively();
		right->SubdivideRecursively();
	}
}

/*
	TopLevelSBVH
*/

TopLevelSBVH::TopLevelSBVH(vector<Instance*> instances, vector<int> idxs) {
	instance_vector = instances;
	indices = idxs;
}

void TopLevelSBVH::AddInstance(Instance* instance) {
	instance_vector.push_back(instance);
}

TopLevelSBVH::~TopLevelSBVH() {
	delete left;
	delete right;
}

void TopLevelSBVH::Rebuild() {
	// Recursively delete children if we are not a leaf
	if (!IsLeaf()) {
		delete left;
		delete right;
	}
	
	// Start with all primitives in the root node
	indices.resize(instance_vector.size());
	for (int i = 0; i < indices.size(); i++) {
		indices[i] = i;
	}

	SubdivideRecursively();
	UpdateBounds();
}

bool TopLevelSBVH::Traverse(Ray& ray, int& material, float3& N, float& t, int* c) {
	if (c) { (*c)++; } // If we are doing BVH debugging, increment the count of BVH intersections

	float initial_t = t; // Reference distance
	float found_t = t;

	if (IsLeaf()) {
		for (int i = 0; i < indices.size(); i++) {
			float t_aabb = FLT_MAX;

			// We're entering an instance-level BVH, so we transform the ray by the inverse of its local transform
			Ray transformed_ray = ray;
			mat4 inv_transform = instance_vector[indices[i]]->transform.Inverted();
			transformed_ray.O = inv_transform.TransformPoint(transformed_ray.O);
			transformed_ray.D = normalize(inv_transform.TransformVector(transformed_ray.D));

			SBVH* bvh = instance_vector[indices[i]]->mesh->bvh;

			if (bvh->IntersectAABB(transformed_ray, t_aabb) && t_aabb < t) {
				bvh->Traverse(transformed_ray, material, N, t, c);

				// If we find a closer intersection than what we previously found
				if (t < found_t) {
					// Transform the normal (in case of translation) and normalize the normal (in case of scaling)
					N = normalize(instance_vector[indices[i]]->transform.TransformVector(N));
					found_t = t;
				}
			}
		}
	}
	else {
		float t_left = FLT_MAX;		// Left child AABB intersection distance
		float t_right = FLT_MAX;	// Right child AABB intersection distance

		if (left->IntersectAABB(ray, t_left) && t_left < t) {
			if (right->IntersectAABB(ray, t_right) && t_right < t) {
				// Both children are intersected, traverse the closest child first
				if (t_left < t_right) {
					left->Traverse(ray, material, N, t, c);
					right->Traverse(ray, material, N, t, c);
				}
				else {
					right->Traverse(ray, material, N, t, c);
					left->Traverse(ray, material, N, t, c);
				}
			}
			else {
				// Only the left child is intersected
				left->Traverse(ray, material, N, t, c);
			}
		}
		else if (right->IntersectAABB(ray, t_right) && t_right < t) {
			// Only the right child is intersected
			right->Traverse(ray, material, N, t, c);
		}
	}

	return t < initial_t;
}

void TopLevelSBVH::Print() {
	if (IsLeaf()) {
		cout << "[" << indices.size() << "]";
	}
	else {
		cout << "[";
		left->Print();
		right->Print();
		cout << "]";
	}
}

float TopLevelSBVH::SplitCost(vector<int> idxs) {
	float3 min_b = make_float3(FLT_MAX);
	float3 max_b = -min_b;

	// Calculate AABB bounds
	for (int i = 0; i < idxs.size(); i++) {
		const SBVH* bvh = instance_vector[indices[i]]->mesh->bvh;

		min_b = fminf(min_b, bvh->min_bound);
		max_b = fmaxf(max_b, bvh->max_bound);
	}

	float3 dims = max_b - min_b; // Dimensions of the AABB
	float area = 2 * dims.x * dims.y + 2 * dims.x * dims.z + 2 * dims.y * dims.z; // AABB surface area

	return idxs.size() * area; // SAH heuristic
}

float TopLevelSBVH::PartitionBinningSAH(vector<int>& l, vector<int>& r, int bin_count) {
	// Calculate the centroid AABB (used as a reference for primitive positions)
	float3 c_min_bound = make_float3(FLT_MAX);
	float3 c_max_bound = -c_min_bound;

	for (int i = 0; i < indices.size(); i++) {
		float3 aabb_center = instance_vector[indices[i]]->mesh->aabb_center;
		aabb_center = instance_vector[indices[i]]->transform.TransformPoint(aabb_center);

		c_min_bound = fminf(c_min_bound, aabb_center);
		c_max_bound = fmaxf(c_max_bound, aabb_center);
	}

	float base_cost = SplitCost(indices);	// The "cost" of this node before splitting
	float best_split_cost = FLT_MAX;		// The best split we've found thus far (no split by default)
	int best_split_plane;					// The plane to split over (0 = X; 1 = Y; 2 = Z)
	float best_split_pos;					// The position of the split plane

	// Find the best split using binning SAH
	for (int i = 0; i < 3; i++) { // X, Y, Z
		if (c_min_bound.at(i) == c_max_bound.at(i)) { continue; }				// If all primitives have the same position on this plane, don't split
		float interval = (c_max_bound.at(i) - c_min_bound.at(i)) / bin_count; // Interval between bins

		for (int j = 0; j < bin_count; j++) {
			float pos = c_min_bound.at(i) + j * interval; // The position of the bin

			vector<int> left_indices;
			vector<int> right_indices;

			// Perform the split for this bin
			for (int k = 0; k < indices.size(); k++) {
				float3 aabb_center = instance_vector[indices[k]]->mesh->aabb_center;
				aabb_center = instance_vector[indices[k]]->transform.TransformPoint(aabb_center);

				if (aabb_center.at(i) <= pos) {
					left_indices.push_back(indices[k]);
				}
				else {
					right_indices.push_back(indices[k]);
				}
			}

			float split_cost = SplitCost(left_indices) + SplitCost(right_indices); // Calculate the combined cost of these two splits

			// If it's less than the best split cost we've thus far found, we save this split
			if (split_cost + EPSILON < best_split_cost) {
				best_split_cost = split_cost;
				best_split_plane = i;
				best_split_pos = pos;
			}
		}
	}

	// If there is no value in splitting, don't split and exit early
	if (base_cost <= best_split_cost) { return base_cost - best_split_cost; }

	// Perform the best split that we found
	for (int i = 0; i < indices.size(); i++) {
		float3 aabb_center = instance_vector[indices[i]]->mesh->aabb_center;
		aabb_center = instance_vector[indices[i]]->transform.TransformPoint(aabb_center);

		if (aabb_center.at(best_split_plane) <= best_split_pos) {
			l.push_back(indices[i]);
		}
		else {
			r.push_back(indices[i]);
		}
	}

	return base_cost - best_split_cost;
}

bool TopLevelSBVH::Subdivide() {
	if (indices.size() < 4) { return false; } // Leaves have at least 2 primitives in them

	// These are passed to Partition() as references
	vector<int> left_indices;
	vector<int> right_indices;

	// If there is value in splitting according to Partition() implementation
	if (PartitionBinningSAH(left_indices, right_indices) > 0.0f) {
		// Turn this SBVH from a leaf into a node with children
		left = new TopLevelSBVH(instance_vector, left_indices);
		right = new TopLevelSBVH(instance_vector, right_indices);

		indices.clear();

		return true;
	}

	return false;
}

void TopLevelSBVH::SubdivideRecursively() {
	// If there is value in subdividing this SBVH, do so and attempt to subdivide its children
	if (Subdivide()) {
		left->SubdivideRecursively();
		right->SubdivideRecursively();
	}
}

void TopLevelSBVH::UpdateBounds() {
	// If this BVH is a leaf then we determine the AABB bounds from its primitives (mesh-level SBVHs)
	// If it's not a leaf, we recursively update its children's bounds and determine our own based on theirs
	if (IsLeaf()) {
		min_bound = make_float3(FLT_MAX);
		max_bound = -min_bound;

		for (int i = 0; i < indices.size(); i++) {
			const Instance* instance = instance_vector[indices[i]];

			float3 minb = instance->mesh->bvh->min_bound;
			float3 maxb = instance->mesh->bvh->max_bound;

			// Transform the mesh AABB bounds according to the instance transform
			minb = instance->transform.TransformPoint(minb);
			maxb = instance->transform.TransformPoint(maxb);

			min_bound = fminf(min_bound, minb);
			max_bound = fmaxf(max_bound, maxb);
		}
	}
	else {
		left->UpdateBounds();
		right->UpdateBounds();

		min_bound = fminf(left->min_bound, right->min_bound);
		max_bound = fmaxf(left->max_bound, right->max_bound);
	}
}