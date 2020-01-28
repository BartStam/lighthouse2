#include "core_settings.h"

bool PointInAABB(float3 min_b, float3 max_b, float3 point) {
	return (point.x + EPSILON >= min_b.x) & (point.x - EPSILON <= max_b.x)
		 & (point.y + EPSILON >= min_b.y) & (point.y - EPSILON <= max_b.y)
		 & (point.z + EPSILON >= min_b.z) & (point.z - EPSILON <= max_b.z);
}

/*
	SBVH (abstract)
*/

bool SBVH::IntersectAABB(Ray& ray, float& t) {
	return IntersectAABB(ray, min_bound, max_bound, t);
}

bool SBVH::IntersectAABB(Ray& ray, float3 min_b, float3 max_b, float& t) {
	// Taken from: https://gamedev.stackexchange.com/a/18459

	float3 dir_frac = make_float3(0, 0, 0);

	// D is the unit direction of the ray
	dir_frac.x = 1.0f / ray.D.x;
	dir_frac.y = 1.0f / ray.D.y;
	dir_frac.z = 1.0f / ray.D.z;

	// O is the origin of ray
	float t1 = (min_b.x - ray.O.x) * dir_frac.x;
	float t2 = (max_b.x - ray.O.x) * dir_frac.x;
	float t3 = (min_b.y - ray.O.y) * dir_frac.y;
	float t4 = (max_b.y - ray.O.y) * dir_frac.y;
	float t5 = (min_b.z - ray.O.z) * dir_frac.z;
	float t6 = (max_b.z - ray.O.z) * dir_frac.z;

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
	indices.resize(idxs.size());
	for (int i = 0; i < idxs.size(); i++) {
		indices[i] = idxs[i];
	}
}

MeshLevelSBVH::~MeshLevelSBVH() {
	delete left;
	delete right;
}

void MeshLevelSBVH::Rebuild() {
	// Recursively delete children if we are not a leaf
	if (!is_leaf) {
		delete left;
		delete right;
	}
	
	// Start with all primitives in the root node
	indices.resize(mesh->vcount / 3);
	for (int i = 0; i < indices.size(); i++) {
		indices[i] = i;
	}

	// Calculate root node cost (with AABB bounds tightly bounding the geometry)
	cost = SplitCost(indices);
	SubdivideRecursively();
}

bool MeshLevelSBVH::Traverse(Ray& ray, int& material, float3& N, float& t, int* c) {
	if (c) { (*c)++; } // If we are doing BVH debugging, increment the count of BVH intersections

	float initial_t = t; // Reference distance
	float found_t = t;

	if (is_leaf) {
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

float MeshLevelSBVH::Cost() {
	if (is_leaf) {
		return cost;
	}
	else {
		return left->Cost() + right->Cost();
	}
}

void MeshLevelSBVH::Print() {
	if (is_leaf) {
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

float MeshLevelSBVH::SplitCost(int count, float3 min_b, float3 max_b, int split_plane, float split_pos) {
	float3 dims = max_b - min_b;
	float area = 2 * dims.x * dims.y + 2 * dims.x * dims.z + 2 * dims.y * dims.z;

	return count * area;
}

void MeshLevelSBVH::ClipAABB(vector<int> idxs, float3& min_b, float3& max_b, int split_plane, float split_pos) {
	// Sutherland-Hodgman from https://en.wikipedia.org/wiki/Sutherland%E2%80%93Hodgman_algorithm
	float3* tri = new float3[3];

	float3 clipped_min_b = make_float3(FLT_MAX);
	float3 clipped_max_b = make_float3(-FLT_MAX);

	float3 dims = max_b - min_b;
	float before_area = 2 * dims.x * dims.y + 2 * dims.x * dims.z + 2 * dims.y * dims.z;

	// For every triangle
	for (int i = 0; i < idxs.size(); i++) {
		tri[0] = mesh->triangles[idxs[i]].vertex0;
		tri[1] = mesh->triangles[idxs[i]].vertex1;
		tri[2] = mesh->triangles[idxs[i]].vertex2;

		for (int j = 0; j < 3; j++) { // For all three vertices in this triangle
			float3 point = tri[j];
			float3 prev_point = tri[(j + 2) % 3];

			if (point.at(split_plane) <= split_pos) {
				if (prev_point.at(split_plane) >= split_pos) {
					Ray ray(prev_point, point - prev_point);
					float t = (split_pos - ray.O.at(split_plane)) / ray.D.at(split_plane);
					float3 intersection_point = ray.Point(t);

					clipped_min_b = fminf(clipped_min_b, intersection_point);
					clipped_max_b = fmaxf(clipped_max_b, intersection_point);
				}
				clipped_min_b = fminf(clipped_min_b, point);
				clipped_max_b = fmaxf(clipped_max_b, point);
			}
			else if (prev_point.at(split_plane) <= split_pos) {
				Ray ray(prev_point, point - prev_point);
				float t = (split_pos - ray.O.at(split_plane)) / ray.D.at(split_plane);
				float3 intersection_point = ray.Point(t);

				clipped_min_b = fminf(clipped_min_b, intersection_point);
				clipped_max_b = fmaxf(clipped_max_b, intersection_point);
			}
		}
	}

	dims = clipped_max_b - clipped_min_b;
	float after_area = 2 * dims.x * dims.y + 2 * dims.x * dims.z + 2 * dims.y * dims.z;

	if (after_area < before_area) {
		min_b = clipped_min_b;
		max_b = clipped_max_b;
	}


	//cout << "Count:      " << idxs.size() << endl;
	//cout << "Before:     " << before_area << endl;
	//cout << "After:      " << after_area << endl;
	//cout << "Difference: " << before_area - after_area << endl;
	//cout << endl;

	delete[] tri;

	//cout << "Clipped" << endl;
	//cout << "  Outside:   " << prev_point.x << ", " << prev_point.y << ", " << prev_point.z << endl;
	//cout << "  Inside:    " << point.x << ", " << point.y << ", " << point.z << endl;
	//cout << "  AABB min:  " << min_b.x << ", " << min_b.y << ", " << min_b.z << endl;
	//cout << "  AABB max:  " << max_b.x << ", " << max_b.y << ", " << max_b.z << endl;
	//cout << "  Intersect: " << intersection_point.x << ", " << intersection_point.y << ", " << intersection_point.z << endl;
}

float MeshLevelSBVH::PartitionBinningSAH(vector<int>& l, float3& l_min, float3& l_max, float& l_cost, vector<int>& r, float3& r_min, float3& r_max, float& r_cost, int bin_count) {
	// Calculate the centroid AABB (used as a reference for primitive positions)
	float3 c_min_bound = make_float3(FLT_MAX);
	float3 c_max_bound = -c_min_bound;

	for (int i = 0; i < indices.size(); i++) {
		c_min_bound = fminf(c_min_bound, mesh->tri_centers[indices[i]]);
		c_max_bound = fmaxf(c_max_bound, mesh->tri_centers[indices[i]]);
	}

	float best_split_cost = FLT_MAX;	// The best split we've found thus far (no split by default)
	int best_split_plane;				// The plane to split over (0 = X; 1 = Y; 2 = Z)
	float best_split_pos;				// The position of the split plane

	// Find the best split using binning SAH
	for (int i = 0; i < 3; i++) { // X, Y, Z
		if (c_min_bound.at(i) == c_max_bound.at(i)) { continue; }				// If all primitives have the same position on this plane, don't split
		float interval = (c_max_bound.at(i) - c_min_bound.at(i)) / bin_count;	// Interval between bins

		for (int j = 0; j < bin_count; j++) {
			float pos = c_min_bound.at(i) + j * interval; // The position of the bin

			// Perform the split for this bin
			for (int k = 0; k < indices.size(); k++) {
				float3 tri_center = mesh->tri_centers[indices[k]];

				if (tri_center.at(i) <= pos) {
					l.push_back(indices[k]);
				}
				else {
					r.push_back(indices[k]);
				}
			}

			// Calculate the cost of this split
			float left_cost = SplitCost(l);
			float right_cost = SplitCost(r);
			float split_cost = left_cost + right_cost;

			// If it's less than the best split cost we've thus far found, we save this split
			if (split_cost + EPSILON < best_split_cost) {
				best_split_cost = split_cost;
				best_split_plane = i;
				best_split_pos = pos;
				l_cost = left_cost;
				r_cost = right_cost;
			}

			l.clear();
			r.clear();
		}
	}

	// If there is no value in splitting, don't split and exit early
	if (cost - EPSILON <= best_split_cost) { return cost - best_split_cost; }

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

	return cost - best_split_cost; // Indicate that there was value in splitting
}

float MeshLevelSBVH::PartitionBinningKD(vector<int>& l, float3& l_min, float3& l_max, float& l_cost, vector<int>& r, float3& r_min, float3& r_max, float& r_cost, int bin_count) {
	// Calculate the base cost of this node
	float3 dims = max_bound - min_bound; // AABB dimensions of parent node
	float area = 2 * dims.x * dims.y + 2 * dims.x * dims.z + 2 * dims.y * dims.z;
	float base_cost = indices.size() * area;

	float best_split_cost = FLT_MAX;	// The best split we've found thus far (no split by default)
	int best_split_plane;				// The plane to split over (0 = X; 1 = Y; 2 = Z)
	float best_split_pos;				// The position of the split plane

	// Find the best split using binning SAH
	for (int i = 0; i < 3; i++) { // X, Y, Z
		float interval = dims.at(i) / (bin_count + 1); // Interval between bins

		for (int j = 1; j <= bin_count; j++) {
			float pos = min_bound.at(i) + j * interval; // The position of the bin

			// Define left and right AABB bounds
			float3 l_min_bound = min_bound;
			float3 l_max_bound = max_bound;
			l_max_bound.at(i) = pos;
			float3 r_min_bound = min_bound;
			float3 r_max_bound = max_bound;
			r_min_bound.at(i) = pos;

			// Perform the split for this bin
			for (int k = 0; k < indices.size(); k++) {
				float3 tri_min_bound = mesh->tri_min_bounds[indices[k]];
				float3 tri_max_bound = mesh->tri_max_bounds[indices[k]];

				if (tri_min_bound.at(i) <= pos) {
					l.push_back(indices[k]);

					if (tri_max_bound.at(i) > pos) {
						r.push_back(indices[k]);
					}
				}
				else {
					r.push_back(indices[k]);
				}
			}

			// Calculate the cost of this split
			float left_cost = SplitCost(l.size(), l_min_bound, l_max_bound, i, pos);
			float right_cost = SplitCost(r.size(), r_min_bound, r_max_bound, i, pos);
			float split_cost = left_cost + right_cost;

			// If it's less than the best split cost we've thus far found, we save this split
			if (split_cost + EPSILON < best_split_cost) {
				best_split_cost = split_cost;
				best_split_plane = i;
				best_split_pos = pos;
				l_cost = left_cost;
				r_cost = right_cost;
			}

			l.clear();
			r.clear();
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

	return base_cost - best_split_cost; // Indicate that there was value in splitting
}

float MeshLevelSBVH::PartitionBinningKD_Clip(vector<int>& l, float3& l_min, float3& l_max, float& l_cost, vector<int>& r, float3& r_min, float3& r_max, float& r_cost, int bin_count) {
	// Calculate the base cost of this node
	float3 dims = max_bound - min_bound; // AABB dimensions of parent node
	float area = 2 * dims.x * dims.y + 2 * dims.x * dims.z + 2 * dims.y * dims.z;
	float base_cost = indices.size() * area;

	float best_split_cost = FLT_MAX;	// The best split we've found thus far (no split by default)
	int best_split_plane;				// The plane to split over (0 = X; 1 = Y; 2 = Z)
	float best_split_pos;				// The position of the split plane

	// Find the best split using binning SAH
	for (int i = 0; i < 3; i++) { // X, Y, Z
		float interval = dims.at(i) / (bin_count + 1); // Interval between bins

		for (int j = 1; j <= bin_count; j++) {
			float pos = min_bound.at(i) + j * interval; // The position of the bin

			// Define left and right AABB bounds
			float3 l_min_bound = min_bound;
			float3 l_max_bound = max_bound;
			l_max_bound.at(i) = pos;
			float3 r_min_bound = min_bound;
			float3 r_max_bound = max_bound;
			r_min_bound.at(i) = pos;

			// Perform the split for this bin
			for (int k = 0; k < indices.size(); k++) {
				float3 tri_min_bound = mesh->tri_min_bounds[indices[k]];
				float3 tri_max_bound = mesh->tri_max_bounds[indices[k]];

				if (tri_min_bound.at(i) <= pos) {
					l.push_back(indices[k]);

					if (tri_max_bound.at(i) > pos) {
						r.push_back(indices[k]);
					}
				}
				else {
					r.push_back(indices[k]);
				}
			}

			// Calculate the cost of this split
			float left_cost = SplitCost(l.size(), l_min_bound, l_max_bound, i, pos);
			float right_cost = SplitCost(r.size(), r_min_bound, r_max_bound, i, pos);
			float split_cost = left_cost + right_cost;

			// If it's less than the best split cost we've thus far found, we save this split
			if (split_cost + EPSILON < best_split_cost) {
				best_split_cost = split_cost;
				best_split_plane = i;
				best_split_pos = pos;
				l_cost = left_cost;
				r_cost = right_cost;
			}

			l.clear();
			r.clear();
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
	float3 l_dims = l_max - l_min;
	float l_area = 2 * dims.x * dims.y + 2 * dims.x * dims.z + 2 * dims.y * dims.z;

	r_min = min_bound;
	r_min.at(best_split_plane) = best_split_pos;
	r_max = max_bound;
	float3 r_dims = r_max - r_min;
	float r_area = 2 * dims.x * dims.y + 2 * dims.x * dims.z + 2 * dims.y * dims.z;

	// Clip bounds
	if (l_area > 1) {
		ClipAABB(l, l_min, l_max, best_split_plane, best_split_pos);
	}
	
	if (r_area > 1) {
		ClipAABB(r, r_min, r_max, best_split_plane, best_split_pos);
	}

	// Recalculate cost
	best_split_cost = SplitCost(l.size(), l_min, l_max, best_split_plane, best_split_pos) + SplitCost(r.size(), r_min, r_max, best_split_plane, best_split_pos);

	return base_cost - best_split_cost; // Indicate that there was value in splitting
}

float MeshLevelSBVH::PartitionBinningKDSAH(vector<int>& l, float3& l_min, float3& l_max, float& l_cost, vector<int>& r, float3& r_min, float3& r_max, float& r_cost, int bin_count) {
	vector<int> left_SAH;
	left_SAH.reserve(indices.size());
	float3 l_min_SAH = make_float3(FLT_MAX);
	float3 l_max_SAH = make_float3(-FLT_MAX);
	float l_cost_SAH;

	vector<int> right_SAH;
	right_SAH.reserve(indices.size());
	float3 r_min_SAH = make_float3(FLT_MAX);
	float3 r_max_SAH = make_float3(-FLT_MAX);
	float r_cost_SAH;

	float score_SAH = PartitionBinningSAH(left_SAH, l_min_SAH, l_max_SAH, l_cost_SAH, right_SAH, r_min_SAH, r_max_SAH, r_cost_SAH, bin_count);

	vector<int> left_KD;
	left_KD.reserve(indices.size());
	float3 l_min_KD = make_float3(FLT_MAX);
	float3 l_max_KD = make_float3(-FLT_MAX);
	float l_cost_KD;

	vector<int> right_KD;
	right_KD.reserve(indices.size());
	float3 r_min_KD = make_float3(FLT_MAX);
	float3 r_max_KD = make_float3(-FLT_MAX);
	float r_cost_KD;

	float score_KD = PartitionBinningKD(left_KD, l_min_KD, l_max_KD, l_cost_KD, right_KD, r_min_KD, r_max_KD, r_cost_KD, bin_count);

	if (score_SAH > score_KD) {
		l = left_SAH;
		l_min = l_min_SAH;
		l_max = l_max_SAH;
		l_cost = l_cost_SAH;
		r = right_SAH;
		r_min = r_min_SAH;
		r_max = r_max_SAH;
		r_cost = r_cost_SAH;
		return score_SAH;
	}
	else {
		l = left_KD;
		l_min = l_min_KD;
		l_max = l_max_KD;
		l_cost = l_cost_KD;
		r = right_KD;
		r_min = r_min_KD;
		r_max = r_max_KD;
		r_cost = r_cost_KD;
		return score_KD;
	}
}

float MeshLevelSBVH::PartitionBinningKDSAH_R(vector<int>& l, float3& l_min, float3& l_max, float& l_cost, vector<int>& r, float3& r_min, float3& r_max, float& r_cost, int bin_count) {
	vector<int> left_SAH;
	left_SAH.reserve(indices.size());
	float3 l_min_SAH = make_float3(FLT_MAX);
	float3 l_max_SAH = make_float3(-FLT_MAX);
	float l_cost_SAH;

	vector<int> right_SAH;
	right_SAH.reserve(indices.size());
	float3 r_min_SAH = make_float3(FLT_MAX);
	float3 r_max_SAH = make_float3(-FLT_MAX);
	float r_cost_SAH;

	float score_SAH = PartitionBinningSAH(left_SAH, l_min_SAH, l_max_SAH, l_cost_SAH, right_SAH, r_min_SAH, r_max_SAH, r_cost_SAH, bin_count);

	float3 dims = max_bound - min_bound;
	float parent_area = 2 * dims.x * dims.y + 2 * dims.x * dims.z + 2 * dims.y * dims.z;
	float overlap_area = (min(l_max_SAH.x, r_max_SAH.x) - max(l_min_SAH.x, r_min_SAH.x))
					   * (min(l_max_SAH.y, r_max_SAH.y) - max(l_min_SAH.y, r_min_SAH.y))
					   * (min(l_max_SAH.z, r_max_SAH.z) - max(l_min_SAH.z, r_min_SAH.z));

	if ((overlap_area / parent_area) > ALPHA) {
		l = left_SAH;
		l_min = l_min_SAH;
		l_max = l_max_SAH;
		l_cost = l_cost_SAH;
		r = right_SAH;
		r_min = r_min_SAH;
		r_max = r_max_SAH;
		r_cost = r_cost_SAH;

		return score_SAH;
	}

	vector<int> left_KD;
	left_KD.reserve(indices.size());
	float3 l_min_KD = make_float3(FLT_MAX);
	float3 l_max_KD = make_float3(-FLT_MAX);
	float l_cost_KD;

	vector<int> right_KD;
	right_KD.reserve(indices.size());
	float3 r_min_KD = make_float3(FLT_MAX);
	float3 r_max_KD = make_float3(-FLT_MAX);
	float r_cost_KD;

	float score_KD = PartitionBinningKD(left_KD, l_min_KD, l_max_KD, l_cost_KD, right_KD, r_min_KD, r_max_KD, r_cost_KD, bin_count);

	if (score_SAH > score_KD) {
		l = left_SAH;
		l_min = l_min_SAH;
		l_max = l_max_SAH;
		l_cost = l_cost_SAH;
		r = right_SAH;
		r_min = r_min_SAH;
		r_max = r_max_SAH;
		r_cost = r_cost_SAH;

		return score_SAH;
	}
	else {
		l = left_KD;
		l_min = l_min_KD;
		l_max = l_max_KD;
		l_cost = l_cost_KD;
		r = right_KD;
		r_min = r_min_KD;
		r_max = r_max_KD;
		r_cost = r_cost_KD;

		return score_KD;
	}
}

bool MeshLevelSBVH::Subdivide() {
	if (indices.size() < 4) { return false; } // Leaves have at least 2 primitives in them

	// These are passed to Partition() as references
	vector<int> left_indices;
	left_indices.reserve(indices.size());
	float3 l_min = make_float3(FLT_MAX);
	float3 l_max = make_float3(-FLT_MAX);
	float l_cost;

	vector<int> right_indices;
	right_indices.reserve(indices.size());
	float3 r_min = make_float3(FLT_MAX);
	float3 r_max = make_float3(-FLT_MAX);
	float r_cost;

	// If there is value in splitting according to Partition() implementation
	if (PartitionBinningKD(left_indices, l_min, l_max, l_cost, right_indices, r_min, r_max, r_cost) > 0.0f) {
		// Turn this SBVH from a leaf into a node with children
		left = new MeshLevelSBVH(mesh, left_indices);
		left->min_bound = l_min;
		left->max_bound = l_max;
		left->cost = l_cost;
		
		right = new MeshLevelSBVH(mesh, right_indices);
		right->min_bound = r_min;
		right->max_bound = r_max;
		right->cost = r_cost;
		
		cost = 0;
		indices.clear();
		is_leaf = false;

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

	indices.resize(idxs.size());
	for (int i = 0; i < idxs.size(); i++) {
		indices[i] = idxs[i];
	}
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
	if (!is_leaf) {
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

	if (is_leaf) {
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

float TopLevelSBVH::Cost() {
	if (is_leaf) {
		float total_cost = 0;
		for (int i = 0; i < indices.size(); i++) {
			total_cost += instance_vector[indices[i]]->mesh->bvh->Cost();
		}
		return total_cost;
	}
	else {
		return left->Cost() + right->Cost();
	}
}

void TopLevelSBVH::Print() {
	if (is_leaf) {
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
		is_leaf = false;

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
	if (is_leaf) {
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