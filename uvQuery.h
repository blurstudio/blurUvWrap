#include <vector>

// Sweep line algorithm for finding which triangle contains a given query point
// but for a list of query points
void sweep(
		const std::vector<uv_t> &qPoints, // the UV query points
		const std::vector<uv_t> &uvs,     // the triangulated UV's
		const std::vector<size_t> &tris,  // the flattened triangle indexes

		std::vector<size_t> &out,         // the tri-index per qPoint
		std::vector<size_t> &missing      // Any qPoints that weren't in a triangle
);


