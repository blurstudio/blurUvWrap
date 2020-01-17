#pragma once
#include <vector>
#include <utility>
#include "uvMath.h"

typedef std::array<size_t, 2> edge_t;

struct edgeHash {
	// Use the boost 
	template <class T1>
	size_t operator()(const std::array<T1, 2> &edge) const {
		std::hash<T1> hasher;
		size_t h1 = hasher(edge[0]);
		size_t h2 = hasher(edge[1]);
		return h1 ^ (h2 + 0x9e3779b9 + (h1 << 6) + (h1 >> 2));
	}
};

// Sweep line algorithm for finding which triangle contains each of a list of query points
void sweep(
	const std::vector<uv_t> &qPoints, // the UV query points
	const std::vector<uv_t> &uvs,     // the triangulated UV's
	const std::vector<size_t> &tris,  // the flattened triangle indexes

	std::vector<size_t> &out,         // the tri-index per qPoint
	std::vector<size_t> &missing,     // Any qPoints that weren't in a triangle
	std::vector<uv_t> &barys          // The barycentric coordinates
);

// Find the closest border edge by brute force ... Eventually make something faster
size_t closestBruteForceEdge(
	const std::vector<uv_t> &startPoints,   // The uv start points
	const std::vector<uv_t> &dirVectors,    // The direction vectors of each edge
	const std::vector<double> &squaredLens, // The Squared lengths of each edge
	const uv_t &pt
);

// Handle query UVs that aren't in the area covered by the control uv set
void handleMissing(
	const std::vector<uv_t> &uvs,             // The Controlling UV coords
	const std::vector<uv_t> &qPoints,         // The Query uv coordinates
	const std::vector<edge_t> &borders,       // A list of border edge index pairs
	const std::vector<size_t> &missing,       // A list of UV indices that weren't found
	const std::vector<size_t> &borderToTri,   // List of triangle indices that the border edges are part of
	const std::vector<size_t> &tris,          // the flattened triangle indexes
	std::vector<size_t> &triIdxs,             // List of triangle indices that each uv is contained by
	std::vector<uv_t> &barys                  // List of barycentric coordinates per query uv
);


// Take the uv barycentric mapping and turn it into a vertex barycetnric mapping
void getVertCorrelation(
	size_t numVerts,                      // The number of vertices in the query mesh
	size_t numUVs,                        // The number of uvs in the query mesh
	const std::vector<size_t> &triIdxs,   // The tri-index per query point
	const std::vector<size_t> &vertTris,  // The triangle *vertex* indices
	const std::vector<uv_t> &barys,       // The first two barycentric coordinates of each point in the triangle
	const std::vector<size_t> &uvToVert,  // Map from the query uvIdx to the query vert Idx
	std::vector<double> &flatBarys, // The flattened barycentric coordinates
	std::vector<size_t> &flatIdxs,  // The flattened barycentric indices
	std::vector<size_t> &flatRanges // The count of barycentric weights per index
);


