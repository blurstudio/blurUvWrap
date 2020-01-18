#include <vector>
#include <unordered_set>
#include <algorithm> // stable_sort
#include <numeric> // iota
#include <cmath>

#include "uvMath.h"
#include "uvQuery.h"

using std::abs;
using std::sqrt;
using std::min;
using std::max;

// Build a vector of indexes that sort another vector
template <typename T>
std::vector<size_t> argsort(const std::vector<T> &v) {
	// initialize original index locations
	std::vector<size_t> ret(v.size());
	iota(ret.begin(), ret.end(), 0);

	// sort indexes based on comparing values in v
	// using std::stable_sort instead of std::sort
	// to avoid unnecessary index re-orderings
	// when v contains elements of equal values 
	std::stable_sort(ret.begin(), ret.end(),
		[&v](size_t i1, size_t i2) {return v[i1] < v[i2];});

	return ret;
}

// Sweep line algorithm for finding which triangle contains a given query point
// but for a list of query points
bool sweep(
		const std::vector<uv_t> &qPoints, // the UV query points
		const std::vector<uv_t> &uvs,     // the triangulated UV's
		const std::vector<size_t> &tris,  // the flattened triangle indexes

		std::vector<size_t> &out,         // the tri-index per qPoint
		std::vector<size_t> &missing,     // Any qPoints that weren't in a triangle
		std::vector<uv_t> &barys          // The barycentric coordinates
){
	// build a data structure indexing into tris based on the
	// min/max bounding box of each triangle
	size_t numTris = tris.size() / 3;
	std::vector<double> ymxs(numTris), ymns(numTris), xmxs(numTris), xmns(numTris);
	for (size_t i=0; i<numTris; ++i){
		size_t a = 3*i;
		const size_t &ta0 = tris[a];
		const size_t &ta1 = tris[a+1];
		const size_t &ta2 = tris[a+2];

		const uv_t &uv0 = uvs[ta0];
		const uv_t &uv1 = uvs[ta1];
		const uv_t &uv2 = uvs[ta2];
		
		xmns[i] = min(uv0[0], min(uv1[0], uv2[0]));
		ymns[i] = min(uv0[1], min(uv1[1], uv2[1]));
		xmxs[i] = max(uv0[0], max(uv1[0], uv2[0]));
		ymxs[i] = max(uv0[1], max(uv1[1], uv2[1]));
	}

	std::vector<double> qpx;
	for (auto & uv : qPoints){
		qpx.push_back(uv[0]);
	}
	std::vector<size_t> qpSIdxs = argsort(qpx);
	std::vector<size_t> mxSIdxs = argsort(xmxs);
	std::vector<size_t> mnSIdxs = argsort(xmns);
    size_t qpSIdx, mxSIdx, mnSIdx;
    qpSIdx = mxSIdx = mnSIdx = 0;
	size_t qpIdx = qpSIdxs[qpSIdx];
	size_t mxIdx = mxSIdxs[mxSIdx];
	size_t mnIdx = mnSIdxs[mnSIdx];
	double qp = qpx[qpIdx];
	double mx = xmxs[mxIdx];
	double mn = xmns[mnIdx];

	out.resize(qPoints.size()); // the tri-index per qPoint
	barys.resize(qPoints.size()); // the first 2 barycentric coordinates of the query point in the triangle

    // skip any triangles to the left of the first query point
    // The algorithm will stop once we hit the last query point

	std::vector<bool> skip(numTris);
	for (size_t i=0; i<xmxs.size(); ++i){
		skip[i] = qp > xmxs[i];
	}
	std::vector<bool> activeTris(numTris, false);
	std::unordered_set<size_t> atSet;

	while (true) {
		if (mn <= mx && mn <= qp){
			size_t aTriIdx = mnSIdxs[mnSIdx];
			if (!activeTris[aTriIdx] && !skip[aTriIdx]){
				activeTris[aTriIdx] = true;
				atSet.insert(aTriIdx);
			}
			mnSIdx += 1;
			if (mnSIdx != numTris){
				mnIdx = mnSIdxs[mnSIdx];
				mn = xmns[mnIdx];
			}
			else {
				mn = xmxs[mxSIdxs.back()] + 1;
			}
		}
		else if (qp <= mx){
			// check query points between adding and removing
			uv_t qPoint = qPoints[qpIdx];
			double yv = qPoint[1];

			// a linear search is faster than more complex collections here in my tests
			bool found = false;
			for (auto &t : atSet){
				if (ymns[t] <= yv && yv <= ymxs[t]){
					double aa, bb;
					triBary(qPoint, uvs[tris[3 * t]], uvs[tris[3 * t + 1]], uvs[tris[3 * t + 2]], aa, bb);
					if ((aa >= 0) && (bb >= 0) && (aa + bb <= 1.0)) {
						out[qpIdx] = t;
						barys[qpIdx] = { aa, bb };
						found = true;
						break;
					}
				}
			}
			if (!found){
				missing.push_back(qpIdx);
			}

			qpSIdx += 1;
			if (qpSIdx == qpSIdxs.size()){
				break; // we've reached the end
			}
			qpIdx = qpSIdxs[qpSIdx];
			qp = qpx[qpIdx];
		}
		else {
			// always remove triangles last
			size_t aTriIdx = mxSIdxs[mxSIdx];
			if (activeTris[aTriIdx] && !skip[aTriIdx]){
				activeTris[aTriIdx] = false;
				atSet.erase(aTriIdx);
			}

			mxSIdx += 1;
			if (mxSIdx != mxSIdxs.size()){
				mxIdx = mxSIdxs[mxSIdx];
				mx = xmxs[mxIdx];
			}
		}
	}
	return true;
}


size_t closestBruteForceEdge(
	const std::vector<uv_t> &a,
	const std::vector<uv_t> &d,
	const std::vector<double> &dr2,
	const uv_t &pt
) {
	// Get the closest edge index via brute force
	// A is the collection of start points
	// D is the collection of direction vectors
	// DR2 is the colletion of squared lengths of the direction vectors
	// Pt is the single point to check
	size_t minIdx = 0;
	double minVal = std::numeric_limits<double>::max();

	for (size_t i = 0; i < a.size(); ++i) {
		const uv_t &aa = a[i];
		const uv_t &dd = d[i];

		// Get the lerp value of the projection of the point onto a->d
		// dot((pt - a[i]), d) / dr2[i]
		double lerp = (((pt[0] - aa[0]) * dd[0]) + ((pt[1] - aa[1]) * dd[1])) / dr2[i];
		// clamp(lerp, 0, 1)
		lerp = (lerp < 0.0) ? 0.0 : ((lerp > 1.0) ? 1.0 : lerp);

		// Get the vector from the point to the projection
		// (d * lerp) + a - pt
		double c0 = (dd[0] * lerp) + aa[0] - pt[0];
		double c1 = (dd[1] * lerp) + aa[1] - pt[1];

		// Square that length
		double l2 = (c0 *c0) + (c1 * c1);

		if (l2 < minVal) {
			minVal = l2;
			minIdx = i;
		}
	}
	return minIdx;
}

bool handleMissing(
	const std::vector<uv_t> &uvs,
	const std::vector<uv_t> &qPoints,
	const std::vector<edge_t> &borders,
	const std::vector<size_t> &missing,
	const std::vector<size_t> &borderToTri,
	const std::vector<size_t> &tris,  // the flattened triangle indexes
	std::vector<size_t> &triIdxs,
	std::vector<uv_t> &barys
) {
	float tol = 1.0f;

	// In the future, I will definitely need to do a
	// better search algorithm for this, but for now I can just handle it
	// with a brute force search

	std::vector<uv_t> starts;
	std::vector<uv_t> bDiff;
	std::vector<double> bLens2;
	bDiff.reserve(borders.size());
	bLens2.reserve(borders.size());
	starts.reserve(borders.size());
	for (size_t i = 0; i < borders.size(); ++i) {
		uv_t uv1 = uvs[borders[i][0]];
		uv_t uv2 = uvs[borders[i][1]];
		uv_t diff = { uv2[0] - uv1[0], uv2[1] - uv1[1] };
		double delta = diff[1] - diff[0];
		bLens2.push_back(delta * delta);
		bDiff.push_back(std::move(diff));
		starts.push_back(uv1);
	}

	for (auto &mIdx : missing) {
		uv_t mp = qPoints[mIdx];
		size_t closestEdge = closestBruteForceEdge(starts, bDiff, bLens2, mp);
		size_t triIdx = borderToTri[closestEdge];

		double b1, b2;
		triBary(mp, uvs[tris[(3*triIdx)]], uvs[tris[(3*triIdx)+1]], uvs[tris[(3*triIdx)+2]], b1, b2);
		triIdxs[mIdx] = triIdx;
		barys[mIdx] = { b1, b2 };
	}
	return true;
}

// Take the uv barycentric mapping and turn it into a vertex barycetnric mapping
// This is done by averaging the influence when a vertex has multiple uvs
bool getVertCorrelation(
	size_t numVerts,                      // The number of vertices in the query mesh
	size_t numUVs,                        // The number of uvs in the query mesh
	const std::vector<size_t> &triIdxs,   // The tri-index per query point
	const std::vector<size_t> &vertTris,  // The triangle *vertex* indices
	const std::vector<uv_t> &barys,       // The first two barycentric coordinates of each point in the triangle
	const std::vector<size_t> &uvToVert,  // Map from the query uvIdx to the query vert Idx

	std::vector<double> &flatBarys, // The flattened barycentric coordinates
	std::vector<size_t> &flatIdxs,  // The flattened barycentric indices
	std::vector<size_t> &flatRanges // The count of barycentric weights per index
) {
	// Invert the uvToVert 
	std::vector<std::vector<size_t>> vertToUvs;
	vertToUvs.resize(numVerts);
	for (size_t uvIdx = 0; uvIdx < uvToVert.size(); ++uvIdx) {
		vertToUvs[uvToVert[uvIdx]].push_back(uvIdx);
	}

	std::vector<std::vector<double>> outBarys;
	std::vector<std::vector<size_t>> outIdxs;
	outBarys.resize(numVerts);
	outIdxs.resize(numVerts);

	size_t flatNum = 0;
	for (size_t vIdx = 0; vIdx < numVerts; ++vIdx) {
		std::vector<size_t> &uvIdxs = vertToUvs[vIdx];
		double idxInv = 1.0 / uvIdxs.size();

		for (auto &uvIdx : uvIdxs) {
			// add the weighted value
			size_t triIdx = triIdxs[uvIdx];
			flatNum += 3;

			// barycentric weights for the verts
			uv_t bary = barys[uvIdx];
			double b0 = bary[0];
			double b1 = bary[1];
			double b2 = 1.0 - b0 - b1;
			outBarys[vIdx].push_back(b0*idxInv);
			outBarys[vIdx].push_back(b1*idxInv);
			outBarys[vIdx].push_back(b2*idxInv);

			// vertIdxs a, b, c
			size_t via = vertTris[3*triIdx + 0];
			size_t vib = vertTris[3*triIdx + 1];
			size_t vic = vertTris[3*triIdx + 2];
			outIdxs[vIdx].push_back(via);
			outIdxs[vIdx].push_back(vib);
			outIdxs[vIdx].push_back(vic);
		}
	}

	// Flatten the outputs
	flatBarys.reserve(flatNum);
	flatIdxs.reserve(flatNum);
	flatRanges.reserve(numVerts+1);
	flatRanges.push_back(0);
	for (size_t i = 0; i < outBarys.size(); ++i) {
		size_t count = 0;
		for (size_t j = 0; j < outBarys[i].size(); ++j) {
			double ob = outBarys[i][j];
			if (ob != 0.0) {
				count++;
				flatBarys.push_back(ob);
				flatIdxs.push_back(outIdxs[i][j]);
			}
		}
		flatRanges.push_back(flatRanges.back() + count);
	}
	return true;
}

