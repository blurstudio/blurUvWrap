#include <vector>
#include <array>
#include <list>
#include <iterator>
#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include <numeric>
#include <cmath>

#define EPS 1e-7

using std::abs;
using std::sqrt;
using std::min;
using std::max;

typedef std::array<double, 2> uv_t;
typedef std::array<size_t, 3> tri_t;

// The vector between two uvs
inline uv_t uvDiff(const uv_t &a, const uv_t &b){
	uv_t out = {a[0] - b[0], a[1] - b[1]};
	return out;
}

// dot product
inline double uvDot(const uv_t &a, const uv_t &b){
	return a[0]*b[0] + a[1]*b[1];
}

// Signed triangle area
inline double triArea(const uv_t &a, const uv_t &b, const uv_t &c){
    return (a[0] * (c[1] - b[1]) + b[0] * (a[1] - c[1]) + c[0] * (b[1] - a[1])) / 2.0;
}

// Signed triangle area assuming point C is at the origin
inline double biArea(const uv_t &a, const uv_t &b){
    return a[1]*b[0] - a[0]*b[1] / 2.0;
}

// Check if the point is inside the triangle
bool pointInTri(const uv_t &p, const uv_t &a, const uv_t &b, const uv_t &c, double tol=EPS){
	double area = abs(triArea(a, b, c));
    double chis = abs(triArea(p, b, c)) + abs(triArea(a, p, c)) + abs(triArea(a, b, p));
	return abs(area - chis) < tol;
}

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
	stable_sort(ret.begin(), ret.end(),
		[&v](size_t i1, size_t i2) {return v[i1] < v[i2];});

	return ret;
}

// Sweep line algorithm for finding which triangle contains a given query point
// but for a list of query points
void sweep(
		const std::vector<uv_t> &qPoints, // the UV query points
		const std::vector<uv_t> &uvs, // the triangulated UV's
		const std::vector<tri_t> &tris, // the triangle indexes

		std::vector<size_t> &out, // the tri-index per qPoint
		std::vector<size_t> &missing // Any qPoints that weren't in a triangle
){
	// build a data structure indexing into tris based on the
	// min/max bounding box of each triangle
	size_t numTris = tris.size();
	std::vector<size_t> ymxs(numTris), ymns(numTris), xmxs(numTris), xmns(numTris);
	for (size_t i=0; i<numTris; ++i){
		const auto &t = tris[i];
		xmns[i] = min(uvs[t[0]][0], min(uvs[t[1]][0], uvs[t[2]][0]));
		ymns[i] = min(uvs[t[0]][1], min(uvs[t[1]][1], uvs[t[2]][1]));
		xmxs[i] = max(uvs[t[0]][0], max(uvs[t[1]][0], uvs[t[2]][0]));
		ymxs[i] = max(uvs[t[0]][1], max(uvs[t[1]][1], uvs[t[2]][1]));
	}

	std::vector<double> qpx;
	for (auto & uv : uvs){
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
	missing.resize(qPoints.size()); // Any qPoints that weren't in a triangle

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
				mn = xmxs[mxSIdxs[-1]] + 1;
			}
		}
		else if (qp <= mx){
			// check query points between adding and removing
			uv_t qPoint = qPoints[qpIdx];
			double yv = qPoint[1];
			// a linear search is faster than more complex collections here
			bool found = false;
			for (auto &t : atSet){
				if (ymns[t] <= yv && yv <= ymxs[t]){
					const tri_t &tri = tris[t];
					if (pointInTri(qPoint, uvs[tri[0]], uvs[tri[1]], uvs[tri[2]])){
						out[qpIdx] = t;
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
}

// Build the mean value cooridinates of a point in a polygon
// MVC's are an extension of barycentric coordinates to arbitrary polygons
std::vector<double> meanValueCoords(const std::vector<uv_t> &corners, const uv_t &p, double tol=EPS){
	size_t dim = corners.size();
	std::vector<double> bary(dim, 0.0);
	std::vector<uv_t> spokes;
	std::vector<double> spokeLens;
	spokes.reserve(dim);
	spokeLens.reserve(dim);
	uv_t zero = {0.0, 0.0};

	for (const auto & c : corners){
		uv_t bak = uvDiff(c, p);
		spokes.push_back(bak);
		spokeLens.push_back(sqrt(uvDot(bak, bak)));
	}

	// Check if P is on top of any of the corners
	for (size_t i=0; i<dim; ++i){
		if (spokeLens[i] < tol){
			bary[i] = 1.0;
			return bary;
		}
	}

	std::vector<double> areas, dots;
	areas.reserve(dim);
	dots.reserve(dim);
	for (size_t i=0; i<dim; ++i){
		size_t j = (i + 1) % dim;
		areas.push_back(abs(biArea(spokes[i], spokes[j])));
		dots.push_back(uvDot(spokes[i], spokes[j]));
	}

	// Check if P is on top of any of the edges
	for (size_t i=0; i<dim; ++i){
		if (areas[i] < tol && dots[i] < 0.0){
			// If I'm here, I know that p lies on the line
			// between the corners at i and i+1.
			// Return the lerp value
			size_t j = (i + 1) % dim;
			uv_t c1 = corners[i];
			uv_t c2 = corners[j];
			uv_t base = uvDiff(c1, c2);
			double proj = uvDot(uvDiff(p, c2), base);
			double b = proj / uvDot(base, base);
			bary[i] = b;
			bary[j] = 1.0 - b;
			return bary;
		}
	}

	std::vector<double> tees;
	tees.reserve(dim);
	for (size_t i=0; i<dim; ++i){
		size_t j = (i + 1) % dim;
		double t = areas[i] / (spokeLens[i]*spokeLens[j] + dots[i]);
		tees.push_back(t);
	}

	double total;
	for (size_t j=0; j<dim; ++j){
		size_t i = (j + 1) % dim; // notice i/j swap
		double x = (tees[i] + tees[j]) / spokeLens[i];
		bary[i] = x;
		total += x;
	}

	for (size_t i=0; i<dim; ++i){
		bary[i] /= total;
	}

	return bary;
}



