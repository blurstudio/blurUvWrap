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

// Check if the given point can be clipped off of the given polygon as a triangle
bool isEar(
	std::list<const uv_t*> &polygon, // A linked list of uv pointers
	std::list<const uv_t*>::iterator &it, // An iterator to a given uv
	double tol=EPS
){
	auto prev = (it == polygon.begin()) ? std::prev(polygon.end()) : std::prev(it);
	auto nxt = std::next(it);
	if (nxt == polygon.end()) nxt = polygon.begin();

	const uv_t *a = *prev;
	const uv_t *b = *it;
	const uv_t *c = *nxt;

	double signedArea = triArea(*a, *b, *c);

	// Check the triangle is wound correctly
	if (signedArea > 0.0) return false;

    // Check that the triangle has non-zero area
    // we already know the area is negative from above
    if (-signedArea < tol) return false;

	// Check that no other points are inside this triangle
	for (const auto &p : polygon){
		if ((p != a) && (p != b) && (p != c)){
            if (pointInTri(*p, *a, *b, *c, tol)) {
				return false;
			}
		}
	}

	return true;
}

// Helper function to either add or remove a value from an unordered_set
// Written as a template because ... well ... the type I use here is really long
template <typename T>
void _updateSetMembership(std::unordered_set<T> &set, T &value, bool shouldBeMember){
	if (set.find(value) == set.end()){
		// Currently is not a member
		if (shouldBeMember) set.insert(value);
	}
	else {
		// Currently is a member
		if (!shouldBeMember) set.erase(value);
	}
}

// Run the ear-clipping algorithm to triangulate a polygon
std::vector<tri_t> earclip(const std::vector<size_t> &idxs, const std::vector<uv_t> &verts, double tol=EPS){
	size_t numPts = idxs.size();
	
	std::list<const uv_t *> polygon;
	std::unordered_set< std::list<const uv_t *>::iterator > earSet;
	std::unordered_map<const uv_t *, size_t> idxMap;

	for (const auto &idx : idxs){
		const uv_t *ptr = &(verts[idx]);
		polygon.push_back(ptr);
		idxMap[ptr] = idx;
	}

	// Check the polygon for any initial ears
	for (auto it=polygon.begin(); it != polygon.end(); ++it){
		if (isEar(polygon, it, tol)){
			earSet.insert(it);
		}
	}

	// Loop over the ears until I'm down to 3 points
	std::vector<tri_t> out;
	while (!earSet.empty() && numPts >=3){
		// Get the neighboring verts

		auto earIt = *earSet.begin();
		auto prevIt = (earIt == polygon.begin()) ? std::prev(polygon.end()) : std::prev(earIt);
		auto nxtIt = std::next(earIt);
		if (nxtIt == polygon.end()) nxtIt = polygon.begin();

		// Store the triangle
		tri_t pb = {idxMap[*prevIt], idxMap[*earIt], idxMap[*nxtIt]};
		out.push_back(std::move(pb));

		// Remove that vert from the triangle
		polygon.erase(earIt);

		// Update the ear property of the neighboring verts
		bool prevEar = isEar(polygon, prevIt, tol);
		bool nxtEar = isEar(polygon, nxtIt, tol);
		_updateSetMembership(earSet, prevIt, prevEar);
		_updateSetMembership(earSet, nxtIt, nxtEar);
	}

	return out;
}




