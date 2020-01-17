#pragma once
#include <array>
#include <cmath>

typedef std::array<double, 2> uv_t;

// The vector between two uvs
inline uv_t uvDiff(const uv_t &a, const uv_t &b){
	//uv_t out = {a[0] - b[0], a[1] - b[1]};
	//return out;
	return {a[0] - b[0], a[1] - b[1]};
}

// dot product
inline double uvDot(const uv_t &a, const uv_t &b){
	return a[0]*b[0] + a[1]*b[1];
}

// Signed triangle area assuming the third point is at the origin
// Clockwise is positive
inline double biArea(const uv_t &a, const uv_t &b){
    return (b[0]*a[1] - a[0]*b[1]) * 0.5;
}

// Signed triangle area. Clockwise is positive
inline double triArea(const uv_t &a, const uv_t &b, const uv_t &c){
	return biArea(uvDiff(a, c), uvDiff(b, c));
}

// Get the first two barycentric coordinates of a 2d triangle
inline void triBary(const uv_t &p, const uv_t &a, const uv_t &b, const uv_t &c, double &u, double &v) {
	double v0x = a[0] - c[0];
	double v0y = a[1] - c[1];
	double v1x = b[0] - c[0];
	double v1y = b[1] - c[1];
	double v2x = p[0] - c[0];
	double v2y = p[1] - c[1];

	double den = 1.0 / (v0x * v1y - v1x * v0y);
	u = (v2x * v1y - v1x * v2y) * den;
	v = (v0x * v2y - v2x * v0y) * den;
}

// Check that the point is inside the triangle
inline bool pointInTri(const uv_t &p, const uv_t &a, const uv_t &b, const uv_t &c){
	// Check that the barycentric coordinates are all positive
	double aa, bb;
	triBary(p, a, b, c, aa, bb);
	return ((aa >= 0) && (bb >= 0) && (aa + bb <= 1.0));
}

