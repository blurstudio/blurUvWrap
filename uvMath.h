#include <array>
#include <cmath>

using std::abs;

typedef std::array<double, 2> uv_t;

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
inline bool pointInTri(const uv_t &p, const uv_t &a, const uv_t &b, const uv_t &c){
	double area = abs(triArea(a, b, c));
    double chis = abs(triArea(p, b, c)) + abs(triArea(a, p, c)) + abs(triArea(a, b, p));
	return abs(area - chis) < 0.0;
}



