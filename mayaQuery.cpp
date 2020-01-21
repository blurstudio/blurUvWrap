#include "blurUvWrap.h"
#include "uvQuery.h"

#include <vector>
#include <array>
#include <algorithm>
#include <memory>
#include <unordered_set>
#include <unordered_map>
#include <utility>

#include <maya/MItGeometry.h>
#include <maya/MFloatVectorArray.h>
#include <maya/MGlobal.h>
#include <maya/MFnMesh.h>
#include <maya/MFnEnumAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnCompoundAttribute.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MFnStringData.h>
#include <maya/MFnMeshData.h>
#include <maya/MMeshIntersector.h>
#include <maya/MFnMeshData.h>
#include <maya/MFloatMatrix.h>
#include <maya/MBoundingBox.h>
#include <maya/MPointArray.h>
#include <maya/MFloatArray.h>
#include <maya/MString.h>
#include <maya/MStringArray.h>
#include <maya/MFnFloatArrayData.h>
#include <maya/MFnIntArrayData.h>
#include <maya/MArrayDataBuilder.h>
#include <maya/MMatrix.h>
#include <maya/MMatrixArray.h>


// Get a pointer to the MArrayDataHandle if it exists
// This is good for dealing with paintable attributes other than the built-in weights
std::unique_ptr<MArrayDataHandle> getArrayDataPtr(
	MDataBlock &block,
	MObject &parPlug,
	MObject &childPlug,
	unsigned int multiIndex
) {
	MStatus status;
	MArrayDataHandle weightList = block.outputArrayValue(parPlug, &status);
	status = weightList.jumpToElement(multiIndex);
	if (status) {
		MDataHandle weightsStructure = weightList.inputValue(&status);
		if (status) {
			MArrayDataHandle weights = weightsStructure.child(childPlug);
			return std::make_unique<MArrayDataHandle>(weights);
		}
	}
	return nullptr;
}

float readArrayDataPtr(std::unique_ptr<MArrayDataHandle> &ptr, unsigned idx, float defaultVal) {
	float wVal = defaultVal;
	MStatus status;
	if (ptr) {
		status = ptr->jumpToElement(idx);
		if (status) {
			wVal = ptr->inputValue().asFloat();
		}
	}
	return wVal;
}

// Get a mapping from the uv indices to the vertex indices
bool getUvToVert(const MFnMesh &mesh, const MString* uvSet, std::vector<size_t> &uvToVert){
	MIntArray uvCounts, uvIdxs, vertCounts, vertIdxs;
	mesh.getAssignedUVs(uvCounts, uvIdxs, uvSet); // numUvsPerFace, flatUvIdxs
	mesh.getVertices(vertCounts, vertIdxs);

	size_t numUvs = mesh.numUVs(*uvSet);

	uvToVert.resize(numUvs);
	unsigned uvCursor = 0;
	for (unsigned faceIdx = 0; faceIdx < uvCounts.length(); ++faceIdx) {
		// Search for border edges
		for (unsigned countOffset = 0; countOffset < uvCounts[faceIdx]; ++countOffset) {
			unsigned uv1 = uvIdxs[uvCursor + countOffset];
			uvToVert[uv1] = vertIdxs[uvCursor + countOffset];
		}
		uvCursor += uvCounts[faceIdx];
	}
	return true;
}

// Get the array of uvs from a mesh
std::vector<uv_t> getUvArray(const MFnMesh &mesh, const MString* uvSet) {
	std::vector<uv_t> uvs;
	MFloatArray uArray, vArray;
	mesh.getUVs(uArray, vArray, uvSet); // uvs
	uvs.reserve(uArray.length());
	for (unsigned int i = 0; i < uArray.length(); ++i) {
		uv_t x = { uArray[i], vArray[i] };
		uvs.push_back(std::move(x));
	}
	return uvs;
}

// stupid utility function to always build the edge_t object in sorted order
edge_t sortEdge(size_t a, size_t b) {
	if (a < b) return { a, b };
	return { b, a };
}

// Parse a mesh and get all the crazy data that I need to get the bind
bool getTriangulation(
	const MFnMesh &mesh, const MString* uvSet,

	std::vector<size_t> &flatUvTriIdxs,   // The flattened uv triangle indexes
	std::vector<size_t> &triToFaceIdx,    // Vector to get the faceIdx given the triangleIdx
	std::vector<size_t> &faceRanges,      // The faceVert index range for a given faceIdx
	std::vector<edge_t> &borders,         // The list of border edges
	std::vector<size_t> &borderTriIdx,    // Get the triangle index based off the border edge index
	std::vector<size_t> &uvToVert,        // map from uvIdx to vertIdx
	std::vector<size_t> &flatVertTriIdxs  // Flattened vertex triangle indices
){
	// Get the triangulated UVs
	// Also get the face index of each uv triangle
	std::unordered_map<edge_t, size_t, edgeHash> borderToTri;
	MIntArray triCounts, triIdxOffsets, uvCounts, uvIdxs, vertCounts, vertIdxs;

	mesh.getTriangleOffsets(triCounts, triIdxOffsets); // numTrisPerFace, flatTriIdxs
	mesh.getAssignedUVs(uvCounts, uvIdxs, uvSet); // numUvsPerFace, flatUvIdxs
	mesh.getVertices(vertCounts, vertIdxs);

	size_t numUvs = mesh.numUVs(*uvSet);

	flatUvTriIdxs.reserve(triIdxOffsets.length());
	flatVertTriIdxs.reserve(triIdxOffsets.length());
	triToFaceIdx.reserve(triIdxOffsets.length());
	faceRanges.reserve(uvCounts.length() + 1);
	faceRanges.push_back(0);
	uvToVert.resize(numUvs);

	unsigned triCursor = 0;
	unsigned uvCursor = 0;
	for (unsigned faceIdx = 0; faceIdx < triCounts.length(); ++faceIdx) {
		unsigned numFaceTris = triCounts[faceIdx];
		unsigned uvCount = uvCounts[faceIdx];
		faceRanges.push_back(faceRanges.back() + uvCount);

		// Search for border edges
		for (unsigned ei = 0; ei < uvCount; ++ei) {
			unsigned ein = (ei + 1) % uvCount;
			unsigned uv1 = uvIdxs[uvCursor + ei];
			unsigned uv2 = uvIdxs[uvCursor + ein];
			uvToVert[uv1] = vertIdxs[uvCursor + ei];

			edge_t edge = sortEdge(uv1, uv2);

			// Populate the map for keeping the triIdx for later
			auto mapIt = borderToTri.find(edge);
			if (mapIt == borderToTri.end()) borderToTri[edge] = 0;
			else borderToTri.erase(mapIt);
		}

		// Build the uv triangle idxs
		for (unsigned triNum = 0; triNum < numFaceTris; ++triNum) {
			size_t triIdx = triToFaceIdx.size();
			triToFaceIdx.push_back(faceIdx);
			unsigned triStart = (triCursor + triNum) * 3;

			// Keep track of the vertex indices of the triangles
			size_t x = vertIdxs[triIdxOffsets[triStart + 0]];
			size_t y = vertIdxs[triIdxOffsets[triStart + 1]];
			size_t z = vertIdxs[triIdxOffsets[triStart + 2]];
			flatVertTriIdxs.push_back(x);
			flatVertTriIdxs.push_back(y);
			flatVertTriIdxs.push_back(z);

			// get the 3 uv idxs of a triangle
			size_t a = uvIdxs[triIdxOffsets[triStart + 0]];
			size_t b = uvIdxs[triIdxOffsets[triStart + 1]];
			size_t c = uvIdxs[triIdxOffsets[triStart + 2]];

			// Add those indices to the flat triangle list
			flatUvTriIdxs.push_back(a);
			flatUvTriIdxs.push_back(b);
			flatUvTriIdxs.push_back(c);

			// Check for border edges of the triangle
			auto ab = borderToTri.find(sortEdge(a, b));
			auto bc = borderToTri.find(sortEdge(b, c));
			auto ac = borderToTri.find(sortEdge(a, c));

			// If the edge is a border, keep track of the triIdx for it
			if (ab != borderToTri.end()) ab->second = triIdx;
			if (bc != borderToTri.end()) bc->second = triIdx;
			if (ac != borderToTri.end()) ac->second = triIdx;

		}
		triCursor += triCounts[faceIdx];
		uvCursor += uvCounts[faceIdx];
	}

	// Put the border edges and border tri indices into a simpler structure
	for (auto &kv : borderToTri) {
		borders.push_back(kv.first);
		borderTriIdx.push_back(kv.second);
	}
	return true;
}

// Given a control and target mesh, get the flattened barycentric bind
bool getBindData(const MFnMesh &fnRestCtrl, const MFnMesh &fnRestMesh, MString *ctrlUvName, MString *uvName, short projType,
	std::vector<double> &flatBarys, std::vector<size_t> &flatRanges, std::vector<size_t> &flatIdxs
) {
	bool success;
	// Get the UVs
	std::vector<uv_t> qPoints = getUvArray(fnRestMesh, uvName);
	std::vector<uv_t> uvs = getUvArray(fnRestCtrl, ctrlUvName);

	// Get the triangles and data structures
	std::vector<size_t> flatUvTriIdxs;   // The flattened uv triangle indexes
	std::vector<size_t> triToFaceIdx;    // Vector to get the faceIdx given the triangleIdx
	std::vector<size_t> faceRanges;      // The faceVert index range for a given faceIdx
	std::vector<edge_t> borderEdges;     // The pairs of edge idxs
	std::vector<size_t> borderTriIdx;    // Get the triangle index based off the border edge index
	std::vector<size_t> uvToVert;        // Map from the uvIdx to the vertex Idx
	std::vector<size_t> flatVertTriIdxs; // Flattened vertex triangle indices
	success = getTriangulation(fnRestCtrl, ctrlUvName, flatUvTriIdxs, triToFaceIdx, faceRanges, borderEdges, borderTriIdx, uvToVert, flatVertTriIdxs);
	if (!success) return false;

	// Sweep the UVs
	std::vector<size_t> triIdxs;     // the tri-index per qPoint
	std::vector<size_t> missing;     // Any qPoints that weren't in a triangle
	std::vector<uv_t> barys;         // The barycentric coordinates of each point in the triangle
	success = sweep(qPoints, uvs, flatUvTriIdxs, triIdxs, missing, barys);
	if (!success) return false;

	// Project any missing uvs to the closest uv border edge
	// And add the triangle to the output
	success = handleMissing(uvs, qPoints, borderEdges, missing, borderTriIdx, flatUvTriIdxs, triIdxs, barys);
	if (!success) return false;

	size_t numVerts = fnRestMesh.numVertices();
	size_t numUVs = fnRestMesh.numUVs(*uvName);
	std::vector<size_t> qUvToVert; // Map the uvIdx to the vertex idx for the query mesh
	success = getUvToVert(fnRestMesh, uvName, qUvToVert);
	if (!success) return false;

	// Translate the UV barycentric coordinates to vertices
	success = getVertCorrelation(numVerts, numUVs, triIdxs, flatVertTriIdxs, barys, qUvToVert, flatBarys, flatIdxs, flatRanges);
	if (!success) return false;

	return true;
}


std::vector<MMatrix> getMeshBasis(const MFnMesh &fnmesh, const MString * uvSet){
	// Get the face-vert tangents and normals
	MFloatVectorArray normals, tangents, binormals, rawTangents;
	MPointArray points;
	fnmesh.getVertexNormals(false, normals, MSpace::kObject);
	fnmesh.getTangents(rawTangents, MSpace::kObject, uvSet);
	fnmesh.getPoints(points);

	// pick an arbitrary (but consistent) face-vert tangent to use for the basis
	tangents.setLength(normals.length());
	MIntArray counts, idxs;
	fnmesh.getVertices(counts, idxs);
	size_t ptr = 0;
	for (size_t faceIdx=0; faceIdx<counts.length(); ++faceIdx){
		UINT c = counts[faceIdx];
		for (size_t j=ptr; j<ptr+c; ++j){
			tangents[idxs[j]] = rawTangents[j];
		}
		ptr += c;
	}

	// Calculate the binormal and build the matrices
	//MMatrixArray meshBasis;
	//meshBasis.setLength(normals.length());
	std::vector<MMatrix> meshBasis;
	meshBasis.reserve(normals.length());

	for (size_t i=0; i<normals.length(); ++i){
		MFloatVector n = normals[i];
		if (n*n < 1e-14f) n = MFloatVector(0.0f, 1.0f, 0.0f);

		MFloatVector b = (n ^ tangents[i]).normal();
		if (b*b < 1e-14f) b = MFloatVector(1.0f, 0.0f, 0.0f);

		MFloatVector t = (b ^ n).normal();
		if (t*t < 1e-14f) t = MFloatVector(0.0f, 0.0f, 1.0f);

		MPoint &p = points[i];

		// Get a reference and update it directly
		//MMatrix &mat = meshBasis[i];
		MMatrix mat;
		mat[0][0] = b[0]; mat[0][1] = b[1]; mat[0][2] = b[2]; mat[0][3] = 0.0;
		mat[1][0] = n[0]; mat[1][1] = n[1]; mat[1][2] = n[2]; mat[1][3] = 0.0;
		mat[2][0] = t[0]; mat[2][1] = t[1]; mat[2][2] = t[2]; mat[2][3] = 0.0;
		mat[3][0] = p[0]; mat[3][1] = p[1]; mat[3][2] = p[2]; mat[3][3] = 1.0;

		meshBasis.push_back(mat);
	}

	return meshBasis;
}




