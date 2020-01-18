#pragma once
#include <vector>
#include <array>
#include <memory>

#include <maya/MArrayDataHandle.h>
#include <maya/MDataBlock.h>
#include <maya/MObject.h>
#include <maya/MFnMesh.h>
#include <maya/MString.h>

// Get a pointer to the MArrayDataHandle if it exists
// This is good for dealing with paintable attributes other than the built-in weights
std::unique_ptr<MArrayDataHandle> getArrayDataPtr(
	MDataBlock &block,
	MObject &parPlug,
	MObject &childPlug,
	unsigned int multiIndex
);

// Read the pointer from getArrayDataPtr
float readArrayDataPtr(std::unique_ptr<MArrayDataHandle> &ptr, unsigned idx, float defaultVal); 

// Get a mapping from the uv indices to the vertex indices
bool getUvToVert(const MFnMesh &mesh, const MString* uvSet, std::vector<size_t> &uvToVert);

// Get the array of uvs from a mesh
std::vector<uv_t> getUvArray(const MFnMesh &mesh, const MString* uvSet); 

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
);

// Given a control and target mesh, get the flattened barycentric bind
bool getBindData(const MFnMesh &fnRestCtrl, const MFnMesh &fnRestMesh, MString *ctrlUvName, MString *uvName, short projType,
	std::vector<double> &flatBarys, std::vector<size_t> &flatRanges, std::vector<size_t> &flatIdxs
); 


MMatrixArray getMeshBasis(const MFnMesh &fnmesh, const MString * uvSet);

