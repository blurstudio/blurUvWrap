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


#define CHECKSTAT(status, msg) {MStatus tStat = (status);  if ( !tStat ) {  MGlobal::displayError(msg); return tStat; }}

#define MM_UVEDGE  0
#define MM_UV      1
#define MM_VERT    2
#define MM_POINT   3

#define EPS 1e-7


MTypeId UvWrapDeformer::id(0x00122707);
MObject UvWrapDeformer::aControlRestMesh; // The non-deforming rest of the controller mesh
MObject UvWrapDeformer::aControlMesh; // The deforming controller mesh
MObject UvWrapDeformer::aControlInvWorld; // The inverse world of the controller mesh
MObject UvWrapDeformer::aControlUvName; // The name of the uv channel to sample on the controller mesh

MObject UvWrapDeformer::aRestMesh; // The non-deforming controlled mesh. Usually the Orig object. Multi
MObject UvWrapDeformer::aUvName; // The name of the uv channel to sample on the deformed object list

MObject UvWrapDeformer::aGlobalOffset; // The global offset multiplier
MObject UvWrapDeformer::aOffsetList; // The compound parent for the per-deformed values. Multi
MObject UvWrapDeformer::aOffsetMult; // The per-deformed offset multiplier
MObject UvWrapDeformer::aOffsetWeights; // The per-deformed offset weightmap. Mutli

MObject UvWrapDeformer::aMismatchHandler; // Enum for how to handle uvs that don't match perfectly


void* UvWrapDeformer::creator() { return new UvWrapDeformer(); }
MStatus UvWrapDeformer::initialize() {
	MStatus status;
	MFnNumericAttribute nAttr;
	MFnEnumAttribute eAttr;
	MFnTypedAttribute tAttr;
    MFnMatrixAttribute mAttr;
    MFnCompoundAttribute cAttr;
	MFnStringData sData;
	MFnMeshData mData;

	// The non-deforming rest of the controller mesh
    aControlRestMesh = tAttr.create("controlRest", "crm", MFnData::kMesh, mData.create(), &status);
	CHECKSTAT(status, "Error creating aControlRestMesh");
	CHECKSTAT(addAttribute(aControlRestMesh), "Error adding aControlRestMesh");
	CHECKSTAT(attributeAffects(aControlRestMesh, outputGeom), "Error affecting aControlRestMesh");
	// The deforming controller mesh
    aControlMesh = tAttr.create("controlMesh", "cm", MFnData::kMesh, mData.create(), &status);
	CHECKSTAT(status, "Error creating aControlMesh");
	CHECKSTAT(addAttribute(aControlMesh), "Error adding aControlMesh");
	CHECKSTAT(attributeAffects(aControlMesh, outputGeom), "Error affecting aControlMesh");
	// The inverse world of the controller mesh
	aControlInvWorld = mAttr.create("controlInv", "ci", MFnMatrixAttribute::kDouble, &status);
	CHECKSTAT(status, "Error creating aControlMesh");
	CHECKSTAT(addAttribute(aControlInvWorld), "Error adding aControlInvWorld");
	CHECKSTAT(attributeAffects(aControlInvWorld, outputGeom), "Error affecting aControlInvWorld");
	// The name of the uv channel to sample on the controller mesh
	aControlUvName = tAttr.create("controlUVName", "cuvn", MFnData::kString, sData.create(), &status);
	CHECKSTAT(status, "Error creating aControlUvName");
	tAttr.setStorable(true);
	CHECKSTAT(addAttribute(aControlUvName), "Error adding aControlUvName");
	CHECKSTAT(attributeAffects(aControlUvName, outputGeom), "Error affecting aControlUvName");

	// The non-deforming controlled mesh. Usually the Orig object. Multi
    aRestMesh = tAttr.create("restMesh", "rm", MFnData::kMesh, mData.create(), &status);
	CHECKSTAT(status, "Error creating aRestMesh");
	CHECKSTAT(addAttribute(aRestMesh), "Error adding aRestMesh");
	CHECKSTAT(attributeAffects(aRestMesh, outputGeom), "Error affecting aRestMesh");
	// The name of the uv channel to sample on the deformed object list
	aUvName = tAttr.create("uvName", "uvn", MFnData::kString, sData.create(), &status);
	CHECKSTAT(status, "Error creating aUvName");
	tAttr.setStorable(true);
	CHECKSTAT(addAttribute(aUvName), "Error adding aUvName");
	CHECKSTAT(attributeAffects(aUvName, outputGeom), "Error affecting aUvName");


	// The global offset multiplier
	aGlobalOffset = nAttr.create("globalOffsetMult", "gom", MFnNumericData::kFloat, 1.0f, &status);
	CHECKSTAT(status, "Error creating aGlobalOffset");
	nAttr.setKeyable(true);
	CHECKSTAT(addAttribute(aGlobalOffset), "Error adding aGlobalOffset");
	CHECKSTAT(attributeAffects(aGlobalOffset, outputGeom), "Error affecting aGlobalOffset");


	// The per-deformed offset multiplier. Child of aOffsetList
	aOffsetMult = nAttr.create("offset", "of", MFnNumericData::kFloat, 1.0f, &status);
	CHECKSTAT(status, "Error creating aOffsetMult");
	nAttr.setKeyable(true);
	// The per-deformed offset weightmap. Mutli. Child of aOffsetList
	aOffsetWeights = nAttr.create("offsetWeights", "ow", MFnNumericData::kFloat, 1.0f, &status);
	CHECKSTAT(status, "Error creating aOffsetWeights");
	nAttr.setArray(true);
	nAttr.setUsesArrayDataBuilder(true);
	// The compound parent for the per-deformed values. Multi
	aOffsetList = cAttr.create("offsetList", "ol", &status);
	CHECKSTAT(status, "Error creating aOffsetList");
	cAttr.setArray(true);
	cAttr.setUsesArrayDataBuilder(true);
	CHECKSTAT(cAttr.addChild(aOffsetWeights), "Error adding aOffsetWeights");
	CHECKSTAT(cAttr.addChild(aOffsetMult), "Error adding aOffsetMult");
	CHECKSTAT(addAttribute(aOffsetList), "Errod adding aOffsetList");
	CHECKSTAT(attributeAffects(aOffsetMult, outputGeom), "Error affecting aOffsetMult");
	CHECKSTAT(attributeAffects(aOffsetWeights, outputGeom), "Error affecting aOffsetWeights");


	// Enum for how to handle uvs that don't match perfectly
	aMismatchHandler = eAttr.create("mismatchHandler", "mmh", MM_UVEDGE, &status);
	CHECKSTAT(status, "Error creating aMismatchHandler");
	eAttr.setKeyable(false);
	eAttr.setChannelBox(true);
    eAttr.addField("UV Edge", MM_UVEDGE);
    eAttr.addField("UV Point", MM_UV);
    eAttr.addField("Closest Surface", MM_POINT);
    eAttr.addField("Closest Vertex", MM_VERT);
	CHECKSTAT(addAttribute(aMismatchHandler), "Error adding aMismatchHandler");
	CHECKSTAT(attributeAffects(aMismatchHandler, outputGeom), "Error affecting aMismatchHandler");

	return MStatus::kSuccess;
}



std::vector<uv_t> getUvArray(MFnMesh &mesh, const MString* uvSet) {
	// Get the array of uvs
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

edge_t sortEdge(size_t a, size_t b) {
	if (a < b) return { a, b };
	return { b, a };
}



void getTriangulation(
	MFnMesh &mesh, const MString* uvSet,

	std::vector<size_t> &flatUvTriIdxs, // The flattened uv triangle indexes
	std::vector<size_t> &triToFaceIdx,  // Vector to get the faceIdx given the triangleIdx
	std::vector<size_t> &faceRanges,    // The faceVert index range for a given faceIdx
	std::vector<edge_t> &borders,       // The list of border edges
	std::vector<size_t> &borderTriIdx   // Get the triangle index based off the border edge index
){
	// Get the triangulated UVs
	// Also get the face index of each uv triangle
	std::unordered_map<edge_t, size_t> borderToTri;
	MIntArray triCounts, triIdxOffsets, uvCounts, uvIdxs;
	mesh.getTriangleOffsets(triCounts, triIdxOffsets); // numTrisPerFace, flatTriIdxs
	mesh.getAssignedUVs(uvCounts, uvIdxs, uvSet); // numUvsPerFace, flatUvIdxs

	flatUvTriIdxs.reserve(triIdxOffsets.length());
	triToFaceIdx.reserve(triIdxOffsets.length());
	faceRanges.reserve(uvCounts.length() + 1);
	faceRanges.push_back(0);

	size_t triCursor = 0;
	size_t uvCursor = 0;
	for (unsigned faceIdx = 0; faceIdx < triCounts.length(); ++faceIdx) {
		unsigned numFaceTris = triCounts[faceIdx];
		unsigned uvCount = uvCounts[faceIdx];
		faceRanges.push_back(faceRanges.back() + uvCount);

		// Search for border edges
		for (unsigned ei = 0; ei < uvCount; ++ei) {
			unsigned ein = (ei + 1) % uvCount;
			unsigned uv1 = uvIdxs[uvCursor + ei];
			unsigned uv2 = uvIdxs[uvCursor + ein];

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
			// get the 3 uv idxs of a triangle
			size_t a = uvIdxs[uvCursor + triIdxOffsets[triStart + 0]];
			size_t b = uvIdxs[uvCursor + triIdxOffsets[triStart + 1]];
			size_t c = uvIdxs[uvCursor + triIdxOffsets[triStart + 2]];

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
	std::vector<size_t> borderTriIdx;
	for (auto &kv : borderToTri) {
		borders.push_back(kv.first);
		borderTriIdx.push_back(kv.second);
	}

}

// Get a pointer to the MArrayDataHandle if it exists
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

float readArrayDataPtr(std::unique_ptr<MArrayDataHandle> &ptr, unsigned int idx, float defaultVal) {
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

MStatus UvWrapDeformer::deform(
    MDataBlock& block, MItGeometry& iter,
    const MMatrix& wm, unsigned int multiIndex
) {
    MStatus status;

	// Load all the data from the custom plugs
    float env = block.inputValue(envelope, &status).asFloat();
    if (env == 0.0f) return status;

    MObject ctrlMesh = block.inputValue(aControlMesh, &status).asMesh();
	if (ctrlMesh.isNull()) return MStatus::kInvalidParameter;
    MFnMesh fnCtrlMesh(ctrlMesh);

    MObject restCtrl = block.inputValue(aControlRestMesh, &status).asMesh();
	if (restCtrl.isNull()) return MStatus::kInvalidParameter;
    MFnMesh fnRestCtrl(restCtrl);

    MObject restMesh = block.inputValue(aRestMesh, &status).asMesh();
	if (restMesh.isNull()) return MStatus::kInvalidParameter;
    MFnMesh fnRestMesh(restMesh);

    MMatrix cWInv = block.inputValue(aControlInvWorld, &status).asMatrix();
	MString ctrlUvName = block.inputValue(aControlUvName, &status).asString();
	MString uvName = block.inputValue(aUvName, &status).asString();
    float globalMult = block.inputValue(aGlobalOffset, &status).asFloat();
    short projType = block.inputValue(aMismatchHandler, &status).asShort();


	// Get the UVs
	std::vector<uv_t> qPoints = getUvArray(fnRestMesh, &uvName);
	std::vector<uv_t> uvs = getUvArray(fnRestCtrl, &ctrlUvName);

	// Get the triangles and data structures
	std::vector<size_t> flatUvTriIdxs; // The flattened uv triangle indexes
	std::vector<size_t> triToFaceIdx;  // Vector to get the faceIdx given the triangleIdx
	std::vector<size_t> faceRanges;    // The faceVert index range for a given faceIdx
	std::vector<edge_t> borderEdges;   // The pairs of edge idxs
	std::vector<size_t> borderTriIdx;  // Get the triangle index based off the border edge index

	getTriangulation(fnRestCtrl, &ctrlUvName, flatUvTriIdxs, triToFaceIdx, faceRanges, borderEdges, borderTriIdx);

	// Sweep the UVs
	std::vector<size_t> out;         // the tri-index per qPoint
	std::vector<size_t> missing;     // Any qPoints that weren't in a triangle
	std::vector<uv_t> barys;         // The barycentric coordinates of each point in the triangle
	sweep(qPoints, uvs, flatUvTriIdxs, out, missing, barys);

	// Project any missing uvs to the closest uv border edge
	// And add the triangle to the output
	// TODO: Find the barycentric coords of the missing points as well
	handleMissing(uvs, borderEdges, missing, borderTriIdx, out);


	// Because there's no default constructor, get a pointer to a MArrayDataHandle
	// That way I can just check for null and I don't have to re-walk the data block for the plug I want
	std::unique_ptr<MArrayDataHandle> hOffsetPtr = getArrayDataPtr(block, aOffsetList, aOffsetWeights, multiIndex);
	std::unique_ptr<MArrayDataHandle> hWeightPtr = getArrayDataPtr(block, weightList, weights, multiIndex);

	for (; !iter.isDone(); iter.next()) {
		float oVal = readArrayDataPtr(hOffsetPtr, iter.index(), 1.0);
		float wVal = readArrayDataPtr(hWeightPtr, iter.index(), 1.0);

		// TEMPORARY: Just do a push to make sure that the deformer is properly running
		MPoint pt = iter.position();
		MVector n = iter.normal();
		pt += (n * (oVal * wVal * env));
		iter.setPosition(pt);

	}

    return MStatus::kSuccess;
}

