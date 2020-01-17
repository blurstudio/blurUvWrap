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

MObject UvWrapDeformer::aBindInfos; // The compound parent per mesh for bind information
	MObject UvWrapDeformer::aBindBarys; // The flattened barycentric coordinate list
	MObject UvWrapDeformer::aBindIdxs; // The flattened vert index list
	MObject UvWrapDeformer::aBindRanges; // The number of coordinates to mix per vertex





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
	MFnFloatArrayData faData;
	MFnIntArrayData iaData;

	// The non-deforming rest of the controller mesh
    aControlRestMesh = tAttr.create("controlRest", "crm", MFnData::kMesh, mData.create(), &status);
	CHECKSTAT(status, "Error creating aControlRestMesh");
	CHECKSTAT(addAttribute(aControlRestMesh), "Error adding aControlRestMesh");

	// The deforming controller mesh
    aControlMesh = tAttr.create("controlMesh", "cm", MFnData::kMesh, mData.create(), &status);
	CHECKSTAT(status, "Error creating aControlMesh");
	CHECKSTAT(addAttribute(aControlMesh), "Error adding aControlMesh");

	// The inverse world of the controller mesh
	aControlInvWorld = mAttr.create("controlInv", "ci", MFnMatrixAttribute::kDouble, &status);
	CHECKSTAT(status, "Error creating aControlMesh");
	CHECKSTAT(addAttribute(aControlInvWorld), "Error adding aControlInvWorld");

	// The name of the uv channel to sample on the controller mesh
	aControlUvName = tAttr.create("controlUVName", "cuvn", MFnData::kString, sData.create(), &status);
	CHECKSTAT(status, "Error creating aControlUvName");
	tAttr.setStorable(true);
	CHECKSTAT(addAttribute(aControlUvName), "Error adding aControlUvName");
	// The non-deforming controlled mesh. Usually the Orig object. Multi
    aRestMesh = tAttr.create("restMesh", "rm", MFnData::kMesh, mData.create(), &status);
	tAttr.setArray(true);
	tAttr.setUsesArrayDataBuilder(true);
	CHECKSTAT(status, "Error creating aRestMesh");
	CHECKSTAT(addAttribute(aRestMesh), "Error adding aRestMesh");

	// The name of the uv channel to sample on the deformed object list
	aUvName = tAttr.create("uvName", "uvn", MFnData::kString, sData.create(), &status);
	CHECKSTAT(status, "Error creating aUvName");
	tAttr.setStorable(true);
	CHECKSTAT(addAttribute(aUvName), "Error adding aUvName");

	// The global offset multiplier
	aGlobalOffset = nAttr.create("globalOffsetMult", "gom", MFnNumericData::kFloat, 1.0f, &status);
	CHECKSTAT(status, "Error creating aGlobalOffset");
	nAttr.setKeyable(true);
	CHECKSTAT(addAttribute(aGlobalOffset), "Error adding aGlobalOffset");

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

	aBindBarys = tAttr.create("bindBarys", "bbs", MFnData::kFloatArray, faData.create(), &status);
	CHECKSTAT(status, "Error creating aBindBarys");
	tAttr.setStorable(false);

	aBindIdxs = tAttr.create("bindIdxs", "bis", MFnData::kIntArray, iaData.create(), &status);
	CHECKSTAT(status, "Error creating aBindIdxs");
	tAttr.setStorable(false);

	aBindRanges = tAttr.create("bindRanges", "brs", MFnData::kIntArray, iaData.create(), &status);
	CHECKSTAT(status, "Error creating aBindRanges");
	tAttr.setStorable(false);

	aBindInfos = cAttr.create("bindInfos", "bi", &status);
	CHECKSTAT(status, "Error creating aBindInfos");
	cAttr.setStorable(false);
	cAttr.setArray(true);
	// cAttr.setUsesArrayDataBuilder(true);
	CHECKSTAT(cAttr.addChild(aBindBarys), "Error adding aBindBarys");
	CHECKSTAT(cAttr.addChild(aBindIdxs), "Error adding aBindIdxs");
	CHECKSTAT(cAttr.addChild(aBindRanges), "Error adding aBindRanges");
	CHECKSTAT(addAttribute(aBindInfos), "Error adding aBindInfos");


	CHECKSTAT(attributeAffects(aControlRestMesh, aBindInfos ), "Error affecting aControlRestMesh 1");
	CHECKSTAT(attributeAffects(aControlRestMesh, aBindBarys ), "Error affecting aControlRestMesh 2");
	CHECKSTAT(attributeAffects(aControlRestMesh, aBindIdxs  ), "Error affecting aControlRestMesh 3");
	CHECKSTAT(attributeAffects(aControlRestMesh, aBindRanges), "Error affecting aControlRestMesh 4");

	CHECKSTAT(attributeAffects(aControlUvName, aBindInfos ), "Error affecting aControlUvName 1");
	CHECKSTAT(attributeAffects(aControlUvName, aBindBarys ), "Error affecting aControlUvName 2");
	CHECKSTAT(attributeAffects(aControlUvName, aBindIdxs  ), "Error affecting aControlUvName 3");
	CHECKSTAT(attributeAffects(aControlUvName, aBindRanges), "Error affecting aControlUvName 4");

	CHECKSTAT(attributeAffects(aRestMesh, aBindInfos ), "Error affecting aRestMesh 1");
	CHECKSTAT(attributeAffects(aRestMesh, aBindBarys ), "Error affecting aRestMesh 2");
	CHECKSTAT(attributeAffects(aRestMesh, aBindIdxs  ), "Error affecting aRestMesh 3");
	CHECKSTAT(attributeAffects(aRestMesh, aBindRanges), "Error affecting aRestMesh 4");

	CHECKSTAT(attributeAffects(aUvName, aBindInfos ), "Error affecting aUvName 1");
	CHECKSTAT(attributeAffects(aUvName, aBindBarys ), "Error affecting aUvName 2");
	CHECKSTAT(attributeAffects(aUvName, aBindIdxs  ), "Error affecting aUvName 3");
	CHECKSTAT(attributeAffects(aUvName, aBindRanges), "Error affecting aUvName 4");

	CHECKSTAT(attributeAffects(aMismatchHandler, aBindInfos ), "Error affecting aMismatchHandler 1");
	CHECKSTAT(attributeAffects(aMismatchHandler, aBindBarys ), "Error affecting aMismatchHandler 2");
	CHECKSTAT(attributeAffects(aMismatchHandler, aBindIdxs  ), "Error affecting aMismatchHandler 3");
	CHECKSTAT(attributeAffects(aMismatchHandler, aBindRanges), "Error affecting aMismatchHandler 4");

	CHECKSTAT(attributeAffects(aControlMesh, outputGeom), "Error affecting aControlMesh");
	CHECKSTAT(attributeAffects(aControlInvWorld, outputGeom), "Error affecting aControlInvWorld");
	CHECKSTAT(attributeAffects(aGlobalOffset, outputGeom), "Error affecting aGlobalOffset");
	CHECKSTAT(attributeAffects(aOffsetMult, outputGeom), "Error affecting aOffsetMult");
	CHECKSTAT(attributeAffects(aOffsetWeights, outputGeom), "Error affecting aOffsetWeights");
	CHECKSTAT(attributeAffects(aOffsetList, outputGeom), "Error affecting aOffsetList");

	CHECKSTAT(attributeAffects(aBindBarys,  outputGeom), "Error affecting aBindBarys");
	CHECKSTAT(attributeAffects(aBindIdxs,   outputGeom), "Error affecting aBindIdxs");
	CHECKSTAT(attributeAffects(aBindRanges, outputGeom), "Error affecting aBindRanges");
	CHECKSTAT(attributeAffects(aBindInfos,  outputGeom), "Error affecting aBindInfos");

	return MStatus::kSuccess;
}



void getUvToVert(
	const MFnMesh &mesh, const MString* uvSet,
	std::vector<size_t> &uvToVert        // map from uvIdx to vertIdx
){
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
}

std::vector<uv_t> getUvArray(const MFnMesh &mesh, const MString* uvSet) {
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

}

// Take the uv barycentric mapping
// and turn it into a vertex barycetnric mapping
// This is done by averaging the influence when a vertex has multiple uvs
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
) {
	// Invert the uvToVert 
	std::vector<std::vector<size_t>> vertToUvs;
	vertToUvs.resize(numVerts);
	for (size_t uvIdx = 0; uvIdx < uvToVert.size(); ++uvIdx) {
		vertToUvs[uvToVert[uvIdx]].push_back(uvIdx);
	}

	// TODO: If a vert has multiple UV's, ignore any missing unless all are missing
	// Get the next missing index
	// If there are no missing indices, then use a highly unlikely index value to test against
	//size_t miss = (missing.empty()) ? std::numeric_limits<size_t>::max() : missing.front();

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




void getBindData(const MFnMesh &fnRestCtrl, const MFnMesh &fnRestMesh, MString *ctrlUvName, MString *uvName, short projType,
	MFloatArray &mFlatBarys, MIntArray &mFlatRanges, MIntArray &mFlatIdxs
) {

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
	getTriangulation(fnRestCtrl, ctrlUvName, flatUvTriIdxs, triToFaceIdx, faceRanges, borderEdges, borderTriIdx, uvToVert, flatVertTriIdxs);

	// Sweep the UVs
	std::vector<size_t> triIdxs;     // the tri-index per qPoint
	std::vector<size_t> missing;     // Any qPoints that weren't in a triangle
	std::vector<uv_t> barys;         // The barycentric coordinates of each point in the triangle
	sweep(qPoints, uvs, flatUvTriIdxs, triIdxs, missing, barys);

	// Project any missing uvs to the closest uv border edge
	// And add the triangle to the output
	// TODO: Find the barycentric coords of the missing points as well
	handleMissing(uvs, qPoints, borderEdges, missing, borderTriIdx, flatUvTriIdxs, triIdxs, barys);

	size_t numVerts = fnRestMesh.numVertices();
	size_t numUVs = fnRestMesh.numUVs(*uvName);
	std::vector<size_t> qUvToVert; // Map the uvIdx to the vertex idx for the query mesh
	getUvToVert(fnRestMesh, uvName, qUvToVert);

	// Translate the UV barycentric coordinates to vertices
	std::vector<double> flatBarys;  // The flattened barycentric coordinates
	std::vector<size_t> flatIdxs;   // The flattened barycentric indices
	std::vector<size_t> flatRanges; // The count of barycentric weights per index
	getVertCorrelation(numVerts, numUVs, triIdxs, flatVertTriIdxs, barys, qUvToVert, flatBarys, flatIdxs, flatRanges);

	// Copy the data to the output maya arrays
	mFlatRanges.setLength((unsigned)flatRanges.size());
	for (size_t i = 0; i < flatIdxs.size(); ++i) mFlatIdxs[(unsigned)i] = (int)flatIdxs[i];
	for (size_t i = 0; i < flatRanges.size(); ++i) mFlatRanges[(unsigned)i] = (int)flatRanges[i];
	for (size_t i = 0; i < flatBarys.size(); ++i) mFlatBarys[(unsigned)i] = (int)flatBarys[i];
}





MStatus UvWrapDeformer::compute(const MPlug& plug, MDataBlock &block) {
	MStatus status;
	if (plug == aBindInfos || plug == aBindBarys || plug == aBindIdxs || plug == aBindRanges) {



		// Get the required inputs from the data block
		MPlug par = plug.parent(&status);
		if (!status) return status;
		unsigned multiIndex = par.logicalIndex(&status);
		if (!status) return status;

		MObject restCtrl = block.inputValue(aControlRestMesh, &status).asMesh();
		if (restCtrl.isNull()) return MStatus::kInvalidParameter;
		MFnMesh fnRestCtrl(restCtrl);

		// Get the proper index of the rest mesh
		MArrayDataHandle haRestMesh = block.inputValue(aRestMesh, &status);
		haRestMesh.jumpToElement(multiIndex);
		MDataHandle hRestMesh = haRestMesh.inputValue(&status);
		MObject restMesh = hRestMesh.asMesh();
		if (restMesh.isNull()) return MStatus::kInvalidParameter;
		MFnMesh fnRestMesh(restMesh);

		MString ctrlUvName = block.inputValue(aControlUvName, &status).asString();
		MString uvName = block.inputValue(aUvName, &status).asString();
		short projType = block.inputValue(aMismatchHandler, &status).asShort();

		// Get the bind data
		MFloatArray mFlatBarys;
		MIntArray mFlatRanges;
		MIntArray mFlatIdxs;
		getBindData(fnRestCtrl, fnRestMesh, &ctrlUvName, &uvName, projType, mFlatBarys, mFlatRanges, mFlatIdxs);

		// Set the data to the output plugs
		// TODO: Fix this, I'm doing it wrong
		MArrayDataHandle bindList = block.inputArrayValue(aBindInfos);
		status = bindList.jumpToElement(multiIndex);
		if (!status) return status;
		MDataHandle bindHandle = bindList.inputValue(&status);
		if (!status) return status;

		MFnFloatArrayData fad;
		MFnIntArrayData iad;

		// TODO: Check if the inputs are connected and only overwrite the data if not
		MObject moBary = fad.create(mFlatBarys);
		MDataHandle baryHandle = bindHandle.child(aBindBarys);
		status = baryHandle.setMObject(moBary);
		if (!status) return status;

		MObject moIdxs = iad.create(mFlatIdxs);
		MDataHandle idxHandle = bindHandle.child(aBindIdxs);
		status = idxHandle.setMObject(moIdxs);
		if (!status) return status;

		MObject moRanges = iad.create(mFlatRanges);
		MDataHandle rangeHandle = bindHandle.child(aBindRanges);
		status = rangeHandle.setMObject(moRanges);
		if (!status) return status;

		block.setClean(aBindBarys);
		block.setClean(aBindIdxs);
		block.setClean(aBindRanges);



	}
	else {
		return MPxDeformerNode::compute(plug, block);
	}
	return MStatus::kSuccess;
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

    MMatrix cWInv = block.inputValue(aControlInvWorld, &status).asMatrix();
    float globalMult = block.inputValue(aGlobalOffset, &status).asFloat();
    short projType = block.inputValue(aMismatchHandler, &status).asShort();


	/*
	// Make sure that the plug for this array index exists
	MArrayDataHandle haBindInfos = block.outputArrayValue(aBindInfos);
	status = haBindInfos.jumpToElement(multiIndex);
	MDataHandle hBindInfos, hBindBarys;
	if (status) {
		hBindInfos = haBindInfos.inputValue(&status);
	}
	else {
		// Make sure to build the array index if it doesn't exist
		MArrayDataBuilder biBuilder = haBindInfos.builder(&status);
		if (!status) return status;
		hBindInfos = biBuilder.addElement(multiIndex, &status);
	}
	if (!status) return status;

	// Get the bind data handles for the current index
	MDataHandle hBindBarys = hBindInfos.child(aBindBarys);
	MObject oBindBarys = hBindBarys.data();
	MDataHandle hBindIdxs = hBindInfos.child(aBindIdxs);
	MObject oBindIdxs = hBindIdxs.data();
	MDataHandle hBindRanges = hBindInfos.child(aBindRanges);
	MObject oBindRanges = hBindRanges.data();

	MFloatArray flatBarys;
	MIntArray flatIdxs, flatRanges;
	if (oBindBarys.isNull() || oBindIdxs.isNull() || oBindRanges.isNull()) {
		// If any of the data is null, build the bind here
		MFloatArray mFlatBarys;
		MIntArray mFlatRanges;
		MIntArray mFlatIdxs;
		// TODO:
		// status = getBindData(fnRestCtrl, fnRestMesh, &ctrlUvName, &uvName, projType, mFlatBarys, mFlatRanges, mFlatIdxs);



	}
	else {
		MFnFloatArrayData dBindBarys(oBindBarys, &status);
		if (!status) return status;
		flatBarys = dBindBarys.array();

		MFnIntArrayData dBindIdxs(oBindIdxs, &status);
		if (!status) return status;
		flatIdxs = dBindIdxs.array();

		MFnIntArrayData dBindRanges(oBindRanges, &status);
		if (!status) return status;
		flatRanges = dBindRanges.array();
	}






	// Because there's no default constructor, get a pointer to a MArrayDataHandle
	// That way I can just check for null and I don't have to re-walk the data block for the plug I want
	std::unique_ptr<MArrayDataHandle> hOffsetPtr = getArrayDataPtr(block, aOffsetList, aOffsetWeights, multiIndex);
	std::unique_ptr<MArrayDataHandle> hWeightPtr = getArrayDataPtr(block, weightList, weights, multiIndex);

	MPointArray ctrlVerts;
	fnCtrlMesh.getPoints(ctrlVerts);

	for (; !iter.isDone(); iter.next()) {
		unsigned idx = iter.index();
		float wVal = readArrayDataPtr(hWeightPtr, idx, 1.0) * env;
		if (wVal == 0.0) continue;

		float oVal = readArrayDataPtr(hOffsetPtr, idx, 1.0);

		MPoint pt;
		for (unsigned i = flatRanges[idx]; i < flatRanges[idx + 1]; ++i) {
			pt += (MVector)ctrlVerts[flatIdxs[i]] * flatBarys[i];
		}
		if (wVal < 1.0) {
			MPoint cp = iter.position();
			pt = ((pt - cp) * wVal) + cp;
		}
		iter.setPosition(pt);
	}

	*/
    return MStatus::kSuccess;
}

