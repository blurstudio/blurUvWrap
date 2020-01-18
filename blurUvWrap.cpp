#include "blurUvWrap.h"
#include "uvQuery.h"
#include "mayaQuery.h"

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

	// Store the flat bind barycentric coords
	aBindBarys = tAttr.create("bindBarys", "bbs", MFnData::kFloatArray, faData.create(), &status);
	CHECKSTAT(status, "Error creating aBindBarys");
	tAttr.setStorable(false);

	// Store the flat indices per barycoord
	aBindIdxs = tAttr.create("bindIdxs", "bis", MFnData::kIntArray, iaData.create(), &status);
	CHECKSTAT(status, "Error creating aBindIdxs");
	tAttr.setStorable(false);

	// Store the range of indices to apply to each deformed vert
	aBindRanges = tAttr.create("bindRanges", "brs", MFnData::kIntArray, iaData.create(), &status);
	CHECKSTAT(status, "Error creating aBindRanges");
	tAttr.setStorable(false);

	// The holder for the bind data
	aBindInfos = cAttr.create("bindInfos", "bi", &status);
	CHECKSTAT(status, "Error creating aBindInfos");
	cAttr.setStorable(false);
	cAttr.setArray(true);
	cAttr.setUsesArrayDataBuilder(true);
	CHECKSTAT(cAttr.addChild(aBindBarys), "Error adding aBindBarys");
	CHECKSTAT(cAttr.addChild(aBindIdxs), "Error adding aBindIdxs");
	CHECKSTAT(cAttr.addChild(aBindRanges), "Error adding aBindRanges");
	CHECKSTAT(addAttribute(aBindInfos), "Error adding aBindInfos");

	CHECKSTAT(attributeAffects(aControlRestMesh, outputGeom ), "Error affecting aControlRestMesh 1");
	CHECKSTAT(attributeAffects(aControlUvName, outputGeom ), "Error affecting aControlUvName 1");
	CHECKSTAT(attributeAffects(aRestMesh, outputGeom ), "Error affecting aRestMesh 1");
	CHECKSTAT(attributeAffects(aUvName, outputGeom ), "Error affecting aUvName 1");
	CHECKSTAT(attributeAffects(aMismatchHandler, outputGeom ), "Error affecting aMismatchHandler 1");

	CHECKSTAT(attributeAffects(aControlMesh, outputGeom), "Error affecting aControlMesh");
	CHECKSTAT(attributeAffects(aControlInvWorld, outputGeom), "Error affecting aControlInvWorld");
	CHECKSTAT(attributeAffects(aGlobalOffset, outputGeom), "Error affecting aGlobalOffset");
	CHECKSTAT(attributeAffects(aOffsetMult, outputGeom), "Error affecting aOffsetMult");
	CHECKSTAT(attributeAffects(aOffsetWeights, outputGeom), "Error affecting aOffsetWeights");
	CHECKSTAT(attributeAffects(aOffsetList, outputGeom), "Error affecting aOffsetList");

	CHECKSTAT(attributeAffects(outputGeom, aBindBarys), "Error affecting aBindBarys");
	CHECKSTAT(attributeAffects(outputGeom, aBindIdxs), "Error affecting aBindIdxs");
	CHECKSTAT(attributeAffects(outputGeom, aBindRanges), "Error affecting aBindRanges");
	CHECKSTAT(attributeAffects(outputGeom, aBindInfos), "Error affecting aBindInfos");

	return MStatus::kSuccess;
}

MStatus UvWrapDeformer::setDependentsDirty(const MPlug& plugBeingDirtied, MPlugArray& affectedPlugs) {
	// Extract the geom index from the dirty plug and set the dirty flag so we know that we need to
	// re-read the binding data.
	const char * nn = plugBeingDirtied.name().asChar();

	if (plugBeingDirtied.isElement()) {
		if (plugBeingDirtied == aRestMesh) {
			// If the rest mesh changes, then recalculate that bind
			unsigned int geomIndex = plugBeingDirtied.logicalIndex();
			_dirty[geomIndex] = true;
		}
	}
	else if (plugBeingDirtied == aControlRestMesh) {
		// If the rest control changes, then recalculate *everything*
		for (auto &x : _dirty) {
			x.second = true;
		}
	}
	return MS::kSuccess;
}

MStatus UvWrapDeformer::compute(const MPlug& plug, MDataBlock &block) {
	MStatus status;
	if (plug == aBindInfos || plug == aBindBarys || plug == aBindIdxs || plug == aBindRanges) {

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


	// If I'm dirty, or this multi-index hasn't been computed yet
	if (
		_dirty[multiIndex] ||
		_coords.find(multiIndex) == _coords.end() ||
		_idxs.find(multiIndex) == _idxs.end() ||
		_ranges.find(multiIndex) == _ranges.end()
	) {
		// Get the required inputs from the data block
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

		short projType = block.inputValue(aMismatchHandler, &status).asShort();

		MString ctrlUvName = block.inputValue(aControlUvName, &status).asString();
		MString uvName = block.inputValue(aUvName, &status).asString();

		if (ctrlUvName == NULL) {
			MStringArray msa;
			fnCtrlMesh.getUVSetNames(msa);
			ctrlUvName = msa[0];
		}

		if (uvName == NULL) {
			MStringArray msa;
			fnRestMesh.getUVSetNames(msa);
			uvName = msa[0];
		}

		// Get the bind data
		std::vector<double> flatBarys;
		std::vector<size_t> flatRanges;
		std::vector<size_t> flatIdxs;
		bool success = getBindData(fnRestCtrl, fnRestMesh, &ctrlUvName, &uvName, projType, flatBarys, flatRanges, flatIdxs);

		_coords[multiIndex] = flatBarys;
		_idxs[multiIndex] = flatIdxs;
		_ranges[multiIndex] = flatRanges;

		// Get each face-vertex normal and tangent
		// Average the normals, and take the last tangent found 'cause its an easy algorithm
		// Re-normalize everything and store in an array of matrices
		// Get the input point positions in that matrix space, and store

		_dirty[multiIndex] = false;
	}

	std::vector<double> flatBarys = _coords[multiIndex];
	std::vector<size_t> flatIdxs = _idxs[multiIndex];
	std::vector<size_t> flatRanges = _ranges[multiIndex];

	if (flatBarys.empty() || flatIdxs.empty() || flatRanges.empty()) return MStatus::kFailure;

	// Because there's no default constructor, get a pointer to a MArrayDataHandle
	// That way I can just check for null and I don't have to re-walk the data block for the plug I want
	std::unique_ptr<MArrayDataHandle> hOffsetPtr = getArrayDataPtr(block, aOffsetList, aOffsetWeights, multiIndex);
	std::unique_ptr<MArrayDataHandle> hWeightPtr = getArrayDataPtr(block, weightList, weights, multiIndex);

	MPointArray ctrlVerts;
	fnCtrlMesh.getPoints(ctrlVerts);


	// Get each face-vertex normal and tangent
	// Average the normals, and take the last tangent found 'cause its an easy algorithm


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

    return MStatus::kSuccess;
}

