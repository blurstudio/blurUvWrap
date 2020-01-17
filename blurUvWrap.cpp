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

