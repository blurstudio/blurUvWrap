#include "blurUvWrap.h"

#include <vector>
#include <array>
#include <algorithm>
#include <maya/MItGeometry.h>
#include <maya/MFloatVectorArray.h>
#include <maya/MGlobal.h>
#include <maya/MFnMesh.h>
#include <maya/MFnEnumAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnCompoundAttribute.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MFnStringData.h>
#include <maya/MMeshIntersector.h>
#include <maya/MFnMeshData.h>
#include <maya/MFloatMatrix.h>
#include <maya/MBoundingBox.h>
#include <maya/MPointArray.h>

#define CHECKSTAT(status, msg) {MStatus tStat = (status);  if ( tStat ) {  MGlobal::displayError(msg); return tStat; }}

#define MM_UVEDGE  0
#define MM_UV      1
#define MM_VERT    2
#define MM_POINT   3

#define EPS 1e-7

typedef std::array<double, 2> uv_t;

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


	// The non-deforming rest of the controller mesh
    aControlRestMesh = tAttr.create("controlRest", "cr", MFnData::kMesh, &status);
	CHECKSTAT(status, "Error creating aControlRestMesh");
	CHECKSTAT(addAttribute(aControlRestMesh), "Error adding aControlRestMesh");
	CHECKSTAT(attributeAffects(aControlRestMesh, outputGeom), "Error affecting aControlRestMesh");
	// The deforming controller mesh
    aControlMesh = tAttr.create("controlMesh", "cm", MFnData::kMesh, &status);
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
    aRestMesh = tAttr.create("restMesh", "rm", MFnData::kMesh, &status);
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
	aGlobalOffset = nAttr.create("globalOffset", "go", MFnNumericData::kFloat, 1.0f, &status);
	CHECKSTAT(status, "Error creating aGlobalOffset");
	nAttr.setKeyable(true);
	CHECKSTAT(addAttribute(aGlobalOffset), "Error adding aGlobalOffset");
	CHECKSTAT(attributeAffects(aGlobalOffset, outputGeom), "Error affecting aGlobalOffset");


	// The per-deformed offset multiplier. Child of aOffsetList
	aOffsetMult = nAttr.create("offset", "of", MFnNumericData::kFloat, 1.0f, &status);
	CHECKSTAT(status, "Error creating aOffsetMult");
	nAttr.setKeyable(true);
	CHECKSTAT(attributeAffects(aOffsetMult, outputGeom), "Error affecting aOffsetMult");
	// The per-deformed offset weightmap. Mutli. Child of aOffsetList
	aOffsetWeights = nAttr.create("offsetWeights", "ow", MFnNumericData::kFloat, 1.0f, &status);
	CHECKSTAT(status, "Error creating aOffsetWeights");
	nAttr.setArray(true);
	CHECKSTAT(attributeAffects(aOffsetWeights, outputGeom), "Error affecting aOffsetWeights");
	// The compound parent for the per-deformed values. Multi
	aOffsetList = cAttr.create("offsetList", "ol", &status);
	CHECKSTAT(status, "Error creating aOffsetList");
	cAttr.setArray(true);
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
	CHECKSTAT(attributeAffects(aMismatchHandler, outputGeom), "Error affecting aMismatchHandler");

	return MStatus::kSuccess;
}







void getTriangulation(MFnMesh &mesh){
	std::vector<size_t> triVerts;
	MIntArray triCounts, mTriVerts, vertCounts, faceIdxs;
	mesh.getTriangles(triCounts, mTriVerts);
	mesh.getVertices(vertCounts, faceIdxs);

	// Copy to a Vector for convenience
	triVerts.reserve(mTriVerts.length());
	for (size_t i=0; i<mTriVerts.length(); ++i){
		triVerts.push_back(mTriVerts[i]);
	}





}


/*


aOffsetMult; 
aOffsetWeights; 

aMismatchHandler; 


*/


MStatus UvWrapDeformer::deform(
    MDataBlock& block, MItGeometry& iter,
    const MMatrix& wm, unsigned int multiIndex
) {
    MStatus status;

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


	MArrayDataHandle hOffsetList = block.outputArrayValue(aOffsetList, &status);
	status = hOffsetList.jumpToElement(multiIndex);
	if (!status) return status; 
	MDataHandle hOffsetCompound = hOffsetList.inputValue(&status);
	if (!status) return status; 

	float localMult = hOffsetCompound.child(aOffsetMult).asFloat();
	MArrayDataHandle hOffWeights = hOffsetCompound.child(aOffsetWeights);

	/*
	// perVert getWeight example
	hOffWeights.jumpToElement(vertIdx);
	hMult = hOffWeights.inputValue(&status);
	float vertMult = 0.0f;
	if (status) vertMult = hMult.asFloat(); // maya doesn't store if non-zero
	*/

    short projType = block.inputValue(aMismatchHandler, &status).asShort();


	for (; !iter.isDone(); iter.next()) {
		// Loop the iterator
	
	}


    return MStatus::kSuccess;
}
