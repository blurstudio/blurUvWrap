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
MObject UvWrapDeformer::aTargetMesh;
MObject UvWrapDeformer::aTargetWorld;
MObject UvWrapDeformer::aSourceInvWorld;
MObject UvWrapDeformer::aOffsetList;
MObject UvWrapDeformer::aOffsetMult;
MObject UvWrapDeformer::aOffsetWeights;
MObject UvWrapDeformer::aMismatchHandler;
MObject UvWrapDeformer::aSourceUvName;
MObject UvWrapDeformer::aTargetUvName;


void* UvWrapDeformer::creator() { return new UvWrapDeformer(); }
MStatus UvWrapDeformer::initialize() {
	MStatus status;
	MFnNumericAttribute nAttr;
	MFnEnumAttribute eAttr;
	MFnTypedAttribute tAttr;
    MFnMatrixAttribute mAttr;
    MFnCompoundAttribute cAttr;
	MFnStringData sData;

    aTargetMesh = tAttr.create("target", "t", MFnData::kMesh, &status);

	CHECKSTAT(status, "Error creating aTargetMesh");
	CHECKSTAT(addAttribute(aTargetMesh), "Error adding aTargetMesh");
	CHECKSTAT(attributeAffects(aTargetMesh, outputGeom), "Error affecting aTargetMesh");

    aTargetWorld = mAttr.create("targetWorld", "tw", MFnMatrixAttribute::kDouble, &status);
	CHECKSTAT(status, "Error creating aTargetWorld");
    CHECKSTAT(addAttribute(aTargetWorld), "Error adding aTargetWorld");
    CHECKSTAT(attributeAffects(aTargetWorld, outputGeom), "Error affecting aTargetWorld");

	aSourceInvWorld = mAttr.create("sourceInvWorld", "siw", MFnMatrixAttribute::kDouble, &status);
	CHECKSTAT(status, "Error creating aSourceInvWorld");
	mAttr.setArray(true);
    CHECKSTAT(addAttribute(aSourceInvWorld), "Error adding siw");
    CHECKSTAT(attributeAffects(aSourceInvWorld, outputGeom), "Error affecting siw");

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

	aOffsetMult = nAttr.create("offsetMult", "om", MFnNumericData::kFloat, 1.0f, &status);
	CHECKSTAT(status, "Error creating aOffsetMult");
	nAttr.setKeyable(true);
	CHECKSTAT(addAttribute(aOffsetMult), "Error adding aOffsetMult");
	CHECKSTAT(attributeAffects(aOffsetMult, outputGeom), "Error affecting aOffsetMult");

	aOffsetWeights = nAttr.create("offsetWeights", "ow", MFnNumericData::kFloat, 1.0f, &status);
	CHECKSTAT(status, "Error creating aOffsetWeights");
	nAttr.setArray(true);
	CHECKSTAT(attributeAffects(aOffsetWeights, outputGeom), "Error affecting aOffsetWeights");

	aOffsetList = cAttr.create("offsetList", "ol", &status);
	CHECKSTAT(status, "Error creating aOffsetList");
	cAttr.setArray(true);
	CHECKSTAT(cAttr.addChild(aOffsetWeights), "Error adding aOffsetWeights");
	CHECKSTAT(addAttribute(aOffsetList), "Errod adding aOffsetList");

	aSourceUvName = tAttr.create("sourceUv", "suv", MFnData::kString, sData.create(), &status);
	CHECKSTAT(status, "Error creating aSourceUvName");
	tAttr.setStorable(true);
	CHECKSTAT(addAttribute(aSourceUvName), "Error adding aSourceUvName");
	CHECKSTAT(attributeAffects(aSourceUvName, outputGeom), "Error affecting aSourceUvName");
	
	aTargetUvName = tAttr.create("targetUv", "tuv", MFnData::kString, sData.create(), &status);
	CHECKSTAT(status, "Error creating aTargetUvName");
	tAttr.setStorable(true);
	CHECKSTAT(addAttribute(aTargetUvName), "Error adding aTargetUvName");
	CHECKSTAT(attributeAffects(aTargetUvName, outputGeom), "Error affecting aTargetUvName");

	return MStatus::kSuccess;
}






MStatus UvWrapDeformer::deform(
    MDataBlock& block, MItGeometry& iter,
    const MMatrix& m, unsigned int multiIndex
) {
    MStatus status;

    float env = block.inputValue(envelope, &status).asFloat();
    if (env == 0.0f) return status;

    MObject target = block.inputValue(aTargetMesh, &status).asMesh();
	if (target.isNull()) return MStatus::kInvalidParameter;
    MFnMesh fnTarget(target);

    float maxParam = block.inputValue(aMaxParam, &status).asFloat();
    bool reverse = block.inputValue(aReverse, &status).asBool();
    bool biDir = block.inputValue(aBidirectional, &status).asBool();
    bool cin = block.inputValue(aClosestIfNone, &status).asBool();
    short projType = block.inputValue(aProjectionType, &status).asShort();
    short vSpace = block.inputValue(aVectorSpace, &status).asShort();
    MMatrix tWInv = block.inputValue(aTargetInvWorld, &status).asMatrix();
	double* projVector = block.inputValue(aProjectionVector, &status).asDouble3();

	MMatrix tranMatInv = m * tWInv;
	MMatrix tranMat = tranMatInv.inverse();

    MVector vec;
	MPoint ctr;
    bool perVert = false;
    bool doClosest = false;
    switch (projType){
        case PA_POSX: vec = MFloatVector::xAxis; break;
        case PA_POSY: vec = MFloatVector::yAxis; break;
        case PA_POSZ: vec = MFloatVector::zAxis; break;
			vec = (vec * tranMatInv).normal();
            break;
        case PA_VECTOR:
            vec.x = projVector[0];
            vec.y = projVector[1];
            vec.z = projVector[2];
			if (vSpace == VS_OBJECT) vec *= tranMatInv;
			else if (vSpace == VS_WORLD) vec *= tWInv;
            break;

        case PA_CENTER:
			vec = MFnDagNode(target).boundingBox().center();
            perVert = true; break;
        case PA_TOWARDS:
			// Because maya treats points and vectors differently
			// Gotta make sure to use an MPoint
            ctr.x = projVector[0];
            ctr.y = projVector[1];
            ctr.z = projVector[2];
			if (vSpace == VS_OBJECT) ctr *= tranMatInv;
			else if (vSpace == VS_WORLD) ctr *= tWInv;
			vec = ctr;
            perVert = true; break;
        case PA_NORMAL:
            perVert = true; break;

        case PA_CLOSEST:
        case PA_CLOSESTSM:
            doClosest = true; break;
    }

    MMeshIntersector octree;
	MObject smoothMeshPar, smoothMesh;
	if (doClosest || cin) {
        if (projType == PA_CLOSESTSM){
            MFnMeshData smoothMeshParFn;
            MMeshSmoothOptions smoothOpt;
            smoothMeshPar = smoothMeshParFn.create();
            smoothOpt.setDivisions(2);
            smoothOpt.setKeepBorderEdge(true);
            smoothOpt.setSubdivisionType(MMeshSmoothOptions::kCatmullClark);
            smoothMesh = fnTarget.generateSmoothMesh(smoothMeshPar, &smoothOpt);
            octree.create(smoothMesh);
        }
        else {
            octree.create(target);
        }
	}

    MMeshIsectAccelParams mmAccelParams = fnTarget.autoUniformGridParams();

    for (size_t i=0; !iter.isDone(); iter.next(), i++) {
        float w = weightValue(block, multiIndex, iter.index());
        if (w == 0.0f) continue;
		MPoint pt = iter.position();
		MPoint tpt = pt * tranMatInv; // target space point

        bool sect = false;
		MVector pvec(vec);

        if (perVert){
            if (projType == PA_NORMAL){
				pvec = (iter.normal() * -1) * tranMatInv;
			}
            else { pvec = pvec - tpt; }
			pvec.normalize();
        }
		if (reverse) pvec *= -1;

        if (!doClosest){
			MFloatPoint hit;
            sect = fnTarget.closestIntersection(
                tpt, pvec, nullptr, nullptr, true, MSpace::kObject, maxParam, false, &mmAccelParams,
                hit, nullptr, nullptr, nullptr, nullptr, nullptr, 1e-6f, &status 
            );
			if (!sect && biDir) {
				sect = fnTarget.closestIntersection(
					tpt, pvec*-1, nullptr, nullptr, true, MSpace::kObject, maxParam, false, &mmAccelParams,
					hit, nullptr, nullptr, nullptr, nullptr, nullptr, 1e-6f, &status 
				);
			}

            if (sect){
                iter.setPosition(env * w * ((MPoint(hit) * tranMat) - pt) + pt);
                continue;
            }
        }

        if (doClosest || (!sect && cin)){
            MPointOnMesh res;
            octree.getClosestPoint(tpt, res, maxParam);
			iter.setPosition(env * w * ((MPoint(res.getPoint()) * tranMat) - pt) + pt);
        }
    }
    return status;
}
