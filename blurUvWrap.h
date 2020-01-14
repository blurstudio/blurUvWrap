#pragma once
#include <maya/MPxDeformerNode.h>
#include <maya/MTypeId.h>
#include <maya/MPlug.h>
#include <maya/MString.h>
#include <maya/MDataBlock.h>
#include <maya/MMatrix.h>
#include <maya/MDagModifier.h>
#include <maya/MPxGPUDeformer.h>
#include <maya/MGPUDeformerRegistry.h>
#include <maya/MOpenCLInfo.h>
#include <maya/MFnNumericAttribute.h>
#include <vector>

#define DEFORMER_NAME "blurUvWrap"


class UvWrapDeformer : public MPxDeformerNode {
public:
	UvWrapDeformer() {};
	virtual ~UvWrapDeformer() {};




    static void* creator();
    static MStatus initialize();
    virtual MStatus deform(MDataBlock& block, MItGeometry& iter, const MMatrix& mat, unsigned int multiIndex);
    static MTypeId id;

	static MObject aControlRestMesh;
	static MObject aControlMesh;
	static MObject aControlInvWorld;
	static MObject aControlUvName;
	static MObject aRestMesh;
	static MObject aUvName;
	static MObject aGlobalOffset;
	static MObject aOffsetList;
	static MObject aOffsetMult;
	static MObject aOffsetWeights;
	static MObject aMismatchHandler;

};

