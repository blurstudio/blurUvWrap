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
#include <maya/MVectorArray.h>
#include <maya/MPointArray.h>

#include <vector>
#include <unordered_map>

#define DEFORMER_NAME "blurUvWrap"

class UvWrapDeformer : public MPxDeformerNode {
public:
	UvWrapDeformer() {};
	virtual ~UvWrapDeformer() {};

    static void* creator();
    static MStatus initialize();

	//virtual MStatus compute(const MPlug& plug, MDataBlock& block);
    virtual MStatus deform(MDataBlock& block, MItGeometry& iter, const MMatrix& mat, unsigned int multiIndex);
	virtual MStatus setDependentsDirty(const MPlug& plugBeingDirtied, MPlugArray& affectedPlugs);

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

	static MObject aBindInfos;
	static MObject aBindBarys;
	static MObject aBindIdxs;
	static MObject aBindRanges;
private:

  std::unordered_map<unsigned, bool> _dirty;
  std::unordered_map<unsigned, std::vector<double>> _coords;
  std::unordered_map<unsigned, std::vector<size_t>> _idxs;
  std::unordered_map<unsigned, std::vector<size_t>> _ranges;
  std::unordered_map<unsigned, MPointArray> _offsets;

};

