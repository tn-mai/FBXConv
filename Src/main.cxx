/****************************************************************************************

   Copyright (C) 2015 Autodesk, Inc.
   All rights reserved.

   Use of this software is subject to the terms of the Autodesk license agreement
   provided at the time of installation or download, or which otherwise accompanies
   this software in either electronic or hard copy form.

****************************************************************************************/

/////////////////////////////////////////////////////////////////////////
//
// The example illustrates how to:
//        1) get normals of mesh.
//        2) get smoothing info of mesh.
//        3) compute smoothing info from normals.
//        4) convert hard/soft edges info to smoothing group info.
//
//Background knowledge:
//There are two kinds of smoothing info: 
//1. Smoothing groups info which is saved by polygon. It usually come from 3ds Max, because 3ds Max can set smoothing groups for polygon.
//2. Hard/soft edges info which is saved by edge. It usually come from Maya, because Maya can set hard/soft edges.
//
//steps:
// 1. initialize FBX sdk object.
// 2. load fbx scene form the specified file.
// 3. Get root node of the scene.
// 4. Recursively traverse each node in the scene.
// 5. Get normals of mesh, according to different mapping mode and reference mode.
// 6. Recursively traverse each node in the scene.
// 7. Computing smoothing info from normals or convert smoothing info
// 8. Get smoothing info of mesh, according to different mapping mode and reference mode.
// 9. Destroy all objects.
//
/////////////////////////////////////////////////////////////////////////

#include "../Common/Common.h"
#include <fbxsdk.h>
#include <stdint.h>
#include <vector>
#include <algorithm>
#include <numeric>
#include <fstream>

#define SAMPLE_FILENAME "TestData/block1.fbx"

namespace /* unnamed */ {

  template<typename T>
  std::ofstream& Output(std::ofstream& ofs, T value) {
	const unsigned char* p = reinterpret_cast<unsigned char*>(&value);
	for (int i = 0; i < sizeof(T); ++i) {
	  ofs << p[i];
	}
	return ofs;
  }

  struct Vector3 {
	float x, y, z;
	Vector3() {}
	Vector3(float xx, float yy, float zz) : x(xx), y(yy), z(zz) {}
	Vector3(const FbxVector4& v) : x(static_cast<float>(v[0])), y(static_cast<float>(v[1])), z(static_cast<float>(v[2])) {}
	Vector3& operator==(const FbxVector4& v) { *this = Vector3(v); return *this; }
  };
  struct Vector4 {
	float x, y, z, w;
	Vector4() {}
	Vector4(float xx, float yy, float zz, float ww) : x(xx), y(yy), z(zz), w(ww) {}
	Vector4(const FbxVector4& v) : x(static_cast<float>(v[0])), y(static_cast<float>(v[1])), z(static_cast<float>(v[2])), w(static_cast<float>(v[3])) {}
	Vector4& operator==(const FbxVector4& v) { *this = Vector4(v); return *this; }
  };
  struct Position2 {
	uint16_t u, v;
	Position2() {}
	Position2(const FbxVector2& v) : u(static_cast<uint16_t>(v[0] * 65535U)), v(static_cast<uint16_t>(v[1] * 65535U)) {}
	Position2& operator==(const FbxVector2& v) { *this = Position2(v); return *this; }
  };
  struct Quaternion {
	float x, y, z, w;
	Quaternion() {}
	Quaternion(float xx, float yy, float zz, float ww) : x(xx), y(yy), z(zz), w(ww) {}
	Quaternion(const FbxQuaternion& v) : x(static_cast<float>(v[0])), y(static_cast<float>(v[1])), z(static_cast<float>(v[2])), w(static_cast<float>(v[3])) {}
	Quaternion& operator==(const FbxQuaternion& v) { *this = Quaternion(v); return *this; }
  };
  struct RotTrans {
	Quaternion rot;
	Vector3 trans;
  };
  struct Vertex {
	Vector3    position; ///< 頂点座標. 12
	uint8_t    weight[4]; ///< 頂点ブレンディングの重み. 0-255 = 0.0-1.0として量子化した値を格納する. 4
	Vector3    normal; ///< 頂点ノーマル. 12
	uint8_t    boneID[4]; ///< 頂点ブレンディングのボーンID. 4
	Position2  texCoord[2]; ///< ディフューズ(withメタルネス)マップ座標, ノーマル(withラフネス)マップ座標. 8
	Vector4    tangent; ///< 頂点タンジェント. 16

	Vertex() {}
  };
  struct Animation {
	uint8_t  nameLength;
	char     name[24];
	bool     loopFlag;
	uint16_t keyFrameCount;
	float    totalFrames;
	struct KeyFrame {
	  float time;
	  std::vector<RotTrans> pose;
	};
	std::vector<KeyFrame> list;
  };

  Vector3 Cross(const Vector3& lhs, const Vector3& rhs) {
	return Vector3(lhs.y * rhs.z - lhs.z * rhs.y, lhs.z * rhs.x - lhs.x * rhs.z, lhs.x * rhs.y - lhs.y * rhs.x);
  }
  float Dot(const Vector3& lhs, const Vector3& rhs) {
	return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
  }
  Vector3 Normalize(const Vector3& v) {
	const float invNorm = 1.0f / std::sqrt(Dot(v, v));
	return Vector3(v.x * invNorm, v.y * invNorm, v.z * invNorm);
  }
  constexpr bool operator==(const Vector3& lhs, const Vector3& rhs) { return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z; }
  constexpr bool operator==(const Vector4& lhs, const Vector4& rhs) { return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z && lhs.w == rhs.w; }
  constexpr bool operator==(const Position2& lhs, const Position2& rhs) { return lhs.u == rhs.u && lhs.v == rhs.v; }
  constexpr bool operator==(const Vertex& lhs, const Vertex& rhs) {
	return
	  lhs.position == rhs.position &&
	  lhs.weight[0] == rhs.weight[0] && lhs.weight[1] == rhs.weight[1] && lhs.weight[2] == rhs.weight[2] && lhs.weight[3] == rhs.weight[3] &&
	  lhs.normal == rhs.normal &&
	  lhs.boneID[0] == rhs.boneID[0] && lhs.boneID[1] == rhs.boneID[1] && lhs.boneID[2] == rhs.boneID[2] && lhs.boneID[3] == rhs.boneID[3] &&
	  lhs.texCoord[0] == rhs.texCoord[0] && lhs.texCoord[1] == rhs.texCoord[1] &&
	  lhs.tangent == rhs.tangent;
  }
  std::ofstream& operator<<(std::ofstream& ofs, const Vertex& v) {
	Output(ofs, v.position.x);
	Output(ofs, v.position.y);
	Output(ofs, v.position.z);
	ofs << v.weight[0] << v.weight[1] << v.weight[2] << v.weight[3];
	Output(ofs, v.normal.x);
	Output(ofs, v.normal.y);
	Output(ofs, v.normal.z);
	ofs << v.boneID[0] << v.boneID[1] << v.boneID[2] << v.boneID[3];
	Output(ofs, v.texCoord[0].u);
	Output(ofs, v.texCoord[0].v);
	Output(ofs, v.texCoord[1].u);
	Output(ofs, v.texCoord[1].v);
	Output(ofs, v.tangent.x);
	Output(ofs, v.tangent.y);
	Output(ofs, v.tangent.z);
	Output(ofs, v.tangent.w);
	return ofs;
  }

  struct NormalElementTypeInfo {
	typedef FbxGeometryElementNormal Type;
	static Type* Get(FbxMesh* p) { return p->GetElementNormal(); }
	static const char* Name() { return "normal"; }
  };

  struct TangentElementTypeInfo {
	typedef FbxGeometryElementTangent Type;
	static Type* Get(FbxMesh* p) { return p->GetElementTangent(); }
	static const char* Name() { return "tangent"; }
  };

  template<typename ElementTypeInfoT> void GetMeshElement(FbxMesh* lMesh);

  template<typename T>
  struct Element {
	const FbxMesh* pm;
	const T* p;
	FbxGeometryElement::EMappingMode mappingMode;
	FbxGeometryElement::EReferenceMode referenceMode;

	Element(const FbxMesh* pMesh, const T* pElement) : pm(pMesh), p(pElement) {
	  mappingMode = p->GetMappingMode();
	  referenceMode = p->GetReferenceMode();
	}
	~Element() {}

	FbxVector4 Get(int polyIndex, int posInPoly) const {
	  switch (mappingMode) {
	  case FbxGeometryElement::eByControlPoint:
		return GetByControlPoint(polyIndex, posInPoly);
	  case FbxGeometryElement::eByPolygonVertex:
		return GetByPolygonVertex(polyIndex, posInPoly);
	  default:
		return FbxVector4(0, 0, 0, 1);
	  }
	}

  private:
	//Let's get element of target vertex, since the mapping mode of element is by control point
	FbxVector4 GetByControlPoint(int polyIndex, int posInPoly) const {
	  const int lVertexIndex = pm->GetPolygonVertex(polyIndex, posInPoly);
	  int lNormalIndex;
	  switch (referenceMode) {
	  case FbxGeometryElement::eDirect:
		//reference mode is direct, the normal index is same as vertex index.
		//get normals by the index of control vertex
		lNormalIndex = lVertexIndex;
		break;
	  case FbxGeometryElement::eIndexToDirect:
		//reference mode is index-to-direct, get normals by the index-to-direct
		lNormalIndex = p->GetIndexArray().GetAt(lVertexIndex);
		break;
	  default:
		lNormalIndex = 0;
		break;
	  }
	  return p->GetDirectArray().GetAt(lNormalIndex);
	}

	//mapping mode is by polygon-vertex.
	//we can get element by retrieving polygon-vertex.
	FbxVector4 GetByPolygonVertex(int polyIndex, int posInPoly) const {
	  const int lIndexByPolygonVertex = polyIndex * 3 + posInPoly;
	  int lNormalIndex = 0;
	  switch (referenceMode) {
	  case FbxGeometryElement::eDirect:
		//reference mode is direct, the normal index is same as lIndexByPolygonVertex.
		lNormalIndex = lIndexByPolygonVertex;
		break;
	  case FbxGeometryElement::eIndexToDirect:
		//reference mode is index-to-direct, get normals by the index-to-direct
		lNormalIndex = p->GetIndexArray().GetAt(lIndexByPolygonVertex);
		break;
	  default:
		lNormalIndex = 0;
		break;
	  }
	  return p->GetDirectArray().GetAt(lNormalIndex);
	}
  };
  typedef Element<FbxLayerElementTangent> TangentElement;
//  typedef Element<FbxLayerElementBinormal> BinormalElement;

  struct BoneWeightInfo {
	BoneWeightInfo(int vi, int bi, float w) : vertexIndex(static_cast<uint16_t>(vi)), boneIndex(static_cast<uint16_t>(bi)), weight(w) {}
	uint16_t vertexIndex;
	uint16_t boneIndex;
	float weight;
  };

  struct BoneWeight {
	BoneWeight() : boneIndex{ -1, -1, -1, -1 }, weight{ 0, 0, 0, 0 } {}
	void Add(int index, float w) {
	  for (int i = 0; i < 4; ++i) {
		if (boneIndex[i] == -1) {
		  boneIndex[i] = index;
		  weight[i] = w;
		  break;
		}
	  }
	}
	int boneIndex[4];
	float weight[4];
  };

  struct Mesh {
	size_t RawSize() const { return sizeof(uint32_t) + sizeof(uint32_t) + sizeof(uint8_t) + nameLength; }
	size_t Size() const { return (RawSize() + 3UL) & ~3UL; }

	uint32_t iboOffset;
	uint32_t iboSize;
	uint8_t nameLength;
	char name[55];
	std::vector<RotTrans> bindPose;
	std::vector<Animation> animationList;
  };

  std::vector<Mesh> meshList;
  std::vector<Vertex> vbo;
  std::vector<uint16_t> ibo;

  /** get the weight of the bones at the vertex index.
  */
  std::vector<BoneWeight> GetBoneWeightList(const FbxMesh* pMesh) {
	std::vector<BoneWeight> result;
	const int count = pMesh->GetDeformerCount(FbxDeformer::eSkin);
	if (count) {
	  const FbxSkin* pSkin = static_cast<const FbxSkin*>(pMesh->GetDeformer(0, FbxDeformer::eSkin));
	  const int boneCount = pSkin->GetClusterCount();
	  std::vector<BoneWeightInfo> weightList;
	  weightList.reserve(static_cast<size_t>(pMesh->GetPolygonCount() * 3));
	  for (int boneIndex = 0; boneIndex < boneCount; ++boneIndex) {
		const FbxCluster* pCluster = pSkin->GetCluster(boneIndex);
		const int pointCount = pCluster->GetControlPointIndicesCount();
		const int* pVertexIndices = pCluster->GetControlPointIndices();
		const double* pWeights = pCluster->GetControlPointWeights();
		for (int j = 0; j < pointCount; ++j) {
		  weightList.push_back(BoneWeightInfo(pVertexIndices[j], boneIndex, static_cast<float>(pWeights[j])));
		}
	  }
	  std::sort(weightList.begin(), weightList.end(), [](const BoneWeightInfo& lhs, const BoneWeightInfo& rhs) { return lhs.vertexIndex < rhs.vertexIndex; });
	  result.resize(weightList.back().vertexIndex + 1);
	  for (const auto& e : weightList) {
		result[e.vertexIndex].Add(e.boneIndex, e.weight);
	  }
	  for (auto& e : result) {
		float totalWeight = 0.0f;
		for (int i = 0; i < 4; ++i) {
		  totalWeight += e.weight[i];
		}
		if (totalWeight > FLT_EPSILON) {
		  for (int i = 0; i < 4; ++i) {
			e.weight[i] /= totalWeight;
			e.weight[i] *= 255.0f;
		  }
		  int a = std::accumulate(e.weight, e.weight + 4, 0, [](int sum, float e) { return sum + static_cast<int>(e); });
		  for (; a < 255; ++a) {
			float dummy;
			const int i0 = std::modf(e.weight[0], &dummy) >= std::modf(e.weight[1], &dummy) ? 0 : 1;
			const int i1 = std::modf(e.weight[2], &dummy) >= std::modf(e.weight[3], &dummy) ? 2 : 3;
			const int i2 = std::modf(e.weight[i0], &dummy) >= std::modf(e.weight[i1], &dummy) ? i0 : i1;
			e.weight[i2] = std::floor(e.weight[i2]) + 1.0f;
		  }
		}
	  }
	}
	return result;
  }

  std::vector<RotTrans> GetBindPose(const FbxMesh* pMesh) {
	std::vector<RotTrans> result;
	const int count = pMesh->GetDeformerCount(FbxDeformer::eSkin);
	if (count) {
	  const FbxNode* pNode = pMesh->GetNode();
	  const FbxVector4 vT = pNode->GetGeometricTranslation(FbxNode::eSourcePivot);
	  const FbxVector4 vR = pNode->GetGeometricRotation(FbxNode::eSourcePivot);
	  const FbxVector4 vS = pNode->GetGeometricScaling(FbxNode::eSourcePivot);
	  const FbxAMatrix mtxNode(vT, vR, vS);

	  const FbxSkin* pSkin = static_cast<const FbxSkin*>(pMesh->GetDeformer(0, FbxDeformer::eSkin));
	  const int boneCount = pSkin->GetClusterCount();
	  result.reserve(boneCount);
	  for (int boneIndex = 0; boneIndex < boneCount; ++boneIndex) {
		const FbxCluster* pCluster = pSkin->GetCluster(boneIndex);
		FbxAMatrix mtxA, mtxB;
		pCluster->GetTransformMatrix(mtxA);
		pCluster->GetTransformLinkMatrix(mtxB);
		const FbxAMatrix m = mtxB.Inverse() * mtxA * mtxNode;
		const RotTrans rt = { m.GetQ(), m.GetT() };
		result.push_back(rt);
	  }
	}
	return result;
  }

  /** the output file format.

  the endianness is little endian.

  char[3]           "MSH"
  uint8_t           mesh count.
  uint32_t          vbo offset by the top of file(32bit alignment).
  uint32_t          vbo byte size(32bit alignment).
  uint32_t          ibo byte size(32bit alignment).
  [
    uint32_t        ibo offset.
    uint32_t        ibo size.
    uint8_t         mesh name length.
    char[length]    mesh name.
    padding         (4 - (length + 1) % 4) % 4 byte.
  ] x (mesh count)
  vbo               vbo data.
  ibo               ibo data.

  uint16_t          bone count.
  uint16_t          animation count.
  [
    RotTrans        rotation and translation for the bind pose.
  ] x (bone count)
  [
    uint8_t         animation name length.
    char[24]        animation name.
    bool            loop flag
    uint16_t        key frame count.
    float           total frames.
    [
      float         frame.
      [
        RotTrans    rotation and translation.
      ] x (bone count)
	] x (key frame count)
  ] x (animation count)
  */
  void Output(const char* filename) {
	std::ofstream ofs(filename, std::ios_base::binary);
	ofs << 'M' << 'S' << 'H';
	ofs << static_cast<uint8_t>(meshList.size());
	const size_t n = std::accumulate(meshList.begin(), meshList.end(), 0UL, [](size_t acc, const Mesh& m) { return acc + m.Size(); });
	Output(ofs, static_cast<uint32_t>(n + 16));
	Output(ofs, static_cast<uint32_t>(vbo.size() * sizeof(Vertex)));
	Output(ofs, static_cast<uint32_t>(ibo.size() * sizeof(uint16_t)));
	for (const Mesh& m : meshList) {
	  Output(ofs, m.iboOffset);
	  Output(ofs, m.iboSize);
	  Output(ofs, m.nameLength);
	  for (uint8_t i = 0; i < m.nameLength; ++i) {
		ofs << m.name[i];
	  }
	  for (size_t i = m.RawSize(); i < m.Size(); ++i) {
		ofs << '\0';
	  }
	}
	for (auto& v : vbo) { ofs << v; }
	for (auto& i : ibo) { Output(ofs, i); }
	ofs.close();
  }

  std::string GetOutputFilename(const char* inputfilename) {
	std::string s(inputfilename);
	return s.substr(0, s.find_last_of('.')) + ".msh";
  }
} // unnamed namespace

static bool gVerbose = true;

//set pCompute true to compute smoothing from normals by default 
//set pConvertToSmoothingGroup true to convert hard/soft edge info to smoothing group info by default
void GetSmoothing(FbxManager* pSdkManager, FbxNode* pNode, bool pCompute = false, bool pConvertToSmoothingGroup = false);

//get mesh normals info
void Convert(FbxNode* pNode);

int main(int argc, char** argv)
{
    FbxManager* lSdkManager = NULL;
    FbxScene* lScene = NULL;
    bool lResult;

    // Prepare the FBX SDK.
    InitializeSdkObjects(lSdkManager, lScene);

    // Load the scene.
    // The example can take a FBX file as an argument.
	FbxString lFilePath("");
	for( int i = 1, c = argc; i < c; ++i )
	{
		if( FbxString(argv[i]) == "-test" ) gVerbose = false;
		else if( lFilePath.IsEmpty() ) lFilePath = argv[i];
	}
	if( lFilePath.IsEmpty() ) lFilePath = SAMPLE_FILENAME;

	FBXSDK_printf("\n\nFile: %s\n\n", lFilePath.Buffer());
	lResult = LoadScene(lSdkManager, lScene, lFilePath.Buffer());

    if(lResult == false)
    {
        FBXSDK_printf("\n\nAn error occurred while loading the scene...");
    }
    else 
    {
        if(!lScene)
        {
            FBX_ASSERT_NOW("null scene");
        }

        //get root node of the fbx scene
        FbxNode* lRootNode = lScene->GetRootNode();

        //get normals info, if there're mesh in the scene
        Convert(lRootNode);

        //set me true to compute smoothing info from normals
        bool lComputeFromNormals = false;
        //set me true to convert hard/soft edges info to smoothing groups info
        bool lConvertToSmoothingGroup = false;
        //get smoothing info, if there're mesh in the scene
        GetSmoothing(lSdkManager, lRootNode, lComputeFromNormals, lConvertToSmoothingGroup);

		if (!meshList.empty()) {
		  const std::string s = GetOutputFilename(lFilePath.Buffer());
		  Output(s.c_str());
		}
    }

    //Destroy all objects created by the FBX SDK.
    DestroySdkObjects(lSdkManager, lResult);
    return 0;
}

//get mesh smoothing info
//set pCompute true to compute smoothing from normals by default 
//set pConvertToSmoothingGroup true to convert hard/soft edge info to smoothing group info by default
void GetSmoothing(FbxManager* pSdkManager, FbxNode* pNode, bool pCompute, bool pConvertToSmoothingGroup)
{
    if(!pNode || !pSdkManager)
        return;

    //get mesh
    FbxMesh* lMesh = pNode->GetMesh();
    if(lMesh)
    {
        //print mesh node name
        FBXSDK_printf("current mesh node: %s\n", pNode->GetName());

        //if there's no smoothing info in fbx file, but you still want to get smoothing info.
        //please compute smoothing info from normals.
        //Another case to recompute smoothing info from normals is:
        //If users edit normals manually in 3ds Max or Maya and export the scene to FBX with smoothing info,
        //The smoothing info may NOT match with normals.
        //the mesh called "fbx_customNormals" in Normals.fbx is the case. All edges are hard, but normals actually represent the "soft" looking.
        //Generally, the normals in fbx file holds the smoothing result you'd like to get.
        //If you want to get correct smoothing info(smoothing group or hard/soft edges) which match with normals,
        //please drop the original smoothing info of fbx file, and recompute smoothing info from normals.
        //if you want to get soft/hard edge info, please call FbxGeometryConverter::ComputeEdgeSmoothingFromNormals().
        //if you want to get smoothing group info, please get soft/hard edge info first by ComputeEdgeSmoothingFromNormals() 
        //And then call FbxGeometryConverter::ComputePolygonSmoothingFromEdgeSmoothing().
        if(pCompute)
        {
            FbxGeometryConverter lGeometryConverter(pSdkManager);
            lGeometryConverter.ComputeEdgeSmoothingFromNormals(lMesh);
            //convert soft/hard edge info to smoothing group info
            if(pConvertToSmoothingGroup)
                lGeometryConverter.ComputePolygonSmoothingFromEdgeSmoothing(lMesh);
        }

        //if there is smoothing groups info in your fbx file, but you want to get hard/soft edges info
        //please use following code:
        //FbxGeometryConverter lGeometryConverter(lSdkManager);
        //lGeometryConverter.ComputeEdgeSmoothingFromPolygonSmoothing(lMesh);

        //get smoothing info
        FbxGeometryElementSmoothing* lSmoothingElement = lMesh->GetElementSmoothing();
        if(lSmoothingElement)
        {
            //mapping mode is by edge. The mesh usually come from Maya, because Maya can set hard/soft edges.
            //we can get smoothing info(which edges are soft, which edges are hard) by retrieving each edge. 
            if( lSmoothingElement->GetMappingMode() == FbxGeometryElement::eByEdge )
            {
                //Let's get smoothing of each edge, since the mapping mode of smoothing element is by edge
                for(int lEdgeIndex = 0; lEdgeIndex < lMesh->GetMeshEdgeCount(); lEdgeIndex++)
                {
                    int lSmoothingIndex = 0;
                    //reference mode is direct, the smoothing index is same as edge index.
                    //get smoothing by the index of edge
                    if( lSmoothingElement->GetReferenceMode() == FbxGeometryElement::eDirect )
                        lSmoothingIndex = lEdgeIndex;

                    //reference mode is index-to-direct, get smoothing by the index-to-direct
                    if(lSmoothingElement->GetReferenceMode() == FbxGeometryElement::eIndexToDirect)
                        lSmoothingIndex = lSmoothingElement->GetIndexArray().GetAt(lEdgeIndex);

                    //Got smoothing of each vertex.
                    int lSmoothingFlag = lSmoothingElement->GetDirectArray().GetAt(lSmoothingIndex);
                    if( gVerbose ) FBXSDK_printf("hard/soft value for edge[%d]: %d \n", lEdgeIndex, lSmoothingFlag);
                    //add your custom code here, to output smoothing or get them into a list, such as KArrayTemplate<int>
                    //. . .
                }//end for lEdgeIndex
            }//end eByEdge
            //mapping mode is by polygon. The mesh usually come from 3ds Max, because 3ds Max can set smoothing groups for polygon.
            //we can get smoothing info(smoothing group ID for each polygon) by retrieving each polygon. 
            else if(lSmoothingElement->GetMappingMode() == FbxGeometryElement::eByPolygon)
            {
                //Let's get smoothing of each polygon, since the mapping mode of smoothing element is by polygon.
                for(int lPolygonIndex = 0; lPolygonIndex < lMesh->GetPolygonCount(); lPolygonIndex++)
                {
                    int lSmoothingIndex = 0;
                    //reference mode is direct, the smoothing index is same as polygon index.
                    if( lSmoothingElement->GetReferenceMode() == FbxGeometryElement::eDirect )
                        lSmoothingIndex = lPolygonIndex;

                    //reference mode is index-to-direct, get smoothing by the index-to-direct
                    if(lSmoothingElement->GetReferenceMode() == FbxGeometryElement::eIndexToDirect)
                        lSmoothingIndex = lSmoothingElement->GetIndexArray().GetAt(lPolygonIndex);

                    //Got smoothing of each polygon.
                    int lSmoothingFlag = lSmoothingElement->GetDirectArray().GetAt(lSmoothingIndex);
                    if( gVerbose ) FBXSDK_printf("smoothing group ID for polygon[%d]: %d \n", lPolygonIndex, lSmoothingFlag);
                    //add your custom code here, to output normals or get them into a list, such as KArrayTemplate<int>
                    //. . .

                }//end for lPolygonIndex //PolygonCount

            }//end eByPolygonVertex
        }//end if lSmoothingElement
    }//end if lMesh

    //recursively traverse each node in the scene
    int i, lCount = pNode->GetChildCount();
    for (i = 0; i < lCount; i++)
    {
        GetSmoothing(pSdkManager, pNode->GetChild(i), pCompute, pConvertToSmoothingGroup);
    }
}

//get mesh normals info
void Convert(FbxNode* pNode)
{
    if(!pNode)
        return;

    //get mesh
    const FbxMesh* lMesh = pNode->GetMesh();
	if (lMesh) {
	  if (const auto pElement = lMesh->GetElementTangent()) {
		//print mesh node name
		FBXSDK_printf("current mesh node: %s\n", pNode->GetName());

		const std::vector<BoneWeight> boneWeightList = GetBoneWeightList(lMesh);

		Mesh mesh = { 0 };
		mesh.iboOffset = ibo.size() * sizeof(uint16_t);
		mesh.nameLength = static_cast<uint8_t>(std::strlen(pNode->GetName()));
		if (mesh.nameLength > 23) {
		  mesh.nameLength = 23;
		}
		std::copy(pNode->GetName(), pNode->GetName() + mesh.nameLength, mesh.name);

		FBXSDK_printf("tangent:\n");
		FbxStringList UVSetNameList;
		lMesh->GetUVSetNames(UVSetNameList);
		const TangentElement tangentList(lMesh, pElement);
//		const BinormalElement binormalList(lMesh, lMesh->GetElementBinormal());
		const int count = lMesh->GetPolygonCount();
		vbo.reserve(vbo.size() + count * 3);
		ibo.reserve(ibo.size() + count * 3);
		for (int i = 0; i < count; ++i) {
		  for (int pos = 0; pos < 3; ++pos) {
			const int index = lMesh->GetPolygonVertex(i, pos);
			Vertex v;
			v.position = lMesh->GetControlPointAt(index);
			v.position = Vector3(v.position.x, v.position.z, -v.position.y);
			FbxVector4 vNormal;
			lMesh->GetPolygonVertexNormal(i, pos, vNormal);
			v.normal = vNormal;
			v.normal = Vector3(v.normal.x, v.normal.z, -v.normal.y);
			FbxVector2 vTexCoord;
			bool unmapped;
			lMesh->GetPolygonVertexUV(i, pos, UVSetNameList[0], vTexCoord, unmapped);
			v.texCoord[0] = vTexCoord;
			v.texCoord[1] = v.texCoord[0];
			v.tangent = tangentList.Get(i, pos);
#if 1
			v.tangent = Vector4(v.tangent.x, v.tangent.z, -v.tangent.y, 1.0f);
			if (v.normal.y > 0.9f) {
			  v.tangent.w = -1.0f;
			}
//			Vector3 b = binormalList.Get(i, pos);
//			b.y *= -1.0f;
//			const Vector3 b2 = Normalize(Cross(v.normal, Vector3(v.tangent.x, v.tangent.y, v.tangent.z)));
//			if (Dot(b, b2) < 0.0f) {
//			  v.tangent.w = -1.0f;
//			}
#else
			v.tangent = Vector4(-v.tangent.x, v.tangent.z, -v.tangent.y, v.tangent.w);
			const Vector3 t(v.tangent.x, v.tangent.y, v.tangent.z);
			Vector3 b = binormalList.Get(i, pos);
			b = Vector3(-b.x, b.z, -b.y);
			v.tangent.w = Dot(Cross(t, b), b) < 0.0f ? -1.0f : 1.0f;
#endif
			if (index < boneWeightList.size()) {
			  const BoneWeight& weightData = boneWeightList[index];
			  for (int i = 0; i < 4; ++i) {
				v.boneID[i] = static_cast<uint8_t>(weightData.boneIndex[i] != -1 ? weightData.boneIndex[i] : 0);
				v.weight[i] = static_cast<uint8_t>(weightData.weight[i]);
			  }
			} else {
			  v.boneID[0] = v.boneID[1] = v.boneID[2] = v.boneID[3] = 0;
			  v.weight[0] = 255; v.weight[1] = v.weight[2] = v.weight[3] = 0;
			}

			auto itr = std::find(vbo.begin(), vbo.end(), v);
			if (itr == vbo.end()) {
			  ibo.push_back(static_cast<uint16_t>(vbo.size()));
			  vbo.push_back(v);
			} else {
			  ibo.push_back(itr - vbo.begin());
			}
		  }
		}
		mesh.iboSize = ibo.size() - mesh.iboOffset / sizeof(uint16_t);

		mesh.bindPose = GetBindPose(lMesh);

		meshList.push_back(mesh);
	  }
	}

    //recursively traverse each node in the scene
    int i, lCount = pNode->GetChildCount();
    for (i = 0; i < lCount; i++)
    {
        Convert(pNode->GetChild(i));
    }
}
