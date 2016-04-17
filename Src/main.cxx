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

//#define DEBUG_SHOW_ROTTRANS_INFO

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
	Vector3& operator/=(float f) { x /= f; y /= f; z /= f; return *this; }
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
	Quaternion(const Vector3& axis, float angle) {
	  const float s = std::sin(angle * 0.5f);
	  x = axis.x * s;
	  y = axis.y * s;
	  z = axis.z * s;
	  w = std::cos(angle * 0.5f);
	}
	Quaternion& operator==(const FbxQuaternion& v) { *this = Quaternion(v); return *this; }
	Quaternion& operator*=(float f) { x *= f; y *= f; z *= f; w *= f; return *this; }
  };

  struct RotTrans {
	Quaternion rot;
	Vector3 trans;
  };

  struct Bone {
	RotTrans rt;
	int32_t parentIndex; ///< if this value has less than 0, it is the root bone.
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
	float    totalTime;
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
  float Length(const Vector3& v) {
	return std::sqrt(Dot(v, v));
  }
  float Length(const Quaternion& q) {
	return std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  }
  Vector3 Normalize(const Vector3& v) {
	const float invNorm = 1.0f / Length(v);
	return Vector3(v.x * invNorm, v.y * invNorm, v.z * invNorm);
  }
  Quaternion Normalize(const Quaternion& q) {
	const float invNorm = 1.0f / Length(q);
	return Quaternion(q.x * invNorm, q.y * invNorm, q.z * invNorm, q.w * invNorm);
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

  FbxAMatrix ToMatrix(const Quaternion& q) {
	const float xx = 2.0f * q.x * q.x;
	const float xy = 2.0f * q.x * q.y;
	const float xz = 2.0f * q.x * q.z;
	const float xw = 2.0f * q.x * q.w;
	const float yy = 2.0f * q.y * q.y;
	const float yz = 2.0f * q.y * q.z;
	const float yw = 2.0f * q.y * q.w;
	const float zz = 2.0f * q.z * q.z;
	const float zw = 2.0f * q.z * q.w;

	FbxAMatrix m;

	m[0][0] = 1.0f - yy - zz;
	m[0][1] = xy + zw;
	m[0][2] = xz - yw;
	m[0][3] = 0.0f;

	m[1][0] = xy - zw;
	m[1][1] = 1.0f - xx - zz;
	m[1][2] = yz + xw;
	m[1][3] = 0.0f;

	m[2][0] = xz + yw;
	m[2][1] = yz - xw;
	m[2][2] = 1.0f - xx - yy;
	m[2][3] = 0.0f;

	m[3][0] = 0.0f;
	m[3][1] = 0.0f;
	m[3][2] = 0.0f;
	m[3][3] = 1.0f;

	return m;
  }

  FbxAMatrix ToMatrix(const RotTrans& rt) {
	FbxAMatrix m = ToMatrix(rt.rot);
	m[3][0] = rt.trans.x;
	m[3][1] = rt.trans.y;
	m[3][2] = rt.trans.z;
	return m;
  }

  std::pair<Vector3, float> GetAxisAngle(const Quaternion& q) {
	const float angle = 2.0f * std::acos(q.w);
	const float n = std::sqrt(1.0f - q.w * q.w);
	const float invN = std::abs(n) > FLT_EPSILON ? 1.0f / n : 0.0f;
	const Vector3 axis(q.x * invN, q.y * invN, q.z * invN);
	return std::make_pair(axis, angle);
  }
  FbxAMatrix FbxMatrixToFbxAMatrix(const FbxMatrix& m) {
	FbxAMatrix result;
	static_cast<FbxDouble4x4&>(result) = m;
    return result;
  }
#if 0
  void Decompose(const FbxAMatrix& m, Quaternion* q, Vector3* scale, Vector3* trans) {
	if (trans) {
	  trans->x = m.Get(3, 0);
	  trans->y = m.Get(3, 1);
	  trans->z = m.Get(3, 2);
	}
	Vector3 v0(m.Get(0, 0), m.Get(0, 1), m.Get(0, 2));
	Vector3 v1(m.Get(1, 0), m.Get(1, 1), m.Get(1, 2));
	Vector3 v2(m.Get(2, 0), m.Get(2, 1), m.Get(2, 2));
	const Vector3 s(Length(v0), Length(v1), Length(v2));
	if (scale) {
	  *scale = s;
	}
	if (q) {
	  // ここ(http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/)のAlternative Methodを使う.
	  v0 /= s.x;
	  v1 /= s.y;
	  v2 /= s.z;
	  *q = Quaternion(
		std::sqrt(std::max(0.0f, 1.0f + v0.x - v1.y - v2.z)),
		std::sqrt(std::max(0.0f, 1.0f - v0.x + v1.y - v2.z)),
		std::sqrt(std::max(0.0f, 1.0f - v0.x - v1.y + v2.z)),
		std::sqrt(std::max(0.0f, 1.0f + v0.x + v1.y + v2.z)));
	  *q *= 0.5f;
	  q->x = std::copysignf(q->x, v2.y - v1.z);
	  q->y = std::copysignf(q->y, v0.z - v2.x);
	  q->z = std::copysignf(q->z, v1.x - v0.y);
	}
  }
#endif
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
		  return;
		}
	  }
	  FBXSDK_printf("weight list over.\n");
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
  };

  struct AnimationKeyframes {
	std::string name;
	std::vector<FbxTime> time;
  };

  std::vector<Mesh> meshList;
  std::vector<Vertex> vbo;
  std::vector<uint16_t> ibo;
  std::vector<Bone> bindPose;
  std::vector<Animation> animationList;

  /** get the first skin in the node tree.
  */
  const FbxSkin* GetSkinForAnimation(const FbxNode* pNode) {
	if (const FbxMesh* pMesh = const_cast<FbxNode*>(pNode)->GetMesh()) {
	  if (const int count = pMesh->GetDeformerCount(FbxDeformer::eSkin)) {
		if (const FbxSkin* pSkin = static_cast<const FbxSkin*>(pMesh->GetDeformer(0, FbxDeformer::eSkin))) {
		  if (const int boneCount = pSkin->GetClusterCount()) {
			return pSkin;
		  }
		}
	  }
	}
	const int childCount = pNode->GetChildCount();
	for (int i = 0; i < childCount; ++i) {
	  if (const FbxSkin* pSkin = GetSkinForAnimation(pNode->GetChild(i))) {
		return pSkin;
	  }
	}
	return nullptr;
  }
  
  /** get the first skeleton node in the node tree.
  */
  const FbxNode* GetSkeletonNodeForAnimation(const FbxNode* pNode) {
	const FbxNodeAttribute* pAttr = const_cast<FbxNode*>(pNode)->GetNodeAttribute();
	if (pAttr && pAttr->GetAttributeType() == FbxNodeAttribute::eSkeleton) {
	  return pNode;
	}
	const int childCount = pNode->GetChildCount();
	for (int i = 0; i < childCount; ++i) {
	  if (const FbxNode* p = GetSkeletonNodeForAnimation(pNode->GetChild(i))) {
		return p;
	  }
	}
	return nullptr;
  }

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
	  int maxVertexIndex = 0;
	  for (int boneIndex = 0; boneIndex < boneCount; ++boneIndex) {
		const FbxCluster* pCluster = pSkin->GetCluster(boneIndex);
		const int pointCount = pCluster->GetControlPointIndicesCount();
		const int* pVertexIndices = pCluster->GetControlPointIndices();
		const double* pWeights = pCluster->GetControlPointWeights();
		for (int j = 0; j < pointCount; ++j) {
		  weightList.push_back(BoneWeightInfo(pVertexIndices[j], boneIndex, static_cast<float>(pWeights[j])));
		  maxVertexIndex = std::max(maxVertexIndex, pVertexIndices[j]);
		}
	  }
	  std::sort(weightList.begin(), weightList.end(), [](const BoneWeightInfo& lhs, const BoneWeightInfo& rhs) { return lhs.weight > rhs.weight; });
	  result.resize(maxVertexIndex + 1);
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
		  // check weight acuracy.
		  {
			const int w0 = static_cast<int>(e.weight[0]);
			const int w1 = static_cast<int>(e.weight[1]);
			const int w2 = static_cast<int>(e.weight[2]);
			const int w3 = static_cast<int>(e.weight[3]);
			if (w0 + w1 + w2 + w3 != 255) {
			  FBXSDK_printf("invalid weight found: %d, %d, %d, %d\n", w0, w1, w2, w3);
			}
		  }
		}
	  }
	}
	return result;
  }

  FbxAMatrix GetGeometry(const FbxNode* pNode) {
	const FbxVector4 vT = pNode->GetGeometricTranslation(FbxNode::eSourcePivot);
	const FbxVector4 vR = pNode->GetGeometricRotation(FbxNode::eSourcePivot);
	const FbxVector4 vS = pNode->GetGeometricScaling(FbxNode::eSourcePivot);
	return FbxAMatrix(vT, vR, vS);
  }

  FbxAMatrix GetPoseMatrix(FbxPose* pPose, const FbxNode* pNode) {
    const int nodeIndex = pPose->Find(pNode);
    return FbxMatrixToFbxAMatrix(pPose->GetMatrix(nodeIndex));
  }

  FbxAMatrix GetAxisConversionMatrix() {
	FbxAMatrix m;
#if 1
	m.SetIdentity();
#else
	m.SetRow(0, FbxVector4(1, 0, 0, 0));
	m.SetRow(1, FbxVector4(0, 0, 1, 0));
	m.SetRow(2, FbxVector4(0, -1, 0, 0));
	m.SetRow(3, FbxVector4(0, 0, 0, 1));
#endif
	return m;
  }

  std::vector<Bone> GetBindPose(const FbxMesh* pMesh, FbxPose* pPose) {
	std::vector<Bone> result;
	const FbxAMatrix mtxInvGlobalPose = GetPoseMatrix(pPose, pMesh->GetNode()).Inverse();
	const int count = pMesh->GetDeformerCount(FbxDeformer::eSkin);
	if (count) {
	  const FbxAMatrix mtxAxisConversion = GetAxisConversionMatrix();
	  const FbxAMatrix mtxNode = GetGeometry(pMesh->GetNode()).Inverse();
	  const FbxSkin* pSkin = static_cast<const FbxSkin*>(pMesh->GetDeformer(0, FbxDeformer::eSkin));
	  const int boneCount = pSkin->GetClusterCount();
	  result.reserve(boneCount);
	  for (int boneIndex = 0; boneIndex < boneCount; ++boneIndex) {
		const FbxCluster* pCluster = pSkin->GetCluster(boneIndex);
		int32_t parentIndex = -1;
		const FbxNode* pNode = pCluster->GetLink();
		if (const FbxNode* pParent = pNode->GetParent()) {
		  if (const FbxNodeAttribute* pAttr = pParent->GetNodeAttribute()) {
			if (pAttr->GetAttributeType() == FbxNodeAttribute::eSkeleton) {
			  for (parentIndex = boneCount - 1; parentIndex >= 0; --parentIndex) {
				if (pParent == pSkin->GetCluster(parentIndex)->GetLink()) {
				  break;
				}
			  }
			}
		  }
		}
		const FbxAMatrix mtxPose = GetPoseMatrix(pPose, pCluster->GetLink());
		const FbxAMatrix mtxInvClusterRelativeCurrent = mtxInvGlobalPose * mtxPose;
		FbxAMatrix mtxTransform, mtxTransformLink;
		pCluster->GetTransformMatrix(mtxTransform);
		pCluster->GetTransformLinkMatrix(mtxTransformLink);
		const FbxAMatrix mtxClusterRelativeInit = mtxTransformLink.Inverse() * mtxTransform * mtxNode;
		const FbxAMatrix m = (mtxAxisConversion * mtxClusterRelativeInit).Inverse();
		const RotTrans rt{ m.GetQ(), m.GetT() };
		result.push_back({ rt, parentIndex });
	  }
	}
	return result;
  }

  void GetCurveKeyframes(const FbxAnimCurve* pCurve, std::vector<FbxTime>& list) {
	if (pCurve) {
	  const int keyCount = pCurve->KeyGetCount();
	  for (int i = 0; i < keyCount; ++i) {
		list.push_back(pCurve->KeyGetTime(i));
	  }
	}
  }

  void GetTranslationKeyframes(
	const FbxNode& node,
	const FbxAnimLayer* pLayer,
	const char* pChannel,
	std::vector<FbxTime>& list
  ) {
	const FbxAnimCurve* pCurve = const_cast<FbxNode&>(node).LclTranslation.GetCurve(const_cast<FbxAnimLayer*>(pLayer), pChannel);
	GetCurveKeyframes(pCurve, list);
	const int childCount = node.GetChildCount();
	for (int i = 0; i < childCount; ++i) {
	  GetTranslationKeyframes(*node.GetChild(i), pLayer, pChannel, list);
	}
  }

  void GetRotationKeyframes(
	const FbxNode& node,
	const FbxAnimLayer* pLayer,
	const char* pChannel,
	std::vector<FbxTime>& list
  ) {
	const FbxAnimCurve* pCurve = const_cast<FbxNode&>(node).LclRotation.GetCurve(const_cast<FbxAnimLayer*>(pLayer), pChannel);
	GetCurveKeyframes(pCurve, list);
	const int childCount = node.GetChildCount();
	for (int i = 0; i < childCount; ++i) {
	  GetRotationKeyframes(*node.GetChild(i), pLayer, pChannel, list);
	}
  }

  /** Get All animation key frames.

    Get all key frames related skeleton node from animation stack in FBX scene.

	@param skeleton  The node associated with the key frames.
	@param scene     FBX scene object that contain skeleton and key frames.

	@return A vector of key frame related to the skeleton in FBX scene.
  */
  std::vector<AnimationKeyframes> GetKeyframes(const FbxNode& skeleton, const FbxScene& scene) {
	std::vector<AnimationKeyframes> result;
	const FbxNodeAttribute* pAttr = const_cast<FbxNode&>(skeleton).GetNodeAttribute();
	if (!pAttr || pAttr->GetAttributeType() != FbxNodeAttribute::eSkeleton) {
	  return result;
	}
	const int numStacks = scene.GetSrcObjectCount<FbxAnimStack>();
	result.reserve(numStacks);
	for (int i = 0; i < numStacks; ++i) {
	  std::vector<FbxTime> timeList;
	  const FbxAnimStack* pStack = scene.GetSrcObject<FbxAnimStack>(i);
	  const int numLayers = pStack->GetMemberCount<FbxAnimLayer>();
	  for (int j = 0; j < numLayers; ++j) {
		const FbxAnimLayer* pLayer = pStack->GetMember<FbxAnimLayer>(j);
		GetTranslationKeyframes(skeleton, pLayer, FBXSDK_CURVENODE_COMPONENT_X, timeList);
		GetTranslationKeyframes(skeleton, pLayer, FBXSDK_CURVENODE_COMPONENT_Y, timeList);
		GetTranslationKeyframes(skeleton, pLayer, FBXSDK_CURVENODE_COMPONENT_Z, timeList);
		GetRotationKeyframes(skeleton, pLayer, FBXSDK_CURVENODE_COMPONENT_X, timeList);
		GetRotationKeyframes(skeleton, pLayer, FBXSDK_CURVENODE_COMPONENT_Y, timeList);
		GetRotationKeyframes(skeleton, pLayer, FBXSDK_CURVENODE_COMPONENT_Z, timeList);
	  }
	  std::sort(timeList.begin(), timeList.end());
	  timeList.erase(std::unique(timeList.begin(), timeList.end()), timeList.end());
	  result.push_back({ pStack->GetName(), timeList });
	}
	return result;
  }

  /** Get and convert animation data from FBX.

	Get all skin animation data from FbxScene, and Convert to the own data format.
	Finally, store to the global variable of 'animationList'.

	'animationList' output to the any file by Output() function.

	@param scene FBX Scene object that contains skin animation data.

	Puseud code of the matrix creation:
	TransformationMatrixOfCluster
	  = inverse(nodeThatHaveMesh.EvaluateGlobalTransform(time))
	  * cluster.GetLink()->EvaluateGlobalTransform(time)
	  * inverse(cluster.GetTransformLinkMatrix())
	  * cluster.GetTransformMatrix()
	  * GetGeometry(mesh.GetNode())

	@see ComputeClusterDeformation(), DrawMesh() in ViewScene sample code in Autodesk FBX SDK 2016.1.2
  */
  void GetAnimation(FbxScene& scene) {
	const FbxSkin* pSkin = GetSkinForAnimation(scene.GetRootNode());
	const FbxNode* pSkeleton = GetSkeletonNodeForAnimation(scene.GetRootNode());
	if (!pSkin || !pSkeleton) {
	  return;
	}
	const std::vector<AnimationKeyframes> keyframeList = GetKeyframes(*pSkeleton, scene);
	if (keyframeList.empty()) {
	  return;
	}
	struct ClusterInfo {
	  FbxNode* pNode;
	  FbxAMatrix mtxCurrent;
	  FbxAMatrix mtxInvLink;
	};
	const int clusterCount = pSkin->GetClusterCount();
	std::vector<ClusterInfo> clusterNodeList;
	clusterNodeList.reserve(clusterCount);
	for (int i = 0; i < clusterCount; ++i) {
	  FbxCluster* pCluster = const_cast<FbxSkin*>(pSkin)->GetCluster(i);
	  FbxAMatrix mtxCurrent, mtxLink;
	  pCluster->GetTransformLinkMatrix(mtxLink);
	  pCluster->GetTransformMatrix(mtxCurrent);
	  clusterNodeList.push_back({ pCluster->GetLink(), mtxCurrent, mtxLink.Inverse() });
	}

	const FbxAMatrix mtxAxisConversion = GetAxisConversionMatrix();

	FbxArray<FbxString*> animStackNameArray;
	scene.FillAnimStackNameArray(animStackNameArray);
	const int animStackCount = animStackNameArray.GetCount();
	const int meshCount = scene.GetMemberCount<FbxMesh>();
	for (int i = 0; i < animStackCount; ++i) {
	  FbxAnimStack* pCurrentAnimationStack = scene.FindMember<FbxAnimStack>(animStackNameArray[i]->Buffer());
	  scene.SetCurrentAnimationStack(pCurrentAnimationStack);
	  const char* pName = [](const char* p) { for (const char* q = p; *q; ++q) { if (*q == '|') { return ++q; } } return p; }(pCurrentAnimationStack->GetName());
	  const std::vector<FbxTime>& keyframes = keyframeList[i].time;
	  if (keyframes.empty()) {
		continue;
	  }
	  Animation  animation;
	  animation.nameLength = static_cast<uint8_t>(std::strlen(pName));
	  std::copy(pName, pName + animation.nameLength, animation.name);
	  animation.totalTime = static_cast<float>(keyframes.back().GetSecondDouble());
	  animation.loopFlag = true;
	  FBXSDK_printf("%s: %fsec(keys=%d)\n", pCurrentAnimationStack->GetName(), animation.totalTime, keyframes.size());
	  animation.list.reserve(keyframes.size());
	  for (int meshIndex = 0; meshIndex < meshCount; ++meshIndex) {
		FbxMesh* pMesh = scene.GetMember<FbxMesh>(meshIndex);
		if (pMesh->GetDeformerCount(FbxDeformer::eSkin) > 0) {
		  FbxNode* pNode = pMesh->GetNode();
		  FbxAMatrix mtxGlobalGeometry = GetGeometry(pNode);
		  for (auto keyframe : keyframes) {
			Animation::KeyFrame kf;
			kf.time = static_cast<float>(keyframe.GetSecondDouble());
			kf.pose.reserve(clusterCount);
#ifdef DEBUG_SHOW_ROTTRANS_INFO
			FBXSDK_printf("time=%f\n",kf.time);
#endif // DEBUG_SHOW_ROTTRANS_INFO
			FbxAMatrix mtxInvGlobalPosition = (pNode->EvaluateGlobalTransform(keyframe) * mtxGlobalGeometry).Inverse();
			for (auto cluster : clusterNodeList) {
			  FbxAMatrix mtxClusterPosition = cluster.pNode->EvaluateGlobalTransform(keyframe);
			  FbxAMatrix m = (mtxInvGlobalPosition * mtxClusterPosition) * (cluster.mtxInvLink * (cluster.mtxCurrent * mtxGlobalGeometry));
			  //m = mtxAxisConversion * m;
			  RotTrans rt{ m.GetQ(), m.GetT() };
			  rt.rot = Quaternion(rt.rot.x, rt.rot.z, -rt.rot.y, rt.rot.w);
			  rt.trans = Vector3(rt.trans.x, rt.trans.z, -rt.trans.y);
#if 0
			  // verify conversion accuracy.
			  {
				const double threshold = (1.0e-6);
				FbxAMatrix m0 = ToMatrix(rt);
				FbxAMatrix m1 = m * m0.Inverse();
				if (!m1.IsIdentity(threshold)) {
				  FBXSDK_printf("non accurate conversion found.\n");
				}
			  }
#endif
#ifdef DEBUG_SHOW_ROTTRANS_INFO
			  FBXSDK_printf("  %-10s:(%+1.3f, %+1.3f, %+1.3f, %+1.3f) (%+1.3f, %+1.3f, %+1.3f)\n", cluster.pNode->GetName(), rt.rot.w, rt.rot.x, rt.rot.y, rt.rot.z, rt.trans.x, rt.trans.y, rt.trans.z);
#endif // DEBUG_SHOW_ROTTRANS_INFO
			  kf.pose.push_back(rt);
			}
			animation.list.push_back(kf);
		  }
		  animationList.push_back(animation);
		}
	  }
	}
  }

  /** the output file format.

  the endianness is little endian.
  all mesh uses same skeleton in the file.

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

  padding           (4 - (ibo byte size % 4) % 4 byte.

  uint16_t          bone count.
  uint16_t          animation count.

  [
    RotTrans        rotation and translation for the bind pose.
	int32_t         parent bone index.
  ] x (bone count)

  [
    uint8_t         animation name length.
    char[24]        animation name.
    bool            loop flag
    uint16_t        key frame count.
    float           total time.
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

	for (size_t i = (4 - ((ibo.size() * sizeof(uint16_t)) % 4)) % 4; i; --i) {
	  ofs << '\0';
	}

	Output(ofs, static_cast<uint16_t>(bindPose.size()));
	Output(ofs, static_cast<uint16_t>(animationList.size()));

	for (const Bone& e : bindPose) {
	  Output(ofs, e.rt.rot.x);
	  Output(ofs, e.rt.rot.y);
	  Output(ofs, e.rt.rot.z);
	  Output(ofs, e.rt.rot.w);
	  Output(ofs, e.rt.trans.x);
	  Output(ofs, e.rt.trans.y);
	  Output(ofs, e.rt.trans.z);
	  Output(ofs, e.parentIndex);
	}

	for (const Animation& anm : animationList) {
	  Output(ofs, anm.nameLength);
	  for (auto c : anm.name) {
		ofs << c;
	  }
	  Output(ofs, static_cast<uint8_t>(anm.loopFlag));
	  Output(ofs, static_cast<uint16_t>(anm.list.size()));
	  Output(ofs, anm.totalTime);
	  for (const Animation::KeyFrame& key : anm.list) {
		Output(ofs, key.time);
		for (const RotTrans& rt : key.pose) {
		  Output(ofs, rt.rot.x);
		  Output(ofs, rt.rot.y);
		  Output(ofs, rt.rot.z);
		  Output(ofs, rt.rot.w);
		  Output(ofs, rt.trans.x);
		  Output(ofs, rt.trans.y);
		  Output(ofs, rt.trans.z);
		}
	  }
	}

	ofs.close();
  }

  std::string GetOutputFilename(const char* inputfilename) {
	std::string s(inputfilename);
	return s.substr(0, s.find_last_of('.')) + ".msh";
  }
} // unnamed namespace

static bool gVerbose = true;
static bool gApplyGlobalGeometry = true;
static double gScale = 1.0;

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
		if ((FbxString(argv[i]) == "-scale") && (i + 1 <= c)) {
			gScale = std::max(0.0000001, atof(argv[i + 1]));
			++i;
		} else if ((FbxString(argv[i]) == "-local") && (i + 1 <= c)) {
			gApplyGlobalGeometry = false;	
		} else if (FbxString(argv[i]) == "-test") {
			gVerbose = false;
		} else if (lFilePath.IsEmpty()) {
			lFilePath = argv[i];
		}
	}
	if (lFilePath.IsEmpty()) {
		FBXSDK_printf(
			"FBXConv.exe ver.1.0\n"
			"Usage: FBXConv.exe [-local] [-scale s] [infile]\n\n"
			"-local  : Ignore the parent nodes. It should be given for animated data.\n"
			"-scale s: Scale the mesh(default=1.0).\n"
			"infile  : Input FBX filename.\n"
			"          The output filename is the extension is replaced to 'msh'.\n"
		);
		return 0;
	}

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

		{
		  FBXSDK_printf("pose count:%d\n", lScene->GetPoseCount());
		}

		{
		  const char* p;
		  switch (lScene->GetGlobalSettings().GetTimeMode()) {
		  case FbxTime::eDefaultMode: p = "eDefaultMode"; break;
		  case FbxTime::eFrames120: p = "eFrames120"; break;
		  case FbxTime::eFrames100: p = "eFrames100"; break;
		  case FbxTime::eFrames60: p = "eFrames60"; break;
		  case FbxTime::eFrames50: p = "eFrames50"; break;
		  case FbxTime::eFrames48: p = "eFrames48"; break;
		  case FbxTime::eFrames30: p = "eFrames30"; break;
		  case FbxTime::eFrames30Drop: p = "eFrames30Drop"; break;
		  case FbxTime::eNTSCDropFrame: p = "eNTSCDropFrame"; break;
		  case FbxTime::eNTSCFullFrame: p = "eNTSCFullFrame"; break;
		  case FbxTime::ePAL: p = "ePAL"; break;
		  case FbxTime::eFrames24: p = "eFrames24"; break;
		  case FbxTime::eFrames1000: p = "eFrames1000"; break;
		  case FbxTime::eFilmFullFrame: p = "eFilmFullFrame"; break;
		  case FbxTime::eCustom: p = "eCustom"; break;
		  case FbxTime::eFrames96: p = "eFrames96"; break;
		  case FbxTime::eFrames72: p = "eFrames72"; break;
		  case FbxTime::eFrames59dot94: p = "eFrames59dot94"; break;
		  default: p = "unknown"; break;
		  }
		  FBXSDK_printf("time mode:%s\n", p);
		}

		//get root node of the fbx scene
		FbxNode* lRootNode = lScene->GetRootNode();

		//get normals info, if there're mesh in the scene
		Convert(lRootNode);
		GetAnimation(*lScene);

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

		int boneWeightListSize = static_cast<int>(boneWeightList.size());
		if (boneWeightList.size() > INT_MAX) {
		  FBXSDK_printf("WARN: too many boneWeightList(%ld).", boneWeightList.size());
		  boneWeightListSize = 0;
		}

		FbxAMatrix trsMatrix;
		if (gApplyGlobalGeometry) {
		  trsMatrix = pNode->EvaluateGlobalTransform();
		} else {
		  trsMatrix[1][1] = 0;
		  trsMatrix[1][2] = -1;
		  trsMatrix[2][1] = 1;
		  trsMatrix[2][2] = 0;
		}
		FbxAMatrix normalMatrix;
		normalMatrix.SetR(trsMatrix.GetR());
		if (gScale != 1.0) {
		  const FbxAMatrix mtxScale(FbxVector4(), FbxVector4(), FbxVector4(gScale, gScale, gScale));
		  trsMatrix = mtxScale * trsMatrix;
		}
		const FbxVector4* const pControlPoints = lMesh->GetControlPoints();
		for (int i = 0; i < count; ++i) {
		  for (int pos = 0; pos < 3; ++pos) {
			const int index = lMesh->GetPolygonVertex(i, pos);
			Vertex v;
			v.position = trsMatrix.MultT(pControlPoints[index]);
			FbxVector4 vNormal;
			lMesh->GetPolygonVertexNormal(i, pos, vNormal);
			v.normal = normalMatrix.MultT(vNormal);
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
			if (index < boneWeightListSize) {
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

		if (bindPose.empty()) {
		  if (FbxPose* pPose = pNode->GetScene()->GetPose(0)) {
			auto tmp = GetBindPose(lMesh, pPose);
			if (!tmp.empty()) {
			  bindPose = tmp;
			}
		  }
		}

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
