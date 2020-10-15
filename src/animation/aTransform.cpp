#include "aTransform.h"
#include <Eigen/Dense>
#pragma warning(disable : 4244)

Eigen::Matrix4d getHomoMatrix(const ATransform& a) {
	Eigen::Matrix4d homo = Eigen::Matrix4d::Identity();
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			homo(i, j) = a.m_rotation[i][j];
		}
	}
	for (int i = 0; i < 3; i++) {
		homo(i, 3) = a.m_translation[i];
	}
	return homo;

}

ATransform::ATransform() : m_rotation(IdentityMat3), m_translation(vec3Zero)
{
}

ATransform::ATransform(const mat3& rot, const vec3& offset) : m_rotation(rot), m_translation(offset)
{
}

ATransform::ATransform(const ATransform& m)
{
    *this = m;
}

// Assignment operators
ATransform& ATransform::operator = (const ATransform& orig)
{
    if (&orig == this)
    {
        return *this;
    }
    m_rotation = orig.m_rotation;
    m_translation = orig.m_translation;
    return *this;
}

ATransform ATransform::Inverse() const
{
	// TODO: compute the inverse of a transform given the current rotation and translation components
	ATransform inv;
	inv.m_rotation = m_rotation.Transpose();
	inv.m_translation = -1 * inv.m_rotation * m_translation;

	return inv;
}


vec3 ATransform::RotTrans(const vec3& vecToTransform) const
{
	// TODO: Transform the input vector based on this transform's rotation and translation components
	return m_rotation * vecToTransform + m_translation;
}

vec3 ATransform::Rotate(const vec3& vecToTransform) const
{
	// TODO: Transform the input direction based on this transform's rotation component
	return m_rotation * vecToTransform;
}

vec3 ATransform::Translate(const vec3& vecToTransform) const
{
	// TODO: Transform the input vector based on this transform's translation component	
	return m_translation + vecToTransform;
}

ATransform operator * (const ATransform& H1, const ATransform& H2)
{
	// TODO: implement the equivalent of multiplying  H1 and H2 transformation matrices and return the result
	ATransform result;
	Eigen::Matrix4d homo1 = getHomoMatrix(H1);
	Eigen::Matrix4d homo2 = getHomoMatrix(H2);
	Eigen::Matrix4d homo = homo1 * homo2;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			result.m_rotation[i][j] = homo(i, j);
		}
	}
	for (int i = 0; i < 3; i++) {
		result.m_translation[i] = homo(i, 3);
	}
	return result;
}

vec3 operator * (const ATransform& A, const vec3& v)
{
	return A.RotTrans(v);
}

void ATransform::WriteToGLMatrix(float* m)
{
	m[0] = m_rotation[0][0]; m[4] = m_rotation[0][1]; m[8] = m_rotation[0][2];  m[12] = m_translation[0];
	m[1] = m_rotation[1][0]; m[5] = m_rotation[1][1]; m[9] = m_rotation[1][2];  m[13] = m_translation[1];
	m[2] = m_rotation[2][0]; m[6] = m_rotation[2][1]; m[10] = m_rotation[2][2]; m[14] = m_translation[2];
	m[3] = 0.0f;    m[7] = 0.0f;    m[11] = 0.0f;    m[15] = 1.0f;
}

void ATransform::ReadFromGLMatrix(float* m)
{
	m_rotation[0][0] = m[0]; m_rotation[0][1] = m[4]; m_rotation[0][2] = m[8];  m_translation[0] = m[12];
	m_rotation[1][0] = m[1]; m_rotation[1][1] = m[5]; m_rotation[1][2] = m[9];  m_translation[1] = m[13];
	m_rotation[2][0] = m[2]; m_rotation[2][1] = m[6]; m_rotation[2][2] = m[10]; m_translation[2] = m[14];
}

std::ostream& operator << (std::ostream& s, const ATransform& t)
{
    vec3 anglesRad;
    t.m_rotation.ToEulerAngles(mat3::ZXY,anglesRad);
    s << "R: " << anglesRad << " T: " << t.m_translation << " ";
    return s;
}





