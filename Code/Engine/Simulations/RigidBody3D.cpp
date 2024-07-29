#include "RigidBody3D.hpp"
#include "Engine/Renderer/Renderer.hpp"
#include "Engine/Core/VertexUtils.hpp"
#include "Engine/Core/Vertex_PCU.hpp"
#include "Engine/Core/Vertex_PCUTBN.hpp"
#include "Engine/Core/OBJLoader.hpp"
#include "Engine/Core/XmlUtils.hpp"
#include "Engine/Core/ErrorWarningAssert.hpp"
#include "Engine/Math/MathUtils.hpp"
#include "Engine/Simulations/Collider3D.hpp"

//-----------------------------------------------------------------------------------------------
RigidBody3D::RigidBody3D(std::string xmlFileName, float totalMass)
{
	//Initialize collision mesh
	m_collider = new Collider3D(this);

	//Initialize XML reading
	XmlDocument modelXml;
	modelXml.LoadFile(xmlFileName.c_str());
	XmlElement* rootElement = modelXml.RootElement();
	if (rootElement == nullptr)
	{
		DebuggerPrintf("COULD NOT LOAD .OBJ FILE!\n");
		return;
	}

	//Get File Path
	std::string filePath = ParseXmlAttribute(*rootElement, "path", "Path Does Not Exist!");
	if (filePath == "Path Does Not Exist!")
	{
		DebuggerPrintf("COULD NOT LOAD .OBJ FILE!\n");
		return;
	}

	//Transform
	XmlElement* transformElement = rootElement->FirstChildElement();
	Mat44 transform;
	Vec3 iBasis = Vec3(ParseXmlAttribute(*transformElement, "x", Vec3()));
	Vec3 jBasis = Vec3(ParseXmlAttribute(*transformElement, "y", Vec3()));
	Vec3 kBasis = Vec3(ParseXmlAttribute(*transformElement, "z", Vec3()));
	Vec3 translation = Vec3(ParseXmlAttribute(*transformElement, "t", Vec3()));
	float scale = ParseXmlAttribute(*transformElement, "scale", 0.0f);
	transform.SetIJKT3D(iBasis, jBasis, kBasis, translation);
	transform.AppendScaleUniform3D(scale);

	//Load OBJ model vertex positions
 	g_theOBJLoader->LoadIntoRigidBody(filePath, this, transform);

	//Set Mass
	m_mass = totalMass;

	// TODO CALCULATE ACCURATE INERTIA TENSORS FOR ANY OBJ OBJECT
	//Setting inertia tensors
	if (filePath == "Data/Models/Cubeoid_vf.obj")
	{
		iBasis = Vec3(m_mass / 12.0f * ((1.0f * 1.0f) + (0.6f * 0.6f)), 0.0f, 0.0f);
		jBasis = Vec3(0.0f, m_mass / 12.0f * ((10.0f * 10.0f) + (0.6f * 0.6f)), 0.0f);
		kBasis = Vec3(0.0f, 0.0f, m_mass / 12.0f * ((10.0f * 10.0f) + (1.0f * 1.0f)));
		m_inertiaTensor.SetIJK3D(iBasis, jBasis, kBasis);
		m_inverseInertiaTensor = m_inertiaTensor.GetInverse();
	}
	else //if (filePath == "Data/Models/Cube_vf.obj")
	{
		iBasis = Vec3(m_mass / 12.0f * ((3.0f * 3.0f) + (3.0f * 3.0f)), 0.0f, 0.0f);
		jBasis = Vec3(0.0f, m_mass / 12.0f * ((3.0f * 3.0f) + (3.0f * 3.0f)), 0.0f);
		kBasis = Vec3(0.0f, 0.0f, m_mass / 12.0f * ((3.0f * 3.0f) + (3.0f * 3.0f)));
		m_inertiaTensor.SetIJK3D(iBasis, jBasis, kBasis);
		m_inverseInertiaTensor = m_inertiaTensor.GetInverse();
	}
}

//-----------------------------------------------------------------------------------------------
void RigidBody3D::Update(float deltaSeconds)
{
	ComputeForcesAndTorque();
	Integrate(deltaSeconds);
}

//-----------------------------------------------------------------------------------------------
void RigidBody3D::ComputeForcesAndTorque()
{
	//Clear Forces
	m_force = Vec3();
	m_torque = Vec3();

	if (m_isGravityEnabled)
	{
		m_force += m_gravity * m_mass;
	}
}

//-----------------------------------------------------------------------------------------------
void RigidBody3D::Integrate(float deltaSeconds)
{
	//Linear Motion
	Vec3 deltaLinearMomentum = m_force * deltaSeconds;
	m_linearMomentum += deltaLinearMomentum;
	m_velocity = m_linearMomentum / m_mass;

	//Angular Motion
	Vec3 deltaAngularMomentum = m_torque * deltaSeconds;
	m_angularMomentum += deltaAngularMomentum;
	Mat33 worldInverseInertiaTrensor = m_rotation;
	worldInverseInertiaTrensor.Append(m_inverseInertiaTensor);
	worldInverseInertiaTrensor.Append(m_rotation.GetOrthonormalInverse());
	m_angularVelocity = worldInverseInertiaTrensor.TransformVectorQuantity3D(m_angularMomentum);
	Vec3 rotatedIbasis = CrossProduct3D(m_angularVelocity, m_rotation.GetIBasis3D()) * deltaSeconds;
	Vec3 rotatedJbasis = CrossProduct3D(m_angularVelocity, m_rotation.GetJBasis3D()) * deltaSeconds;
	Vec3 rotatedKbasis = CrossProduct3D(m_angularVelocity, m_rotation.GetKBasis3D()) * deltaSeconds;
	m_rotation.SetIJK3D(m_rotation.GetIBasis3D() + rotatedIbasis, m_rotation.GetJBasis3D() + rotatedJbasis, m_rotation.GetKBasis3D() + rotatedKbasis);
	m_rotation.Orthonormalize_XFwd_YLeft_ZUp();

	//Position Update
	m_position += m_velocity * deltaSeconds;
}

//-----------------------------------------------------------------------------------------------
void RigidBody3D::ApplyForceAndTorque(Vec3 const& force, Vec3 const& impactLocation)
{
	//Force
	m_force += force;
	
	//Torque
	Vec3 displacementToCenterOfMass = impactLocation - m_position;
	m_torque += CrossProduct3D(displacementToCenterOfMass, force);
}

//-----------------------------------------------------------------------------------------------
void RigidBody3D::ApplyImpulse(Vec3 const& force, Vec3 const& impactLocation)
{
	//Linear impulse
	m_linearMomentum += force;
	m_velocity = m_linearMomentum / m_mass;


	//Torque
	Vec3 displacementToCenterOfMass = impactLocation - m_position;
	m_torque = CrossProduct3D(displacementToCenterOfMass, force);
	m_angularMomentum += m_torque;
	Mat33 worldInverseInertiaTrensor = m_rotation;
	worldInverseInertiaTrensor.Append(m_inverseInertiaTensor);
	worldInverseInertiaTrensor.Append(m_rotation.GetOrthonormalInverse());
	m_angularVelocity = worldInverseInertiaTrensor.TransformVectorQuantity3D(m_angularMomentum);
}

//-----------------------------------------------------------------------------------------------
void RigidBody3D::DebudRender(Renderer* renderer) const
{
	Mat44 renderMatrix(m_rotation);
	renderMatrix.SetTranslation3D(m_position);
	renderer->SetModelConstants(renderMatrix);
	renderer->BindTexture(nullptr);
	std::vector<Vertex_PCU> verts;

	//Local Basis Vectors
	AddVertsForSphere3D(verts, Vec3(), 0.05f, Rgba8::WHITE);
	AddVertsForArrow3D(verts, Vec3(), Vec3() + Vec3(1.0f, 0.0f, 0.0f), 0.025f, Rgba8::RED);
	AddVertsForArrow3D(verts, Vec3(), Vec3() + Vec3(0.0f, 1.0f, 0.0f), 0.025f, Rgba8::GREEN);
	AddVertsForArrow3D(verts, Vec3(), Vec3() + Vec3(0.0f, 0.0f, 1.0f), 0.025f, Rgba8::BLUE);
	renderer->DrawVertexArray(int(verts.size()), verts.data(), PrimitiveTopology::D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
	verts.clear();

	//Velocity and Angular Velocity Vectors
	renderMatrix.SetIJK3D(Vec3(1.0f, 0.0f, 0.0f), Vec3(0.0f, 1.0f, 0.0f), Vec3(0.0f, 0.0f, 1.0f));
	renderer->SetModelConstants(renderMatrix);
	AddVertsForArrow3D(verts, Vec3(), Vec3() + m_velocity, 0.025f, Rgba8::YELLOW);
	AddVertsForArrow3D(verts, Vec3(), Vec3() + m_angularVelocity.GetNormalized(), 0.025f, Rgba8::MAGENTA);
	renderer->DrawVertexArray(int(verts.size()), verts.data(), PrimitiveTopology::D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
}
