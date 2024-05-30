#pragma once
#include "Constraint3D.hpp"
#include "Particles3D.hpp"
#include "Engine/Math/Vec3.hpp"
#include "Engine/Math/DPVec4.hpp"
#include "Engine/Math/Vec4.hpp"
#include "Engine/Math/Vec3.hpp"
#include "Engine/Math/Vec2.hpp"
#include "Engine/Math/OBB3.hpp"
#include "Engine/Math/AABB3.hpp"
#include "Engine/Math/Capsule3.hpp"
#include "Engine/Math/Sphere3.hpp"
#include "Engine/Math/Cylinder3.hpp"
#include "Engine/Math/EulerAngles.hpp"
#include "Engine/Math/IntVec3.hpp"
#include <vector>
#include <cstdint>

//-----------------------------------------------------------------------------------------------
struct	Vertex_PCU;
struct	Vertex_PCUTBN;
struct	Spring2D;
struct	Mat44;
class	Texture;
class	Shader;
class	StructuredBuffer;
class	ConstantBuffer;
class	Renderer;
class	VertexBuffer;
class	Query;

//-----------------------------------------------------------------------------------------------
enum class CollisionType
{
	SPHERES,
	CAPSULES,
	COUNT
};

//-----------------------------------------------------------------------------------------------
struct GPUVertex_PCUTBN
{
	Vec3 m_position;
	Vec4 m_color;
	Vec2 m_uvTexCoords;
	Vec3 m_tangent;
	Vec3 m_binormal;
	Vec3 m_normal;
};

//-----------------------------------------------------------------------------------------------
//Polymorphic Collision Objects 
struct CollisionObject
{
public:
	CollisionObject(Vec3 const& boundingDiscCenter, float boundingDiscRadius);
	virtual ~CollisionObject() {}
	
public:
	uint64_t	m_macroBitRegions = 0;
	uint64_t	m_microBitRegions = 0;
	Vec3		m_boundingDiscCenter;
	float		m_boundingDiscRadius = 0.0f;
};
struct SphereCollisionObject : public CollisionObject
{
public:
	SphereCollisionObject(Vec3 const& boundingDiscCenter, float boundingDiscRadius, Sphere3 const& sphere);
	~SphereCollisionObject() {}

public:
	Sphere3 m_sphere;
};
struct AABBCollisionObject : public CollisionObject
{
public:
	AABBCollisionObject(Vec3 const& boundingDiscCenter, float boundingDiscRadius, AABB3 const& aabb);
	~AABBCollisionObject() {}

public:
	AABB3 m_aabb;
};
struct OBBCollisionObject : public CollisionObject
{
public:
	OBBCollisionObject(Vec3 const& boundingDiscCenter, float boundingDiscRadius, OBB3 const& obb);
	~OBBCollisionObject() {}
	
public:
	OBB3 m_obb;
};
struct CapsuleCollisionObject : public CollisionObject
{
public:
	CapsuleCollisionObject(Vec3 const& boundingDiscCenter, float boundingDiscRadius, Capsule3 const& capsule);
	~CapsuleCollisionObject() {}
		
public:
	Capsule3	m_capsule;
	Vec3		m_collisionNormalStart;
	Vec3		m_jacobiCorrectionStart;
	int			m_jacobiConstraintTotalStart;
	int			m_isCollidingStart;
	Vec3		m_collisionNormalEnd;
	Vec3		m_jacobiCorrectionEnd;
	int			m_jacobiConstraintTotalEnd;
	int			m_isCollidingEnd;
};
struct CylinderCollisionObject : public CollisionObject
{
public:
	CylinderCollisionObject(Vec3 const& boundingDiscCenter, float boundingDiscRadius, Cylinder3 const& cylinder);
	~CylinderCollisionObject() {}
	
public:
	Cylinder3 m_cylinder;
};

//-----------------------------------------------------------------------------------------------
struct RopeSimualtionConstantBufferVariables
{
	int		m_totalParticles;
	float	m_ropeRadius;
	float	m_gravityCoefficient;
	float	m_physicsTimestep;
	float	m_dampingCoefficient;
	int		m_totalSolverIterations;
	float	m_desiredDistance;
	float	m_compressionCoefficient;
	float	m_stretchingCoefficient;
	float	m_kineticFrictionCoefficient;
	float	m_bendingConstraintDistance;
	float	m_bendingCoefficient;
	Vec3	m_worldBoundsMins;
	int		m_padding1;
	Vec3	m_worldBoundsMaxs;
	int		m_totalCollisionObjects;
	int		m_totalAABBs;
	int		m_totalOBBs;
	int		m_totalCylinders;
	int		m_totalCapsules;
	int		m_totalSpheres;
	int		m_isSelfCollisionEnabled;
	int		m_padding3[2];
};

//-----------------------------------------------------------------------------------------------
struct GameConstantBufferVariables
{
	Vec3	m_grabPosition;
	int		m_isGrabbing;
	Vec3	m_grabDirection;
	int		m_shouldLock;
	int		m_unlockAllParticles;
	int		m_isMouseBeingScrolled;
	float	m_mouseScrollDelta;
	int		m_shouldLockParticle;
};

//-----------------------------------------------------------------------------------------------
struct GameStructuredBufferVariables
{
	float	m_originalGrabLengthFromCamera;
	int		m_isRopeBeingGrabbed = 0;
	int		m_grabbedRopeParticleIndex = -1;
	int		m_wasPointAlreadyLocked = 0;
};


//-----------------------------------------------------------------------------------------------
class RopeSimulation3D
{
public:
	explicit	RopeSimulation3D(Renderer* renderer, AABB3 worldBounds, int totalParticles, float totalMassOfRope, float dampingCoefficient,
				float stretchCoefficient, float compressionCoefficient, float bendingCoefficient, float staticFrictionCoefficient,
				float kineticFrictionCoefficient, int totalSolverIterations, Vec3 start, Vec3 end, float physicsTimestep,
				CollisionType collisionType, bool isSelfCollisionEnabled, int totalCollisionObjects, int totalAABBs, int totalOBBs, int totalCylinders,
				int totalCapsules, int totalSpheres);
	RopeSimulation3D(){};
	~RopeSimulation3D();
	void		Startup();
	void		Shutdown();

	//Update/Render
	void		Update(float deltaSeconds);
	void		Render() const;

	//Misc Public Methods
	float		GetCurrentLengthOfTheRope();
	void		ClearShapeReferences();
	void		UpdateGrabbedRopeParticle(int sentParticleIndex, Vec3 const& newPosition);
	void		AttachRopeParticle(int const& particleIndex);
	void		UnattachRopeParticle(int const& particleIndex);
	void		UpdateCollisionObjectBitRegions();
	void		InitializeGPUCollisionObjects();

private:
	//GPU/CPU Functions
	void		InitializeShaders();
	void		UpdateGPUBuffers();
	void		UpdateCPU();
	void		UpdateGPU();
	 
	//Gauss Seidel CPU
	void		UpdateGaussSeidel();
	void		ProjectConstraintsGaussSeidel();
	void		ProjectDistanceConstraintGaussSeidel(int constraintIndex);
	void		ProjectBendingConstraintGaussSeidel(int constraintIndex);
	void		ProjectCollisionConstraintsSpheresGaussSeidel(int sentParticleIndex);
	void		ProjectWorldBoundsConstraintsSpheresGaussSeidel(int sentParticleIndex);
	void		ProjectCollisionConstraintsCapsulesGaussSeidel(int sentCapsuleIndex);
	void		ProjectWorldBoundsConstraintsCapsulesGaussSeidel(int sentCapsuleIndex);

	//Jacobi CPU
	void		UpdateJacobi();
	void		ProjectConstraintsJacobi();
	void		ProjectDistanceConstraintJacobi(int constraintIndex);
	void		ProjectBendingConstraintJacobi(int constraintIndex);
	void		ProjectCollisionConstraintsSpheresJacobi(int sentParticleIndex);
	void		ProjectWorldBoundsConstraintsSpheresJacobi(int sentParticleIndex);
	void		ProjectCollisionConstraintsCapsulesJacobi(int sentCapsuleIndex);
	void		ProjectWorldBoundsConstraintsCapsulesJacobi(int sentCapsuleIndex);

	//Bit Regions and Collision Constraint Generation
	void		InitializeBitRegions();
	void		BitRegionDetectionAllParticles();
	void		BitRegionDetectionSingleParticle(int const& sentParticleIndex);
	void		BitRegionDetectionAllCapsules();
	void		BitRegionDetectionSingleCapsule(int const& sentCapsuleIndex);
	void		BitRegionDetectionAllCollisionObjects();
	void		AssignBitRegionsParticle(int sentParticleIndex, int currentRegion, int regionDifferenceX, int regionDifferenceY, bool isMicro = true);
	void		AssignBitRegionsCapsule(int sentCapsuleIndex, int currentRegion, int regionDifferenceX, int regionDifferenceY, bool isMicro = true);
	void		AssignBitRegionsCollisionObject(int sentCollisionIndex, int currentRegion, int regionDifferenceX, int regionDifferenceY, bool isMicro = true);

	//Debug Render Functions
	void		DebugRenderParticles() const;
	void		DebugRenderBitRegions() const;

	//Misc Private Methods
	int			GetRegionIndexForCoords(IntVec3 const& coords);
	IntVec3		GetCoordsForRegionIndex(int const& region);
	void		UpdateCollisionCapsulesFromParticle(int const& sentParticleIndex);
	void		UpdateProposedParticlesFromCollisionCapsule(int const& sentCapsuleIndex);
	void		UpdateNeighboringCollisionCapsules(int const& sentCapsuleIndex);
	void		AddCollidingRopeParticle(int const& particleIndex);
	void		UnaddCollidingRopeParticle(int const& particleIndex);
	int			GetPreviousAttachmentOrCollisionParticleIndex(int const& particleIndex) const;
	int			GetNextAttachmentOrCollisionParticleIndex(int const& particleIndex) const;
	void		GenerateConstraints();
	void		DeleteConstraints();

public:
	//Rope Variables
	std::vector<Constraint3D>				m_distanceConstraints;
	std::vector<Constraint3D>				m_bendingConstraints;
	std::vector<CapsuleCollisionObject>		m_collisionCapsules;
	std::vector<CollisionObject*>			m_collisionObjects;
	std::vector<Vertex_PCU>					m_verts;
	std::vector<int>						m_attachedParticleIndices;
	std::vector<int>						m_collisionParticleIndices;
	Texture*								m_texture = nullptr;
	Particles3D								m_particles;
	Vec3									m_ropeStartPosition;
	Vec3									m_ropeEndPosition;
	float									m_physicsTimestep = 0.0005f;
	float									m_physicsDebt = 0.0f;
	float									m_desiredDistance = 0.0f;
	float									m_bendingConstraintDistance = 0.0f;
	float									m_bendingCoefficient = 0.0f;
	float									m_stretchingCoefficient = 0.0f;
	float									m_compressionCoefficient = 0.0f;
	float									m_dampingCoefficient = 0.0f;
	float									m_gravityCoefficient = 9.81f;
	int										m_totalSolverIterations = 10;
	int										m_numberOfParticlesInRope = 0;
	float									m_ropeRadius = 0.015f;
	float									m_staticFrictionCoefficient = 0.0f;
	float									m_kineticFrictionCoefficient = 0.0f;
	float									m_totalLengthOfRope = 0.0f;
	float									m_totalMassOfRope = 0.0f;
	float									m_constraintsStartTime = 0.0f;
	float									m_constraintsEndTime = 0.0f;
	CollisionType							m_collisionType = CollisionType::SPHERES;
	Vec3									m_grabbedParticlePosition;
	int										m_grabbedParticleIndex = -1;
	int										m_totalSubsteps;
	float									m_substepMultiplier;
	bool									isMaxLength = false;
	bool									m_isGravityEnabled = true;
	bool									m_isDebugRope = false;
	bool									m_isDebugBitRegions = false;
	bool									m_isGPUSimulated = true;
	bool									m_isJacobiSolver = false;
	bool									m_isSelfCollisionEnabled = true;
	bool									m_isHierarchical = false;

	//Bit Bucket Variables / Collision Variables
	VertexBuffer*							m_bitRegionVertexBuffer = nullptr;
	std::vector<AABB3>						m_macroBitRegionBounds;
	std::vector<std::vector<AABB3>>			m_microBitRegionBounds;
	float									m_worldScaleX = 0.0f;
	float									m_worldScaleY = 0.0f;
	float									m_macroScaleX = 0.0f;  
	float									m_macroScaleY = 0.0f;
	float									m_microScaleX = 0.0f;
	float									m_microScaleY = 0.0f;
	float									m_bitRegionScale = 1.0f / 8.0f;
	int										m_bitRegionDebugVertCount = 0;
	int										m_totalCollisionObjects = 0;
	int										m_totalAABBs = 0;
	int										m_totalOBBs = 0;
	int										m_totalCylinders = 0;
	int										m_totalCapsules = 0;
	int										m_totalSpheres = 0;
	float									m_simulationStartTime = 0.0f;
	float									m_simulationEndTime = 0.0f;
	int										m_collisionCount = 0;
	AABB3									m_worldBounds;

	//Geometry and Compute Shader Variables
	Renderer*								m_renderer = nullptr;
	Shader*									m_csGameInteraction = nullptr;
	Shader*									m_csInitialUpdates = nullptr;
	Shader*									m_csProjectNonCollisionConstraints = nullptr;
	Shader*									m_csUpdateAfterNonCollisionConstraints = nullptr;
	Shader*									m_csProjectCollisionConstraintsSpheres = nullptr;
	Shader*									m_csInitializeBitRegionsCapsules = nullptr;
	Shader*									m_csUpdateParticlesFromCapsules = nullptr;
	Shader*									m_csCapsuleCollisionsPhaseOne = nullptr;
	Shader*									m_csCapsuleCollisionsPhaseTwo = nullptr;
	Shader*									m_csFinalUpdates = nullptr;
	Shader*									m_csInitializeCollisionObjects = nullptr;
	Shader*									m_csAddVerts = nullptr;
	Shader*									m_csTangentSpace = nullptr;
	Shader*									m_geometryShaderCylinderSides = nullptr;
	Shader*									m_geometryShaderHemisphereTop = nullptr;
	Shader*									m_geometryShaderHemisphereBottom = nullptr;
	Shader*									m_renderShader = nullptr;

	ConstantBuffer*							m_cbRopeData = nullptr;
	ConstantBuffer*							m_cbGameInteraciton = nullptr;
	StructuredBuffer*						m_sbGameInteraction = nullptr;
	StructuredBuffer*						m_sbParticlePositions = nullptr;
	StructuredBuffer*						m_sbParticleVelocities = nullptr;
	StructuredBuffer*						m_sbParticleProposedPositions = nullptr;
	StructuredBuffer*						m_sbParticleJacobiCorrections = nullptr;
	StructuredBuffer*						m_sbParticleCollisionNormals = nullptr;
	StructuredBuffer*						m_sbParticleMacroBitRegions = nullptr;
	StructuredBuffer*						m_sbParticleMicroBitRegions = nullptr;
	StructuredBuffer*						m_sbParticleMasses = nullptr;
	StructuredBuffer*						m_sbParticleInverseMasses = nullptr;
	StructuredBuffer*						m_sbParticleIsAttached = nullptr;
	StructuredBuffer*						m_sbAABBs = nullptr;
	StructuredBuffer*						m_sbOBBs = nullptr;
	StructuredBuffer*						m_sbSpheres = nullptr;
	StructuredBuffer*						m_sbCapsules = nullptr;
	StructuredBuffer*						m_sbCylinders = nullptr;
	StructuredBuffer*						m_sbRopeCapsules = nullptr;
	StructuredBuffer*						m_structuredBufferVerts = nullptr;
	Query*									m_startGPUQuery = nullptr;
	Query*									m_endGPUQuery = nullptr;
	GameConstantBufferVariables				m_gameInteractionVariables;
	int										m_totalVerts = 0;
	int										m_dispatchThreadX = 1;
	int										m_dispatchThreadY = 1;
	int										m_dispatchThreadZ = 1;
	RopeSimualtionConstantBufferVariables	m_constantBufferVars;
	bool									m_readyToQuery = false;
	bool									m_hasGPUSwitchOccured = false;
	bool									m_shouldRunGameUpdateComputeShader = false;
};