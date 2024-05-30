#include "Camera.hpp"
#include "Engine/Window/Window.hpp"
#include "Engine/Input/InputSystem.hpp"
#include "Engine/Math/MathUtils.hpp"
#include "Engine/Core/ErrorWarningAssert.hpp"

//-----------------------------------------------------------------------------------------------
void Camera::SetOrthographicView(Vec2 const& bottomLeft, Vec2 const& topRight, float near, float far)
{
	m_mode = eMode_Orthographic;
	m_orthographicBottomLeft = bottomLeft;
	m_orthographicTopRight = topRight;
	m_orthographicNear = near;
	m_orthographicFar = far;
}

//-----------------------------------------------------------------------------------------------
void Camera::SetPerspectiveView(float aspect, float fov, float near, float far)
{
	m_mode = eMode_Perspective;
	m_perspectiveAspect = aspect;
	m_perspectiveFOV = fov;
	m_perspectiveNear = near;
	m_perspectiveFar = far;
}

//-----------------------------------------------------------------------------------------------
void Camera::SetRenderBasis(Vec3 const& iBasis, Vec3 const& jBasis, Vec3 const& kBasis)
{
	m_renderIBasis = iBasis;
	m_renderJBasis = jBasis;
	m_renderKBasis = kBasis;
}

//-----------------------------------------------------------------------------------------------
void Camera::SetTransform(const Vec3& position, const EulerAngles& orientation)
{
	m_position = position;
	m_orientation = orientation;
}

//-----------------------------------------------------------------------------------------------
Vec2 Camera::GetOrthographicBottomLeft() const
{
	return m_orthographicBottomLeft;
}

//-----------------------------------------------------------------------------------------------
Vec2 Camera::GetOrthographicTopRight() const
{
	return m_orthographicTopRight;
}

//-----------------------------------------------------------------------------------------------
void Camera::Translate2D(const Vec2& tranlsation2D)
{
	m_orthographicBottomLeft.x += tranlsation2D.x;
	m_orthographicTopRight.x += tranlsation2D.x;
	m_orthographicBottomLeft.y += tranlsation2D.y;
	m_orthographicTopRight.y += tranlsation2D.y;
}

//-----------------------------------------------------------------------------------------------
Mat44 Camera::GetOrthographicMatrix() const
{
	Mat44 orthographicMatrix;
	orthographicMatrix = orthographicMatrix.CreateOrthoProjection(m_orthographicBottomLeft.x, m_orthographicTopRight.x, m_orthographicBottomLeft.y, m_orthographicTopRight.y, m_orthographicNear, m_orthographicFar);
	return orthographicMatrix;
}

//-----------------------------------------------------------------------------------------------
Mat44 Camera::GetPerspectiveMatrix() const
{
	Mat44 perspectiveMatrix;
	perspectiveMatrix = perspectiveMatrix.CreatePerspectiveProjection(m_perspectiveFOV, m_perspectiveAspect, m_perspectiveNear, m_perspectiveFar);
	return perspectiveMatrix;
}

//-----------------------------------------------------------------------------------------------
Mat44 Camera::GetProjectionMatrix() const
{
	Mat44 ProjectionMatrix;
	if (m_mode == eMode_Orthographic)
	{
		ProjectionMatrix = GetOrthographicMatrix();
		ProjectionMatrix.Append(GetRenderMatrix());
		return ProjectionMatrix;
	}
	else
	{
		ProjectionMatrix = GetPerspectiveMatrix();
		ProjectionMatrix.Append(GetRenderMatrix());
		return ProjectionMatrix;
	}
}

//-----------------------------------------------------------------------------------------------
Mat44 Camera::GetRenderMatrix() const
{
	Mat44 renderMatrix;
	renderMatrix.SetIJK3D(m_renderIBasis, m_renderJBasis, m_renderKBasis);
	return renderMatrix;
}

//-----------------------------------------------------------------------------------------------
Mat44 Camera::GetViewMatrix() const
{
	Mat44 viewMatrix;
	
	viewMatrix = m_orientation.GetAsMatrix_XFwd_YLeft_ZUp();
	viewMatrix.SetTranslation3D(m_position);
	viewMatrix = viewMatrix.GetOrthonormalInverse();
	return viewMatrix;
}


//-----------------------------------------------------------------------------------------------
Vec3 Camera::GetWorldMouseDirection()
{
	Mat44 cameraMatrix = m_orientation.GetAsMatrix_XFwd_YLeft_ZUp();
	IntVec2 clientDimensions = g_theWindow->GetClientDimensions();
	IntVec2 clientCenter = g_theWindow->GetClientCenter();
	IntVec2 cursorClientPos = g_theInput->GetCursorClientPosition();
	
	float viewSpaceHeight = (m_perspectiveNear * TanDegrees(m_perspectiveFOV * 0.5f)) * 2.0f;
	float viewSpaceWidth = m_perspectiveAspect * viewSpaceHeight;
	float cursorViewSpaceX = RangeMapClamped(float(cursorClientPos.x), 0.0f, float(clientDimensions.x), 0.0f, viewSpaceWidth);
	float cursorViewSpaceY = RangeMapClamped(float(cursorClientPos.y), 0.0f, float(clientDimensions.y), 0.0f, viewSpaceHeight);
	float clientCenterViewSpaceX = RangeMapClamped(float(clientCenter.x), 0.0f, float(clientDimensions.x), 0.0f, viewSpaceWidth);
	float clientCenterViewSpaceY = RangeMapClamped(float(clientCenter.y), 0.0f, float(clientDimensions.y), 0.0f, viewSpaceHeight);

	float xOffset = cursorViewSpaceX - clientCenterViewSpaceX;
	float yOffset = cursorViewSpaceY - clientCenterViewSpaceY;

	Vec3 a = m_perspectiveNear * cameraMatrix.GetIBasis3D();
	Vec3 b = -xOffset * cameraMatrix.GetJBasis3D();
	Vec3 c = -yOffset * cameraMatrix.GetKBasis3D();

	Vec3 direction = (a + b + c).GetNormalized();
	return direction;
}

//-----------------------------------------------------------------------------------------------
Vec3 Camera::GetLocalDirectionFromScreenPosition(IntVec2 screenPosition)
{
	Mat44 cameraMatrix = m_orientation.GetAsMatrix_XFwd_YLeft_ZUp();
	IntVec2 clientDimensions = g_theWindow->GetClientDimensions();
	IntVec2 clientCenter = g_theWindow->GetClientCenter();
	//DebuggerPrintf("Client Position = (%d, %d)\n", screenPosition.x, screenPosition.y);

	float viewSpaceHeight = (m_perspectiveNear * TanDegrees(m_perspectiveFOV * 0.5f)) * 2.0f;
	float viewSpaceWidth = m_perspectiveAspect * viewSpaceHeight;
	float cursorViewSpaceX = RangeMapClamped(float(screenPosition.x), 0.0f, float(clientDimensions.x), 0.0f, viewSpaceWidth);
	float cursorViewSpaceY = RangeMapClamped(float(screenPosition.y), 0.0f, float(clientDimensions.y), 0.0f, viewSpaceHeight);
	float clientCenterViewSpaceX = RangeMapClamped(float(clientCenter.x), 0.0f, float(clientDimensions.x), 0.0f, viewSpaceWidth);
	float clientCenterViewSpaceY = RangeMapClamped(float(clientCenter.y), 0.0f, float(clientDimensions.y), 0.0f, viewSpaceHeight);

	float xOffset = cursorViewSpaceX - clientCenterViewSpaceX;
	float yOffset = cursorViewSpaceY - clientCenterViewSpaceY;

	Vec3 a = m_perspectiveNear * Vec3(1.0f, 0.0f, 0.0f);
	Vec3 b = -xOffset * Vec3(0.0f, 1.0f, 0.0f);
	Vec3 c = yOffset * Vec3(0.0f, 0.0f, 1.0f);

	Vec3 direction = (a + b + c).GetNormalized();
	return direction;
}
