//
//  application.cpp
//
#include <chrono>
#include <thread>

#include "Renderer/DeviceContext.h"
#include "Renderer/model.h"
#include "Renderer/shader.h"
#include "Renderer/Samplers.h"

#include "application.h"
#include "Fileio.h"
#include <assert.h>

#include "Renderer/OffscreenRenderer.h"

#include "Scene.h"

Application * application = NULL;

#include <time.h>
#include <windows.h>

static bool gIsInitialized( false );
static unsigned __int64 gTicksPerSecond;
static unsigned __int64 gStartTicks;

/*
====================================
GetTimeSeconds
====================================
*/
int GetTimeMicroseconds() {
	if ( false == gIsInitialized ) {
		gIsInitialized = true;

		// Get the high frequency counter's resolution
		QueryPerformanceFrequency( (LARGE_INTEGER *)&gTicksPerSecond );

		// Get the current time
		QueryPerformanceCounter( (LARGE_INTEGER *)&gStartTicks );

		return 0;
	}

	unsigned __int64 tick;
	QueryPerformanceCounter( (LARGE_INTEGER *)&tick );

	const double ticks_per_micro = (double)( gTicksPerSecond / 1000000 );

	const unsigned __int64 timeMicro = (unsigned __int64)( (double)( tick - gStartTicks ) / ticks_per_micro );
	return (int)timeMicro;
}

/*
========================================================================================================

Application

========================================================================================================
*/

/*
====================================================
Application::Initialize
====================================================
*/
void Application::Initialize() {
	//FillDiamond();

	InitializeGLFW();
	InitializeVulkan();

	scene = new Scene;
	scene->Initialize();
	scene->Reset();

	m_models.reserve( scene->bodies.size() );
	for ( int i = 0; i < scene->bodies.size(); i++ ) {
		Model * model = new Model();
		model->BuildFromShape( scene->bodies[ i ].shape );
		model->MakeVBO( &deviceContext );

		m_models.push_back( model );
	}

	m_mousePosition = Vec2( 0, 0 );
	m_cameraPositionTheta = acosf( -1.0f ) / 2.0f;
	m_cameraPositionPhi = 0;
	m_cameraRadius = 15.0f;
	m_cameraFocusPoint = Vec3( 0, 0, 3 );

	m_isPaused = true;
	m_stepFrame = false;
}

/*
====================================================
Application::~Application
====================================================
*/
Application::~Application() {
	Cleanup();
}

/*
====================================================
Application::InitializeGLFW
====================================================
*/
void Application::InitializeGLFW() {
	glfwInit();

	glfwWindowHint( GLFW_CLIENT_API, GLFW_NO_API );

	glfwWindow = glfwCreateWindow( WINDOW_WIDTH, WINDOW_HEIGHT, "Physics", nullptr, nullptr );

	glfwSetWindowUserPointer( glfwWindow, this );
	glfwSetWindowSizeCallback( glfwWindow, Application::OnWindowResized );

	glfwSetInputMode( glfwWindow, GLFW_CURSOR, GLFW_CURSOR_DISABLED );
	glfwSetInputMode( glfwWindow, GLFW_STICKY_KEYS, GLFW_TRUE );
	glfwSetCursorPosCallback( glfwWindow, Application::OnMouseMoved );
	glfwSetScrollCallback( glfwWindow, Application::OnMouseWheelScrolled );
	glfwSetKeyCallback( glfwWindow, Application::OnKeyboard );
}

/*
====================================================
Application::GetGLFWRequiredExtensions
====================================================
*/
std::vector< const char * > Application::GetGLFWRequiredExtensions() const {
	std::vector< const char * > extensions;

	const char ** glfwExtensions;
	uint32_t glfwExtensionCount = 0;
	glfwExtensions = glfwGetRequiredInstanceExtensions( &glfwExtensionCount );

	for ( uint32_t i = 0; i < glfwExtensionCount; i++ ) {
		extensions.push_back( glfwExtensions[ i ] );
	}

	if ( m_enableLayers ) {
		extensions.push_back( VK_EXT_DEBUG_REPORT_EXTENSION_NAME );
	}

	return extensions;
}

/*
====================================================
Application::InitializeVulkan
====================================================
*/
bool Application::InitializeVulkan() {
	//
	//	Vulkan Instance
	//
	{
		std::vector< const char * > extensions = GetGLFWRequiredExtensions();
		if ( !deviceContext.CreateInstance( m_enableLayers, extensions ) ) {
			printf( "ERROR: Failed to create vulkan instance\n" );
			assert( 0 );
			return false;
		}
	}

	//
	//	Vulkan Surface for GLFW Window
	//
	if ( VK_SUCCESS != glfwCreateWindowSurface( deviceContext.m_vkInstance, glfwWindow, nullptr, &deviceContext.m_vkSurface ) ) {
		printf( "ERROR: Failed to create window sruface\n" );
		assert( 0 );
		return false;
	}

	int windowWidth;
	int windowHeight;
	glfwGetWindowSize( glfwWindow, &windowWidth, &windowHeight );

	//
	//	Vulkan Device
	//
	if ( !deviceContext.CreateDevice() ) {
		printf( "ERROR: Failed to create device\n" );
		assert( 0 );
		return false;
	}

	//
	//	Create SwapChain
	//
	if ( !deviceContext.CreateSwapChain( windowWidth, windowHeight ) ) {
		printf( "ERROR: Failed to create swapchain\n" );
		assert( 0 );
		return false;
	}

	//
	//	Initialize texture samplers
	//
	Samplers::InitializeSamplers( &deviceContext );

	//
	//	Command Buffers
	//
	if ( !deviceContext.CreateCommandBuffers() ) {
		printf( "ERROR: Failed to create command buffers\n" );
		assert( 0 );
		return false;
	}

	//
	//	Uniform Buffer
	//
	m_uniformBuffer.Allocate( &deviceContext, NULL, sizeof( float ) * 16 * 4 * 128, VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT );

	//
	//	Offscreen rendering
	//
	InitOffscreen( &deviceContext, deviceContext.m_swapChain.m_windowWidth, deviceContext.m_swapChain.m_windowHeight );

	//
	//	Full screen texture rendering
	//
	{
		bool result;
		FillFullScreenQuad( m_modelFullScreen );
		for ( int i = 0; i < m_modelFullScreen.m_vertices.size(); i++ ) {
			m_modelFullScreen.m_vertices[ i ].xyz[ 1 ] *= -1.0f;
		}
		m_modelFullScreen.MakeVBO( &deviceContext );

		result = m_copyShader.Load( &deviceContext, "DebugImage2D" );
		if ( !result ) {
			printf( "ERROR: Failed to load copy shader\n" );
			assert( 0 );
			return false;
		}

		Descriptors::CreateParms_t descriptorParms;
		memset( &descriptorParms, 0, sizeof( descriptorParms ) );
		descriptorParms.numUniformsFragment = 1;
		descriptorParms.numImageSamplers = 1;
		result = m_copyDescriptors.Create( &deviceContext, descriptorParms );
		if ( !result ) {
			printf( "ERROR: Failed to create copy descriptors\n" );
			assert( 0 );
			return false;
		}

		Pipeline::CreateParms_t pipelineParms;
		memset( &pipelineParms, 0, sizeof( pipelineParms ) );
		pipelineParms.renderPass = deviceContext.m_swapChain.m_vkRenderPass;
		pipelineParms.descriptors = &m_copyDescriptors;
		pipelineParms.shader = &m_copyShader;
		pipelineParms.width = deviceContext.m_swapChain.m_windowWidth;
		pipelineParms.height = deviceContext.m_swapChain.m_windowHeight;
		pipelineParms.cullMode = Pipeline::CULL_MODE_NONE;
		pipelineParms.depthTest = false;
		pipelineParms.depthWrite = false;
		result = m_copyPipeline.Create( &deviceContext, pipelineParms );
		if ( !result ) {
			printf( "ERROR: Failed to create copy pipeline\n" );
			assert( 0 );
			return false;
		}
	}

	return true;
}

/*
====================================================
Application::Cleanup
====================================================
*/
void Application::Cleanup() {
	CleanupOffscreen( &deviceContext );

	m_copyShader.Cleanup( &deviceContext );
	m_copyDescriptors.Cleanup( &deviceContext );
	m_copyPipeline.Cleanup( &deviceContext );
	m_modelFullScreen.Cleanup( deviceContext );

	// Delete the screen so that it can clean itself up
	delete scene;
	scene = NULL;

	// Delete models
	for ( int i = 0; i < m_models.size(); i++ ) {
		m_models[ i ]->Cleanup( deviceContext );
		delete m_models[ i ];
	}
	m_models.clear();

	// Delete Uniform Buffer Memory
	m_uniformBuffer.Cleanup( &deviceContext );

	// Delete Samplers
	Samplers::Cleanup( &deviceContext );

	// Delete Device Context
	deviceContext.Cleanup();

	// Delete GLFW
	glfwDestroyWindow( glfwWindow );
	glfwTerminate();
}

/*
====================================================
Application::OnWindowResized
====================================================
*/
void Application::OnWindowResized( GLFWwindow * window, int windowWidth, int windowHeight ) {
	if ( 0 == windowWidth || 0 == windowHeight ) {
		return;
	}

	Application * application = reinterpret_cast< Application * >( glfwGetWindowUserPointer( window ) );
	application->ResizeWindow( windowWidth, windowHeight );
}

/*
====================================================
Application::ResizeWindow
====================================================
*/
void Application::ResizeWindow( int windowWidth, int windowHeight ) {
	deviceContext.ResizeWindow( windowWidth, windowHeight );

	//
	//	Full screen texture rendering
	//
	{
		bool result;
		m_copyPipeline.Cleanup( &deviceContext );

		Pipeline::CreateParms_t pipelineParms;
		memset( &pipelineParms, 0, sizeof( pipelineParms ) );
		pipelineParms.renderPass = deviceContext.m_swapChain.m_vkRenderPass;
		pipelineParms.descriptors = &m_copyDescriptors;
		pipelineParms.shader = &m_copyShader;
		pipelineParms.width = windowWidth;
		pipelineParms.height = windowHeight;
		pipelineParms.cullMode = Pipeline::CULL_MODE_NONE;
		pipelineParms.depthTest = false;
		pipelineParms.depthWrite = false;
		result = m_copyPipeline.Create( &deviceContext, pipelineParms );
		if ( !result ) {
			printf( "Unable to build pipeline!\n" );
			assert( 0 );
			return;
		}
	}
}

/*
====================================================
Application::OnMouseMoved
====================================================
*/
void Application::OnMouseMoved( GLFWwindow * window, double x, double y ) {
	Application * application = reinterpret_cast< Application * >( glfwGetWindowUserPointer( window ) );
	application->MouseMoved( (float)x, (float)y );
}

/*
====================================================
Application::MouseMoved
====================================================
*/
void Application::MouseMoved( float x, float y ) {
	Vec2 newPosition = Vec2( x, y );
	Vec2 ds = newPosition - m_mousePosition;
	m_mousePosition = newPosition;

	float sensitivity = 0.01f;
	m_cameraPositionTheta += ds.y * sensitivity;
	m_cameraPositionPhi += ds.x * sensitivity;

	if ( m_cameraPositionTheta < 0.14f ) {
		m_cameraPositionTheta = 0.14f;
	}
	if ( m_cameraPositionTheta > 3.0f ) {
		m_cameraPositionTheta = 3.0f;
	}
}

/*
====================================================
Application::OnMouseWheelScrolled
====================================================
*/
void Application::OnMouseWheelScrolled( GLFWwindow * window, double x, double y ) {
	Application * application = reinterpret_cast< Application * >( glfwGetWindowUserPointer( window ) );
	application->MouseScrolled( (float)y );
}

/*
====================================================
Application::MouseScrolled
====================================================
*/
void Application::MouseScrolled( float z ) {
	m_cameraRadius -= z;
	if ( m_cameraRadius < 0.5f ) {
		m_cameraRadius = 0.5f;
	}
}

/*
====================================================
Application::OnKeyboard
====================================================
*/
void Application::OnKeyboard( GLFWwindow * window, int key, int scancode, int action, int modifiers ) {
	Application * application = reinterpret_cast< Application * >( glfwGetWindowUserPointer( window ) );
	application->Keyboard( key, scancode, action, modifiers );
}

/*
====================================================
Application::Keyboard
====================================================
*/
void Application::Keyboard( int key, int scancode, int action, int modifiers ) {
	if ( GLFW_KEY_R == key && GLFW_RELEASE == action ) {
		scene->Reset();
	}
	if ( GLFW_KEY_T == key && GLFW_RELEASE == action ) {
		m_isPaused = !m_isPaused;
	}
	if ( GLFW_KEY_Y == key && ( GLFW_PRESS == action || GLFW_REPEAT == action ) ) {
		m_stepFrame = m_isPaused && !m_stepFrame;
	}
}

/*
====================================================
Application::MainLoop
====================================================
*/
void Application::MainLoop() {
	static int timeLastFrame = 0;
	static int numSamples = 0;
	static float avgTime = 0.0f;
	static float maxTime = 0.0f;

	while ( !glfwWindowShouldClose( glfwWindow ) ) {
		int time					= GetTimeMicroseconds();
		float dt_us					= (float)time - (float)timeLastFrame;
		if ( dt_us < 16000.0f ) {
			int x = 16000 - (int)dt_us;
			std::this_thread::sleep_for( std::chrono::microseconds( x ) );
			dt_us = 16000;
			time = GetTimeMicroseconds();
		}
		timeLastFrame = time;
		printf( "\ndt_ms: %.1f    ", dt_us * 0.001f );

		// Get User Input
		glfwPollEvents();

		// If the time is greater than 33ms (30fps)
		// then force the time difference to smaller
		// to prevent super large simulation steps.
		if ( dt_us > 33000.0f ) {
			dt_us = 33000.0f;
		}

		bool runPhysics = true;
		if ( m_isPaused ) {
			dt_us = 0.0f;
			runPhysics = false;
			if ( m_stepFrame ) {
				dt_us = 16667.0f;
				m_stepFrame = false;
				runPhysics = true;
			}
			numSamples = 0;
			maxTime = 0.0f;
		}
		float dt_sec = dt_us * 0.001f * 0.001f;

		// Run Update
		if ( runPhysics ) {
			int startTime = GetTimeMicroseconds();
			for ( int i = 0; i < 2; i++ ) {
				scene->Update( dt_sec * 0.5f );
			}
			int endTime = GetTimeMicroseconds();

			dt_us = (float)endTime - (float)startTime;
			if ( dt_us > maxTime ) {
				maxTime = dt_us;
			}

			avgTime = ( avgTime * float( numSamples ) + dt_us ) / float( numSamples + 1 );
			numSamples++;

			printf( "frame dt_ms: %.2f %.2f %.2f", avgTime * 0.001f, maxTime * 0.001f, dt_us * 0.001f );
		}

		// Draw the Scene
		DrawFrame();
	}
}

/*
====================================================
Application::UpdateUniforms
====================================================
*/
void Application::UpdateUniforms() {
	m_renderModels.clear();

	uint32_t uboByteOffset = 0;
	uint32_t cameraByteOFfset = 0;
	uint32_t shadowByteOffset = 0;

	struct camera_t {
		Mat4 matView;
		Mat4 matProj;
		Mat4 pad0;
		Mat4 pad1;
	};
	camera_t camera;

	//
	//	Update the uniform buffers
	//
	{
		unsigned char * mappedData = (unsigned char *)m_uniformBuffer.MapBuffer( &deviceContext );

		//
		// Update the uniform buffer with the camera information
		//
		{
			Vec3 camPos = Vec3( 10, 0, 5 ) * 1.25f;
			Vec3 camLookAt = Vec3( 0, 0, 1 );
			Vec3 camUp = Vec3( 0, 0, 1 );

			camPos.x = cosf( m_cameraPositionPhi ) * sinf( m_cameraPositionTheta );
			camPos.y = sinf( m_cameraPositionPhi ) * sinf( m_cameraPositionTheta );
			camPos.z = cosf( m_cameraPositionTheta );
			camPos *= m_cameraRadius;

			camPos += m_cameraFocusPoint;

			camLookAt = m_cameraFocusPoint;

			int windowWidth;
			int windowHeight;
			glfwGetWindowSize( glfwWindow, &windowWidth, &windowHeight );

			const float zNear   = 0.1f;
			const float zFar    = 1000.0f;
			const float fovy	= 45.0f;
			const float aspect	= (float)windowHeight / (float)windowWidth;
			camera.matProj.PerspectiveVulkan( fovy, aspect, zNear, zFar );
			camera.matProj = camera.matProj.Transpose();

			camera.matView.LookAt( camPos, camLookAt, camUp );
			camera.matView = camera.matView.Transpose();

			// Update the uniform buffer for the camera matrices
			memcpy( mappedData + uboByteOffset, &camera, sizeof( camera ) );

			cameraByteOFfset = uboByteOffset;

			// update offset into the buffer
			uboByteOffset += deviceContext.GetAligendUniformByteOffset( sizeof( camera ) );
		}

		//
		// Update the uniform buffer with the shadow camera information
		//
		{
			Vec3 camPos = Vec3( 1, 1, 1 ) * 75.0f;
			Vec3 camLookAt = Vec3( 0, 0, 0 );
			Vec3 camUp = Vec3( 0, 0, 1 );
			Vec3 tmp = camPos.Cross( camUp );
			camUp = tmp.Cross( camPos );
			camUp.Normalize();

			extern FrameBuffer g_shadowFrameBuffer;
			const int windowWidth = g_shadowFrameBuffer.m_parms.width;
			const int windowHeight = g_shadowFrameBuffer.m_parms.height;

			const float halfWidth = 60.0f;
			const float xmin	= -halfWidth;
			const float xmax	= halfWidth;
			const float ymin	= -halfWidth;
			const float ymax	= halfWidth;
			const float zNear	= 25.0f;
			const float zFar	= 175.0f;
			camera.matProj.OrthoVulkan( xmin, xmax, ymin, ymax, zNear, zFar );
			camera.matProj = camera.matProj.Transpose();

			camera.matView.LookAt( camPos, camLookAt, camUp );
			camera.matView = camera.matView.Transpose();

			// Update the uniform buffer for the camera matrices
			memcpy( mappedData + uboByteOffset, &camera, sizeof( camera ) );

			shadowByteOffset = uboByteOffset;

			// update offset into the buffer
			uboByteOffset += deviceContext.GetAligendUniformByteOffset( sizeof( camera ) );
		}

		//
		//	Update the uniform buffer with the body positions/orientations
		//
		for ( int i = 0; i < scene->bodies.size(); i++ ) {
			Body & body = scene->bodies[ i ];

			Vec3 fwd = body.orientation.RotatePoint( Vec3( 1, 0, 0 ) );
			Vec3 up = body.orientation.RotatePoint( Vec3( 0, 0, 1 ) );

			Mat4 matOrient;
			matOrient.Orient( body.position, fwd, up );
			matOrient = matOrient.Transpose();

			// Update the uniform buffer with the orientation of this body
			memcpy( mappedData + uboByteOffset, matOrient.ToPtr(), sizeof( matOrient ) );

			RenderModel renderModel;
			renderModel.model = m_models[ i ];
			renderModel.uboByteOffset = uboByteOffset;
			renderModel.uboByteSize = sizeof( matOrient );
			renderModel.pos = body.position;
			renderModel.orient = body.orientation;
			m_renderModels.push_back( renderModel );

			uboByteOffset += deviceContext.GetAligendUniformByteOffset( sizeof( matOrient ) );
		}

		m_uniformBuffer.UnmapBuffer( &deviceContext );
	}
}

/*
====================================================
Application::DrawFrame
====================================================
*/
void Application::DrawFrame() {
	UpdateUniforms();

	//
	//	Begin the render frame
	//
	const uint32_t imageIndex = deviceContext.BeginFrame();

	// Draw everything in an offscreen buffer
	DrawOffscreen( &deviceContext, imageIndex, &m_uniformBuffer, m_renderModels.data(), (int)m_renderModels.size() );

	//
	//	Draw the offscreen framebuffer to the swap chain frame buffer
	//
 	deviceContext.BeginRenderPass();
	{
		extern FrameBuffer g_offscreenFrameBuffer;
		VkCommandBuffer cmdBuffer = deviceContext.m_vkCommandBuffers[ imageIndex ];

		// Binding the pipeline is effectively the "use shader" we had back in our opengl apps
		m_copyPipeline.BindPipeline( cmdBuffer );

		// Descriptor is how we bind our buffers and images
		Descriptor descriptor = m_copyPipeline.GetFreeDescriptor();
		descriptor.BindImage( VK_IMAGE_LAYOUT_GENERAL, g_offscreenFrameBuffer.m_imageColor.m_vkImageView, Samplers::m_samplerStandard, 0 );
		descriptor.BindDescriptor( &deviceContext, cmdBuffer, &m_copyPipeline );
		m_modelFullScreen.DrawIndexed( cmdBuffer );
	}
 	deviceContext.EndRenderPass();

	//
	//	End the render frame
	//
	deviceContext.EndFrame();
}