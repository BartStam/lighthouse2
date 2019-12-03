/* platform.cpp - Copyright 2019 Utrecht University

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

	   http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.

   This file:

   Implementation of platform-specific functionality:
   - Application entry point
   - Window management though GLFW
   This system uses OpenGL for rasterization-based taks, which are limited to:
   - Rendering full-screen quads using textures filled by cores
   - Providing the cores with rendertargets
   - Post-processing in the basicapp project.
*/

#include "platform.h"

#pragma comment( linker, "/subsystem:windows /ENTRY:mainCRTStartup" )

//  +-----------------------------------------------------------------------------+
//  |  Thread::start                                                              |
//  |  Entry point for threads.                                             LH2'19|
//  +-----------------------------------------------------------------------------+
uint sthread_proc( void* param ) { Thread* tp = (Thread*)param; tp->run(); return 0; }
void Thread::start()
{
	thread = std::thread( sthread_proc, this );
#ifdef _MSC_VER
	SetThreadPriority( thread.native_handle(), THREAD_PRIORITY_NORMAL );
#else
	// pthread_setschedprio(thread.native_handle(), ...);
#endif
}

//  +-----------------------------------------------------------------------------+
//  |  OpenGL helper functions.                                             LH2'19|
//  +-----------------------------------------------------------------------------+
void _CheckGL( const char* f, int l )
{
	GLenum error = glGetError();
	if (error != GL_NO_ERROR)
	{
		const char *errStr = "UNKNOWN ERROR";
		if (error == 0x500) errStr = "INVALID ENUM";
		else if (error == 0x502) errStr = "INVALID OPERATION";
		else if (error == 0x501) errStr = "INVALID VALUE";
		else if (error == 0x506) errStr = "INVALID FRAMEBUFFER OPERATION";

		FatalError( "GL error %d: %s at %s:%d\n", error, errStr, f, l );
	}
}

GLuint CreateVBO( const GLfloat* data, const uint size )
{
	GLuint id;
	glGenBuffers( 1, &id );
	glBindBuffer( GL_ARRAY_BUFFER, id );
	glBufferData( GL_ARRAY_BUFFER, size, data, GL_STATIC_DRAW );
	CheckGL();
	return id;
}

void BindVBO( const uint idx, const uint N, const GLuint id )
{
	glEnableVertexAttribArray( idx );
	glBindBuffer( GL_ARRAY_BUFFER, id );
	glVertexAttribPointer( idx, N, GL_FLOAT, GL_FALSE, 0, (void*)0 );
	CheckGL();
}

void CheckShader( GLuint shader, const char* vshader, const char* fshader )
{
	char buffer[1024];
	memset( buffer, 0, 1024 );
	GLsizei length = 0;
	glGetShaderInfoLog( shader, 1024, &length, buffer );
	CheckGL();
	FATALERROR_IF( length > 0 && strstr( buffer, "ERROR" ), "Shader compile error:\n%s", buffer );
}

void CheckProgram( GLuint id, const char* vshader, const char* fshader )
{
	char buffer[1024];
	memset( buffer, 0, 1024 );
	GLsizei length = 0;
	glGetProgramInfoLog( id, 1024, &length, buffer );
	CheckGL();
	FATALERROR_IF( length > 0, "Shader link error:\n%s", buffer );
}

void DrawQuad()
{
	static GLuint vao = 0;
	if (!vao)
	{
		// generate buffers
		static const GLfloat verts[] = { -1, -1, 0, 1, -1, 0, -1, 1, 0, 1, -1, 0, -1, 1, 0, 1, 1, 0 };
		static const GLfloat uvdata[] = { 0, 1, 1, 1, 0, 0, 1, 1, 0, 0, 1, 0 };
		GLuint vertexBuffer = CreateVBO( verts, sizeof( verts ) );
		GLuint UVBuffer = CreateVBO( uvdata, sizeof( uvdata ) );
		glGenVertexArrays( 1, &vao );
		glBindVertexArray( vao );
		BindVBO( 0, 3, vertexBuffer );
		BindVBO( 1, 2, UVBuffer );
		glBindVertexArray( 0 );
		CheckGL();
	}
	glBindVertexArray( vao );
	glDrawArrays( GL_TRIANGLES, 0, 6 );
	glBindVertexArray( 0 );
	CheckGL();
}

//  +-----------------------------------------------------------------------------+
//  |  DrawShapeOnScreen			    										  |
//  |  Draws the specified shape from the given vertices using GL.                |
//  |  *verts* are the 2D screen coordinates.                                     |
//  |  *colors* are the respective rgba colors of each vertex.                    |
//  |  *GLshape* can be:													      |
//  |     GL_POINTS       *verts* are individual points (squares of size *width)  |
//  |     GL_LINES        *verts* consists of vertex pairs					      |
//  |     GL_LINE_STRIP   *verts* consists of consequtive vertices			      |
//  |     GL_TRIANGLE_FAN *verts* consists of  consequtive verts around a poly    |
//  |  *width* is the line/point width in pixels.                           LH2'19|
//  +-----------------------------------------------------------------------------+
void DrawShapeOnScreen(std::vector<float2> verts, std::vector<float4> colors, uint GLshape, float width)
{
	if (verts.size() == 0) return;

	// Create VBOs
	static GLuint vboID = 0;
	GLuint vertexBuffer = CreateVBO((const GLfloat*)&verts[0], (uint) verts.size() * sizeof(float2));
	GLuint colorBuffer = CreateVBO((const GLfloat*)&colors[0], (uint) colors.size() * sizeof(float4));
	glGenVertexArrays(1, &vboID);
	glBindVertexArray(vboID);
	BindVBO(0, 2, vertexBuffer); // 0 = position attribute, 2 = floats in vertex
	BindVBO(3, 4, colorBuffer);  // 3 = color attribute, 4 = floats in color
	glBindVertexArray(0);
	CheckGL();

	// Draw
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glBindVertexArray(vboID);
	glPointSize(width);
	glLineWidth(width);
	glDrawArrays(GLshape, 0, (GLsizei)verts.size());
	glBindVertexArray(0);
	CheckGL();
	glDisable(GL_BLEND);
}

//  +-----------------------------------------------------------------------------+
//  |  OpenGL texture wrapper class.                                        LH2'19|
//  +-----------------------------------------------------------------------------+
GLTexture::GLTexture( uint w, uint h, uint type )
{
	width = w;
	height = h;
	glGenTextures( 1, &ID );
	glBindTexture( GL_TEXTURE_2D, ID );
	if (type == DEFAULT)
	{
		// regular texture
		glTexImage2D( GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_BGR, GL_UNSIGNED_BYTE, 0 );
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST );
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST );
	}
	else /* type == FLOAT */
	{
		// floating point texture
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE );
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE );
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST );
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST );
		glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA32F, width, height, 0, GL_RGBA, GL_FLOAT, 0 );
	}
	CheckGL();
}

GLTexture::GLTexture( const char* fileName, int filter )
{
	GLuint textureType = GL_TEXTURE_2D;
	glGenTextures( 1, &ID );
	glBindTexture( textureType, ID );
	FREE_IMAGE_FORMAT fif = FIF_UNKNOWN;
	fif = FreeImage_GetFileType( fileName, 0 );
	if (fif == FIF_UNKNOWN) fif = FreeImage_GetFIFFromFilename( fileName );
	FIBITMAP* tmp = FreeImage_Load( fif, fileName );
	FATALERROR_IF( !tmp, "File %s not found", fileName );
	FIBITMAP* dib = FreeImage_ConvertTo24Bits( tmp );
	FreeImage_Unload( tmp );
	width = FreeImage_GetWidth( dib );
	height = FreeImage_GetHeight( dib );
	uint* data = new uint[width * height];
	uchar* line = new uchar[width * 3];
	for (uint y = 0; y < height; y++)
	{
		memcpy( line, FreeImage_GetScanLine( dib, height - 1 - y ), width * 3 );
		for (uint x = 0; x < width; x++) data[y * width + x] = (line[x * 3 + 0] << 16) + (line[x * 3 + 1] << 8) + line[x * 3 + 2];
	}
	FreeImage_Unload( dib );
	glTexParameteri( textureType, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE );
	glTexParameteri( textureType, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE );
	glTexParameteri( textureType, GL_TEXTURE_MAG_FILTER, filter );
	glTexParameteri( textureType, GL_TEXTURE_MIN_FILTER, filter );
	glTexImage2D( textureType, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data );
	CheckGL();
}

GLTexture::~GLTexture()
{
	glDeleteTextures( 1, &ID );
	CheckGL();
}

void GLTexture::Bind()
{
	glBindTexture( GL_TEXTURE_2D, ID );
	CheckGL();
}

void GLTexture::CopyFrom( Bitmap* src )
{
	glBindTexture( GL_TEXTURE_2D, ID );
	glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, src->pixels );
	CheckGL();
}

void GLTexture::CopyTo( Bitmap* dst )
{
	glBindTexture( GL_TEXTURE_2D, ID );
	glGetTexImage( GL_TEXTURE_2D, 0, GL_RGBA, GL_UNSIGNED_BYTE, dst->pixels );
	CheckGL();
}

//  +-----------------------------------------------------------------------------+
//  |  Shader class implementation.                                         LH2'19|
//  +-----------------------------------------------------------------------------+
Shader::Shader( const char* vfile, const char* pfile )
{
	Init( vfile, pfile );
}

Shader::~Shader()
{
	glDetachShader( ID, pixel );
	glDetachShader( ID, vertex );
	glDeleteShader( pixel );
	glDeleteShader( vertex );
	glDeleteProgram( ID );
	CheckGL();
}

void Shader::Init( const char* vfile, const char* pfile )
{
	string vsText = TextFileRead( vfile );
	string fsText = TextFileRead( pfile );
	FATALERROR_IF( vsText.size() == 0, "File %s not found", vfile );
	FATALERROR_IF( fsText.size() == 0, "File %s not found", pfile );
	const char* vertexText = vsText.c_str();
	const char* fragmentText = fsText.c_str();
	Compile( vertexText, fragmentText );
}

void Shader::Compile( const char* vtext, const char* ftext )
{
	vertex = glCreateShader( GL_VERTEX_SHADER );
	pixel = glCreateShader( GL_FRAGMENT_SHADER );
	glShaderSource( vertex, 1, &vtext, 0 );
	glCompileShader( vertex );
	CheckShader( vertex, vtext, ftext );
	glShaderSource( pixel, 1, &ftext, 0 );
	glCompileShader( pixel );
	CheckShader( pixel, vtext, ftext );
	ID = glCreateProgram();
	glAttachShader( ID, vertex );
	glAttachShader( ID, pixel );
	glBindAttribLocation( ID, 0, "pos" );
	glBindAttribLocation( ID, 1, "tuv" );
	glLinkProgram( ID );
	CheckProgram( ID, vtext, ftext );
	CheckGL();
}

void Shader::Bind()
{
	glUseProgram( ID );
	CheckGL();
}

void Shader::Unbind()
{
	glUseProgram( 0 );
	CheckGL();
}

void Shader::SetInputTexture( uint slot, const char* name, GLTexture* texture )
{
	glActiveTexture( GL_TEXTURE0 + slot );
	glBindTexture( GL_TEXTURE_2D, texture->ID );
	glUniform1i( glGetUniformLocation( ID, name ), slot );
	CheckGL();
}

void Shader::SetInputMatrix( const char* name, const mat4& matrix )
{
	const GLfloat* data = (const GLfloat*)&matrix;
	glUniformMatrix4fv( glGetUniformLocation( ID, name ), 1, GL_FALSE, data );
	CheckGL();
}

void Shader::SetFloat( const char* name, const float v )
{
	glUniform1f( glGetUniformLocation( ID, name ), v );
	CheckGL();
}

void Shader::SetInt( const char* name, const int v )
{
	glUniform1i( glGetUniformLocation( ID, name ), v );
	CheckGL();
}

void Shader::SetUInt( const char* name, const uint v )
{
	glUniform1ui( glGetUniformLocation( ID, name ), v );
	CheckGL();
}

//  +-----------------------------------------------------------------------------+
//  |  Below this point: glad 0.1.28.                                       LH2'19|
//  +-----------------------------------------------------------------------------+

/*

	OpenGL loader generated by glad 0.1.33 on Mon Sep 23 13:29:33 2019.

	Language/Generator: C/C++
	Specification: gl
	APIs: gl=4.6
	Profile: core
	Extensions:
		GL_EXT_memory_object,
		GL_EXT_memory_object_fd,
		GL_EXT_memory_object_win32
	Loader: True
	Local files: False
	Omit khrplatform: False
	Reproducible: False

	Commandline:
		--profile="core" --api="gl=4.6" --generator="c" --spec="gl" --extensions="GL_EXT_memory_object,GL_EXT_memory_object_fd,GL_EXT_memory_object_win32"
	Online:
		https://glad.dav1d.de/#profile=core&language=c&specification=gl&loader=on&api=gl%3D4.6&extensions=GL_EXT_memory_object&extensions=GL_EXT_memory_object_fd&extensions=GL_EXT_memory_object_win32
*/

/*

	OpenGL loader generated by glad 0.1.33 on Mon Sep 23 13:24:30 2019.

	Language/Generator: C/C++
	Specification: gl
	APIs: gl=4.6
	Profile: compatibility
	Extensions:
		GL_EXT_memory_object,
		GL_EXT_memory_object_fd,
		GL_EXT_memory_object_win32
	Loader: True
	Local files: False
	Omit khrplatform: False
	Reproducible: False

	Commandline:
		--profile="compatibility" --api="gl=4.6" --generator="c" --spec="gl" --extensions="GL_EXT_memory_object,GL_EXT_memory_object_fd,GL_EXT_memory_object_win32"
	Online:
		https://glad.dav1d.de/#profile=compatibility&language=c&specification=gl&loader=on&api=gl%3D4.6&extensions=GL_EXT_memory_object&extensions=GL_EXT_memory_object_fd&extensions=GL_EXT_memory_object_win32
*/

static void* get_proc( const char *namez );

#if defined(_WIN32) || defined(__CYGWIN__)
#ifndef _WINDOWS_
#undef APIENTRY
#endif
static HMODULE libGL;

typedef void* (APIENTRYP PFNWGLGETPROCADDRESSPROC_PRIVATE)(const char*);
static PFNWGLGETPROCADDRESSPROC_PRIVATE gladGetProcAddressPtr;

#ifdef _MSC_VER
#ifdef __has_include
#if __has_include(<winapifamily.h>)
#define HAVE_WINAPIFAMILY 1
#endif
#elif _MSC_VER >= 1700 && !_USING_V110_SDK71_
#define HAVE_WINAPIFAMILY 1
#endif
#endif

#if 0
#ifdef HAVE_WINAPIFAMILY
#include <winapifamily.h>
#if !WINAPI_FAMILY_PARTITION(WINAPI_PARTITION_DESKTOP) && WINAPI_FAMILY_PARTITION(WINAPI_PARTITION_APP)
#define IS_UWP 1
#endif
#endif
#endif

static
int open_gl( void )
{
#ifndef IS_UWP
	libGL = LoadLibraryW( L"opengl32.dll" );
	if (libGL != NULL)
	{
		void( *tmp )(void);
		tmp = (void( *)(void)) GetProcAddress( libGL, "wglGetProcAddress" );
		gladGetProcAddressPtr = (PFNWGLGETPROCADDRESSPROC_PRIVATE)tmp;
		return gladGetProcAddressPtr != NULL;
	}
#endif

	return 0;
}

static
void close_gl( void )
{
	if (libGL != NULL)
	{
		FreeLibrary( (HMODULE)libGL );
		libGL = NULL;
	}
}
#else
#include <dlfcn.h>
static void* libGL;

#if !defined(__APPLE__) && !defined(__HAIKU__)
typedef void* (APIENTRYP PFNGLXGETPROCADDRESSPROC_PRIVATE)(const char*);
static PFNGLXGETPROCADDRESSPROC_PRIVATE gladGetProcAddressPtr;
#endif

static
int open_gl( void )
{
#ifdef __APPLE__
	static const char *NAMES[] = {
		"../Frameworks/OpenGL.framework/OpenGL",
		"/Library/Frameworks/OpenGL.framework/OpenGL",
		"/System/Library/Frameworks/OpenGL.framework/OpenGL",
		"/System/Library/Frameworks/OpenGL.framework/Versions/Current/OpenGL"
	};
#else
	static const char *NAMES[] = { "libGL.so.1", "libGL.so" };
#endif

	unsigned int index = 0;
	for (index = 0; index < (sizeof( NAMES ) / sizeof( NAMES[0] )); index++)
	{
		libGL = dlopen( NAMES[index], RTLD_NOW | RTLD_GLOBAL );

		if (libGL != NULL)
		{
		#if defined(__APPLE__) || defined(__HAIKU__)
			return 1;
		#else
			gladGetProcAddressPtr = (PFNGLXGETPROCADDRESSPROC_PRIVATE)dlsym( libGL,
				"glXGetProcAddressARB" );
			return gladGetProcAddressPtr != NULL;
		#endif
		}
	}

	return 0;
}

static
void close_gl( void )
{
	if (libGL != NULL)
	{
		dlclose( libGL );
		libGL = NULL;
	}
}
#endif

static
void* get_proc( const char *namez )
{
	void* result = NULL;
	if (libGL == NULL) return NULL;

#if !defined(__APPLE__) && !defined(__HAIKU__)
	if (gladGetProcAddressPtr != NULL)
	{
		result = gladGetProcAddressPtr( namez );
	}
#endif
	if (result == NULL)
	{
	#if defined(_WIN32) || defined(__CYGWIN__)
		result = (void*)GetProcAddress( (HMODULE)libGL, namez );
	#else
		result = dlsym( libGL, namez );
	#endif
	}

	return result;
}

int gladLoadGL( void )
{
	int status = 0;

	if (open_gl())
	{
		status = gladLoadGLLoader( &get_proc );
		close_gl();
	}

	return status;
}

struct gladGLversionStruct GLVersion = { 0, 0 };

#if defined(GL_ES_VERSION_3_0) || defined(GL_VERSION_3_0)
#define _GLAD_IS_SOME_NEW_VERSION 1
#endif

static int max_loaded_major;
static int max_loaded_minor;

static const char *exts = NULL;
static int num_exts_i = 0;
static char **exts_i = NULL;

static int get_exts( void )
{
#ifdef _GLAD_IS_SOME_NEW_VERSION
	if (max_loaded_major < 3)
	{
	#endif
		exts = (const char *)glGetString( GL_EXTENSIONS );
	#ifdef _GLAD_IS_SOME_NEW_VERSION
	}
	else
	{
		unsigned int index;

		num_exts_i = 0;
		glGetIntegerv( GL_NUM_EXTENSIONS, &num_exts_i );
		if (num_exts_i > 0)
		{
			exts_i = (char **)malloc( (size_t)num_exts_i * (sizeof *exts_i) );
		}

		if (exts_i == NULL)
		{
			return 0;
		}

		for (index = 0; index < (unsigned)num_exts_i; index++)
		{
			const char *gl_str_tmp = (const char*)glGetStringi( GL_EXTENSIONS, index );
			size_t len = strlen( gl_str_tmp );

			char *local_str = (char*)malloc( (len + 1) * sizeof( char ) );
			if (local_str != NULL)
			{
				memcpy( local_str, gl_str_tmp, (len + 1) * sizeof( char ) );
			}
			exts_i[index] = local_str;
		}
	}
#endif
	return 1;
}

static void free_exts( void )
{
	if (exts_i != NULL)
	{
		int index;
		for (index = 0; index < num_exts_i; index++)
		{
			free( (char *)exts_i[index] );
		}
		free( (void *)exts_i );
		exts_i = NULL;
	}
}

static int has_ext( const char *ext )
{
#ifdef _GLAD_IS_SOME_NEW_VERSION
	if (max_loaded_major < 3)
	{
	#endif
		const char *extensions;
		const char *loc;
		const char *terminator;
		extensions = exts;
		if (extensions == NULL || ext == NULL)
		{
			return 0;
		}

		while (1)
		{
			loc = strstr( extensions, ext );
			if (loc == NULL)
			{
				return 0;
			}

			terminator = loc + strlen( ext );
			if ((loc == extensions || *(loc - 1) == ' ') &&
				(*terminator == ' ' || *terminator == '\0'))
			{
				return 1;
			}
			extensions = terminator;
		}
	#ifdef _GLAD_IS_SOME_NEW_VERSION
	}
	else
	{
		int index;
		if (exts_i == NULL) return 0;
		for (index = 0; index < num_exts_i; index++)
		{
			const char *e = exts_i[index];

			if (exts_i[index] != NULL && strcmp( e, ext ) == 0)
			{
				return 1;
			}
		}
	}
#endif

	return 0;
}
int GLAD_GL_VERSION_1_0 = 0;
int GLAD_GL_VERSION_1_1 = 0;
int GLAD_GL_VERSION_1_2 = 0;
int GLAD_GL_VERSION_1_3 = 0;
int GLAD_GL_VERSION_1_4 = 0;
int GLAD_GL_VERSION_1_5 = 0;
int GLAD_GL_VERSION_2_0 = 0;
int GLAD_GL_VERSION_2_1 = 0;
int GLAD_GL_VERSION_3_0 = 0;
int GLAD_GL_VERSION_3_1 = 0;
int GLAD_GL_VERSION_3_2 = 0;
int GLAD_GL_VERSION_3_3 = 0;
int GLAD_GL_VERSION_4_0 = 0;
int GLAD_GL_VERSION_4_1 = 0;
int GLAD_GL_VERSION_4_2 = 0;
int GLAD_GL_VERSION_4_3 = 0;
int GLAD_GL_VERSION_4_4 = 0;
int GLAD_GL_VERSION_4_5 = 0;
int GLAD_GL_VERSION_4_6 = 0;
PFNGLACCUMPROC glad_glAccum = NULL;
PFNGLACTIVESHADERPROGRAMPROC glad_glActiveShaderProgram = NULL;
PFNGLACTIVETEXTUREPROC glad_glActiveTexture = NULL;
PFNGLALPHAFUNCPROC glad_glAlphaFunc = NULL;
PFNGLARETEXTURESRESIDENTPROC glad_glAreTexturesResident = NULL;
PFNGLARRAYELEMENTPROC glad_glArrayElement = NULL;
PFNGLATTACHSHADERPROC glad_glAttachShader = NULL;
PFNGLBEGINPROC glad_glBegin = NULL;
PFNGLBEGINCONDITIONALRENDERPROC glad_glBeginConditionalRender = NULL;
PFNGLBEGINQUERYPROC glad_glBeginQuery = NULL;
PFNGLBEGINQUERYINDEXEDPROC glad_glBeginQueryIndexed = NULL;
PFNGLBEGINTRANSFORMFEEDBACKPROC glad_glBeginTransformFeedback = NULL;
PFNGLBINDATTRIBLOCATIONPROC glad_glBindAttribLocation = NULL;
PFNGLBINDBUFFERPROC glad_glBindBuffer = NULL;
PFNGLBINDBUFFERBASEPROC glad_glBindBufferBase = NULL;
PFNGLBINDBUFFERRANGEPROC glad_glBindBufferRange = NULL;
PFNGLBINDBUFFERSBASEPROC glad_glBindBuffersBase = NULL;
PFNGLBINDBUFFERSRANGEPROC glad_glBindBuffersRange = NULL;
PFNGLBINDFRAGDATALOCATIONPROC glad_glBindFragDataLocation = NULL;
PFNGLBINDFRAGDATALOCATIONINDEXEDPROC glad_glBindFragDataLocationIndexed = NULL;
PFNGLBINDFRAMEBUFFERPROC glad_glBindFramebuffer = NULL;
PFNGLBINDIMAGETEXTUREPROC glad_glBindImageTexture = NULL;
PFNGLBINDIMAGETEXTURESPROC glad_glBindImageTextures = NULL;
PFNGLBINDPROGRAMPIPELINEPROC glad_glBindProgramPipeline = NULL;
PFNGLBINDRENDERBUFFERPROC glad_glBindRenderbuffer = NULL;
PFNGLBINDSAMPLERPROC glad_glBindSampler = NULL;
PFNGLBINDSAMPLERSPROC glad_glBindSamplers = NULL;
PFNGLBINDTEXTUREPROC glad_glBindTexture = NULL;
PFNGLBINDTEXTUREUNITPROC glad_glBindTextureUnit = NULL;
PFNGLBINDTEXTURESPROC glad_glBindTextures = NULL;
PFNGLBINDTRANSFORMFEEDBACKPROC glad_glBindTransformFeedback = NULL;
PFNGLBINDVERTEXARRAYPROC glad_glBindVertexArray = NULL;
PFNGLBINDVERTEXBUFFERPROC glad_glBindVertexBuffer = NULL;
PFNGLBINDVERTEXBUFFERSPROC glad_glBindVertexBuffers = NULL;
PFNGLBITMAPPROC glad_glBitmap = NULL;
PFNGLBLENDCOLORPROC glad_glBlendColor = NULL;
PFNGLBLENDEQUATIONPROC glad_glBlendEquation = NULL;
PFNGLBLENDEQUATIONSEPARATEPROC glad_glBlendEquationSeparate = NULL;
PFNGLBLENDEQUATIONSEPARATEIPROC glad_glBlendEquationSeparatei = NULL;
PFNGLBLENDEQUATIONIPROC glad_glBlendEquationi = NULL;
PFNGLBLENDFUNCPROC glad_glBlendFunc = NULL;
PFNGLBLENDFUNCSEPARATEPROC glad_glBlendFuncSeparate = NULL;
PFNGLBLENDFUNCSEPARATEIPROC glad_glBlendFuncSeparatei = NULL;
PFNGLBLENDFUNCIPROC glad_glBlendFunci = NULL;
PFNGLBLITFRAMEBUFFERPROC glad_glBlitFramebuffer = NULL;
PFNGLBLITNAMEDFRAMEBUFFERPROC glad_glBlitNamedFramebuffer = NULL;
PFNGLBUFFERDATAPROC glad_glBufferData = NULL;
PFNGLBUFFERSTORAGEPROC glad_glBufferStorage = NULL;
PFNGLBUFFERSUBDATAPROC glad_glBufferSubData = NULL;
PFNGLCALLLISTPROC glad_glCallList = NULL;
PFNGLCALLLISTSPROC glad_glCallLists = NULL;
PFNGLCHECKFRAMEBUFFERSTATUSPROC glad_glCheckFramebufferStatus = NULL;
PFNGLCHECKNAMEDFRAMEBUFFERSTATUSPROC glad_glCheckNamedFramebufferStatus = NULL;
PFNGLCLAMPCOLORPROC glad_glClampColor = NULL;
PFNGLCLEARPROC glad_glClear = NULL;
PFNGLCLEARACCUMPROC glad_glClearAccum = NULL;
PFNGLCLEARBUFFERDATAPROC glad_glClearBufferData = NULL;
PFNGLCLEARBUFFERSUBDATAPROC glad_glClearBufferSubData = NULL;
PFNGLCLEARBUFFERFIPROC glad_glClearBufferfi = NULL;
PFNGLCLEARBUFFERFVPROC glad_glClearBufferfv = NULL;
PFNGLCLEARBUFFERIVPROC glad_glClearBufferiv = NULL;
PFNGLCLEARBUFFERUIVPROC glad_glClearBufferuiv = NULL;
PFNGLCLEARCOLORPROC glad_glClearColor = NULL;
PFNGLCLEARDEPTHPROC glad_glClearDepth = NULL;
PFNGLCLEARDEPTHFPROC glad_glClearDepthf = NULL;
PFNGLCLEARINDEXPROC glad_glClearIndex = NULL;
PFNGLCLEARNAMEDBUFFERDATAPROC glad_glClearNamedBufferData = NULL;
PFNGLCLEARNAMEDBUFFERSUBDATAPROC glad_glClearNamedBufferSubData = NULL;
PFNGLCLEARNAMEDFRAMEBUFFERFIPROC glad_glClearNamedFramebufferfi = NULL;
PFNGLCLEARNAMEDFRAMEBUFFERFVPROC glad_glClearNamedFramebufferfv = NULL;
PFNGLCLEARNAMEDFRAMEBUFFERIVPROC glad_glClearNamedFramebufferiv = NULL;
PFNGLCLEARNAMEDFRAMEBUFFERUIVPROC glad_glClearNamedFramebufferuiv = NULL;
PFNGLCLEARSTENCILPROC glad_glClearStencil = NULL;
PFNGLCLEARTEXIMAGEPROC glad_glClearTexImage = NULL;
PFNGLCLEARTEXSUBIMAGEPROC glad_glClearTexSubImage = NULL;
PFNGLCLIENTACTIVETEXTUREPROC glad_glClientActiveTexture = NULL;
PFNGLCLIENTWAITSYNCPROC glad_glClientWaitSync = NULL;
PFNGLCLIPCONTROLPROC glad_glClipControl = NULL;
PFNGLCLIPPLANEPROC glad_glClipPlane = NULL;
PFNGLCOLOR3BPROC glad_glColor3b = NULL;
PFNGLCOLOR3BVPROC glad_glColor3bv = NULL;
PFNGLCOLOR3DPROC glad_glColor3d = NULL;
PFNGLCOLOR3DVPROC glad_glColor3dv = NULL;
PFNGLCOLOR3FPROC glad_glColor3f = NULL;
PFNGLCOLOR3FVPROC glad_glColor3fv = NULL;
PFNGLCOLOR3IPROC glad_glColor3i = NULL;
PFNGLCOLOR3IVPROC glad_glColor3iv = NULL;
PFNGLCOLOR3SPROC glad_glColor3s = NULL;
PFNGLCOLOR3SVPROC glad_glColor3sv = NULL;
PFNGLCOLOR3UBPROC glad_glColor3ub = NULL;
PFNGLCOLOR3UBVPROC glad_glColor3ubv = NULL;
PFNGLCOLOR3UIPROC glad_glColor3ui = NULL;
PFNGLCOLOR3UIVPROC glad_glColor3uiv = NULL;
PFNGLCOLOR3USPROC glad_glColor3us = NULL;
PFNGLCOLOR3USVPROC glad_glColor3usv = NULL;
PFNGLCOLOR4BPROC glad_glColor4b = NULL;
PFNGLCOLOR4BVPROC glad_glColor4bv = NULL;
PFNGLCOLOR4DPROC glad_glColor4d = NULL;
PFNGLCOLOR4DVPROC glad_glColor4dv = NULL;
PFNGLCOLOR4FPROC glad_glColor4f = NULL;
PFNGLCOLOR4FVPROC glad_glColor4fv = NULL;
PFNGLCOLOR4IPROC glad_glColor4i = NULL;
PFNGLCOLOR4IVPROC glad_glColor4iv = NULL;
PFNGLCOLOR4SPROC glad_glColor4s = NULL;
PFNGLCOLOR4SVPROC glad_glColor4sv = NULL;
PFNGLCOLOR4UBPROC glad_glColor4ub = NULL;
PFNGLCOLOR4UBVPROC glad_glColor4ubv = NULL;
PFNGLCOLOR4UIPROC glad_glColor4ui = NULL;
PFNGLCOLOR4UIVPROC glad_glColor4uiv = NULL;
PFNGLCOLOR4USPROC glad_glColor4us = NULL;
PFNGLCOLOR4USVPROC glad_glColor4usv = NULL;
PFNGLCOLORMASKPROC glad_glColorMask = NULL;
PFNGLCOLORMASKIPROC glad_glColorMaski = NULL;
PFNGLCOLORMATERIALPROC glad_glColorMaterial = NULL;
PFNGLCOLORP3UIPROC glad_glColorP3ui = NULL;
PFNGLCOLORP3UIVPROC glad_glColorP3uiv = NULL;
PFNGLCOLORP4UIPROC glad_glColorP4ui = NULL;
PFNGLCOLORP4UIVPROC glad_glColorP4uiv = NULL;
PFNGLCOLORPOINTERPROC glad_glColorPointer = NULL;
PFNGLCOMPILESHADERPROC glad_glCompileShader = NULL;
PFNGLCOMPRESSEDTEXIMAGE1DPROC glad_glCompressedTexImage1D = NULL;
PFNGLCOMPRESSEDTEXIMAGE2DPROC glad_glCompressedTexImage2D = NULL;
PFNGLCOMPRESSEDTEXIMAGE3DPROC glad_glCompressedTexImage3D = NULL;
PFNGLCOMPRESSEDTEXSUBIMAGE1DPROC glad_glCompressedTexSubImage1D = NULL;
PFNGLCOMPRESSEDTEXSUBIMAGE2DPROC glad_glCompressedTexSubImage2D = NULL;
PFNGLCOMPRESSEDTEXSUBIMAGE3DPROC glad_glCompressedTexSubImage3D = NULL;
PFNGLCOMPRESSEDTEXTURESUBIMAGE1DPROC glad_glCompressedTextureSubImage1D = NULL;
PFNGLCOMPRESSEDTEXTURESUBIMAGE2DPROC glad_glCompressedTextureSubImage2D = NULL;
PFNGLCOMPRESSEDTEXTURESUBIMAGE3DPROC glad_glCompressedTextureSubImage3D = NULL;
PFNGLCOPYBUFFERSUBDATAPROC glad_glCopyBufferSubData = NULL;
PFNGLCOPYIMAGESUBDATAPROC glad_glCopyImageSubData = NULL;
PFNGLCOPYNAMEDBUFFERSUBDATAPROC glad_glCopyNamedBufferSubData = NULL;
PFNGLCOPYPIXELSPROC glad_glCopyPixels = NULL;
PFNGLCOPYTEXIMAGE1DPROC glad_glCopyTexImage1D = NULL;
PFNGLCOPYTEXIMAGE2DPROC glad_glCopyTexImage2D = NULL;
PFNGLCOPYTEXSUBIMAGE1DPROC glad_glCopyTexSubImage1D = NULL;
PFNGLCOPYTEXSUBIMAGE2DPROC glad_glCopyTexSubImage2D = NULL;
PFNGLCOPYTEXSUBIMAGE3DPROC glad_glCopyTexSubImage3D = NULL;
PFNGLCOPYTEXTURESUBIMAGE1DPROC glad_glCopyTextureSubImage1D = NULL;
PFNGLCOPYTEXTURESUBIMAGE2DPROC glad_glCopyTextureSubImage2D = NULL;
PFNGLCOPYTEXTURESUBIMAGE3DPROC glad_glCopyTextureSubImage3D = NULL;
PFNGLCREATEBUFFERSPROC glad_glCreateBuffers = NULL;
PFNGLCREATEFRAMEBUFFERSPROC glad_glCreateFramebuffers = NULL;
PFNGLCREATEPROGRAMPROC glad_glCreateProgram = NULL;
PFNGLCREATEPROGRAMPIPELINESPROC glad_glCreateProgramPipelines = NULL;
PFNGLCREATEQUERIESPROC glad_glCreateQueries = NULL;
PFNGLCREATERENDERBUFFERSPROC glad_glCreateRenderbuffers = NULL;
PFNGLCREATESAMPLERSPROC glad_glCreateSamplers = NULL;
PFNGLCREATESHADERPROC glad_glCreateShader = NULL;
PFNGLCREATESHADERPROGRAMVPROC glad_glCreateShaderProgramv = NULL;
PFNGLCREATETEXTURESPROC glad_glCreateTextures = NULL;
PFNGLCREATETRANSFORMFEEDBACKSPROC glad_glCreateTransformFeedbacks = NULL;
PFNGLCREATEVERTEXARRAYSPROC glad_glCreateVertexArrays = NULL;
PFNGLCULLFACEPROC glad_glCullFace = NULL;
PFNGLDEBUGMESSAGECALLBACKPROC glad_glDebugMessageCallback = NULL;
PFNGLDEBUGMESSAGECONTROLPROC glad_glDebugMessageControl = NULL;
PFNGLDEBUGMESSAGEINSERTPROC glad_glDebugMessageInsert = NULL;
PFNGLDELETEBUFFERSPROC glad_glDeleteBuffers = NULL;
PFNGLDELETEFRAMEBUFFERSPROC glad_glDeleteFramebuffers = NULL;
PFNGLDELETELISTSPROC glad_glDeleteLists = NULL;
PFNGLDELETEPROGRAMPROC glad_glDeleteProgram = NULL;
PFNGLDELETEPROGRAMPIPELINESPROC glad_glDeleteProgramPipelines = NULL;
PFNGLDELETEQUERIESPROC glad_glDeleteQueries = NULL;
PFNGLDELETERENDERBUFFERSPROC glad_glDeleteRenderbuffers = NULL;
PFNGLDELETESAMPLERSPROC glad_glDeleteSamplers = NULL;
PFNGLDELETESHADERPROC glad_glDeleteShader = NULL;
PFNGLDELETESYNCPROC glad_glDeleteSync = NULL;
PFNGLDELETETEXTURESPROC glad_glDeleteTextures = NULL;
PFNGLDELETETRANSFORMFEEDBACKSPROC glad_glDeleteTransformFeedbacks = NULL;
PFNGLDELETEVERTEXARRAYSPROC glad_glDeleteVertexArrays = NULL;
PFNGLDEPTHFUNCPROC glad_glDepthFunc = NULL;
PFNGLDEPTHMASKPROC glad_glDepthMask = NULL;
PFNGLDEPTHRANGEPROC glad_glDepthRange = NULL;
PFNGLDEPTHRANGEARRAYVPROC glad_glDepthRangeArrayv = NULL;
PFNGLDEPTHRANGEINDEXEDPROC glad_glDepthRangeIndexed = NULL;
PFNGLDEPTHRANGEFPROC glad_glDepthRangef = NULL;
PFNGLDETACHSHADERPROC glad_glDetachShader = NULL;
PFNGLDISABLEPROC glad_glDisable = NULL;
PFNGLDISABLECLIENTSTATEPROC glad_glDisableClientState = NULL;
PFNGLDISABLEVERTEXARRAYATTRIBPROC glad_glDisableVertexArrayAttrib = NULL;
PFNGLDISABLEVERTEXATTRIBARRAYPROC glad_glDisableVertexAttribArray = NULL;
PFNGLDISABLEIPROC glad_glDisablei = NULL;
PFNGLDISPATCHCOMPUTEPROC glad_glDispatchCompute = NULL;
PFNGLDISPATCHCOMPUTEINDIRECTPROC glad_glDispatchComputeIndirect = NULL;
PFNGLDRAWARRAYSPROC glad_glDrawArrays = NULL;
PFNGLDRAWARRAYSINDIRECTPROC glad_glDrawArraysIndirect = NULL;
PFNGLDRAWARRAYSINSTANCEDPROC glad_glDrawArraysInstanced = NULL;
PFNGLDRAWARRAYSINSTANCEDBASEINSTANCEPROC glad_glDrawArraysInstancedBaseInstance = NULL;
PFNGLDRAWBUFFERPROC glad_glDrawBuffer = NULL;
PFNGLDRAWBUFFERSPROC glad_glDrawBuffers = NULL;
PFNGLDRAWELEMENTSPROC glad_glDrawElements = NULL;
PFNGLDRAWELEMENTSBASEVERTEXPROC glad_glDrawElementsBaseVertex = NULL;
PFNGLDRAWELEMENTSINDIRECTPROC glad_glDrawElementsIndirect = NULL;
PFNGLDRAWELEMENTSINSTANCEDPROC glad_glDrawElementsInstanced = NULL;
PFNGLDRAWELEMENTSINSTANCEDBASEINSTANCEPROC glad_glDrawElementsInstancedBaseInstance = NULL;
PFNGLDRAWELEMENTSINSTANCEDBASEVERTEXPROC glad_glDrawElementsInstancedBaseVertex = NULL;
PFNGLDRAWELEMENTSINSTANCEDBASEVERTEXBASEINSTANCEPROC glad_glDrawElementsInstancedBaseVertexBaseInstance = NULL;
PFNGLDRAWPIXELSPROC glad_glDrawPixels = NULL;
PFNGLDRAWRANGEELEMENTSPROC glad_glDrawRangeElements = NULL;
PFNGLDRAWRANGEELEMENTSBASEVERTEXPROC glad_glDrawRangeElementsBaseVertex = NULL;
PFNGLDRAWTRANSFORMFEEDBACKPROC glad_glDrawTransformFeedback = NULL;
PFNGLDRAWTRANSFORMFEEDBACKINSTANCEDPROC glad_glDrawTransformFeedbackInstanced = NULL;
PFNGLDRAWTRANSFORMFEEDBACKSTREAMPROC glad_glDrawTransformFeedbackStream = NULL;
PFNGLDRAWTRANSFORMFEEDBACKSTREAMINSTANCEDPROC glad_glDrawTransformFeedbackStreamInstanced = NULL;
PFNGLEDGEFLAGPROC glad_glEdgeFlag = NULL;
PFNGLEDGEFLAGPOINTERPROC glad_glEdgeFlagPointer = NULL;
PFNGLEDGEFLAGVPROC glad_glEdgeFlagv = NULL;
PFNGLENABLEPROC glad_glEnable = NULL;
PFNGLENABLECLIENTSTATEPROC glad_glEnableClientState = NULL;
PFNGLENABLEVERTEXARRAYATTRIBPROC glad_glEnableVertexArrayAttrib = NULL;
PFNGLENABLEVERTEXATTRIBARRAYPROC glad_glEnableVertexAttribArray = NULL;
PFNGLENABLEIPROC glad_glEnablei = NULL;
PFNGLENDPROC glad_glEnd = NULL;
PFNGLENDCONDITIONALRENDERPROC glad_glEndConditionalRender = NULL;
PFNGLENDLISTPROC glad_glEndList = NULL;
PFNGLENDQUERYPROC glad_glEndQuery = NULL;
PFNGLENDQUERYINDEXEDPROC glad_glEndQueryIndexed = NULL;
PFNGLENDTRANSFORMFEEDBACKPROC glad_glEndTransformFeedback = NULL;
PFNGLEVALCOORD1DPROC glad_glEvalCoord1d = NULL;
PFNGLEVALCOORD1DVPROC glad_glEvalCoord1dv = NULL;
PFNGLEVALCOORD1FPROC glad_glEvalCoord1f = NULL;
PFNGLEVALCOORD1FVPROC glad_glEvalCoord1fv = NULL;
PFNGLEVALCOORD2DPROC glad_glEvalCoord2d = NULL;
PFNGLEVALCOORD2DVPROC glad_glEvalCoord2dv = NULL;
PFNGLEVALCOORD2FPROC glad_glEvalCoord2f = NULL;
PFNGLEVALCOORD2FVPROC glad_glEvalCoord2fv = NULL;
PFNGLEVALMESH1PROC glad_glEvalMesh1 = NULL;
PFNGLEVALMESH2PROC glad_glEvalMesh2 = NULL;
PFNGLEVALPOINT1PROC glad_glEvalPoint1 = NULL;
PFNGLEVALPOINT2PROC glad_glEvalPoint2 = NULL;
PFNGLFEEDBACKBUFFERPROC glad_glFeedbackBuffer = NULL;
PFNGLFENCESYNCPROC glad_glFenceSync = NULL;
PFNGLFINISHPROC glad_glFinish = NULL;
PFNGLFLUSHPROC glad_glFlush = NULL;
PFNGLFLUSHMAPPEDBUFFERRANGEPROC glad_glFlushMappedBufferRange = NULL;
PFNGLFLUSHMAPPEDNAMEDBUFFERRANGEPROC glad_glFlushMappedNamedBufferRange = NULL;
PFNGLFOGCOORDPOINTERPROC glad_glFogCoordPointer = NULL;
PFNGLFOGCOORDDPROC glad_glFogCoordd = NULL;
PFNGLFOGCOORDDVPROC glad_glFogCoorddv = NULL;
PFNGLFOGCOORDFPROC glad_glFogCoordf = NULL;
PFNGLFOGCOORDFVPROC glad_glFogCoordfv = NULL;
PFNGLFOGFPROC glad_glFogf = NULL;
PFNGLFOGFVPROC glad_glFogfv = NULL;
PFNGLFOGIPROC glad_glFogi = NULL;
PFNGLFOGIVPROC glad_glFogiv = NULL;
PFNGLFRAMEBUFFERPARAMETERIPROC glad_glFramebufferParameteri = NULL;
PFNGLFRAMEBUFFERRENDERBUFFERPROC glad_glFramebufferRenderbuffer = NULL;
PFNGLFRAMEBUFFERTEXTUREPROC glad_glFramebufferTexture = NULL;
PFNGLFRAMEBUFFERTEXTURE1DPROC glad_glFramebufferTexture1D = NULL;
PFNGLFRAMEBUFFERTEXTURE2DPROC glad_glFramebufferTexture2D = NULL;
PFNGLFRAMEBUFFERTEXTURE3DPROC glad_glFramebufferTexture3D = NULL;
PFNGLFRAMEBUFFERTEXTURELAYERPROC glad_glFramebufferTextureLayer = NULL;
PFNGLFRONTFACEPROC glad_glFrontFace = NULL;
PFNGLFRUSTUMPROC glad_glFrustum = NULL;
PFNGLGENBUFFERSPROC glad_glGenBuffers = NULL;
PFNGLGENFRAMEBUFFERSPROC glad_glGenFramebuffers = NULL;
PFNGLGENLISTSPROC glad_glGenLists = NULL;
PFNGLGENPROGRAMPIPELINESPROC glad_glGenProgramPipelines = NULL;
PFNGLGENQUERIESPROC glad_glGenQueries = NULL;
PFNGLGENRENDERBUFFERSPROC glad_glGenRenderbuffers = NULL;
PFNGLGENSAMPLERSPROC glad_glGenSamplers = NULL;
PFNGLGENTEXTURESPROC glad_glGenTextures = NULL;
PFNGLGENTRANSFORMFEEDBACKSPROC glad_glGenTransformFeedbacks = NULL;
PFNGLGENVERTEXARRAYSPROC glad_glGenVertexArrays = NULL;
PFNGLGENERATEMIPMAPPROC glad_glGenerateMipmap = NULL;
PFNGLGENERATETEXTUREMIPMAPPROC glad_glGenerateTextureMipmap = NULL;
PFNGLGETACTIVEATOMICCOUNTERBUFFERIVPROC glad_glGetActiveAtomicCounterBufferiv = NULL;
PFNGLGETACTIVEATTRIBPROC glad_glGetActiveAttrib = NULL;
PFNGLGETACTIVESUBROUTINENAMEPROC glad_glGetActiveSubroutineName = NULL;
PFNGLGETACTIVESUBROUTINEUNIFORMNAMEPROC glad_glGetActiveSubroutineUniformName = NULL;
PFNGLGETACTIVESUBROUTINEUNIFORMIVPROC glad_glGetActiveSubroutineUniformiv = NULL;
PFNGLGETACTIVEUNIFORMPROC glad_glGetActiveUniform = NULL;
PFNGLGETACTIVEUNIFORMBLOCKNAMEPROC glad_glGetActiveUniformBlockName = NULL;
PFNGLGETACTIVEUNIFORMBLOCKIVPROC glad_glGetActiveUniformBlockiv = NULL;
PFNGLGETACTIVEUNIFORMNAMEPROC glad_glGetActiveUniformName = NULL;
PFNGLGETACTIVEUNIFORMSIVPROC glad_glGetActiveUniformsiv = NULL;
PFNGLGETATTACHEDSHADERSPROC glad_glGetAttachedShaders = NULL;
PFNGLGETATTRIBLOCATIONPROC glad_glGetAttribLocation = NULL;
PFNGLGETBOOLEANI_VPROC glad_glGetBooleani_v = NULL;
PFNGLGETBOOLEANVPROC glad_glGetBooleanv = NULL;
PFNGLGETBUFFERPARAMETERI64VPROC glad_glGetBufferParameteri64v = NULL;
PFNGLGETBUFFERPARAMETERIVPROC glad_glGetBufferParameteriv = NULL;
PFNGLGETBUFFERPOINTERVPROC glad_glGetBufferPointerv = NULL;
PFNGLGETBUFFERSUBDATAPROC glad_glGetBufferSubData = NULL;
PFNGLGETCLIPPLANEPROC glad_glGetClipPlane = NULL;
PFNGLGETCOMPRESSEDTEXIMAGEPROC glad_glGetCompressedTexImage = NULL;
PFNGLGETCOMPRESSEDTEXTUREIMAGEPROC glad_glGetCompressedTextureImage = NULL;
PFNGLGETCOMPRESSEDTEXTURESUBIMAGEPROC glad_glGetCompressedTextureSubImage = NULL;
PFNGLGETDEBUGMESSAGELOGPROC glad_glGetDebugMessageLog = NULL;
PFNGLGETDOUBLEI_VPROC glad_glGetDoublei_v = NULL;
PFNGLGETDOUBLEVPROC glad_glGetDoublev = NULL;
PFNGLGETERRORPROC glad_glGetError = NULL;
PFNGLGETFLOATI_VPROC glad_glGetFloati_v = NULL;
PFNGLGETFLOATVPROC glad_glGetFloatv = NULL;
PFNGLGETFRAGDATAINDEXPROC glad_glGetFragDataIndex = NULL;
PFNGLGETFRAGDATALOCATIONPROC glad_glGetFragDataLocation = NULL;
PFNGLGETFRAMEBUFFERATTACHMENTPARAMETERIVPROC glad_glGetFramebufferAttachmentParameteriv = NULL;
PFNGLGETFRAMEBUFFERPARAMETERIVPROC glad_glGetFramebufferParameteriv = NULL;
PFNGLGETGRAPHICSRESETSTATUSPROC glad_glGetGraphicsResetStatus = NULL;
PFNGLGETINTEGER64I_VPROC glad_glGetInteger64i_v = NULL;
PFNGLGETINTEGER64VPROC glad_glGetInteger64v = NULL;
PFNGLGETINTEGERI_VPROC glad_glGetIntegeri_v = NULL;
PFNGLGETINTEGERVPROC glad_glGetIntegerv = NULL;
PFNGLGETINTERNALFORMATI64VPROC glad_glGetInternalformati64v = NULL;
PFNGLGETINTERNALFORMATIVPROC glad_glGetInternalformativ = NULL;
PFNGLGETLIGHTFVPROC glad_glGetLightfv = NULL;
PFNGLGETLIGHTIVPROC glad_glGetLightiv = NULL;
PFNGLGETMAPDVPROC glad_glGetMapdv = NULL;
PFNGLGETMAPFVPROC glad_glGetMapfv = NULL;
PFNGLGETMAPIVPROC glad_glGetMapiv = NULL;
PFNGLGETMATERIALFVPROC glad_glGetMaterialfv = NULL;
PFNGLGETMATERIALIVPROC glad_glGetMaterialiv = NULL;
PFNGLGETMULTISAMPLEFVPROC glad_glGetMultisamplefv = NULL;
PFNGLGETNAMEDBUFFERPARAMETERI64VPROC glad_glGetNamedBufferParameteri64v = NULL;
PFNGLGETNAMEDBUFFERPARAMETERIVPROC glad_glGetNamedBufferParameteriv = NULL;
PFNGLGETNAMEDBUFFERPOINTERVPROC glad_glGetNamedBufferPointerv = NULL;
PFNGLGETNAMEDBUFFERSUBDATAPROC glad_glGetNamedBufferSubData = NULL;
PFNGLGETNAMEDFRAMEBUFFERATTACHMENTPARAMETERIVPROC glad_glGetNamedFramebufferAttachmentParameteriv = NULL;
PFNGLGETNAMEDFRAMEBUFFERPARAMETERIVPROC glad_glGetNamedFramebufferParameteriv = NULL;
PFNGLGETNAMEDRENDERBUFFERPARAMETERIVPROC glad_glGetNamedRenderbufferParameteriv = NULL;
PFNGLGETOBJECTLABELPROC glad_glGetObjectLabel = NULL;
PFNGLGETOBJECTPTRLABELPROC glad_glGetObjectPtrLabel = NULL;
PFNGLGETPIXELMAPFVPROC glad_glGetPixelMapfv = NULL;
PFNGLGETPIXELMAPUIVPROC glad_glGetPixelMapuiv = NULL;
PFNGLGETPIXELMAPUSVPROC glad_glGetPixelMapusv = NULL;
PFNGLGETPOINTERVPROC glad_glGetPointerv = NULL;
PFNGLGETPOLYGONSTIPPLEPROC glad_glGetPolygonStipple = NULL;
PFNGLGETPROGRAMBINARYPROC glad_glGetProgramBinary = NULL;
PFNGLGETPROGRAMINFOLOGPROC glad_glGetProgramInfoLog = NULL;
PFNGLGETPROGRAMINTERFACEIVPROC glad_glGetProgramInterfaceiv = NULL;
PFNGLGETPROGRAMPIPELINEINFOLOGPROC glad_glGetProgramPipelineInfoLog = NULL;
PFNGLGETPROGRAMPIPELINEIVPROC glad_glGetProgramPipelineiv = NULL;
PFNGLGETPROGRAMRESOURCEINDEXPROC glad_glGetProgramResourceIndex = NULL;
PFNGLGETPROGRAMRESOURCELOCATIONPROC glad_glGetProgramResourceLocation = NULL;
PFNGLGETPROGRAMRESOURCELOCATIONINDEXPROC glad_glGetProgramResourceLocationIndex = NULL;
PFNGLGETPROGRAMRESOURCENAMEPROC glad_glGetProgramResourceName = NULL;
PFNGLGETPROGRAMRESOURCEIVPROC glad_glGetProgramResourceiv = NULL;
PFNGLGETPROGRAMSTAGEIVPROC glad_glGetProgramStageiv = NULL;
PFNGLGETPROGRAMIVPROC glad_glGetProgramiv = NULL;
PFNGLGETQUERYBUFFEROBJECTI64VPROC glad_glGetQueryBufferObjecti64v = NULL;
PFNGLGETQUERYBUFFEROBJECTIVPROC glad_glGetQueryBufferObjectiv = NULL;
PFNGLGETQUERYBUFFEROBJECTUI64VPROC glad_glGetQueryBufferObjectui64v = NULL;
PFNGLGETQUERYBUFFEROBJECTUIVPROC glad_glGetQueryBufferObjectuiv = NULL;
PFNGLGETQUERYINDEXEDIVPROC glad_glGetQueryIndexediv = NULL;
PFNGLGETQUERYOBJECTI64VPROC glad_glGetQueryObjecti64v = NULL;
PFNGLGETQUERYOBJECTIVPROC glad_glGetQueryObjectiv = NULL;
PFNGLGETQUERYOBJECTUI64VPROC glad_glGetQueryObjectui64v = NULL;
PFNGLGETQUERYOBJECTUIVPROC glad_glGetQueryObjectuiv = NULL;
PFNGLGETQUERYIVPROC glad_glGetQueryiv = NULL;
PFNGLGETRENDERBUFFERPARAMETERIVPROC glad_glGetRenderbufferParameteriv = NULL;
PFNGLGETSAMPLERPARAMETERIIVPROC glad_glGetSamplerParameterIiv = NULL;
PFNGLGETSAMPLERPARAMETERIUIVPROC glad_glGetSamplerParameterIuiv = NULL;
PFNGLGETSAMPLERPARAMETERFVPROC glad_glGetSamplerParameterfv = NULL;
PFNGLGETSAMPLERPARAMETERIVPROC glad_glGetSamplerParameteriv = NULL;
PFNGLGETSHADERINFOLOGPROC glad_glGetShaderInfoLog = NULL;
PFNGLGETSHADERPRECISIONFORMATPROC glad_glGetShaderPrecisionFormat = NULL;
PFNGLGETSHADERSOURCEPROC glad_glGetShaderSource = NULL;
PFNGLGETSHADERIVPROC glad_glGetShaderiv = NULL;
PFNGLGETSTRINGPROC glad_glGetString = NULL;
PFNGLGETSTRINGIPROC glad_glGetStringi = NULL;
PFNGLGETSUBROUTINEINDEXPROC glad_glGetSubroutineIndex = NULL;
PFNGLGETSUBROUTINEUNIFORMLOCATIONPROC glad_glGetSubroutineUniformLocation = NULL;
PFNGLGETSYNCIVPROC glad_glGetSynciv = NULL;
PFNGLGETTEXENVFVPROC glad_glGetTexEnvfv = NULL;
PFNGLGETTEXENVIVPROC glad_glGetTexEnviv = NULL;
PFNGLGETTEXGENDVPROC glad_glGetTexGendv = NULL;
PFNGLGETTEXGENFVPROC glad_glGetTexGenfv = NULL;
PFNGLGETTEXGENIVPROC glad_glGetTexGeniv = NULL;
PFNGLGETTEXIMAGEPROC glad_glGetTexImage = NULL;
PFNGLGETTEXLEVELPARAMETERFVPROC glad_glGetTexLevelParameterfv = NULL;
PFNGLGETTEXLEVELPARAMETERIVPROC glad_glGetTexLevelParameteriv = NULL;
PFNGLGETTEXPARAMETERIIVPROC glad_glGetTexParameterIiv = NULL;
PFNGLGETTEXPARAMETERIUIVPROC glad_glGetTexParameterIuiv = NULL;
PFNGLGETTEXPARAMETERFVPROC glad_glGetTexParameterfv = NULL;
PFNGLGETTEXPARAMETERIVPROC glad_glGetTexParameteriv = NULL;
PFNGLGETTEXTUREIMAGEPROC glad_glGetTextureImage = NULL;
PFNGLGETTEXTURELEVELPARAMETERFVPROC glad_glGetTextureLevelParameterfv = NULL;
PFNGLGETTEXTURELEVELPARAMETERIVPROC glad_glGetTextureLevelParameteriv = NULL;
PFNGLGETTEXTUREPARAMETERIIVPROC glad_glGetTextureParameterIiv = NULL;
PFNGLGETTEXTUREPARAMETERIUIVPROC glad_glGetTextureParameterIuiv = NULL;
PFNGLGETTEXTUREPARAMETERFVPROC glad_glGetTextureParameterfv = NULL;
PFNGLGETTEXTUREPARAMETERIVPROC glad_glGetTextureParameteriv = NULL;
PFNGLGETTEXTURESUBIMAGEPROC glad_glGetTextureSubImage = NULL;
PFNGLGETTRANSFORMFEEDBACKVARYINGPROC glad_glGetTransformFeedbackVarying = NULL;
PFNGLGETTRANSFORMFEEDBACKI64_VPROC glad_glGetTransformFeedbacki64_v = NULL;
PFNGLGETTRANSFORMFEEDBACKI_VPROC glad_glGetTransformFeedbacki_v = NULL;
PFNGLGETTRANSFORMFEEDBACKIVPROC glad_glGetTransformFeedbackiv = NULL;
PFNGLGETUNIFORMBLOCKINDEXPROC glad_glGetUniformBlockIndex = NULL;
PFNGLGETUNIFORMINDICESPROC glad_glGetUniformIndices = NULL;
PFNGLGETUNIFORMLOCATIONPROC glad_glGetUniformLocation = NULL;
PFNGLGETUNIFORMSUBROUTINEUIVPROC glad_glGetUniformSubroutineuiv = NULL;
PFNGLGETUNIFORMDVPROC glad_glGetUniformdv = NULL;
PFNGLGETUNIFORMFVPROC glad_glGetUniformfv = NULL;
PFNGLGETUNIFORMIVPROC glad_glGetUniformiv = NULL;
PFNGLGETUNIFORMUIVPROC glad_glGetUniformuiv = NULL;
PFNGLGETVERTEXARRAYINDEXED64IVPROC glad_glGetVertexArrayIndexed64iv = NULL;
PFNGLGETVERTEXARRAYINDEXEDIVPROC glad_glGetVertexArrayIndexediv = NULL;
PFNGLGETVERTEXARRAYIVPROC glad_glGetVertexArrayiv = NULL;
PFNGLGETVERTEXATTRIBIIVPROC glad_glGetVertexAttribIiv = NULL;
PFNGLGETVERTEXATTRIBIUIVPROC glad_glGetVertexAttribIuiv = NULL;
PFNGLGETVERTEXATTRIBLDVPROC glad_glGetVertexAttribLdv = NULL;
PFNGLGETVERTEXATTRIBPOINTERVPROC glad_glGetVertexAttribPointerv = NULL;
PFNGLGETVERTEXATTRIBDVPROC glad_glGetVertexAttribdv = NULL;
PFNGLGETVERTEXATTRIBFVPROC glad_glGetVertexAttribfv = NULL;
PFNGLGETVERTEXATTRIBIVPROC glad_glGetVertexAttribiv = NULL;
PFNGLGETNCOLORTABLEPROC glad_glGetnColorTable = NULL;
PFNGLGETNCOMPRESSEDTEXIMAGEPROC glad_glGetnCompressedTexImage = NULL;
PFNGLGETNCONVOLUTIONFILTERPROC glad_glGetnConvolutionFilter = NULL;
PFNGLGETNHISTOGRAMPROC glad_glGetnHistogram = NULL;
PFNGLGETNMAPDVPROC glad_glGetnMapdv = NULL;
PFNGLGETNMAPFVPROC glad_glGetnMapfv = NULL;
PFNGLGETNMAPIVPROC glad_glGetnMapiv = NULL;
PFNGLGETNMINMAXPROC glad_glGetnMinmax = NULL;
PFNGLGETNPIXELMAPFVPROC glad_glGetnPixelMapfv = NULL;
PFNGLGETNPIXELMAPUIVPROC glad_glGetnPixelMapuiv = NULL;
PFNGLGETNPIXELMAPUSVPROC glad_glGetnPixelMapusv = NULL;
PFNGLGETNPOLYGONSTIPPLEPROC glad_glGetnPolygonStipple = NULL;
PFNGLGETNSEPARABLEFILTERPROC glad_glGetnSeparableFilter = NULL;
PFNGLGETNTEXIMAGEPROC glad_glGetnTexImage = NULL;
PFNGLGETNUNIFORMDVPROC glad_glGetnUniformdv = NULL;
PFNGLGETNUNIFORMFVPROC glad_glGetnUniformfv = NULL;
PFNGLGETNUNIFORMIVPROC glad_glGetnUniformiv = NULL;
PFNGLGETNUNIFORMUIVPROC glad_glGetnUniformuiv = NULL;
PFNGLHINTPROC glad_glHint = NULL;
PFNGLINDEXMASKPROC glad_glIndexMask = NULL;
PFNGLINDEXPOINTERPROC glad_glIndexPointer = NULL;
PFNGLINDEXDPROC glad_glIndexd = NULL;
PFNGLINDEXDVPROC glad_glIndexdv = NULL;
PFNGLINDEXFPROC glad_glIndexf = NULL;
PFNGLINDEXFVPROC glad_glIndexfv = NULL;
PFNGLINDEXIPROC glad_glIndexi = NULL;
PFNGLINDEXIVPROC glad_glIndexiv = NULL;
PFNGLINDEXSPROC glad_glIndexs = NULL;
PFNGLINDEXSVPROC glad_glIndexsv = NULL;
PFNGLINDEXUBPROC glad_glIndexub = NULL;
PFNGLINDEXUBVPROC glad_glIndexubv = NULL;
PFNGLINITNAMESPROC glad_glInitNames = NULL;
PFNGLINTERLEAVEDARRAYSPROC glad_glInterleavedArrays = NULL;
PFNGLINVALIDATEBUFFERDATAPROC glad_glInvalidateBufferData = NULL;
PFNGLINVALIDATEBUFFERSUBDATAPROC glad_glInvalidateBufferSubData = NULL;
PFNGLINVALIDATEFRAMEBUFFERPROC glad_glInvalidateFramebuffer = NULL;
PFNGLINVALIDATENAMEDFRAMEBUFFERDATAPROC glad_glInvalidateNamedFramebufferData = NULL;
PFNGLINVALIDATENAMEDFRAMEBUFFERSUBDATAPROC glad_glInvalidateNamedFramebufferSubData = NULL;
PFNGLINVALIDATESUBFRAMEBUFFERPROC glad_glInvalidateSubFramebuffer = NULL;
PFNGLINVALIDATETEXIMAGEPROC glad_glInvalidateTexImage = NULL;
PFNGLINVALIDATETEXSUBIMAGEPROC glad_glInvalidateTexSubImage = NULL;
PFNGLISBUFFERPROC glad_glIsBuffer = NULL;
PFNGLISENABLEDPROC glad_glIsEnabled = NULL;
PFNGLISENABLEDIPROC glad_glIsEnabledi = NULL;
PFNGLISFRAMEBUFFERPROC glad_glIsFramebuffer = NULL;
PFNGLISLISTPROC glad_glIsList = NULL;
PFNGLISPROGRAMPROC glad_glIsProgram = NULL;
PFNGLISPROGRAMPIPELINEPROC glad_glIsProgramPipeline = NULL;
PFNGLISQUERYPROC glad_glIsQuery = NULL;
PFNGLISRENDERBUFFERPROC glad_glIsRenderbuffer = NULL;
PFNGLISSAMPLERPROC glad_glIsSampler = NULL;
PFNGLISSHADERPROC glad_glIsShader = NULL;
PFNGLISSYNCPROC glad_glIsSync = NULL;
PFNGLISTEXTUREPROC glad_glIsTexture = NULL;
PFNGLISTRANSFORMFEEDBACKPROC glad_glIsTransformFeedback = NULL;
PFNGLISVERTEXARRAYPROC glad_glIsVertexArray = NULL;
PFNGLLIGHTMODELFPROC glad_glLightModelf = NULL;
PFNGLLIGHTMODELFVPROC glad_glLightModelfv = NULL;
PFNGLLIGHTMODELIPROC glad_glLightModeli = NULL;
PFNGLLIGHTMODELIVPROC glad_glLightModeliv = NULL;
PFNGLLIGHTFPROC glad_glLightf = NULL;
PFNGLLIGHTFVPROC glad_glLightfv = NULL;
PFNGLLIGHTIPROC glad_glLighti = NULL;
PFNGLLIGHTIVPROC glad_glLightiv = NULL;
PFNGLLINESTIPPLEPROC glad_glLineStipple = NULL;
PFNGLLINEWIDTHPROC glad_glLineWidth = NULL;
PFNGLLINKPROGRAMPROC glad_glLinkProgram = NULL;
PFNGLLISTBASEPROC glad_glListBase = NULL;
PFNGLLOADIDENTITYPROC glad_glLoadIdentity = NULL;
PFNGLLOADMATRIXDPROC glad_glLoadMatrixd = NULL;
PFNGLLOADMATRIXFPROC glad_glLoadMatrixf = NULL;
PFNGLLOADNAMEPROC glad_glLoadName = NULL;
PFNGLLOADTRANSPOSEMATRIXDPROC glad_glLoadTransposeMatrixd = NULL;
PFNGLLOADTRANSPOSEMATRIXFPROC glad_glLoadTransposeMatrixf = NULL;
PFNGLLOGICOPPROC glad_glLogicOp = NULL;
PFNGLMAP1DPROC glad_glMap1d = NULL;
PFNGLMAP1FPROC glad_glMap1f = NULL;
PFNGLMAP2DPROC glad_glMap2d = NULL;
PFNGLMAP2FPROC glad_glMap2f = NULL;
PFNGLMAPBUFFERPROC glad_glMapBuffer = NULL;
PFNGLMAPBUFFERRANGEPROC glad_glMapBufferRange = NULL;
PFNGLMAPGRID1DPROC glad_glMapGrid1d = NULL;
PFNGLMAPGRID1FPROC glad_glMapGrid1f = NULL;
PFNGLMAPGRID2DPROC glad_glMapGrid2d = NULL;
PFNGLMAPGRID2FPROC glad_glMapGrid2f = NULL;
PFNGLMAPNAMEDBUFFERPROC glad_glMapNamedBuffer = NULL;
PFNGLMAPNAMEDBUFFERRANGEPROC glad_glMapNamedBufferRange = NULL;
PFNGLMATERIALFPROC glad_glMaterialf = NULL;
PFNGLMATERIALFVPROC glad_glMaterialfv = NULL;
PFNGLMATERIALIPROC glad_glMateriali = NULL;
PFNGLMATERIALIVPROC glad_glMaterialiv = NULL;
PFNGLMATRIXMODEPROC glad_glMatrixMode = NULL;
PFNGLMEMORYBARRIERPROC glad_glMemoryBarrier = NULL;
PFNGLMEMORYBARRIERBYREGIONPROC glad_glMemoryBarrierByRegion = NULL;
PFNGLMINSAMPLESHADINGPROC glad_glMinSampleShading = NULL;
PFNGLMULTMATRIXDPROC glad_glMultMatrixd = NULL;
PFNGLMULTMATRIXFPROC glad_glMultMatrixf = NULL;
PFNGLMULTTRANSPOSEMATRIXDPROC glad_glMultTransposeMatrixd = NULL;
PFNGLMULTTRANSPOSEMATRIXFPROC glad_glMultTransposeMatrixf = NULL;
PFNGLMULTIDRAWARRAYSPROC glad_glMultiDrawArrays = NULL;
PFNGLMULTIDRAWARRAYSINDIRECTPROC glad_glMultiDrawArraysIndirect = NULL;
PFNGLMULTIDRAWARRAYSINDIRECTCOUNTPROC glad_glMultiDrawArraysIndirectCount = NULL;
PFNGLMULTIDRAWELEMENTSPROC glad_glMultiDrawElements = NULL;
PFNGLMULTIDRAWELEMENTSBASEVERTEXPROC glad_glMultiDrawElementsBaseVertex = NULL;
PFNGLMULTIDRAWELEMENTSINDIRECTPROC glad_glMultiDrawElementsIndirect = NULL;
PFNGLMULTIDRAWELEMENTSINDIRECTCOUNTPROC glad_glMultiDrawElementsIndirectCount = NULL;
PFNGLMULTITEXCOORD1DPROC glad_glMultiTexCoord1d = NULL;
PFNGLMULTITEXCOORD1DVPROC glad_glMultiTexCoord1dv = NULL;
PFNGLMULTITEXCOORD1FPROC glad_glMultiTexCoord1f = NULL;
PFNGLMULTITEXCOORD1FVPROC glad_glMultiTexCoord1fv = NULL;
PFNGLMULTITEXCOORD1IPROC glad_glMultiTexCoord1i = NULL;
PFNGLMULTITEXCOORD1IVPROC glad_glMultiTexCoord1iv = NULL;
PFNGLMULTITEXCOORD1SPROC glad_glMultiTexCoord1s = NULL;
PFNGLMULTITEXCOORD1SVPROC glad_glMultiTexCoord1sv = NULL;
PFNGLMULTITEXCOORD2DPROC glad_glMultiTexCoord2d = NULL;
PFNGLMULTITEXCOORD2DVPROC glad_glMultiTexCoord2dv = NULL;
PFNGLMULTITEXCOORD2FPROC glad_glMultiTexCoord2f = NULL;
PFNGLMULTITEXCOORD2FVPROC glad_glMultiTexCoord2fv = NULL;
PFNGLMULTITEXCOORD2IPROC glad_glMultiTexCoord2i = NULL;
PFNGLMULTITEXCOORD2IVPROC glad_glMultiTexCoord2iv = NULL;
PFNGLMULTITEXCOORD2SPROC glad_glMultiTexCoord2s = NULL;
PFNGLMULTITEXCOORD2SVPROC glad_glMultiTexCoord2sv = NULL;
PFNGLMULTITEXCOORD3DPROC glad_glMultiTexCoord3d = NULL;
PFNGLMULTITEXCOORD3DVPROC glad_glMultiTexCoord3dv = NULL;
PFNGLMULTITEXCOORD3FPROC glad_glMultiTexCoord3f = NULL;
PFNGLMULTITEXCOORD3FVPROC glad_glMultiTexCoord3fv = NULL;
PFNGLMULTITEXCOORD3IPROC glad_glMultiTexCoord3i = NULL;
PFNGLMULTITEXCOORD3IVPROC glad_glMultiTexCoord3iv = NULL;
PFNGLMULTITEXCOORD3SPROC glad_glMultiTexCoord3s = NULL;
PFNGLMULTITEXCOORD3SVPROC glad_glMultiTexCoord3sv = NULL;
PFNGLMULTITEXCOORD4DPROC glad_glMultiTexCoord4d = NULL;
PFNGLMULTITEXCOORD4DVPROC glad_glMultiTexCoord4dv = NULL;
PFNGLMULTITEXCOORD4FPROC glad_glMultiTexCoord4f = NULL;
PFNGLMULTITEXCOORD4FVPROC glad_glMultiTexCoord4fv = NULL;
PFNGLMULTITEXCOORD4IPROC glad_glMultiTexCoord4i = NULL;
PFNGLMULTITEXCOORD4IVPROC glad_glMultiTexCoord4iv = NULL;
PFNGLMULTITEXCOORD4SPROC glad_glMultiTexCoord4s = NULL;
PFNGLMULTITEXCOORD4SVPROC glad_glMultiTexCoord4sv = NULL;
PFNGLMULTITEXCOORDP1UIPROC glad_glMultiTexCoordP1ui = NULL;
PFNGLMULTITEXCOORDP1UIVPROC glad_glMultiTexCoordP1uiv = NULL;
PFNGLMULTITEXCOORDP2UIPROC glad_glMultiTexCoordP2ui = NULL;
PFNGLMULTITEXCOORDP2UIVPROC glad_glMultiTexCoordP2uiv = NULL;
PFNGLMULTITEXCOORDP3UIPROC glad_glMultiTexCoordP3ui = NULL;
PFNGLMULTITEXCOORDP3UIVPROC glad_glMultiTexCoordP3uiv = NULL;
PFNGLMULTITEXCOORDP4UIPROC glad_glMultiTexCoordP4ui = NULL;
PFNGLMULTITEXCOORDP4UIVPROC glad_glMultiTexCoordP4uiv = NULL;
PFNGLNAMEDBUFFERDATAPROC glad_glNamedBufferData = NULL;
PFNGLNAMEDBUFFERSTORAGEPROC glad_glNamedBufferStorage = NULL;
PFNGLNAMEDBUFFERSUBDATAPROC glad_glNamedBufferSubData = NULL;
PFNGLNAMEDFRAMEBUFFERDRAWBUFFERPROC glad_glNamedFramebufferDrawBuffer = NULL;
PFNGLNAMEDFRAMEBUFFERDRAWBUFFERSPROC glad_glNamedFramebufferDrawBuffers = NULL;
PFNGLNAMEDFRAMEBUFFERPARAMETERIPROC glad_glNamedFramebufferParameteri = NULL;
PFNGLNAMEDFRAMEBUFFERREADBUFFERPROC glad_glNamedFramebufferReadBuffer = NULL;
PFNGLNAMEDFRAMEBUFFERRENDERBUFFERPROC glad_glNamedFramebufferRenderbuffer = NULL;
PFNGLNAMEDFRAMEBUFFERTEXTUREPROC glad_glNamedFramebufferTexture = NULL;
PFNGLNAMEDFRAMEBUFFERTEXTURELAYERPROC glad_glNamedFramebufferTextureLayer = NULL;
PFNGLNAMEDRENDERBUFFERSTORAGEPROC glad_glNamedRenderbufferStorage = NULL;
PFNGLNAMEDRENDERBUFFERSTORAGEMULTISAMPLEPROC glad_glNamedRenderbufferStorageMultisample = NULL;
PFNGLNEWLISTPROC glad_glNewList = NULL;
PFNGLNORMAL3BPROC glad_glNormal3b = NULL;
PFNGLNORMAL3BVPROC glad_glNormal3bv = NULL;
PFNGLNORMAL3DPROC glad_glNormal3d = NULL;
PFNGLNORMAL3DVPROC glad_glNormal3dv = NULL;
PFNGLNORMAL3FPROC glad_glNormal3f = NULL;
PFNGLNORMAL3FVPROC glad_glNormal3fv = NULL;
PFNGLNORMAL3IPROC glad_glNormal3i = NULL;
PFNGLNORMAL3IVPROC glad_glNormal3iv = NULL;
PFNGLNORMAL3SPROC glad_glNormal3s = NULL;
PFNGLNORMAL3SVPROC glad_glNormal3sv = NULL;
PFNGLNORMALP3UIPROC glad_glNormalP3ui = NULL;
PFNGLNORMALP3UIVPROC glad_glNormalP3uiv = NULL;
PFNGLNORMALPOINTERPROC glad_glNormalPointer = NULL;
PFNGLOBJECTLABELPROC glad_glObjectLabel = NULL;
PFNGLOBJECTPTRLABELPROC glad_glObjectPtrLabel = NULL;
PFNGLORTHOPROC glad_glOrtho = NULL;
PFNGLPASSTHROUGHPROC glad_glPassThrough = NULL;
PFNGLPATCHPARAMETERFVPROC glad_glPatchParameterfv = NULL;
PFNGLPATCHPARAMETERIPROC glad_glPatchParameteri = NULL;
PFNGLPAUSETRANSFORMFEEDBACKPROC glad_glPauseTransformFeedback = NULL;
PFNGLPIXELMAPFVPROC glad_glPixelMapfv = NULL;
PFNGLPIXELMAPUIVPROC glad_glPixelMapuiv = NULL;
PFNGLPIXELMAPUSVPROC glad_glPixelMapusv = NULL;
PFNGLPIXELSTOREFPROC glad_glPixelStoref = NULL;
PFNGLPIXELSTOREIPROC glad_glPixelStorei = NULL;
PFNGLPIXELTRANSFERFPROC glad_glPixelTransferf = NULL;
PFNGLPIXELTRANSFERIPROC glad_glPixelTransferi = NULL;
PFNGLPIXELZOOMPROC glad_glPixelZoom = NULL;
PFNGLPOINTPARAMETERFPROC glad_glPointParameterf = NULL;
PFNGLPOINTPARAMETERFVPROC glad_glPointParameterfv = NULL;
PFNGLPOINTPARAMETERIPROC glad_glPointParameteri = NULL;
PFNGLPOINTPARAMETERIVPROC glad_glPointParameteriv = NULL;
PFNGLPOINTSIZEPROC glad_glPointSize = NULL;
PFNGLPOLYGONMODEPROC glad_glPolygonMode = NULL;
PFNGLPOLYGONOFFSETPROC glad_glPolygonOffset = NULL;
PFNGLPOLYGONOFFSETCLAMPPROC glad_glPolygonOffsetClamp = NULL;
PFNGLPOLYGONSTIPPLEPROC glad_glPolygonStipple = NULL;
PFNGLPOPATTRIBPROC glad_glPopAttrib = NULL;
PFNGLPOPCLIENTATTRIBPROC glad_glPopClientAttrib = NULL;
PFNGLPOPDEBUGGROUPPROC glad_glPopDebugGroup = NULL;
PFNGLPOPMATRIXPROC glad_glPopMatrix = NULL;
PFNGLPOPNAMEPROC glad_glPopName = NULL;
PFNGLPRIMITIVERESTARTINDEXPROC glad_glPrimitiveRestartIndex = NULL;
PFNGLPRIORITIZETEXTURESPROC glad_glPrioritizeTextures = NULL;
PFNGLPROGRAMBINARYPROC glad_glProgramBinary = NULL;
PFNGLPROGRAMPARAMETERIPROC glad_glProgramParameteri = NULL;
PFNGLPROGRAMUNIFORM1DPROC glad_glProgramUniform1d = NULL;
PFNGLPROGRAMUNIFORM1DVPROC glad_glProgramUniform1dv = NULL;
PFNGLPROGRAMUNIFORM1FPROC glad_glProgramUniform1f = NULL;
PFNGLPROGRAMUNIFORM1FVPROC glad_glProgramUniform1fv = NULL;
PFNGLPROGRAMUNIFORM1IPROC glad_glProgramUniform1i = NULL;
PFNGLPROGRAMUNIFORM1IVPROC glad_glProgramUniform1iv = NULL;
PFNGLPROGRAMUNIFORM1UIPROC glad_glProgramUniform1ui = NULL;
PFNGLPROGRAMUNIFORM1UIVPROC glad_glProgramUniform1uiv = NULL;
PFNGLPROGRAMUNIFORM2DPROC glad_glProgramUniform2d = NULL;
PFNGLPROGRAMUNIFORM2DVPROC glad_glProgramUniform2dv = NULL;
PFNGLPROGRAMUNIFORM2FPROC glad_glProgramUniform2f = NULL;
PFNGLPROGRAMUNIFORM2FVPROC glad_glProgramUniform2fv = NULL;
PFNGLPROGRAMUNIFORM2IPROC glad_glProgramUniform2i = NULL;
PFNGLPROGRAMUNIFORM2IVPROC glad_glProgramUniform2iv = NULL;
PFNGLPROGRAMUNIFORM2UIPROC glad_glProgramUniform2ui = NULL;
PFNGLPROGRAMUNIFORM2UIVPROC glad_glProgramUniform2uiv = NULL;
PFNGLPROGRAMUNIFORM3DPROC glad_glProgramUniform3d = NULL;
PFNGLPROGRAMUNIFORM3DVPROC glad_glProgramUniform3dv = NULL;
PFNGLPROGRAMUNIFORM3FPROC glad_glProgramUniform3f = NULL;
PFNGLPROGRAMUNIFORM3FVPROC glad_glProgramUniform3fv = NULL;
PFNGLPROGRAMUNIFORM3IPROC glad_glProgramUniform3i = NULL;
PFNGLPROGRAMUNIFORM3IVPROC glad_glProgramUniform3iv = NULL;
PFNGLPROGRAMUNIFORM3UIPROC glad_glProgramUniform3ui = NULL;
PFNGLPROGRAMUNIFORM3UIVPROC glad_glProgramUniform3uiv = NULL;
PFNGLPROGRAMUNIFORM4DPROC glad_glProgramUniform4d = NULL;
PFNGLPROGRAMUNIFORM4DVPROC glad_glProgramUniform4dv = NULL;
PFNGLPROGRAMUNIFORM4FPROC glad_glProgramUniform4f = NULL;
PFNGLPROGRAMUNIFORM4FVPROC glad_glProgramUniform4fv = NULL;
PFNGLPROGRAMUNIFORM4IPROC glad_glProgramUniform4i = NULL;
PFNGLPROGRAMUNIFORM4IVPROC glad_glProgramUniform4iv = NULL;
PFNGLPROGRAMUNIFORM4UIPROC glad_glProgramUniform4ui = NULL;
PFNGLPROGRAMUNIFORM4UIVPROC glad_glProgramUniform4uiv = NULL;
PFNGLPROGRAMUNIFORMMATRIX2DVPROC glad_glProgramUniformMatrix2dv = NULL;
PFNGLPROGRAMUNIFORMMATRIX2FVPROC glad_glProgramUniformMatrix2fv = NULL;
PFNGLPROGRAMUNIFORMMATRIX2X3DVPROC glad_glProgramUniformMatrix2x3dv = NULL;
PFNGLPROGRAMUNIFORMMATRIX2X3FVPROC glad_glProgramUniformMatrix2x3fv = NULL;
PFNGLPROGRAMUNIFORMMATRIX2X4DVPROC glad_glProgramUniformMatrix2x4dv = NULL;
PFNGLPROGRAMUNIFORMMATRIX2X4FVPROC glad_glProgramUniformMatrix2x4fv = NULL;
PFNGLPROGRAMUNIFORMMATRIX3DVPROC glad_glProgramUniformMatrix3dv = NULL;
PFNGLPROGRAMUNIFORMMATRIX3FVPROC glad_glProgramUniformMatrix3fv = NULL;
PFNGLPROGRAMUNIFORMMATRIX3X2DVPROC glad_glProgramUniformMatrix3x2dv = NULL;
PFNGLPROGRAMUNIFORMMATRIX3X2FVPROC glad_glProgramUniformMatrix3x2fv = NULL;
PFNGLPROGRAMUNIFORMMATRIX3X4DVPROC glad_glProgramUniformMatrix3x4dv = NULL;
PFNGLPROGRAMUNIFORMMATRIX3X4FVPROC glad_glProgramUniformMatrix3x4fv = NULL;
PFNGLPROGRAMUNIFORMMATRIX4DVPROC glad_glProgramUniformMatrix4dv = NULL;
PFNGLPROGRAMUNIFORMMATRIX4FVPROC glad_glProgramUniformMatrix4fv = NULL;
PFNGLPROGRAMUNIFORMMATRIX4X2DVPROC glad_glProgramUniformMatrix4x2dv = NULL;
PFNGLPROGRAMUNIFORMMATRIX4X2FVPROC glad_glProgramUniformMatrix4x2fv = NULL;
PFNGLPROGRAMUNIFORMMATRIX4X3DVPROC glad_glProgramUniformMatrix4x3dv = NULL;
PFNGLPROGRAMUNIFORMMATRIX4X3FVPROC glad_glProgramUniformMatrix4x3fv = NULL;
PFNGLPROVOKINGVERTEXPROC glad_glProvokingVertex = NULL;
PFNGLPUSHATTRIBPROC glad_glPushAttrib = NULL;
PFNGLPUSHCLIENTATTRIBPROC glad_glPushClientAttrib = NULL;
PFNGLPUSHDEBUGGROUPPROC glad_glPushDebugGroup = NULL;
PFNGLPUSHMATRIXPROC glad_glPushMatrix = NULL;
PFNGLPUSHNAMEPROC glad_glPushName = NULL;
PFNGLQUERYCOUNTERPROC glad_glQueryCounter = NULL;
PFNGLRASTERPOS2DPROC glad_glRasterPos2d = NULL;
PFNGLRASTERPOS2DVPROC glad_glRasterPos2dv = NULL;
PFNGLRASTERPOS2FPROC glad_glRasterPos2f = NULL;
PFNGLRASTERPOS2FVPROC glad_glRasterPos2fv = NULL;
PFNGLRASTERPOS2IPROC glad_glRasterPos2i = NULL;
PFNGLRASTERPOS2IVPROC glad_glRasterPos2iv = NULL;
PFNGLRASTERPOS2SPROC glad_glRasterPos2s = NULL;
PFNGLRASTERPOS2SVPROC glad_glRasterPos2sv = NULL;
PFNGLRASTERPOS3DPROC glad_glRasterPos3d = NULL;
PFNGLRASTERPOS3DVPROC glad_glRasterPos3dv = NULL;
PFNGLRASTERPOS3FPROC glad_glRasterPos3f = NULL;
PFNGLRASTERPOS3FVPROC glad_glRasterPos3fv = NULL;
PFNGLRASTERPOS3IPROC glad_glRasterPos3i = NULL;
PFNGLRASTERPOS3IVPROC glad_glRasterPos3iv = NULL;
PFNGLRASTERPOS3SPROC glad_glRasterPos3s = NULL;
PFNGLRASTERPOS3SVPROC glad_glRasterPos3sv = NULL;
PFNGLRASTERPOS4DPROC glad_glRasterPos4d = NULL;
PFNGLRASTERPOS4DVPROC glad_glRasterPos4dv = NULL;
PFNGLRASTERPOS4FPROC glad_glRasterPos4f = NULL;
PFNGLRASTERPOS4FVPROC glad_glRasterPos4fv = NULL;
PFNGLRASTERPOS4IPROC glad_glRasterPos4i = NULL;
PFNGLRASTERPOS4IVPROC glad_glRasterPos4iv = NULL;
PFNGLRASTERPOS4SPROC glad_glRasterPos4s = NULL;
PFNGLRASTERPOS4SVPROC glad_glRasterPos4sv = NULL;
PFNGLREADBUFFERPROC glad_glReadBuffer = NULL;
PFNGLREADPIXELSPROC glad_glReadPixels = NULL;
PFNGLREADNPIXELSPROC glad_glReadnPixels = NULL;
PFNGLRECTDPROC glad_glRectd = NULL;
PFNGLRECTDVPROC glad_glRectdv = NULL;
PFNGLRECTFPROC glad_glRectf = NULL;
PFNGLRECTFVPROC glad_glRectfv = NULL;
PFNGLRECTIPROC glad_glRecti = NULL;
PFNGLRECTIVPROC glad_glRectiv = NULL;
PFNGLRECTSPROC glad_glRects = NULL;
PFNGLRECTSVPROC glad_glRectsv = NULL;
PFNGLRELEASESHADERCOMPILERPROC glad_glReleaseShaderCompiler = NULL;
PFNGLRENDERMODEPROC glad_glRenderMode = NULL;
PFNGLRENDERBUFFERSTORAGEPROC glad_glRenderbufferStorage = NULL;
PFNGLRENDERBUFFERSTORAGEMULTISAMPLEPROC glad_glRenderbufferStorageMultisample = NULL;
PFNGLRESUMETRANSFORMFEEDBACKPROC glad_glResumeTransformFeedback = NULL;
PFNGLROTATEDPROC glad_glRotated = NULL;
PFNGLROTATEFPROC glad_glRotatef = NULL;
PFNGLSAMPLECOVERAGEPROC glad_glSampleCoverage = NULL;
PFNGLSAMPLEMASKIPROC glad_glSampleMaski = NULL;
PFNGLSAMPLERPARAMETERIIVPROC glad_glSamplerParameterIiv = NULL;
PFNGLSAMPLERPARAMETERIUIVPROC glad_glSamplerParameterIuiv = NULL;
PFNGLSAMPLERPARAMETERFPROC glad_glSamplerParameterf = NULL;
PFNGLSAMPLERPARAMETERFVPROC glad_glSamplerParameterfv = NULL;
PFNGLSAMPLERPARAMETERIPROC glad_glSamplerParameteri = NULL;
PFNGLSAMPLERPARAMETERIVPROC glad_glSamplerParameteriv = NULL;
PFNGLSCALEDPROC glad_glScaled = NULL;
PFNGLSCALEFPROC glad_glScalef = NULL;
PFNGLSCISSORPROC glad_glScissor = NULL;
PFNGLSCISSORARRAYVPROC glad_glScissorArrayv = NULL;
PFNGLSCISSORINDEXEDPROC glad_glScissorIndexed = NULL;
PFNGLSCISSORINDEXEDVPROC glad_glScissorIndexedv = NULL;
PFNGLSECONDARYCOLOR3BPROC glad_glSecondaryColor3b = NULL;
PFNGLSECONDARYCOLOR3BVPROC glad_glSecondaryColor3bv = NULL;
PFNGLSECONDARYCOLOR3DPROC glad_glSecondaryColor3d = NULL;
PFNGLSECONDARYCOLOR3DVPROC glad_glSecondaryColor3dv = NULL;
PFNGLSECONDARYCOLOR3FPROC glad_glSecondaryColor3f = NULL;
PFNGLSECONDARYCOLOR3FVPROC glad_glSecondaryColor3fv = NULL;
PFNGLSECONDARYCOLOR3IPROC glad_glSecondaryColor3i = NULL;
PFNGLSECONDARYCOLOR3IVPROC glad_glSecondaryColor3iv = NULL;
PFNGLSECONDARYCOLOR3SPROC glad_glSecondaryColor3s = NULL;
PFNGLSECONDARYCOLOR3SVPROC glad_glSecondaryColor3sv = NULL;
PFNGLSECONDARYCOLOR3UBPROC glad_glSecondaryColor3ub = NULL;
PFNGLSECONDARYCOLOR3UBVPROC glad_glSecondaryColor3ubv = NULL;
PFNGLSECONDARYCOLOR3UIPROC glad_glSecondaryColor3ui = NULL;
PFNGLSECONDARYCOLOR3UIVPROC glad_glSecondaryColor3uiv = NULL;
PFNGLSECONDARYCOLOR3USPROC glad_glSecondaryColor3us = NULL;
PFNGLSECONDARYCOLOR3USVPROC glad_glSecondaryColor3usv = NULL;
PFNGLSECONDARYCOLORP3UIPROC glad_glSecondaryColorP3ui = NULL;
PFNGLSECONDARYCOLORP3UIVPROC glad_glSecondaryColorP3uiv = NULL;
PFNGLSECONDARYCOLORPOINTERPROC glad_glSecondaryColorPointer = NULL;
PFNGLSELECTBUFFERPROC glad_glSelectBuffer = NULL;
PFNGLSHADEMODELPROC glad_glShadeModel = NULL;
PFNGLSHADERBINARYPROC glad_glShaderBinary = NULL;
PFNGLSHADERSOURCEPROC glad_glShaderSource = NULL;
PFNGLSHADERSTORAGEBLOCKBINDINGPROC glad_glShaderStorageBlockBinding = NULL;
PFNGLSPECIALIZESHADERPROC glad_glSpecializeShader = NULL;
PFNGLSTENCILFUNCPROC glad_glStencilFunc = NULL;
PFNGLSTENCILFUNCSEPARATEPROC glad_glStencilFuncSeparate = NULL;
PFNGLSTENCILMASKPROC glad_glStencilMask = NULL;
PFNGLSTENCILMASKSEPARATEPROC glad_glStencilMaskSeparate = NULL;
PFNGLSTENCILOPPROC glad_glStencilOp = NULL;
PFNGLSTENCILOPSEPARATEPROC glad_glStencilOpSeparate = NULL;
PFNGLTEXBUFFERPROC glad_glTexBuffer = NULL;
PFNGLTEXBUFFERRANGEPROC glad_glTexBufferRange = NULL;
PFNGLTEXCOORD1DPROC glad_glTexCoord1d = NULL;
PFNGLTEXCOORD1DVPROC glad_glTexCoord1dv = NULL;
PFNGLTEXCOORD1FPROC glad_glTexCoord1f = NULL;
PFNGLTEXCOORD1FVPROC glad_glTexCoord1fv = NULL;
PFNGLTEXCOORD1IPROC glad_glTexCoord1i = NULL;
PFNGLTEXCOORD1IVPROC glad_glTexCoord1iv = NULL;
PFNGLTEXCOORD1SPROC glad_glTexCoord1s = NULL;
PFNGLTEXCOORD1SVPROC glad_glTexCoord1sv = NULL;
PFNGLTEXCOORD2DPROC glad_glTexCoord2d = NULL;
PFNGLTEXCOORD2DVPROC glad_glTexCoord2dv = NULL;
PFNGLTEXCOORD2FPROC glad_glTexCoord2f = NULL;
PFNGLTEXCOORD2FVPROC glad_glTexCoord2fv = NULL;
PFNGLTEXCOORD2IPROC glad_glTexCoord2i = NULL;
PFNGLTEXCOORD2IVPROC glad_glTexCoord2iv = NULL;
PFNGLTEXCOORD2SPROC glad_glTexCoord2s = NULL;
PFNGLTEXCOORD2SVPROC glad_glTexCoord2sv = NULL;
PFNGLTEXCOORD3DPROC glad_glTexCoord3d = NULL;
PFNGLTEXCOORD3DVPROC glad_glTexCoord3dv = NULL;
PFNGLTEXCOORD3FPROC glad_glTexCoord3f = NULL;
PFNGLTEXCOORD3FVPROC glad_glTexCoord3fv = NULL;
PFNGLTEXCOORD3IPROC glad_glTexCoord3i = NULL;
PFNGLTEXCOORD3IVPROC glad_glTexCoord3iv = NULL;
PFNGLTEXCOORD3SPROC glad_glTexCoord3s = NULL;
PFNGLTEXCOORD3SVPROC glad_glTexCoord3sv = NULL;
PFNGLTEXCOORD4DPROC glad_glTexCoord4d = NULL;
PFNGLTEXCOORD4DVPROC glad_glTexCoord4dv = NULL;
PFNGLTEXCOORD4FPROC glad_glTexCoord4f = NULL;
PFNGLTEXCOORD4FVPROC glad_glTexCoord4fv = NULL;
PFNGLTEXCOORD4IPROC glad_glTexCoord4i = NULL;
PFNGLTEXCOORD4IVPROC glad_glTexCoord4iv = NULL;
PFNGLTEXCOORD4SPROC glad_glTexCoord4s = NULL;
PFNGLTEXCOORD4SVPROC glad_glTexCoord4sv = NULL;
PFNGLTEXCOORDP1UIPROC glad_glTexCoordP1ui = NULL;
PFNGLTEXCOORDP1UIVPROC glad_glTexCoordP1uiv = NULL;
PFNGLTEXCOORDP2UIPROC glad_glTexCoordP2ui = NULL;
PFNGLTEXCOORDP2UIVPROC glad_glTexCoordP2uiv = NULL;
PFNGLTEXCOORDP3UIPROC glad_glTexCoordP3ui = NULL;
PFNGLTEXCOORDP3UIVPROC glad_glTexCoordP3uiv = NULL;
PFNGLTEXCOORDP4UIPROC glad_glTexCoordP4ui = NULL;
PFNGLTEXCOORDP4UIVPROC glad_glTexCoordP4uiv = NULL;
PFNGLTEXCOORDPOINTERPROC glad_glTexCoordPointer = NULL;
PFNGLTEXENVFPROC glad_glTexEnvf = NULL;
PFNGLTEXENVFVPROC glad_glTexEnvfv = NULL;
PFNGLTEXENVIPROC glad_glTexEnvi = NULL;
PFNGLTEXENVIVPROC glad_glTexEnviv = NULL;
PFNGLTEXGENDPROC glad_glTexGend = NULL;
PFNGLTEXGENDVPROC glad_glTexGendv = NULL;
PFNGLTEXGENFPROC glad_glTexGenf = NULL;
PFNGLTEXGENFVPROC glad_glTexGenfv = NULL;
PFNGLTEXGENIPROC glad_glTexGeni = NULL;
PFNGLTEXGENIVPROC glad_glTexGeniv = NULL;
PFNGLTEXIMAGE1DPROC glad_glTexImage1D = NULL;
PFNGLTEXIMAGE2DPROC glad_glTexImage2D = NULL;
PFNGLTEXIMAGE2DMULTISAMPLEPROC glad_glTexImage2DMultisample = NULL;
PFNGLTEXIMAGE3DPROC glad_glTexImage3D = NULL;
PFNGLTEXIMAGE3DMULTISAMPLEPROC glad_glTexImage3DMultisample = NULL;
PFNGLTEXPARAMETERIIVPROC glad_glTexParameterIiv = NULL;
PFNGLTEXPARAMETERIUIVPROC glad_glTexParameterIuiv = NULL;
PFNGLTEXPARAMETERFPROC glad_glTexParameterf = NULL;
PFNGLTEXPARAMETERFVPROC glad_glTexParameterfv = NULL;
PFNGLTEXPARAMETERIPROC glad_glTexParameteri = NULL;
PFNGLTEXPARAMETERIVPROC glad_glTexParameteriv = NULL;
PFNGLTEXSTORAGE1DPROC glad_glTexStorage1D = NULL;
PFNGLTEXSTORAGE2DPROC glad_glTexStorage2D = NULL;
PFNGLTEXSTORAGE2DMULTISAMPLEPROC glad_glTexStorage2DMultisample = NULL;
PFNGLTEXSTORAGE3DPROC glad_glTexStorage3D = NULL;
PFNGLTEXSTORAGE3DMULTISAMPLEPROC glad_glTexStorage3DMultisample = NULL;
PFNGLTEXSUBIMAGE1DPROC glad_glTexSubImage1D = NULL;
PFNGLTEXSUBIMAGE2DPROC glad_glTexSubImage2D = NULL;
PFNGLTEXSUBIMAGE3DPROC glad_glTexSubImage3D = NULL;
PFNGLTEXTUREBARRIERPROC glad_glTextureBarrier = NULL;
PFNGLTEXTUREBUFFERPROC glad_glTextureBuffer = NULL;
PFNGLTEXTUREBUFFERRANGEPROC glad_glTextureBufferRange = NULL;
PFNGLTEXTUREPARAMETERIIVPROC glad_glTextureParameterIiv = NULL;
PFNGLTEXTUREPARAMETERIUIVPROC glad_glTextureParameterIuiv = NULL;
PFNGLTEXTUREPARAMETERFPROC glad_glTextureParameterf = NULL;
PFNGLTEXTUREPARAMETERFVPROC glad_glTextureParameterfv = NULL;
PFNGLTEXTUREPARAMETERIPROC glad_glTextureParameteri = NULL;
PFNGLTEXTUREPARAMETERIVPROC glad_glTextureParameteriv = NULL;
PFNGLTEXTURESTORAGE1DPROC glad_glTextureStorage1D = NULL;
PFNGLTEXTURESTORAGE2DPROC glad_glTextureStorage2D = NULL;
PFNGLTEXTURESTORAGE2DMULTISAMPLEPROC glad_glTextureStorage2DMultisample = NULL;
PFNGLTEXTURESTORAGE3DPROC glad_glTextureStorage3D = NULL;
PFNGLTEXTURESTORAGE3DMULTISAMPLEPROC glad_glTextureStorage3DMultisample = NULL;
PFNGLTEXTURESUBIMAGE1DPROC glad_glTextureSubImage1D = NULL;
PFNGLTEXTURESUBIMAGE2DPROC glad_glTextureSubImage2D = NULL;
PFNGLTEXTURESUBIMAGE3DPROC glad_glTextureSubImage3D = NULL;
PFNGLTEXTUREVIEWPROC glad_glTextureView = NULL;
PFNGLTRANSFORMFEEDBACKBUFFERBASEPROC glad_glTransformFeedbackBufferBase = NULL;
PFNGLTRANSFORMFEEDBACKBUFFERRANGEPROC glad_glTransformFeedbackBufferRange = NULL;
PFNGLTRANSFORMFEEDBACKVARYINGSPROC glad_glTransformFeedbackVaryings = NULL;
PFNGLTRANSLATEDPROC glad_glTranslated = NULL;
PFNGLTRANSLATEFPROC glad_glTranslatef = NULL;
PFNGLUNIFORM1DPROC glad_glUniform1d = NULL;
PFNGLUNIFORM1DVPROC glad_glUniform1dv = NULL;
PFNGLUNIFORM1FPROC glad_glUniform1f = NULL;
PFNGLUNIFORM1FVPROC glad_glUniform1fv = NULL;
PFNGLUNIFORM1IPROC glad_glUniform1i = NULL;
PFNGLUNIFORM1IVPROC glad_glUniform1iv = NULL;
PFNGLUNIFORM1UIPROC glad_glUniform1ui = NULL;
PFNGLUNIFORM1UIVPROC glad_glUniform1uiv = NULL;
PFNGLUNIFORM2DPROC glad_glUniform2d = NULL;
PFNGLUNIFORM2DVPROC glad_glUniform2dv = NULL;
PFNGLUNIFORM2FPROC glad_glUniform2f = NULL;
PFNGLUNIFORM2FVPROC glad_glUniform2fv = NULL;
PFNGLUNIFORM2IPROC glad_glUniform2i = NULL;
PFNGLUNIFORM2IVPROC glad_glUniform2iv = NULL;
PFNGLUNIFORM2UIPROC glad_glUniform2ui = NULL;
PFNGLUNIFORM2UIVPROC glad_glUniform2uiv = NULL;
PFNGLUNIFORM3DPROC glad_glUniform3d = NULL;
PFNGLUNIFORM3DVPROC glad_glUniform3dv = NULL;
PFNGLUNIFORM3FPROC glad_glUniform3f = NULL;
PFNGLUNIFORM3FVPROC glad_glUniform3fv = NULL;
PFNGLUNIFORM3IPROC glad_glUniform3i = NULL;
PFNGLUNIFORM3IVPROC glad_glUniform3iv = NULL;
PFNGLUNIFORM3UIPROC glad_glUniform3ui = NULL;
PFNGLUNIFORM3UIVPROC glad_glUniform3uiv = NULL;
PFNGLUNIFORM4DPROC glad_glUniform4d = NULL;
PFNGLUNIFORM4DVPROC glad_glUniform4dv = NULL;
PFNGLUNIFORM4FPROC glad_glUniform4f = NULL;
PFNGLUNIFORM4FVPROC glad_glUniform4fv = NULL;
PFNGLUNIFORM4IPROC glad_glUniform4i = NULL;
PFNGLUNIFORM4IVPROC glad_glUniform4iv = NULL;
PFNGLUNIFORM4UIPROC glad_glUniform4ui = NULL;
PFNGLUNIFORM4UIVPROC glad_glUniform4uiv = NULL;
PFNGLUNIFORMBLOCKBINDINGPROC glad_glUniformBlockBinding = NULL;
PFNGLUNIFORMMATRIX2DVPROC glad_glUniformMatrix2dv = NULL;
PFNGLUNIFORMMATRIX2FVPROC glad_glUniformMatrix2fv = NULL;
PFNGLUNIFORMMATRIX2X3DVPROC glad_glUniformMatrix2x3dv = NULL;
PFNGLUNIFORMMATRIX2X3FVPROC glad_glUniformMatrix2x3fv = NULL;
PFNGLUNIFORMMATRIX2X4DVPROC glad_glUniformMatrix2x4dv = NULL;
PFNGLUNIFORMMATRIX2X4FVPROC glad_glUniformMatrix2x4fv = NULL;
PFNGLUNIFORMMATRIX3DVPROC glad_glUniformMatrix3dv = NULL;
PFNGLUNIFORMMATRIX3FVPROC glad_glUniformMatrix3fv = NULL;
PFNGLUNIFORMMATRIX3X2DVPROC glad_glUniformMatrix3x2dv = NULL;
PFNGLUNIFORMMATRIX3X2FVPROC glad_glUniformMatrix3x2fv = NULL;
PFNGLUNIFORMMATRIX3X4DVPROC glad_glUniformMatrix3x4dv = NULL;
PFNGLUNIFORMMATRIX3X4FVPROC glad_glUniformMatrix3x4fv = NULL;
PFNGLUNIFORMMATRIX4DVPROC glad_glUniformMatrix4dv = NULL;
PFNGLUNIFORMMATRIX4FVPROC glad_glUniformMatrix4fv = NULL;
PFNGLUNIFORMMATRIX4X2DVPROC glad_glUniformMatrix4x2dv = NULL;
PFNGLUNIFORMMATRIX4X2FVPROC glad_glUniformMatrix4x2fv = NULL;
PFNGLUNIFORMMATRIX4X3DVPROC glad_glUniformMatrix4x3dv = NULL;
PFNGLUNIFORMMATRIX4X3FVPROC glad_glUniformMatrix4x3fv = NULL;
PFNGLUNIFORMSUBROUTINESUIVPROC glad_glUniformSubroutinesuiv = NULL;
PFNGLUNMAPBUFFERPROC glad_glUnmapBuffer = NULL;
PFNGLUNMAPNAMEDBUFFERPROC glad_glUnmapNamedBuffer = NULL;
PFNGLUSEPROGRAMPROC glad_glUseProgram = NULL;
PFNGLUSEPROGRAMSTAGESPROC glad_glUseProgramStages = NULL;
PFNGLVALIDATEPROGRAMPROC glad_glValidateProgram = NULL;
PFNGLVALIDATEPROGRAMPIPELINEPROC glad_glValidateProgramPipeline = NULL;
PFNGLVERTEX2DPROC glad_glVertex2d = NULL;
PFNGLVERTEX2DVPROC glad_glVertex2dv = NULL;
PFNGLVERTEX2FPROC glad_glVertex2f = NULL;
PFNGLVERTEX2FVPROC glad_glVertex2fv = NULL;
PFNGLVERTEX2IPROC glad_glVertex2i = NULL;
PFNGLVERTEX2IVPROC glad_glVertex2iv = NULL;
PFNGLVERTEX2SPROC glad_glVertex2s = NULL;
PFNGLVERTEX2SVPROC glad_glVertex2sv = NULL;
PFNGLVERTEX3DPROC glad_glVertex3d = NULL;
PFNGLVERTEX3DVPROC glad_glVertex3dv = NULL;
PFNGLVERTEX3FPROC glad_glVertex3f = NULL;
PFNGLVERTEX3FVPROC glad_glVertex3fv = NULL;
PFNGLVERTEX3IPROC glad_glVertex3i = NULL;
PFNGLVERTEX3IVPROC glad_glVertex3iv = NULL;
PFNGLVERTEX3SPROC glad_glVertex3s = NULL;
PFNGLVERTEX3SVPROC glad_glVertex3sv = NULL;
PFNGLVERTEX4DPROC glad_glVertex4d = NULL;
PFNGLVERTEX4DVPROC glad_glVertex4dv = NULL;
PFNGLVERTEX4FPROC glad_glVertex4f = NULL;
PFNGLVERTEX4FVPROC glad_glVertex4fv = NULL;
PFNGLVERTEX4IPROC glad_glVertex4i = NULL;
PFNGLVERTEX4IVPROC glad_glVertex4iv = NULL;
PFNGLVERTEX4SPROC glad_glVertex4s = NULL;
PFNGLVERTEX4SVPROC glad_glVertex4sv = NULL;
PFNGLVERTEXARRAYATTRIBBINDINGPROC glad_glVertexArrayAttribBinding = NULL;
PFNGLVERTEXARRAYATTRIBFORMATPROC glad_glVertexArrayAttribFormat = NULL;
PFNGLVERTEXARRAYATTRIBIFORMATPROC glad_glVertexArrayAttribIFormat = NULL;
PFNGLVERTEXARRAYATTRIBLFORMATPROC glad_glVertexArrayAttribLFormat = NULL;
PFNGLVERTEXARRAYBINDINGDIVISORPROC glad_glVertexArrayBindingDivisor = NULL;
PFNGLVERTEXARRAYELEMENTBUFFERPROC glad_glVertexArrayElementBuffer = NULL;
PFNGLVERTEXARRAYVERTEXBUFFERPROC glad_glVertexArrayVertexBuffer = NULL;
PFNGLVERTEXARRAYVERTEXBUFFERSPROC glad_glVertexArrayVertexBuffers = NULL;
PFNGLVERTEXATTRIB1DPROC glad_glVertexAttrib1d = NULL;
PFNGLVERTEXATTRIB1DVPROC glad_glVertexAttrib1dv = NULL;
PFNGLVERTEXATTRIB1FPROC glad_glVertexAttrib1f = NULL;
PFNGLVERTEXATTRIB1FVPROC glad_glVertexAttrib1fv = NULL;
PFNGLVERTEXATTRIB1SPROC glad_glVertexAttrib1s = NULL;
PFNGLVERTEXATTRIB1SVPROC glad_glVertexAttrib1sv = NULL;
PFNGLVERTEXATTRIB2DPROC glad_glVertexAttrib2d = NULL;
PFNGLVERTEXATTRIB2DVPROC glad_glVertexAttrib2dv = NULL;
PFNGLVERTEXATTRIB2FPROC glad_glVertexAttrib2f = NULL;
PFNGLVERTEXATTRIB2FVPROC glad_glVertexAttrib2fv = NULL;
PFNGLVERTEXATTRIB2SPROC glad_glVertexAttrib2s = NULL;
PFNGLVERTEXATTRIB2SVPROC glad_glVertexAttrib2sv = NULL;
PFNGLVERTEXATTRIB3DPROC glad_glVertexAttrib3d = NULL;
PFNGLVERTEXATTRIB3DVPROC glad_glVertexAttrib3dv = NULL;
PFNGLVERTEXATTRIB3FPROC glad_glVertexAttrib3f = NULL;
PFNGLVERTEXATTRIB3FVPROC glad_glVertexAttrib3fv = NULL;
PFNGLVERTEXATTRIB3SPROC glad_glVertexAttrib3s = NULL;
PFNGLVERTEXATTRIB3SVPROC glad_glVertexAttrib3sv = NULL;
PFNGLVERTEXATTRIB4NBVPROC glad_glVertexAttrib4Nbv = NULL;
PFNGLVERTEXATTRIB4NIVPROC glad_glVertexAttrib4Niv = NULL;
PFNGLVERTEXATTRIB4NSVPROC glad_glVertexAttrib4Nsv = NULL;
PFNGLVERTEXATTRIB4NUBPROC glad_glVertexAttrib4Nub = NULL;
PFNGLVERTEXATTRIB4NUBVPROC glad_glVertexAttrib4Nubv = NULL;
PFNGLVERTEXATTRIB4NUIVPROC glad_glVertexAttrib4Nuiv = NULL;
PFNGLVERTEXATTRIB4NUSVPROC glad_glVertexAttrib4Nusv = NULL;
PFNGLVERTEXATTRIB4BVPROC glad_glVertexAttrib4bv = NULL;
PFNGLVERTEXATTRIB4DPROC glad_glVertexAttrib4d = NULL;
PFNGLVERTEXATTRIB4DVPROC glad_glVertexAttrib4dv = NULL;
PFNGLVERTEXATTRIB4FPROC glad_glVertexAttrib4f = NULL;
PFNGLVERTEXATTRIB4FVPROC glad_glVertexAttrib4fv = NULL;
PFNGLVERTEXATTRIB4IVPROC glad_glVertexAttrib4iv = NULL;
PFNGLVERTEXATTRIB4SPROC glad_glVertexAttrib4s = NULL;
PFNGLVERTEXATTRIB4SVPROC glad_glVertexAttrib4sv = NULL;
PFNGLVERTEXATTRIB4UBVPROC glad_glVertexAttrib4ubv = NULL;
PFNGLVERTEXATTRIB4UIVPROC glad_glVertexAttrib4uiv = NULL;
PFNGLVERTEXATTRIB4USVPROC glad_glVertexAttrib4usv = NULL;
PFNGLVERTEXATTRIBBINDINGPROC glad_glVertexAttribBinding = NULL;
PFNGLVERTEXATTRIBDIVISORPROC glad_glVertexAttribDivisor = NULL;
PFNGLVERTEXATTRIBFORMATPROC glad_glVertexAttribFormat = NULL;
PFNGLVERTEXATTRIBI1IPROC glad_glVertexAttribI1i = NULL;
PFNGLVERTEXATTRIBI1IVPROC glad_glVertexAttribI1iv = NULL;
PFNGLVERTEXATTRIBI1UIPROC glad_glVertexAttribI1ui = NULL;
PFNGLVERTEXATTRIBI1UIVPROC glad_glVertexAttribI1uiv = NULL;
PFNGLVERTEXATTRIBI2IPROC glad_glVertexAttribI2i = NULL;
PFNGLVERTEXATTRIBI2IVPROC glad_glVertexAttribI2iv = NULL;
PFNGLVERTEXATTRIBI2UIPROC glad_glVertexAttribI2ui = NULL;
PFNGLVERTEXATTRIBI2UIVPROC glad_glVertexAttribI2uiv = NULL;
PFNGLVERTEXATTRIBI3IPROC glad_glVertexAttribI3i = NULL;
PFNGLVERTEXATTRIBI3IVPROC glad_glVertexAttribI3iv = NULL;
PFNGLVERTEXATTRIBI3UIPROC glad_glVertexAttribI3ui = NULL;
PFNGLVERTEXATTRIBI3UIVPROC glad_glVertexAttribI3uiv = NULL;
PFNGLVERTEXATTRIBI4BVPROC glad_glVertexAttribI4bv = NULL;
PFNGLVERTEXATTRIBI4IPROC glad_glVertexAttribI4i = NULL;
PFNGLVERTEXATTRIBI4IVPROC glad_glVertexAttribI4iv = NULL;
PFNGLVERTEXATTRIBI4SVPROC glad_glVertexAttribI4sv = NULL;
PFNGLVERTEXATTRIBI4UBVPROC glad_glVertexAttribI4ubv = NULL;
PFNGLVERTEXATTRIBI4UIPROC glad_glVertexAttribI4ui = NULL;
PFNGLVERTEXATTRIBI4UIVPROC glad_glVertexAttribI4uiv = NULL;
PFNGLVERTEXATTRIBI4USVPROC glad_glVertexAttribI4usv = NULL;
PFNGLVERTEXATTRIBIFORMATPROC glad_glVertexAttribIFormat = NULL;
PFNGLVERTEXATTRIBIPOINTERPROC glad_glVertexAttribIPointer = NULL;
PFNGLVERTEXATTRIBL1DPROC glad_glVertexAttribL1d = NULL;
PFNGLVERTEXATTRIBL1DVPROC glad_glVertexAttribL1dv = NULL;
PFNGLVERTEXATTRIBL2DPROC glad_glVertexAttribL2d = NULL;
PFNGLVERTEXATTRIBL2DVPROC glad_glVertexAttribL2dv = NULL;
PFNGLVERTEXATTRIBL3DPROC glad_glVertexAttribL3d = NULL;
PFNGLVERTEXATTRIBL3DVPROC glad_glVertexAttribL3dv = NULL;
PFNGLVERTEXATTRIBL4DPROC glad_glVertexAttribL4d = NULL;
PFNGLVERTEXATTRIBL4DVPROC glad_glVertexAttribL4dv = NULL;
PFNGLVERTEXATTRIBLFORMATPROC glad_glVertexAttribLFormat = NULL;
PFNGLVERTEXATTRIBLPOINTERPROC glad_glVertexAttribLPointer = NULL;
PFNGLVERTEXATTRIBP1UIPROC glad_glVertexAttribP1ui = NULL;
PFNGLVERTEXATTRIBP1UIVPROC glad_glVertexAttribP1uiv = NULL;
PFNGLVERTEXATTRIBP2UIPROC glad_glVertexAttribP2ui = NULL;
PFNGLVERTEXATTRIBP2UIVPROC glad_glVertexAttribP2uiv = NULL;
PFNGLVERTEXATTRIBP3UIPROC glad_glVertexAttribP3ui = NULL;
PFNGLVERTEXATTRIBP3UIVPROC glad_glVertexAttribP3uiv = NULL;
PFNGLVERTEXATTRIBP4UIPROC glad_glVertexAttribP4ui = NULL;
PFNGLVERTEXATTRIBP4UIVPROC glad_glVertexAttribP4uiv = NULL;
PFNGLVERTEXATTRIBPOINTERPROC glad_glVertexAttribPointer = NULL;
PFNGLVERTEXBINDINGDIVISORPROC glad_glVertexBindingDivisor = NULL;
PFNGLVERTEXP2UIPROC glad_glVertexP2ui = NULL;
PFNGLVERTEXP2UIVPROC glad_glVertexP2uiv = NULL;
PFNGLVERTEXP3UIPROC glad_glVertexP3ui = NULL;
PFNGLVERTEXP3UIVPROC glad_glVertexP3uiv = NULL;
PFNGLVERTEXP4UIPROC glad_glVertexP4ui = NULL;
PFNGLVERTEXP4UIVPROC glad_glVertexP4uiv = NULL;
PFNGLVERTEXPOINTERPROC glad_glVertexPointer = NULL;
PFNGLVIEWPORTPROC glad_glViewport = NULL;
PFNGLVIEWPORTARRAYVPROC glad_glViewportArrayv = NULL;
PFNGLVIEWPORTINDEXEDFPROC glad_glViewportIndexedf = NULL;
PFNGLVIEWPORTINDEXEDFVPROC glad_glViewportIndexedfv = NULL;
PFNGLWAITSYNCPROC glad_glWaitSync = NULL;
PFNGLWINDOWPOS2DPROC glad_glWindowPos2d = NULL;
PFNGLWINDOWPOS2DVPROC glad_glWindowPos2dv = NULL;
PFNGLWINDOWPOS2FPROC glad_glWindowPos2f = NULL;
PFNGLWINDOWPOS2FVPROC glad_glWindowPos2fv = NULL;
PFNGLWINDOWPOS2IPROC glad_glWindowPos2i = NULL;
PFNGLWINDOWPOS2IVPROC glad_glWindowPos2iv = NULL;
PFNGLWINDOWPOS2SPROC glad_glWindowPos2s = NULL;
PFNGLWINDOWPOS2SVPROC glad_glWindowPos2sv = NULL;
PFNGLWINDOWPOS3DPROC glad_glWindowPos3d = NULL;
PFNGLWINDOWPOS3DVPROC glad_glWindowPos3dv = NULL;
PFNGLWINDOWPOS3FPROC glad_glWindowPos3f = NULL;
PFNGLWINDOWPOS3FVPROC glad_glWindowPos3fv = NULL;
PFNGLWINDOWPOS3IPROC glad_glWindowPos3i = NULL;
PFNGLWINDOWPOS3IVPROC glad_glWindowPos3iv = NULL;
PFNGLWINDOWPOS3SPROC glad_glWindowPos3s = NULL;
PFNGLWINDOWPOS3SVPROC glad_glWindowPos3sv = NULL;
int GLAD_GL_EXT_memory_object = 0;
int GLAD_GL_EXT_memory_object_fd = 0;
int GLAD_GL_EXT_memory_object_win32 = 0;
PFNGLGETUNSIGNEDBYTEVEXTPROC glad_glGetUnsignedBytevEXT = NULL;
PFNGLGETUNSIGNEDBYTEI_VEXTPROC glad_glGetUnsignedBytei_vEXT = NULL;
PFNGLDELETEMEMORYOBJECTSEXTPROC glad_glDeleteMemoryObjectsEXT = NULL;
PFNGLISMEMORYOBJECTEXTPROC glad_glIsMemoryObjectEXT = NULL;
PFNGLCREATEMEMORYOBJECTSEXTPROC glad_glCreateMemoryObjectsEXT = NULL;
PFNGLMEMORYOBJECTPARAMETERIVEXTPROC glad_glMemoryObjectParameterivEXT = NULL;
PFNGLGETMEMORYOBJECTPARAMETERIVEXTPROC glad_glGetMemoryObjectParameterivEXT = NULL;
PFNGLTEXSTORAGEMEM2DEXTPROC glad_glTexStorageMem2DEXT = NULL;
PFNGLTEXSTORAGEMEM2DMULTISAMPLEEXTPROC glad_glTexStorageMem2DMultisampleEXT = NULL;
PFNGLTEXSTORAGEMEM3DEXTPROC glad_glTexStorageMem3DEXT = NULL;
PFNGLTEXSTORAGEMEM3DMULTISAMPLEEXTPROC glad_glTexStorageMem3DMultisampleEXT = NULL;
PFNGLBUFFERSTORAGEMEMEXTPROC glad_glBufferStorageMemEXT = NULL;
PFNGLTEXTURESTORAGEMEM2DEXTPROC glad_glTextureStorageMem2DEXT = NULL;
PFNGLTEXTURESTORAGEMEM2DMULTISAMPLEEXTPROC glad_glTextureStorageMem2DMultisampleEXT = NULL;
PFNGLTEXTURESTORAGEMEM3DEXTPROC glad_glTextureStorageMem3DEXT = NULL;
PFNGLTEXTURESTORAGEMEM3DMULTISAMPLEEXTPROC glad_glTextureStorageMem3DMultisampleEXT = NULL;
PFNGLNAMEDBUFFERSTORAGEMEMEXTPROC glad_glNamedBufferStorageMemEXT = NULL;
PFNGLTEXSTORAGEMEM1DEXTPROC glad_glTexStorageMem1DEXT = NULL;
PFNGLTEXTURESTORAGEMEM1DEXTPROC glad_glTextureStorageMem1DEXT = NULL;
PFNGLIMPORTMEMORYFDEXTPROC glad_glImportMemoryFdEXT = NULL;
PFNGLIMPORTMEMORYWIN32HANDLEEXTPROC glad_glImportMemoryWin32HandleEXT = NULL;
PFNGLIMPORTMEMORYWIN32NAMEEXTPROC glad_glImportMemoryWin32NameEXT = NULL;
static void load_GL_VERSION_1_0( GLADloadproc load )
{
	if (!GLAD_GL_VERSION_1_0) return;
	glad_glCullFace = (PFNGLCULLFACEPROC)load( "glCullFace" );
	glad_glFrontFace = (PFNGLFRONTFACEPROC)load( "glFrontFace" );
	glad_glHint = (PFNGLHINTPROC)load( "glHint" );
	glad_glLineWidth = (PFNGLLINEWIDTHPROC)load( "glLineWidth" );
	glad_glPointSize = (PFNGLPOINTSIZEPROC)load( "glPointSize" );
	glad_glPolygonMode = (PFNGLPOLYGONMODEPROC)load( "glPolygonMode" );
	glad_glScissor = (PFNGLSCISSORPROC)load( "glScissor" );
	glad_glTexParameterf = (PFNGLTEXPARAMETERFPROC)load( "glTexParameterf" );
	glad_glTexParameterfv = (PFNGLTEXPARAMETERFVPROC)load( "glTexParameterfv" );
	glad_glTexParameteri = (PFNGLTEXPARAMETERIPROC)load( "glTexParameteri" );
	glad_glTexParameteriv = (PFNGLTEXPARAMETERIVPROC)load( "glTexParameteriv" );
	glad_glTexImage1D = (PFNGLTEXIMAGE1DPROC)load( "glTexImage1D" );
	glad_glTexImage2D = (PFNGLTEXIMAGE2DPROC)load( "glTexImage2D" );
	glad_glDrawBuffer = (PFNGLDRAWBUFFERPROC)load( "glDrawBuffer" );
	glad_glClear = (PFNGLCLEARPROC)load( "glClear" );
	glad_glClearColor = (PFNGLCLEARCOLORPROC)load( "glClearColor" );
	glad_glClearStencil = (PFNGLCLEARSTENCILPROC)load( "glClearStencil" );
	glad_glClearDepth = (PFNGLCLEARDEPTHPROC)load( "glClearDepth" );
	glad_glStencilMask = (PFNGLSTENCILMASKPROC)load( "glStencilMask" );
	glad_glColorMask = (PFNGLCOLORMASKPROC)load( "glColorMask" );
	glad_glDepthMask = (PFNGLDEPTHMASKPROC)load( "glDepthMask" );
	glad_glDisable = (PFNGLDISABLEPROC)load( "glDisable" );
	glad_glEnable = (PFNGLENABLEPROC)load( "glEnable" );
	glad_glFinish = (PFNGLFINISHPROC)load( "glFinish" );
	glad_glFlush = (PFNGLFLUSHPROC)load( "glFlush" );
	glad_glBlendFunc = (PFNGLBLENDFUNCPROC)load( "glBlendFunc" );
	glad_glLogicOp = (PFNGLLOGICOPPROC)load( "glLogicOp" );
	glad_glStencilFunc = (PFNGLSTENCILFUNCPROC)load( "glStencilFunc" );
	glad_glStencilOp = (PFNGLSTENCILOPPROC)load( "glStencilOp" );
	glad_glDepthFunc = (PFNGLDEPTHFUNCPROC)load( "glDepthFunc" );
	glad_glPixelStoref = (PFNGLPIXELSTOREFPROC)load( "glPixelStoref" );
	glad_glPixelStorei = (PFNGLPIXELSTOREIPROC)load( "glPixelStorei" );
	glad_glReadBuffer = (PFNGLREADBUFFERPROC)load( "glReadBuffer" );
	glad_glReadPixels = (PFNGLREADPIXELSPROC)load( "glReadPixels" );
	glad_glGetBooleanv = (PFNGLGETBOOLEANVPROC)load( "glGetBooleanv" );
	glad_glGetDoublev = (PFNGLGETDOUBLEVPROC)load( "glGetDoublev" );
	glad_glGetError = (PFNGLGETERRORPROC)load( "glGetError" );
	glad_glGetFloatv = (PFNGLGETFLOATVPROC)load( "glGetFloatv" );
	glad_glGetIntegerv = (PFNGLGETINTEGERVPROC)load( "glGetIntegerv" );
	glad_glGetString = (PFNGLGETSTRINGPROC)load( "glGetString" );
	glad_glGetTexImage = (PFNGLGETTEXIMAGEPROC)load( "glGetTexImage" );
	glad_glGetTexParameterfv = (PFNGLGETTEXPARAMETERFVPROC)load( "glGetTexParameterfv" );
	glad_glGetTexParameteriv = (PFNGLGETTEXPARAMETERIVPROC)load( "glGetTexParameteriv" );
	glad_glGetTexLevelParameterfv = (PFNGLGETTEXLEVELPARAMETERFVPROC)load( "glGetTexLevelParameterfv" );
	glad_glGetTexLevelParameteriv = (PFNGLGETTEXLEVELPARAMETERIVPROC)load( "glGetTexLevelParameteriv" );
	glad_glIsEnabled = (PFNGLISENABLEDPROC)load( "glIsEnabled" );
	glad_glDepthRange = (PFNGLDEPTHRANGEPROC)load( "glDepthRange" );
	glad_glViewport = (PFNGLVIEWPORTPROC)load( "glViewport" );
	glad_glNewList = (PFNGLNEWLISTPROC)load( "glNewList" );
	glad_glEndList = (PFNGLENDLISTPROC)load( "glEndList" );
	glad_glCallList = (PFNGLCALLLISTPROC)load( "glCallList" );
	glad_glCallLists = (PFNGLCALLLISTSPROC)load( "glCallLists" );
	glad_glDeleteLists = (PFNGLDELETELISTSPROC)load( "glDeleteLists" );
	glad_glGenLists = (PFNGLGENLISTSPROC)load( "glGenLists" );
	glad_glListBase = (PFNGLLISTBASEPROC)load( "glListBase" );
	glad_glBegin = (PFNGLBEGINPROC)load( "glBegin" );
	glad_glBitmap = (PFNGLBITMAPPROC)load( "glBitmap" );
	glad_glColor3b = (PFNGLCOLOR3BPROC)load( "glColor3b" );
	glad_glColor3bv = (PFNGLCOLOR3BVPROC)load( "glColor3bv" );
	glad_glColor3d = (PFNGLCOLOR3DPROC)load( "glColor3d" );
	glad_glColor3dv = (PFNGLCOLOR3DVPROC)load( "glColor3dv" );
	glad_glColor3f = (PFNGLCOLOR3FPROC)load( "glColor3f" );
	glad_glColor3fv = (PFNGLCOLOR3FVPROC)load( "glColor3fv" );
	glad_glColor3i = (PFNGLCOLOR3IPROC)load( "glColor3i" );
	glad_glColor3iv = (PFNGLCOLOR3IVPROC)load( "glColor3iv" );
	glad_glColor3s = (PFNGLCOLOR3SPROC)load( "glColor3s" );
	glad_glColor3sv = (PFNGLCOLOR3SVPROC)load( "glColor3sv" );
	glad_glColor3ub = (PFNGLCOLOR3UBPROC)load( "glColor3ub" );
	glad_glColor3ubv = (PFNGLCOLOR3UBVPROC)load( "glColor3ubv" );
	glad_glColor3ui = (PFNGLCOLOR3UIPROC)load( "glColor3ui" );
	glad_glColor3uiv = (PFNGLCOLOR3UIVPROC)load( "glColor3uiv" );
	glad_glColor3us = (PFNGLCOLOR3USPROC)load( "glColor3us" );
	glad_glColor3usv = (PFNGLCOLOR3USVPROC)load( "glColor3usv" );
	glad_glColor4b = (PFNGLCOLOR4BPROC)load( "glColor4b" );
	glad_glColor4bv = (PFNGLCOLOR4BVPROC)load( "glColor4bv" );
	glad_glColor4d = (PFNGLCOLOR4DPROC)load( "glColor4d" );
	glad_glColor4dv = (PFNGLCOLOR4DVPROC)load( "glColor4dv" );
	glad_glColor4f = (PFNGLCOLOR4FPROC)load( "glColor4f" );
	glad_glColor4fv = (PFNGLCOLOR4FVPROC)load( "glColor4fv" );
	glad_glColor4i = (PFNGLCOLOR4IPROC)load( "glColor4i" );
	glad_glColor4iv = (PFNGLCOLOR4IVPROC)load( "glColor4iv" );
	glad_glColor4s = (PFNGLCOLOR4SPROC)load( "glColor4s" );
	glad_glColor4sv = (PFNGLCOLOR4SVPROC)load( "glColor4sv" );
	glad_glColor4ub = (PFNGLCOLOR4UBPROC)load( "glColor4ub" );
	glad_glColor4ubv = (PFNGLCOLOR4UBVPROC)load( "glColor4ubv" );
	glad_glColor4ui = (PFNGLCOLOR4UIPROC)load( "glColor4ui" );
	glad_glColor4uiv = (PFNGLCOLOR4UIVPROC)load( "glColor4uiv" );
	glad_glColor4us = (PFNGLCOLOR4USPROC)load( "glColor4us" );
	glad_glColor4usv = (PFNGLCOLOR4USVPROC)load( "glColor4usv" );
	glad_glEdgeFlag = (PFNGLEDGEFLAGPROC)load( "glEdgeFlag" );
	glad_glEdgeFlagv = (PFNGLEDGEFLAGVPROC)load( "glEdgeFlagv" );
	glad_glEnd = (PFNGLENDPROC)load( "glEnd" );
	glad_glIndexd = (PFNGLINDEXDPROC)load( "glIndexd" );
	glad_glIndexdv = (PFNGLINDEXDVPROC)load( "glIndexdv" );
	glad_glIndexf = (PFNGLINDEXFPROC)load( "glIndexf" );
	glad_glIndexfv = (PFNGLINDEXFVPROC)load( "glIndexfv" );
	glad_glIndexi = (PFNGLINDEXIPROC)load( "glIndexi" );
	glad_glIndexiv = (PFNGLINDEXIVPROC)load( "glIndexiv" );
	glad_glIndexs = (PFNGLINDEXSPROC)load( "glIndexs" );
	glad_glIndexsv = (PFNGLINDEXSVPROC)load( "glIndexsv" );
	glad_glNormal3b = (PFNGLNORMAL3BPROC)load( "glNormal3b" );
	glad_glNormal3bv = (PFNGLNORMAL3BVPROC)load( "glNormal3bv" );
	glad_glNormal3d = (PFNGLNORMAL3DPROC)load( "glNormal3d" );
	glad_glNormal3dv = (PFNGLNORMAL3DVPROC)load( "glNormal3dv" );
	glad_glNormal3f = (PFNGLNORMAL3FPROC)load( "glNormal3f" );
	glad_glNormal3fv = (PFNGLNORMAL3FVPROC)load( "glNormal3fv" );
	glad_glNormal3i = (PFNGLNORMAL3IPROC)load( "glNormal3i" );
	glad_glNormal3iv = (PFNGLNORMAL3IVPROC)load( "glNormal3iv" );
	glad_glNormal3s = (PFNGLNORMAL3SPROC)load( "glNormal3s" );
	glad_glNormal3sv = (PFNGLNORMAL3SVPROC)load( "glNormal3sv" );
	glad_glRasterPos2d = (PFNGLRASTERPOS2DPROC)load( "glRasterPos2d" );
	glad_glRasterPos2dv = (PFNGLRASTERPOS2DVPROC)load( "glRasterPos2dv" );
	glad_glRasterPos2f = (PFNGLRASTERPOS2FPROC)load( "glRasterPos2f" );
	glad_glRasterPos2fv = (PFNGLRASTERPOS2FVPROC)load( "glRasterPos2fv" );
	glad_glRasterPos2i = (PFNGLRASTERPOS2IPROC)load( "glRasterPos2i" );
	glad_glRasterPos2iv = (PFNGLRASTERPOS2IVPROC)load( "glRasterPos2iv" );
	glad_glRasterPos2s = (PFNGLRASTERPOS2SPROC)load( "glRasterPos2s" );
	glad_glRasterPos2sv = (PFNGLRASTERPOS2SVPROC)load( "glRasterPos2sv" );
	glad_glRasterPos3d = (PFNGLRASTERPOS3DPROC)load( "glRasterPos3d" );
	glad_glRasterPos3dv = (PFNGLRASTERPOS3DVPROC)load( "glRasterPos3dv" );
	glad_glRasterPos3f = (PFNGLRASTERPOS3FPROC)load( "glRasterPos3f" );
	glad_glRasterPos3fv = (PFNGLRASTERPOS3FVPROC)load( "glRasterPos3fv" );
	glad_glRasterPos3i = (PFNGLRASTERPOS3IPROC)load( "glRasterPos3i" );
	glad_glRasterPos3iv = (PFNGLRASTERPOS3IVPROC)load( "glRasterPos3iv" );
	glad_glRasterPos3s = (PFNGLRASTERPOS3SPROC)load( "glRasterPos3s" );
	glad_glRasterPos3sv = (PFNGLRASTERPOS3SVPROC)load( "glRasterPos3sv" );
	glad_glRasterPos4d = (PFNGLRASTERPOS4DPROC)load( "glRasterPos4d" );
	glad_glRasterPos4dv = (PFNGLRASTERPOS4DVPROC)load( "glRasterPos4dv" );
	glad_glRasterPos4f = (PFNGLRASTERPOS4FPROC)load( "glRasterPos4f" );
	glad_glRasterPos4fv = (PFNGLRASTERPOS4FVPROC)load( "glRasterPos4fv" );
	glad_glRasterPos4i = (PFNGLRASTERPOS4IPROC)load( "glRasterPos4i" );
	glad_glRasterPos4iv = (PFNGLRASTERPOS4IVPROC)load( "glRasterPos4iv" );
	glad_glRasterPos4s = (PFNGLRASTERPOS4SPROC)load( "glRasterPos4s" );
	glad_glRasterPos4sv = (PFNGLRASTERPOS4SVPROC)load( "glRasterPos4sv" );
	glad_glRectd = (PFNGLRECTDPROC)load( "glRectd" );
	glad_glRectdv = (PFNGLRECTDVPROC)load( "glRectdv" );
	glad_glRectf = (PFNGLRECTFPROC)load( "glRectf" );
	glad_glRectfv = (PFNGLRECTFVPROC)load( "glRectfv" );
	glad_glRecti = (PFNGLRECTIPROC)load( "glRecti" );
	glad_glRectiv = (PFNGLRECTIVPROC)load( "glRectiv" );
	glad_glRects = (PFNGLRECTSPROC)load( "glRects" );
	glad_glRectsv = (PFNGLRECTSVPROC)load( "glRectsv" );
	glad_glTexCoord1d = (PFNGLTEXCOORD1DPROC)load( "glTexCoord1d" );
	glad_glTexCoord1dv = (PFNGLTEXCOORD1DVPROC)load( "glTexCoord1dv" );
	glad_glTexCoord1f = (PFNGLTEXCOORD1FPROC)load( "glTexCoord1f" );
	glad_glTexCoord1fv = (PFNGLTEXCOORD1FVPROC)load( "glTexCoord1fv" );
	glad_glTexCoord1i = (PFNGLTEXCOORD1IPROC)load( "glTexCoord1i" );
	glad_glTexCoord1iv = (PFNGLTEXCOORD1IVPROC)load( "glTexCoord1iv" );
	glad_glTexCoord1s = (PFNGLTEXCOORD1SPROC)load( "glTexCoord1s" );
	glad_glTexCoord1sv = (PFNGLTEXCOORD1SVPROC)load( "glTexCoord1sv" );
	glad_glTexCoord2d = (PFNGLTEXCOORD2DPROC)load( "glTexCoord2d" );
	glad_glTexCoord2dv = (PFNGLTEXCOORD2DVPROC)load( "glTexCoord2dv" );
	glad_glTexCoord2f = (PFNGLTEXCOORD2FPROC)load( "glTexCoord2f" );
	glad_glTexCoord2fv = (PFNGLTEXCOORD2FVPROC)load( "glTexCoord2fv" );
	glad_glTexCoord2i = (PFNGLTEXCOORD2IPROC)load( "glTexCoord2i" );
	glad_glTexCoord2iv = (PFNGLTEXCOORD2IVPROC)load( "glTexCoord2iv" );
	glad_glTexCoord2s = (PFNGLTEXCOORD2SPROC)load( "glTexCoord2s" );
	glad_glTexCoord2sv = (PFNGLTEXCOORD2SVPROC)load( "glTexCoord2sv" );
	glad_glTexCoord3d = (PFNGLTEXCOORD3DPROC)load( "glTexCoord3d" );
	glad_glTexCoord3dv = (PFNGLTEXCOORD3DVPROC)load( "glTexCoord3dv" );
	glad_glTexCoord3f = (PFNGLTEXCOORD3FPROC)load( "glTexCoord3f" );
	glad_glTexCoord3fv = (PFNGLTEXCOORD3FVPROC)load( "glTexCoord3fv" );
	glad_glTexCoord3i = (PFNGLTEXCOORD3IPROC)load( "glTexCoord3i" );
	glad_glTexCoord3iv = (PFNGLTEXCOORD3IVPROC)load( "glTexCoord3iv" );
	glad_glTexCoord3s = (PFNGLTEXCOORD3SPROC)load( "glTexCoord3s" );
	glad_glTexCoord3sv = (PFNGLTEXCOORD3SVPROC)load( "glTexCoord3sv" );
	glad_glTexCoord4d = (PFNGLTEXCOORD4DPROC)load( "glTexCoord4d" );
	glad_glTexCoord4dv = (PFNGLTEXCOORD4DVPROC)load( "glTexCoord4dv" );
	glad_glTexCoord4f = (PFNGLTEXCOORD4FPROC)load( "glTexCoord4f" );
	glad_glTexCoord4fv = (PFNGLTEXCOORD4FVPROC)load( "glTexCoord4fv" );
	glad_glTexCoord4i = (PFNGLTEXCOORD4IPROC)load( "glTexCoord4i" );
	glad_glTexCoord4iv = (PFNGLTEXCOORD4IVPROC)load( "glTexCoord4iv" );
	glad_glTexCoord4s = (PFNGLTEXCOORD4SPROC)load( "glTexCoord4s" );
	glad_glTexCoord4sv = (PFNGLTEXCOORD4SVPROC)load( "glTexCoord4sv" );
	glad_glVertex2d = (PFNGLVERTEX2DPROC)load( "glVertex2d" );
	glad_glVertex2dv = (PFNGLVERTEX2DVPROC)load( "glVertex2dv" );
	glad_glVertex2f = (PFNGLVERTEX2FPROC)load( "glVertex2f" );
	glad_glVertex2fv = (PFNGLVERTEX2FVPROC)load( "glVertex2fv" );
	glad_glVertex2i = (PFNGLVERTEX2IPROC)load( "glVertex2i" );
	glad_glVertex2iv = (PFNGLVERTEX2IVPROC)load( "glVertex2iv" );
	glad_glVertex2s = (PFNGLVERTEX2SPROC)load( "glVertex2s" );
	glad_glVertex2sv = (PFNGLVERTEX2SVPROC)load( "glVertex2sv" );
	glad_glVertex3d = (PFNGLVERTEX3DPROC)load( "glVertex3d" );
	glad_glVertex3dv = (PFNGLVERTEX3DVPROC)load( "glVertex3dv" );
	glad_glVertex3f = (PFNGLVERTEX3FPROC)load( "glVertex3f" );
	glad_glVertex3fv = (PFNGLVERTEX3FVPROC)load( "glVertex3fv" );
	glad_glVertex3i = (PFNGLVERTEX3IPROC)load( "glVertex3i" );
	glad_glVertex3iv = (PFNGLVERTEX3IVPROC)load( "glVertex3iv" );
	glad_glVertex3s = (PFNGLVERTEX3SPROC)load( "glVertex3s" );
	glad_glVertex3sv = (PFNGLVERTEX3SVPROC)load( "glVertex3sv" );
	glad_glVertex4d = (PFNGLVERTEX4DPROC)load( "glVertex4d" );
	glad_glVertex4dv = (PFNGLVERTEX4DVPROC)load( "glVertex4dv" );
	glad_glVertex4f = (PFNGLVERTEX4FPROC)load( "glVertex4f" );
	glad_glVertex4fv = (PFNGLVERTEX4FVPROC)load( "glVertex4fv" );
	glad_glVertex4i = (PFNGLVERTEX4IPROC)load( "glVertex4i" );
	glad_glVertex4iv = (PFNGLVERTEX4IVPROC)load( "glVertex4iv" );
	glad_glVertex4s = (PFNGLVERTEX4SPROC)load( "glVertex4s" );
	glad_glVertex4sv = (PFNGLVERTEX4SVPROC)load( "glVertex4sv" );
	glad_glClipPlane = (PFNGLCLIPPLANEPROC)load( "glClipPlane" );
	glad_glColorMaterial = (PFNGLCOLORMATERIALPROC)load( "glColorMaterial" );
	glad_glFogf = (PFNGLFOGFPROC)load( "glFogf" );
	glad_glFogfv = (PFNGLFOGFVPROC)load( "glFogfv" );
	glad_glFogi = (PFNGLFOGIPROC)load( "glFogi" );
	glad_glFogiv = (PFNGLFOGIVPROC)load( "glFogiv" );
	glad_glLightf = (PFNGLLIGHTFPROC)load( "glLightf" );
	glad_glLightfv = (PFNGLLIGHTFVPROC)load( "glLightfv" );
	glad_glLighti = (PFNGLLIGHTIPROC)load( "glLighti" );
	glad_glLightiv = (PFNGLLIGHTIVPROC)load( "glLightiv" );
	glad_glLightModelf = (PFNGLLIGHTMODELFPROC)load( "glLightModelf" );
	glad_glLightModelfv = (PFNGLLIGHTMODELFVPROC)load( "glLightModelfv" );
	glad_glLightModeli = (PFNGLLIGHTMODELIPROC)load( "glLightModeli" );
	glad_glLightModeliv = (PFNGLLIGHTMODELIVPROC)load( "glLightModeliv" );
	glad_glLineStipple = (PFNGLLINESTIPPLEPROC)load( "glLineStipple" );
	glad_glMaterialf = (PFNGLMATERIALFPROC)load( "glMaterialf" );
	glad_glMaterialfv = (PFNGLMATERIALFVPROC)load( "glMaterialfv" );
	glad_glMateriali = (PFNGLMATERIALIPROC)load( "glMateriali" );
	glad_glMaterialiv = (PFNGLMATERIALIVPROC)load( "glMaterialiv" );
	glad_glPolygonStipple = (PFNGLPOLYGONSTIPPLEPROC)load( "glPolygonStipple" );
	glad_glShadeModel = (PFNGLSHADEMODELPROC)load( "glShadeModel" );
	glad_glTexEnvf = (PFNGLTEXENVFPROC)load( "glTexEnvf" );
	glad_glTexEnvfv = (PFNGLTEXENVFVPROC)load( "glTexEnvfv" );
	glad_glTexEnvi = (PFNGLTEXENVIPROC)load( "glTexEnvi" );
	glad_glTexEnviv = (PFNGLTEXENVIVPROC)load( "glTexEnviv" );
	glad_glTexGend = (PFNGLTEXGENDPROC)load( "glTexGend" );
	glad_glTexGendv = (PFNGLTEXGENDVPROC)load( "glTexGendv" );
	glad_glTexGenf = (PFNGLTEXGENFPROC)load( "glTexGenf" );
	glad_glTexGenfv = (PFNGLTEXGENFVPROC)load( "glTexGenfv" );
	glad_glTexGeni = (PFNGLTEXGENIPROC)load( "glTexGeni" );
	glad_glTexGeniv = (PFNGLTEXGENIVPROC)load( "glTexGeniv" );
	glad_glFeedbackBuffer = (PFNGLFEEDBACKBUFFERPROC)load( "glFeedbackBuffer" );
	glad_glSelectBuffer = (PFNGLSELECTBUFFERPROC)load( "glSelectBuffer" );
	glad_glRenderMode = (PFNGLRENDERMODEPROC)load( "glRenderMode" );
	glad_glInitNames = (PFNGLINITNAMESPROC)load( "glInitNames" );
	glad_glLoadName = (PFNGLLOADNAMEPROC)load( "glLoadName" );
	glad_glPassThrough = (PFNGLPASSTHROUGHPROC)load( "glPassThrough" );
	glad_glPopName = (PFNGLPOPNAMEPROC)load( "glPopName" );
	glad_glPushName = (PFNGLPUSHNAMEPROC)load( "glPushName" );
	glad_glClearAccum = (PFNGLCLEARACCUMPROC)load( "glClearAccum" );
	glad_glClearIndex = (PFNGLCLEARINDEXPROC)load( "glClearIndex" );
	glad_glIndexMask = (PFNGLINDEXMASKPROC)load( "glIndexMask" );
	glad_glAccum = (PFNGLACCUMPROC)load( "glAccum" );
	glad_glPopAttrib = (PFNGLPOPATTRIBPROC)load( "glPopAttrib" );
	glad_glPushAttrib = (PFNGLPUSHATTRIBPROC)load( "glPushAttrib" );
	glad_glMap1d = (PFNGLMAP1DPROC)load( "glMap1d" );
	glad_glMap1f = (PFNGLMAP1FPROC)load( "glMap1f" );
	glad_glMap2d = (PFNGLMAP2DPROC)load( "glMap2d" );
	glad_glMap2f = (PFNGLMAP2FPROC)load( "glMap2f" );
	glad_glMapGrid1d = (PFNGLMAPGRID1DPROC)load( "glMapGrid1d" );
	glad_glMapGrid1f = (PFNGLMAPGRID1FPROC)load( "glMapGrid1f" );
	glad_glMapGrid2d = (PFNGLMAPGRID2DPROC)load( "glMapGrid2d" );
	glad_glMapGrid2f = (PFNGLMAPGRID2FPROC)load( "glMapGrid2f" );
	glad_glEvalCoord1d = (PFNGLEVALCOORD1DPROC)load( "glEvalCoord1d" );
	glad_glEvalCoord1dv = (PFNGLEVALCOORD1DVPROC)load( "glEvalCoord1dv" );
	glad_glEvalCoord1f = (PFNGLEVALCOORD1FPROC)load( "glEvalCoord1f" );
	glad_glEvalCoord1fv = (PFNGLEVALCOORD1FVPROC)load( "glEvalCoord1fv" );
	glad_glEvalCoord2d = (PFNGLEVALCOORD2DPROC)load( "glEvalCoord2d" );
	glad_glEvalCoord2dv = (PFNGLEVALCOORD2DVPROC)load( "glEvalCoord2dv" );
	glad_glEvalCoord2f = (PFNGLEVALCOORD2FPROC)load( "glEvalCoord2f" );
	glad_glEvalCoord2fv = (PFNGLEVALCOORD2FVPROC)load( "glEvalCoord2fv" );
	glad_glEvalMesh1 = (PFNGLEVALMESH1PROC)load( "glEvalMesh1" );
	glad_glEvalPoint1 = (PFNGLEVALPOINT1PROC)load( "glEvalPoint1" );
	glad_glEvalMesh2 = (PFNGLEVALMESH2PROC)load( "glEvalMesh2" );
	glad_glEvalPoint2 = (PFNGLEVALPOINT2PROC)load( "glEvalPoint2" );
	glad_glAlphaFunc = (PFNGLALPHAFUNCPROC)load( "glAlphaFunc" );
	glad_glPixelZoom = (PFNGLPIXELZOOMPROC)load( "glPixelZoom" );
	glad_glPixelTransferf = (PFNGLPIXELTRANSFERFPROC)load( "glPixelTransferf" );
	glad_glPixelTransferi = (PFNGLPIXELTRANSFERIPROC)load( "glPixelTransferi" );
	glad_glPixelMapfv = (PFNGLPIXELMAPFVPROC)load( "glPixelMapfv" );
	glad_glPixelMapuiv = (PFNGLPIXELMAPUIVPROC)load( "glPixelMapuiv" );
	glad_glPixelMapusv = (PFNGLPIXELMAPUSVPROC)load( "glPixelMapusv" );
	glad_glCopyPixels = (PFNGLCOPYPIXELSPROC)load( "glCopyPixels" );
	glad_glDrawPixels = (PFNGLDRAWPIXELSPROC)load( "glDrawPixels" );
	glad_glGetClipPlane = (PFNGLGETCLIPPLANEPROC)load( "glGetClipPlane" );
	glad_glGetLightfv = (PFNGLGETLIGHTFVPROC)load( "glGetLightfv" );
	glad_glGetLightiv = (PFNGLGETLIGHTIVPROC)load( "glGetLightiv" );
	glad_glGetMapdv = (PFNGLGETMAPDVPROC)load( "glGetMapdv" );
	glad_glGetMapfv = (PFNGLGETMAPFVPROC)load( "glGetMapfv" );
	glad_glGetMapiv = (PFNGLGETMAPIVPROC)load( "glGetMapiv" );
	glad_glGetMaterialfv = (PFNGLGETMATERIALFVPROC)load( "glGetMaterialfv" );
	glad_glGetMaterialiv = (PFNGLGETMATERIALIVPROC)load( "glGetMaterialiv" );
	glad_glGetPixelMapfv = (PFNGLGETPIXELMAPFVPROC)load( "glGetPixelMapfv" );
	glad_glGetPixelMapuiv = (PFNGLGETPIXELMAPUIVPROC)load( "glGetPixelMapuiv" );
	glad_glGetPixelMapusv = (PFNGLGETPIXELMAPUSVPROC)load( "glGetPixelMapusv" );
	glad_glGetPolygonStipple = (PFNGLGETPOLYGONSTIPPLEPROC)load( "glGetPolygonStipple" );
	glad_glGetTexEnvfv = (PFNGLGETTEXENVFVPROC)load( "glGetTexEnvfv" );
	glad_glGetTexEnviv = (PFNGLGETTEXENVIVPROC)load( "glGetTexEnviv" );
	glad_glGetTexGendv = (PFNGLGETTEXGENDVPROC)load( "glGetTexGendv" );
	glad_glGetTexGenfv = (PFNGLGETTEXGENFVPROC)load( "glGetTexGenfv" );
	glad_glGetTexGeniv = (PFNGLGETTEXGENIVPROC)load( "glGetTexGeniv" );
	glad_glIsList = (PFNGLISLISTPROC)load( "glIsList" );
	glad_glFrustum = (PFNGLFRUSTUMPROC)load( "glFrustum" );
	glad_glLoadIdentity = (PFNGLLOADIDENTITYPROC)load( "glLoadIdentity" );
	glad_glLoadMatrixf = (PFNGLLOADMATRIXFPROC)load( "glLoadMatrixf" );
	glad_glLoadMatrixd = (PFNGLLOADMATRIXDPROC)load( "glLoadMatrixd" );
	glad_glMatrixMode = (PFNGLMATRIXMODEPROC)load( "glMatrixMode" );
	glad_glMultMatrixf = (PFNGLMULTMATRIXFPROC)load( "glMultMatrixf" );
	glad_glMultMatrixd = (PFNGLMULTMATRIXDPROC)load( "glMultMatrixd" );
	glad_glOrtho = (PFNGLORTHOPROC)load( "glOrtho" );
	glad_glPopMatrix = (PFNGLPOPMATRIXPROC)load( "glPopMatrix" );
	glad_glPushMatrix = (PFNGLPUSHMATRIXPROC)load( "glPushMatrix" );
	glad_glRotated = (PFNGLROTATEDPROC)load( "glRotated" );
	glad_glRotatef = (PFNGLROTATEFPROC)load( "glRotatef" );
	glad_glScaled = (PFNGLSCALEDPROC)load( "glScaled" );
	glad_glScalef = (PFNGLSCALEFPROC)load( "glScalef" );
	glad_glTranslated = (PFNGLTRANSLATEDPROC)load( "glTranslated" );
	glad_glTranslatef = (PFNGLTRANSLATEFPROC)load( "glTranslatef" );
}
static void load_GL_VERSION_1_1( GLADloadproc load )
{
	if (!GLAD_GL_VERSION_1_1) return;
	glad_glDrawArrays = (PFNGLDRAWARRAYSPROC)load( "glDrawArrays" );
	glad_glDrawElements = (PFNGLDRAWELEMENTSPROC)load( "glDrawElements" );
	glad_glGetPointerv = (PFNGLGETPOINTERVPROC)load( "glGetPointerv" );
	glad_glPolygonOffset = (PFNGLPOLYGONOFFSETPROC)load( "glPolygonOffset" );
	glad_glCopyTexImage1D = (PFNGLCOPYTEXIMAGE1DPROC)load( "glCopyTexImage1D" );
	glad_glCopyTexImage2D = (PFNGLCOPYTEXIMAGE2DPROC)load( "glCopyTexImage2D" );
	glad_glCopyTexSubImage1D = (PFNGLCOPYTEXSUBIMAGE1DPROC)load( "glCopyTexSubImage1D" );
	glad_glCopyTexSubImage2D = (PFNGLCOPYTEXSUBIMAGE2DPROC)load( "glCopyTexSubImage2D" );
	glad_glTexSubImage1D = (PFNGLTEXSUBIMAGE1DPROC)load( "glTexSubImage1D" );
	glad_glTexSubImage2D = (PFNGLTEXSUBIMAGE2DPROC)load( "glTexSubImage2D" );
	glad_glBindTexture = (PFNGLBINDTEXTUREPROC)load( "glBindTexture" );
	glad_glDeleteTextures = (PFNGLDELETETEXTURESPROC)load( "glDeleteTextures" );
	glad_glGenTextures = (PFNGLGENTEXTURESPROC)load( "glGenTextures" );
	glad_glIsTexture = (PFNGLISTEXTUREPROC)load( "glIsTexture" );
	glad_glArrayElement = (PFNGLARRAYELEMENTPROC)load( "glArrayElement" );
	glad_glColorPointer = (PFNGLCOLORPOINTERPROC)load( "glColorPointer" );
	glad_glDisableClientState = (PFNGLDISABLECLIENTSTATEPROC)load( "glDisableClientState" );
	glad_glEdgeFlagPointer = (PFNGLEDGEFLAGPOINTERPROC)load( "glEdgeFlagPointer" );
	glad_glEnableClientState = (PFNGLENABLECLIENTSTATEPROC)load( "glEnableClientState" );
	glad_glIndexPointer = (PFNGLINDEXPOINTERPROC)load( "glIndexPointer" );
	glad_glInterleavedArrays = (PFNGLINTERLEAVEDARRAYSPROC)load( "glInterleavedArrays" );
	glad_glNormalPointer = (PFNGLNORMALPOINTERPROC)load( "glNormalPointer" );
	glad_glTexCoordPointer = (PFNGLTEXCOORDPOINTERPROC)load( "glTexCoordPointer" );
	glad_glVertexPointer = (PFNGLVERTEXPOINTERPROC)load( "glVertexPointer" );
	glad_glAreTexturesResident = (PFNGLARETEXTURESRESIDENTPROC)load( "glAreTexturesResident" );
	glad_glPrioritizeTextures = (PFNGLPRIORITIZETEXTURESPROC)load( "glPrioritizeTextures" );
	glad_glIndexub = (PFNGLINDEXUBPROC)load( "glIndexub" );
	glad_glIndexubv = (PFNGLINDEXUBVPROC)load( "glIndexubv" );
	glad_glPopClientAttrib = (PFNGLPOPCLIENTATTRIBPROC)load( "glPopClientAttrib" );
	glad_glPushClientAttrib = (PFNGLPUSHCLIENTATTRIBPROC)load( "glPushClientAttrib" );
}
static void load_GL_VERSION_1_2( GLADloadproc load )
{
	if (!GLAD_GL_VERSION_1_2) return;
	glad_glDrawRangeElements = (PFNGLDRAWRANGEELEMENTSPROC)load( "glDrawRangeElements" );
	glad_glTexImage3D = (PFNGLTEXIMAGE3DPROC)load( "glTexImage3D" );
	glad_glTexSubImage3D = (PFNGLTEXSUBIMAGE3DPROC)load( "glTexSubImage3D" );
	glad_glCopyTexSubImage3D = (PFNGLCOPYTEXSUBIMAGE3DPROC)load( "glCopyTexSubImage3D" );
}
static void load_GL_VERSION_1_3( GLADloadproc load )
{
	if (!GLAD_GL_VERSION_1_3) return;
	glad_glActiveTexture = (PFNGLACTIVETEXTUREPROC)load( "glActiveTexture" );
	glad_glSampleCoverage = (PFNGLSAMPLECOVERAGEPROC)load( "glSampleCoverage" );
	glad_glCompressedTexImage3D = (PFNGLCOMPRESSEDTEXIMAGE3DPROC)load( "glCompressedTexImage3D" );
	glad_glCompressedTexImage2D = (PFNGLCOMPRESSEDTEXIMAGE2DPROC)load( "glCompressedTexImage2D" );
	glad_glCompressedTexImage1D = (PFNGLCOMPRESSEDTEXIMAGE1DPROC)load( "glCompressedTexImage1D" );
	glad_glCompressedTexSubImage3D = (PFNGLCOMPRESSEDTEXSUBIMAGE3DPROC)load( "glCompressedTexSubImage3D" );
	glad_glCompressedTexSubImage2D = (PFNGLCOMPRESSEDTEXSUBIMAGE2DPROC)load( "glCompressedTexSubImage2D" );
	glad_glCompressedTexSubImage1D = (PFNGLCOMPRESSEDTEXSUBIMAGE1DPROC)load( "glCompressedTexSubImage1D" );
	glad_glGetCompressedTexImage = (PFNGLGETCOMPRESSEDTEXIMAGEPROC)load( "glGetCompressedTexImage" );
	glad_glClientActiveTexture = (PFNGLCLIENTACTIVETEXTUREPROC)load( "glClientActiveTexture" );
	glad_glMultiTexCoord1d = (PFNGLMULTITEXCOORD1DPROC)load( "glMultiTexCoord1d" );
	glad_glMultiTexCoord1dv = (PFNGLMULTITEXCOORD1DVPROC)load( "glMultiTexCoord1dv" );
	glad_glMultiTexCoord1f = (PFNGLMULTITEXCOORD1FPROC)load( "glMultiTexCoord1f" );
	glad_glMultiTexCoord1fv = (PFNGLMULTITEXCOORD1FVPROC)load( "glMultiTexCoord1fv" );
	glad_glMultiTexCoord1i = (PFNGLMULTITEXCOORD1IPROC)load( "glMultiTexCoord1i" );
	glad_glMultiTexCoord1iv = (PFNGLMULTITEXCOORD1IVPROC)load( "glMultiTexCoord1iv" );
	glad_glMultiTexCoord1s = (PFNGLMULTITEXCOORD1SPROC)load( "glMultiTexCoord1s" );
	glad_glMultiTexCoord1sv = (PFNGLMULTITEXCOORD1SVPROC)load( "glMultiTexCoord1sv" );
	glad_glMultiTexCoord2d = (PFNGLMULTITEXCOORD2DPROC)load( "glMultiTexCoord2d" );
	glad_glMultiTexCoord2dv = (PFNGLMULTITEXCOORD2DVPROC)load( "glMultiTexCoord2dv" );
	glad_glMultiTexCoord2f = (PFNGLMULTITEXCOORD2FPROC)load( "glMultiTexCoord2f" );
	glad_glMultiTexCoord2fv = (PFNGLMULTITEXCOORD2FVPROC)load( "glMultiTexCoord2fv" );
	glad_glMultiTexCoord2i = (PFNGLMULTITEXCOORD2IPROC)load( "glMultiTexCoord2i" );
	glad_glMultiTexCoord2iv = (PFNGLMULTITEXCOORD2IVPROC)load( "glMultiTexCoord2iv" );
	glad_glMultiTexCoord2s = (PFNGLMULTITEXCOORD2SPROC)load( "glMultiTexCoord2s" );
	glad_glMultiTexCoord2sv = (PFNGLMULTITEXCOORD2SVPROC)load( "glMultiTexCoord2sv" );
	glad_glMultiTexCoord3d = (PFNGLMULTITEXCOORD3DPROC)load( "glMultiTexCoord3d" );
	glad_glMultiTexCoord3dv = (PFNGLMULTITEXCOORD3DVPROC)load( "glMultiTexCoord3dv" );
	glad_glMultiTexCoord3f = (PFNGLMULTITEXCOORD3FPROC)load( "glMultiTexCoord3f" );
	glad_glMultiTexCoord3fv = (PFNGLMULTITEXCOORD3FVPROC)load( "glMultiTexCoord3fv" );
	glad_glMultiTexCoord3i = (PFNGLMULTITEXCOORD3IPROC)load( "glMultiTexCoord3i" );
	glad_glMultiTexCoord3iv = (PFNGLMULTITEXCOORD3IVPROC)load( "glMultiTexCoord3iv" );
	glad_glMultiTexCoord3s = (PFNGLMULTITEXCOORD3SPROC)load( "glMultiTexCoord3s" );
	glad_glMultiTexCoord3sv = (PFNGLMULTITEXCOORD3SVPROC)load( "glMultiTexCoord3sv" );
	glad_glMultiTexCoord4d = (PFNGLMULTITEXCOORD4DPROC)load( "glMultiTexCoord4d" );
	glad_glMultiTexCoord4dv = (PFNGLMULTITEXCOORD4DVPROC)load( "glMultiTexCoord4dv" );
	glad_glMultiTexCoord4f = (PFNGLMULTITEXCOORD4FPROC)load( "glMultiTexCoord4f" );
	glad_glMultiTexCoord4fv = (PFNGLMULTITEXCOORD4FVPROC)load( "glMultiTexCoord4fv" );
	glad_glMultiTexCoord4i = (PFNGLMULTITEXCOORD4IPROC)load( "glMultiTexCoord4i" );
	glad_glMultiTexCoord4iv = (PFNGLMULTITEXCOORD4IVPROC)load( "glMultiTexCoord4iv" );
	glad_glMultiTexCoord4s = (PFNGLMULTITEXCOORD4SPROC)load( "glMultiTexCoord4s" );
	glad_glMultiTexCoord4sv = (PFNGLMULTITEXCOORD4SVPROC)load( "glMultiTexCoord4sv" );
	glad_glLoadTransposeMatrixf = (PFNGLLOADTRANSPOSEMATRIXFPROC)load( "glLoadTransposeMatrixf" );
	glad_glLoadTransposeMatrixd = (PFNGLLOADTRANSPOSEMATRIXDPROC)load( "glLoadTransposeMatrixd" );
	glad_glMultTransposeMatrixf = (PFNGLMULTTRANSPOSEMATRIXFPROC)load( "glMultTransposeMatrixf" );
	glad_glMultTransposeMatrixd = (PFNGLMULTTRANSPOSEMATRIXDPROC)load( "glMultTransposeMatrixd" );
}
static void load_GL_VERSION_1_4( GLADloadproc load )
{
	if (!GLAD_GL_VERSION_1_4) return;
	glad_glBlendFuncSeparate = (PFNGLBLENDFUNCSEPARATEPROC)load( "glBlendFuncSeparate" );
	glad_glMultiDrawArrays = (PFNGLMULTIDRAWARRAYSPROC)load( "glMultiDrawArrays" );
	glad_glMultiDrawElements = (PFNGLMULTIDRAWELEMENTSPROC)load( "glMultiDrawElements" );
	glad_glPointParameterf = (PFNGLPOINTPARAMETERFPROC)load( "glPointParameterf" );
	glad_glPointParameterfv = (PFNGLPOINTPARAMETERFVPROC)load( "glPointParameterfv" );
	glad_glPointParameteri = (PFNGLPOINTPARAMETERIPROC)load( "glPointParameteri" );
	glad_glPointParameteriv = (PFNGLPOINTPARAMETERIVPROC)load( "glPointParameteriv" );
	glad_glFogCoordf = (PFNGLFOGCOORDFPROC)load( "glFogCoordf" );
	glad_glFogCoordfv = (PFNGLFOGCOORDFVPROC)load( "glFogCoordfv" );
	glad_glFogCoordd = (PFNGLFOGCOORDDPROC)load( "glFogCoordd" );
	glad_glFogCoorddv = (PFNGLFOGCOORDDVPROC)load( "glFogCoorddv" );
	glad_glFogCoordPointer = (PFNGLFOGCOORDPOINTERPROC)load( "glFogCoordPointer" );
	glad_glSecondaryColor3b = (PFNGLSECONDARYCOLOR3BPROC)load( "glSecondaryColor3b" );
	glad_glSecondaryColor3bv = (PFNGLSECONDARYCOLOR3BVPROC)load( "glSecondaryColor3bv" );
	glad_glSecondaryColor3d = (PFNGLSECONDARYCOLOR3DPROC)load( "glSecondaryColor3d" );
	glad_glSecondaryColor3dv = (PFNGLSECONDARYCOLOR3DVPROC)load( "glSecondaryColor3dv" );
	glad_glSecondaryColor3f = (PFNGLSECONDARYCOLOR3FPROC)load( "glSecondaryColor3f" );
	glad_glSecondaryColor3fv = (PFNGLSECONDARYCOLOR3FVPROC)load( "glSecondaryColor3fv" );
	glad_glSecondaryColor3i = (PFNGLSECONDARYCOLOR3IPROC)load( "glSecondaryColor3i" );
	glad_glSecondaryColor3iv = (PFNGLSECONDARYCOLOR3IVPROC)load( "glSecondaryColor3iv" );
	glad_glSecondaryColor3s = (PFNGLSECONDARYCOLOR3SPROC)load( "glSecondaryColor3s" );
	glad_glSecondaryColor3sv = (PFNGLSECONDARYCOLOR3SVPROC)load( "glSecondaryColor3sv" );
	glad_glSecondaryColor3ub = (PFNGLSECONDARYCOLOR3UBPROC)load( "glSecondaryColor3ub" );
	glad_glSecondaryColor3ubv = (PFNGLSECONDARYCOLOR3UBVPROC)load( "glSecondaryColor3ubv" );
	glad_glSecondaryColor3ui = (PFNGLSECONDARYCOLOR3UIPROC)load( "glSecondaryColor3ui" );
	glad_glSecondaryColor3uiv = (PFNGLSECONDARYCOLOR3UIVPROC)load( "glSecondaryColor3uiv" );
	glad_glSecondaryColor3us = (PFNGLSECONDARYCOLOR3USPROC)load( "glSecondaryColor3us" );
	glad_glSecondaryColor3usv = (PFNGLSECONDARYCOLOR3USVPROC)load( "glSecondaryColor3usv" );
	glad_glSecondaryColorPointer = (PFNGLSECONDARYCOLORPOINTERPROC)load( "glSecondaryColorPointer" );
	glad_glWindowPos2d = (PFNGLWINDOWPOS2DPROC)load( "glWindowPos2d" );
	glad_glWindowPos2dv = (PFNGLWINDOWPOS2DVPROC)load( "glWindowPos2dv" );
	glad_glWindowPos2f = (PFNGLWINDOWPOS2FPROC)load( "glWindowPos2f" );
	glad_glWindowPos2fv = (PFNGLWINDOWPOS2FVPROC)load( "glWindowPos2fv" );
	glad_glWindowPos2i = (PFNGLWINDOWPOS2IPROC)load( "glWindowPos2i" );
	glad_glWindowPos2iv = (PFNGLWINDOWPOS2IVPROC)load( "glWindowPos2iv" );
	glad_glWindowPos2s = (PFNGLWINDOWPOS2SPROC)load( "glWindowPos2s" );
	glad_glWindowPos2sv = (PFNGLWINDOWPOS2SVPROC)load( "glWindowPos2sv" );
	glad_glWindowPos3d = (PFNGLWINDOWPOS3DPROC)load( "glWindowPos3d" );
	glad_glWindowPos3dv = (PFNGLWINDOWPOS3DVPROC)load( "glWindowPos3dv" );
	glad_glWindowPos3f = (PFNGLWINDOWPOS3FPROC)load( "glWindowPos3f" );
	glad_glWindowPos3fv = (PFNGLWINDOWPOS3FVPROC)load( "glWindowPos3fv" );
	glad_glWindowPos3i = (PFNGLWINDOWPOS3IPROC)load( "glWindowPos3i" );
	glad_glWindowPos3iv = (PFNGLWINDOWPOS3IVPROC)load( "glWindowPos3iv" );
	glad_glWindowPos3s = (PFNGLWINDOWPOS3SPROC)load( "glWindowPos3s" );
	glad_glWindowPos3sv = (PFNGLWINDOWPOS3SVPROC)load( "glWindowPos3sv" );
	glad_glBlendColor = (PFNGLBLENDCOLORPROC)load( "glBlendColor" );
	glad_glBlendEquation = (PFNGLBLENDEQUATIONPROC)load( "glBlendEquation" );
}
static void load_GL_VERSION_1_5( GLADloadproc load )
{
	if (!GLAD_GL_VERSION_1_5) return;
	glad_glGenQueries = (PFNGLGENQUERIESPROC)load( "glGenQueries" );
	glad_glDeleteQueries = (PFNGLDELETEQUERIESPROC)load( "glDeleteQueries" );
	glad_glIsQuery = (PFNGLISQUERYPROC)load( "glIsQuery" );
	glad_glBeginQuery = (PFNGLBEGINQUERYPROC)load( "glBeginQuery" );
	glad_glEndQuery = (PFNGLENDQUERYPROC)load( "glEndQuery" );
	glad_glGetQueryiv = (PFNGLGETQUERYIVPROC)load( "glGetQueryiv" );
	glad_glGetQueryObjectiv = (PFNGLGETQUERYOBJECTIVPROC)load( "glGetQueryObjectiv" );
	glad_glGetQueryObjectuiv = (PFNGLGETQUERYOBJECTUIVPROC)load( "glGetQueryObjectuiv" );
	glad_glBindBuffer = (PFNGLBINDBUFFERPROC)load( "glBindBuffer" );
	glad_glDeleteBuffers = (PFNGLDELETEBUFFERSPROC)load( "glDeleteBuffers" );
	glad_glGenBuffers = (PFNGLGENBUFFERSPROC)load( "glGenBuffers" );
	glad_glIsBuffer = (PFNGLISBUFFERPROC)load( "glIsBuffer" );
	glad_glBufferData = (PFNGLBUFFERDATAPROC)load( "glBufferData" );
	glad_glBufferSubData = (PFNGLBUFFERSUBDATAPROC)load( "glBufferSubData" );
	glad_glGetBufferSubData = (PFNGLGETBUFFERSUBDATAPROC)load( "glGetBufferSubData" );
	glad_glMapBuffer = (PFNGLMAPBUFFERPROC)load( "glMapBuffer" );
	glad_glUnmapBuffer = (PFNGLUNMAPBUFFERPROC)load( "glUnmapBuffer" );
	glad_glGetBufferParameteriv = (PFNGLGETBUFFERPARAMETERIVPROC)load( "glGetBufferParameteriv" );
	glad_glGetBufferPointerv = (PFNGLGETBUFFERPOINTERVPROC)load( "glGetBufferPointerv" );
}
static void load_GL_VERSION_2_0( GLADloadproc load )
{
	if (!GLAD_GL_VERSION_2_0) return;
	glad_glBlendEquationSeparate = (PFNGLBLENDEQUATIONSEPARATEPROC)load( "glBlendEquationSeparate" );
	glad_glDrawBuffers = (PFNGLDRAWBUFFERSPROC)load( "glDrawBuffers" );
	glad_glStencilOpSeparate = (PFNGLSTENCILOPSEPARATEPROC)load( "glStencilOpSeparate" );
	glad_glStencilFuncSeparate = (PFNGLSTENCILFUNCSEPARATEPROC)load( "glStencilFuncSeparate" );
	glad_glStencilMaskSeparate = (PFNGLSTENCILMASKSEPARATEPROC)load( "glStencilMaskSeparate" );
	glad_glAttachShader = (PFNGLATTACHSHADERPROC)load( "glAttachShader" );
	glad_glBindAttribLocation = (PFNGLBINDATTRIBLOCATIONPROC)load( "glBindAttribLocation" );
	glad_glCompileShader = (PFNGLCOMPILESHADERPROC)load( "glCompileShader" );
	glad_glCreateProgram = (PFNGLCREATEPROGRAMPROC)load( "glCreateProgram" );
	glad_glCreateShader = (PFNGLCREATESHADERPROC)load( "glCreateShader" );
	glad_glDeleteProgram = (PFNGLDELETEPROGRAMPROC)load( "glDeleteProgram" );
	glad_glDeleteShader = (PFNGLDELETESHADERPROC)load( "glDeleteShader" );
	glad_glDetachShader = (PFNGLDETACHSHADERPROC)load( "glDetachShader" );
	glad_glDisableVertexAttribArray = (PFNGLDISABLEVERTEXATTRIBARRAYPROC)load( "glDisableVertexAttribArray" );
	glad_glEnableVertexAttribArray = (PFNGLENABLEVERTEXATTRIBARRAYPROC)load( "glEnableVertexAttribArray" );
	glad_glGetActiveAttrib = (PFNGLGETACTIVEATTRIBPROC)load( "glGetActiveAttrib" );
	glad_glGetActiveUniform = (PFNGLGETACTIVEUNIFORMPROC)load( "glGetActiveUniform" );
	glad_glGetAttachedShaders = (PFNGLGETATTACHEDSHADERSPROC)load( "glGetAttachedShaders" );
	glad_glGetAttribLocation = (PFNGLGETATTRIBLOCATIONPROC)load( "glGetAttribLocation" );
	glad_glGetProgramiv = (PFNGLGETPROGRAMIVPROC)load( "glGetProgramiv" );
	glad_glGetProgramInfoLog = (PFNGLGETPROGRAMINFOLOGPROC)load( "glGetProgramInfoLog" );
	glad_glGetShaderiv = (PFNGLGETSHADERIVPROC)load( "glGetShaderiv" );
	glad_glGetShaderInfoLog = (PFNGLGETSHADERINFOLOGPROC)load( "glGetShaderInfoLog" );
	glad_glGetShaderSource = (PFNGLGETSHADERSOURCEPROC)load( "glGetShaderSource" );
	glad_glGetUniformLocation = (PFNGLGETUNIFORMLOCATIONPROC)load( "glGetUniformLocation" );
	glad_glGetUniformfv = (PFNGLGETUNIFORMFVPROC)load( "glGetUniformfv" );
	glad_glGetUniformiv = (PFNGLGETUNIFORMIVPROC)load( "glGetUniformiv" );
	glad_glGetVertexAttribdv = (PFNGLGETVERTEXATTRIBDVPROC)load( "glGetVertexAttribdv" );
	glad_glGetVertexAttribfv = (PFNGLGETVERTEXATTRIBFVPROC)load( "glGetVertexAttribfv" );
	glad_glGetVertexAttribiv = (PFNGLGETVERTEXATTRIBIVPROC)load( "glGetVertexAttribiv" );
	glad_glGetVertexAttribPointerv = (PFNGLGETVERTEXATTRIBPOINTERVPROC)load( "glGetVertexAttribPointerv" );
	glad_glIsProgram = (PFNGLISPROGRAMPROC)load( "glIsProgram" );
	glad_glIsShader = (PFNGLISSHADERPROC)load( "glIsShader" );
	glad_glLinkProgram = (PFNGLLINKPROGRAMPROC)load( "glLinkProgram" );
	glad_glShaderSource = (PFNGLSHADERSOURCEPROC)load( "glShaderSource" );
	glad_glUseProgram = (PFNGLUSEPROGRAMPROC)load( "glUseProgram" );
	glad_glUniform1f = (PFNGLUNIFORM1FPROC)load( "glUniform1f" );
	glad_glUniform2f = (PFNGLUNIFORM2FPROC)load( "glUniform2f" );
	glad_glUniform3f = (PFNGLUNIFORM3FPROC)load( "glUniform3f" );
	glad_glUniform4f = (PFNGLUNIFORM4FPROC)load( "glUniform4f" );
	glad_glUniform1i = (PFNGLUNIFORM1IPROC)load( "glUniform1i" );
	glad_glUniform2i = (PFNGLUNIFORM2IPROC)load( "glUniform2i" );
	glad_glUniform3i = (PFNGLUNIFORM3IPROC)load( "glUniform3i" );
	glad_glUniform4i = (PFNGLUNIFORM4IPROC)load( "glUniform4i" );
	glad_glUniform1fv = (PFNGLUNIFORM1FVPROC)load( "glUniform1fv" );
	glad_glUniform2fv = (PFNGLUNIFORM2FVPROC)load( "glUniform2fv" );
	glad_glUniform3fv = (PFNGLUNIFORM3FVPROC)load( "glUniform3fv" );
	glad_glUniform4fv = (PFNGLUNIFORM4FVPROC)load( "glUniform4fv" );
	glad_glUniform1iv = (PFNGLUNIFORM1IVPROC)load( "glUniform1iv" );
	glad_glUniform2iv = (PFNGLUNIFORM2IVPROC)load( "glUniform2iv" );
	glad_glUniform3iv = (PFNGLUNIFORM3IVPROC)load( "glUniform3iv" );
	glad_glUniform4iv = (PFNGLUNIFORM4IVPROC)load( "glUniform4iv" );
	glad_glUniformMatrix2fv = (PFNGLUNIFORMMATRIX2FVPROC)load( "glUniformMatrix2fv" );
	glad_glUniformMatrix3fv = (PFNGLUNIFORMMATRIX3FVPROC)load( "glUniformMatrix3fv" );
	glad_glUniformMatrix4fv = (PFNGLUNIFORMMATRIX4FVPROC)load( "glUniformMatrix4fv" );
	glad_glValidateProgram = (PFNGLVALIDATEPROGRAMPROC)load( "glValidateProgram" );
	glad_glVertexAttrib1d = (PFNGLVERTEXATTRIB1DPROC)load( "glVertexAttrib1d" );
	glad_glVertexAttrib1dv = (PFNGLVERTEXATTRIB1DVPROC)load( "glVertexAttrib1dv" );
	glad_glVertexAttrib1f = (PFNGLVERTEXATTRIB1FPROC)load( "glVertexAttrib1f" );
	glad_glVertexAttrib1fv = (PFNGLVERTEXATTRIB1FVPROC)load( "glVertexAttrib1fv" );
	glad_glVertexAttrib1s = (PFNGLVERTEXATTRIB1SPROC)load( "glVertexAttrib1s" );
	glad_glVertexAttrib1sv = (PFNGLVERTEXATTRIB1SVPROC)load( "glVertexAttrib1sv" );
	glad_glVertexAttrib2d = (PFNGLVERTEXATTRIB2DPROC)load( "glVertexAttrib2d" );
	glad_glVertexAttrib2dv = (PFNGLVERTEXATTRIB2DVPROC)load( "glVertexAttrib2dv" );
	glad_glVertexAttrib2f = (PFNGLVERTEXATTRIB2FPROC)load( "glVertexAttrib2f" );
	glad_glVertexAttrib2fv = (PFNGLVERTEXATTRIB2FVPROC)load( "glVertexAttrib2fv" );
	glad_glVertexAttrib2s = (PFNGLVERTEXATTRIB2SPROC)load( "glVertexAttrib2s" );
	glad_glVertexAttrib2sv = (PFNGLVERTEXATTRIB2SVPROC)load( "glVertexAttrib2sv" );
	glad_glVertexAttrib3d = (PFNGLVERTEXATTRIB3DPROC)load( "glVertexAttrib3d" );
	glad_glVertexAttrib3dv = (PFNGLVERTEXATTRIB3DVPROC)load( "glVertexAttrib3dv" );
	glad_glVertexAttrib3f = (PFNGLVERTEXATTRIB3FPROC)load( "glVertexAttrib3f" );
	glad_glVertexAttrib3fv = (PFNGLVERTEXATTRIB3FVPROC)load( "glVertexAttrib3fv" );
	glad_glVertexAttrib3s = (PFNGLVERTEXATTRIB3SPROC)load( "glVertexAttrib3s" );
	glad_glVertexAttrib3sv = (PFNGLVERTEXATTRIB3SVPROC)load( "glVertexAttrib3sv" );
	glad_glVertexAttrib4Nbv = (PFNGLVERTEXATTRIB4NBVPROC)load( "glVertexAttrib4Nbv" );
	glad_glVertexAttrib4Niv = (PFNGLVERTEXATTRIB4NIVPROC)load( "glVertexAttrib4Niv" );
	glad_glVertexAttrib4Nsv = (PFNGLVERTEXATTRIB4NSVPROC)load( "glVertexAttrib4Nsv" );
	glad_glVertexAttrib4Nub = (PFNGLVERTEXATTRIB4NUBPROC)load( "glVertexAttrib4Nub" );
	glad_glVertexAttrib4Nubv = (PFNGLVERTEXATTRIB4NUBVPROC)load( "glVertexAttrib4Nubv" );
	glad_glVertexAttrib4Nuiv = (PFNGLVERTEXATTRIB4NUIVPROC)load( "glVertexAttrib4Nuiv" );
	glad_glVertexAttrib4Nusv = (PFNGLVERTEXATTRIB4NUSVPROC)load( "glVertexAttrib4Nusv" );
	glad_glVertexAttrib4bv = (PFNGLVERTEXATTRIB4BVPROC)load( "glVertexAttrib4bv" );
	glad_glVertexAttrib4d = (PFNGLVERTEXATTRIB4DPROC)load( "glVertexAttrib4d" );
	glad_glVertexAttrib4dv = (PFNGLVERTEXATTRIB4DVPROC)load( "glVertexAttrib4dv" );
	glad_glVertexAttrib4f = (PFNGLVERTEXATTRIB4FPROC)load( "glVertexAttrib4f" );
	glad_glVertexAttrib4fv = (PFNGLVERTEXATTRIB4FVPROC)load( "glVertexAttrib4fv" );
	glad_glVertexAttrib4iv = (PFNGLVERTEXATTRIB4IVPROC)load( "glVertexAttrib4iv" );
	glad_glVertexAttrib4s = (PFNGLVERTEXATTRIB4SPROC)load( "glVertexAttrib4s" );
	glad_glVertexAttrib4sv = (PFNGLVERTEXATTRIB4SVPROC)load( "glVertexAttrib4sv" );
	glad_glVertexAttrib4ubv = (PFNGLVERTEXATTRIB4UBVPROC)load( "glVertexAttrib4ubv" );
	glad_glVertexAttrib4uiv = (PFNGLVERTEXATTRIB4UIVPROC)load( "glVertexAttrib4uiv" );
	glad_glVertexAttrib4usv = (PFNGLVERTEXATTRIB4USVPROC)load( "glVertexAttrib4usv" );
	glad_glVertexAttribPointer = (PFNGLVERTEXATTRIBPOINTERPROC)load( "glVertexAttribPointer" );
}
static void load_GL_VERSION_2_1( GLADloadproc load )
{
	if (!GLAD_GL_VERSION_2_1) return;
	glad_glUniformMatrix2x3fv = (PFNGLUNIFORMMATRIX2X3FVPROC)load( "glUniformMatrix2x3fv" );
	glad_glUniformMatrix3x2fv = (PFNGLUNIFORMMATRIX3X2FVPROC)load( "glUniformMatrix3x2fv" );
	glad_glUniformMatrix2x4fv = (PFNGLUNIFORMMATRIX2X4FVPROC)load( "glUniformMatrix2x4fv" );
	glad_glUniformMatrix4x2fv = (PFNGLUNIFORMMATRIX4X2FVPROC)load( "glUniformMatrix4x2fv" );
	glad_glUniformMatrix3x4fv = (PFNGLUNIFORMMATRIX3X4FVPROC)load( "glUniformMatrix3x4fv" );
	glad_glUniformMatrix4x3fv = (PFNGLUNIFORMMATRIX4X3FVPROC)load( "glUniformMatrix4x3fv" );
}
static void load_GL_VERSION_3_0( GLADloadproc load )
{
	if (!GLAD_GL_VERSION_3_0) return;
	glad_glColorMaski = (PFNGLCOLORMASKIPROC)load( "glColorMaski" );
	glad_glGetBooleani_v = (PFNGLGETBOOLEANI_VPROC)load( "glGetBooleani_v" );
	glad_glGetIntegeri_v = (PFNGLGETINTEGERI_VPROC)load( "glGetIntegeri_v" );
	glad_glEnablei = (PFNGLENABLEIPROC)load( "glEnablei" );
	glad_glDisablei = (PFNGLDISABLEIPROC)load( "glDisablei" );
	glad_glIsEnabledi = (PFNGLISENABLEDIPROC)load( "glIsEnabledi" );
	glad_glBeginTransformFeedback = (PFNGLBEGINTRANSFORMFEEDBACKPROC)load( "glBeginTransformFeedback" );
	glad_glEndTransformFeedback = (PFNGLENDTRANSFORMFEEDBACKPROC)load( "glEndTransformFeedback" );
	glad_glBindBufferRange = (PFNGLBINDBUFFERRANGEPROC)load( "glBindBufferRange" );
	glad_glBindBufferBase = (PFNGLBINDBUFFERBASEPROC)load( "glBindBufferBase" );
	glad_glTransformFeedbackVaryings = (PFNGLTRANSFORMFEEDBACKVARYINGSPROC)load( "glTransformFeedbackVaryings" );
	glad_glGetTransformFeedbackVarying = (PFNGLGETTRANSFORMFEEDBACKVARYINGPROC)load( "glGetTransformFeedbackVarying" );
	glad_glClampColor = (PFNGLCLAMPCOLORPROC)load( "glClampColor" );
	glad_glBeginConditionalRender = (PFNGLBEGINCONDITIONALRENDERPROC)load( "glBeginConditionalRender" );
	glad_glEndConditionalRender = (PFNGLENDCONDITIONALRENDERPROC)load( "glEndConditionalRender" );
	glad_glVertexAttribIPointer = (PFNGLVERTEXATTRIBIPOINTERPROC)load( "glVertexAttribIPointer" );
	glad_glGetVertexAttribIiv = (PFNGLGETVERTEXATTRIBIIVPROC)load( "glGetVertexAttribIiv" );
	glad_glGetVertexAttribIuiv = (PFNGLGETVERTEXATTRIBIUIVPROC)load( "glGetVertexAttribIuiv" );
	glad_glVertexAttribI1i = (PFNGLVERTEXATTRIBI1IPROC)load( "glVertexAttribI1i" );
	glad_glVertexAttribI2i = (PFNGLVERTEXATTRIBI2IPROC)load( "glVertexAttribI2i" );
	glad_glVertexAttribI3i = (PFNGLVERTEXATTRIBI3IPROC)load( "glVertexAttribI3i" );
	glad_glVertexAttribI4i = (PFNGLVERTEXATTRIBI4IPROC)load( "glVertexAttribI4i" );
	glad_glVertexAttribI1ui = (PFNGLVERTEXATTRIBI1UIPROC)load( "glVertexAttribI1ui" );
	glad_glVertexAttribI2ui = (PFNGLVERTEXATTRIBI2UIPROC)load( "glVertexAttribI2ui" );
	glad_glVertexAttribI3ui = (PFNGLVERTEXATTRIBI3UIPROC)load( "glVertexAttribI3ui" );
	glad_glVertexAttribI4ui = (PFNGLVERTEXATTRIBI4UIPROC)load( "glVertexAttribI4ui" );
	glad_glVertexAttribI1iv = (PFNGLVERTEXATTRIBI1IVPROC)load( "glVertexAttribI1iv" );
	glad_glVertexAttribI2iv = (PFNGLVERTEXATTRIBI2IVPROC)load( "glVertexAttribI2iv" );
	glad_glVertexAttribI3iv = (PFNGLVERTEXATTRIBI3IVPROC)load( "glVertexAttribI3iv" );
	glad_glVertexAttribI4iv = (PFNGLVERTEXATTRIBI4IVPROC)load( "glVertexAttribI4iv" );
	glad_glVertexAttribI1uiv = (PFNGLVERTEXATTRIBI1UIVPROC)load( "glVertexAttribI1uiv" );
	glad_glVertexAttribI2uiv = (PFNGLVERTEXATTRIBI2UIVPROC)load( "glVertexAttribI2uiv" );
	glad_glVertexAttribI3uiv = (PFNGLVERTEXATTRIBI3UIVPROC)load( "glVertexAttribI3uiv" );
	glad_glVertexAttribI4uiv = (PFNGLVERTEXATTRIBI4UIVPROC)load( "glVertexAttribI4uiv" );
	glad_glVertexAttribI4bv = (PFNGLVERTEXATTRIBI4BVPROC)load( "glVertexAttribI4bv" );
	glad_glVertexAttribI4sv = (PFNGLVERTEXATTRIBI4SVPROC)load( "glVertexAttribI4sv" );
	glad_glVertexAttribI4ubv = (PFNGLVERTEXATTRIBI4UBVPROC)load( "glVertexAttribI4ubv" );
	glad_glVertexAttribI4usv = (PFNGLVERTEXATTRIBI4USVPROC)load( "glVertexAttribI4usv" );
	glad_glGetUniformuiv = (PFNGLGETUNIFORMUIVPROC)load( "glGetUniformuiv" );
	glad_glBindFragDataLocation = (PFNGLBINDFRAGDATALOCATIONPROC)load( "glBindFragDataLocation" );
	glad_glGetFragDataLocation = (PFNGLGETFRAGDATALOCATIONPROC)load( "glGetFragDataLocation" );
	glad_glUniform1ui = (PFNGLUNIFORM1UIPROC)load( "glUniform1ui" );
	glad_glUniform2ui = (PFNGLUNIFORM2UIPROC)load( "glUniform2ui" );
	glad_glUniform3ui = (PFNGLUNIFORM3UIPROC)load( "glUniform3ui" );
	glad_glUniform4ui = (PFNGLUNIFORM4UIPROC)load( "glUniform4ui" );
	glad_glUniform1uiv = (PFNGLUNIFORM1UIVPROC)load( "glUniform1uiv" );
	glad_glUniform2uiv = (PFNGLUNIFORM2UIVPROC)load( "glUniform2uiv" );
	glad_glUniform3uiv = (PFNGLUNIFORM3UIVPROC)load( "glUniform3uiv" );
	glad_glUniform4uiv = (PFNGLUNIFORM4UIVPROC)load( "glUniform4uiv" );
	glad_glTexParameterIiv = (PFNGLTEXPARAMETERIIVPROC)load( "glTexParameterIiv" );
	glad_glTexParameterIuiv = (PFNGLTEXPARAMETERIUIVPROC)load( "glTexParameterIuiv" );
	glad_glGetTexParameterIiv = (PFNGLGETTEXPARAMETERIIVPROC)load( "glGetTexParameterIiv" );
	glad_glGetTexParameterIuiv = (PFNGLGETTEXPARAMETERIUIVPROC)load( "glGetTexParameterIuiv" );
	glad_glClearBufferiv = (PFNGLCLEARBUFFERIVPROC)load( "glClearBufferiv" );
	glad_glClearBufferuiv = (PFNGLCLEARBUFFERUIVPROC)load( "glClearBufferuiv" );
	glad_glClearBufferfv = (PFNGLCLEARBUFFERFVPROC)load( "glClearBufferfv" );
	glad_glClearBufferfi = (PFNGLCLEARBUFFERFIPROC)load( "glClearBufferfi" );
	glad_glGetStringi = (PFNGLGETSTRINGIPROC)load( "glGetStringi" );
	glad_glIsRenderbuffer = (PFNGLISRENDERBUFFERPROC)load( "glIsRenderbuffer" );
	glad_glBindRenderbuffer = (PFNGLBINDRENDERBUFFERPROC)load( "glBindRenderbuffer" );
	glad_glDeleteRenderbuffers = (PFNGLDELETERENDERBUFFERSPROC)load( "glDeleteRenderbuffers" );
	glad_glGenRenderbuffers = (PFNGLGENRENDERBUFFERSPROC)load( "glGenRenderbuffers" );
	glad_glRenderbufferStorage = (PFNGLRENDERBUFFERSTORAGEPROC)load( "glRenderbufferStorage" );
	glad_glGetRenderbufferParameteriv = (PFNGLGETRENDERBUFFERPARAMETERIVPROC)load( "glGetRenderbufferParameteriv" );
	glad_glIsFramebuffer = (PFNGLISFRAMEBUFFERPROC)load( "glIsFramebuffer" );
	glad_glBindFramebuffer = (PFNGLBINDFRAMEBUFFERPROC)load( "glBindFramebuffer" );
	glad_glDeleteFramebuffers = (PFNGLDELETEFRAMEBUFFERSPROC)load( "glDeleteFramebuffers" );
	glad_glGenFramebuffers = (PFNGLGENFRAMEBUFFERSPROC)load( "glGenFramebuffers" );
	glad_glCheckFramebufferStatus = (PFNGLCHECKFRAMEBUFFERSTATUSPROC)load( "glCheckFramebufferStatus" );
	glad_glFramebufferTexture1D = (PFNGLFRAMEBUFFERTEXTURE1DPROC)load( "glFramebufferTexture1D" );
	glad_glFramebufferTexture2D = (PFNGLFRAMEBUFFERTEXTURE2DPROC)load( "glFramebufferTexture2D" );
	glad_glFramebufferTexture3D = (PFNGLFRAMEBUFFERTEXTURE3DPROC)load( "glFramebufferTexture3D" );
	glad_glFramebufferRenderbuffer = (PFNGLFRAMEBUFFERRENDERBUFFERPROC)load( "glFramebufferRenderbuffer" );
	glad_glGetFramebufferAttachmentParameteriv = (PFNGLGETFRAMEBUFFERATTACHMENTPARAMETERIVPROC)load( "glGetFramebufferAttachmentParameteriv" );
	glad_glGenerateMipmap = (PFNGLGENERATEMIPMAPPROC)load( "glGenerateMipmap" );
	glad_glBlitFramebuffer = (PFNGLBLITFRAMEBUFFERPROC)load( "glBlitFramebuffer" );
	glad_glRenderbufferStorageMultisample = (PFNGLRENDERBUFFERSTORAGEMULTISAMPLEPROC)load( "glRenderbufferStorageMultisample" );
	glad_glFramebufferTextureLayer = (PFNGLFRAMEBUFFERTEXTURELAYERPROC)load( "glFramebufferTextureLayer" );
	glad_glMapBufferRange = (PFNGLMAPBUFFERRANGEPROC)load( "glMapBufferRange" );
	glad_glFlushMappedBufferRange = (PFNGLFLUSHMAPPEDBUFFERRANGEPROC)load( "glFlushMappedBufferRange" );
	glad_glBindVertexArray = (PFNGLBINDVERTEXARRAYPROC)load( "glBindVertexArray" );
	glad_glDeleteVertexArrays = (PFNGLDELETEVERTEXARRAYSPROC)load( "glDeleteVertexArrays" );
	glad_glGenVertexArrays = (PFNGLGENVERTEXARRAYSPROC)load( "glGenVertexArrays" );
	glad_glIsVertexArray = (PFNGLISVERTEXARRAYPROC)load( "glIsVertexArray" );
}
static void load_GL_VERSION_3_1( GLADloadproc load )
{
	if (!GLAD_GL_VERSION_3_1) return;
	glad_glDrawArraysInstanced = (PFNGLDRAWARRAYSINSTANCEDPROC)load( "glDrawArraysInstanced" );
	glad_glDrawElementsInstanced = (PFNGLDRAWELEMENTSINSTANCEDPROC)load( "glDrawElementsInstanced" );
	glad_glTexBuffer = (PFNGLTEXBUFFERPROC)load( "glTexBuffer" );
	glad_glPrimitiveRestartIndex = (PFNGLPRIMITIVERESTARTINDEXPROC)load( "glPrimitiveRestartIndex" );
	glad_glCopyBufferSubData = (PFNGLCOPYBUFFERSUBDATAPROC)load( "glCopyBufferSubData" );
	glad_glGetUniformIndices = (PFNGLGETUNIFORMINDICESPROC)load( "glGetUniformIndices" );
	glad_glGetActiveUniformsiv = (PFNGLGETACTIVEUNIFORMSIVPROC)load( "glGetActiveUniformsiv" );
	glad_glGetActiveUniformName = (PFNGLGETACTIVEUNIFORMNAMEPROC)load( "glGetActiveUniformName" );
	glad_glGetUniformBlockIndex = (PFNGLGETUNIFORMBLOCKINDEXPROC)load( "glGetUniformBlockIndex" );
	glad_glGetActiveUniformBlockiv = (PFNGLGETACTIVEUNIFORMBLOCKIVPROC)load( "glGetActiveUniformBlockiv" );
	glad_glGetActiveUniformBlockName = (PFNGLGETACTIVEUNIFORMBLOCKNAMEPROC)load( "glGetActiveUniformBlockName" );
	glad_glUniformBlockBinding = (PFNGLUNIFORMBLOCKBINDINGPROC)load( "glUniformBlockBinding" );
	glad_glBindBufferRange = (PFNGLBINDBUFFERRANGEPROC)load( "glBindBufferRange" );
	glad_glBindBufferBase = (PFNGLBINDBUFFERBASEPROC)load( "glBindBufferBase" );
	glad_glGetIntegeri_v = (PFNGLGETINTEGERI_VPROC)load( "glGetIntegeri_v" );
}
static void load_GL_VERSION_3_2( GLADloadproc load )
{
	if (!GLAD_GL_VERSION_3_2) return;
	glad_glDrawElementsBaseVertex = (PFNGLDRAWELEMENTSBASEVERTEXPROC)load( "glDrawElementsBaseVertex" );
	glad_glDrawRangeElementsBaseVertex = (PFNGLDRAWRANGEELEMENTSBASEVERTEXPROC)load( "glDrawRangeElementsBaseVertex" );
	glad_glDrawElementsInstancedBaseVertex = (PFNGLDRAWELEMENTSINSTANCEDBASEVERTEXPROC)load( "glDrawElementsInstancedBaseVertex" );
	glad_glMultiDrawElementsBaseVertex = (PFNGLMULTIDRAWELEMENTSBASEVERTEXPROC)load( "glMultiDrawElementsBaseVertex" );
	glad_glProvokingVertex = (PFNGLPROVOKINGVERTEXPROC)load( "glProvokingVertex" );
	glad_glFenceSync = (PFNGLFENCESYNCPROC)load( "glFenceSync" );
	glad_glIsSync = (PFNGLISSYNCPROC)load( "glIsSync" );
	glad_glDeleteSync = (PFNGLDELETESYNCPROC)load( "glDeleteSync" );
	glad_glClientWaitSync = (PFNGLCLIENTWAITSYNCPROC)load( "glClientWaitSync" );
	glad_glWaitSync = (PFNGLWAITSYNCPROC)load( "glWaitSync" );
	glad_glGetInteger64v = (PFNGLGETINTEGER64VPROC)load( "glGetInteger64v" );
	glad_glGetSynciv = (PFNGLGETSYNCIVPROC)load( "glGetSynciv" );
	glad_glGetInteger64i_v = (PFNGLGETINTEGER64I_VPROC)load( "glGetInteger64i_v" );
	glad_glGetBufferParameteri64v = (PFNGLGETBUFFERPARAMETERI64VPROC)load( "glGetBufferParameteri64v" );
	glad_glFramebufferTexture = (PFNGLFRAMEBUFFERTEXTUREPROC)load( "glFramebufferTexture" );
	glad_glTexImage2DMultisample = (PFNGLTEXIMAGE2DMULTISAMPLEPROC)load( "glTexImage2DMultisample" );
	glad_glTexImage3DMultisample = (PFNGLTEXIMAGE3DMULTISAMPLEPROC)load( "glTexImage3DMultisample" );
	glad_glGetMultisamplefv = (PFNGLGETMULTISAMPLEFVPROC)load( "glGetMultisamplefv" );
	glad_glSampleMaski = (PFNGLSAMPLEMASKIPROC)load( "glSampleMaski" );
}
static void load_GL_VERSION_3_3( GLADloadproc load )
{
	if (!GLAD_GL_VERSION_3_3) return;
	glad_glBindFragDataLocationIndexed = (PFNGLBINDFRAGDATALOCATIONINDEXEDPROC)load( "glBindFragDataLocationIndexed" );
	glad_glGetFragDataIndex = (PFNGLGETFRAGDATAINDEXPROC)load( "glGetFragDataIndex" );
	glad_glGenSamplers = (PFNGLGENSAMPLERSPROC)load( "glGenSamplers" );
	glad_glDeleteSamplers = (PFNGLDELETESAMPLERSPROC)load( "glDeleteSamplers" );
	glad_glIsSampler = (PFNGLISSAMPLERPROC)load( "glIsSampler" );
	glad_glBindSampler = (PFNGLBINDSAMPLERPROC)load( "glBindSampler" );
	glad_glSamplerParameteri = (PFNGLSAMPLERPARAMETERIPROC)load( "glSamplerParameteri" );
	glad_glSamplerParameteriv = (PFNGLSAMPLERPARAMETERIVPROC)load( "glSamplerParameteriv" );
	glad_glSamplerParameterf = (PFNGLSAMPLERPARAMETERFPROC)load( "glSamplerParameterf" );
	glad_glSamplerParameterfv = (PFNGLSAMPLERPARAMETERFVPROC)load( "glSamplerParameterfv" );
	glad_glSamplerParameterIiv = (PFNGLSAMPLERPARAMETERIIVPROC)load( "glSamplerParameterIiv" );
	glad_glSamplerParameterIuiv = (PFNGLSAMPLERPARAMETERIUIVPROC)load( "glSamplerParameterIuiv" );
	glad_glGetSamplerParameteriv = (PFNGLGETSAMPLERPARAMETERIVPROC)load( "glGetSamplerParameteriv" );
	glad_glGetSamplerParameterIiv = (PFNGLGETSAMPLERPARAMETERIIVPROC)load( "glGetSamplerParameterIiv" );
	glad_glGetSamplerParameterfv = (PFNGLGETSAMPLERPARAMETERFVPROC)load( "glGetSamplerParameterfv" );
	glad_glGetSamplerParameterIuiv = (PFNGLGETSAMPLERPARAMETERIUIVPROC)load( "glGetSamplerParameterIuiv" );
	glad_glQueryCounter = (PFNGLQUERYCOUNTERPROC)load( "glQueryCounter" );
	glad_glGetQueryObjecti64v = (PFNGLGETQUERYOBJECTI64VPROC)load( "glGetQueryObjecti64v" );
	glad_glGetQueryObjectui64v = (PFNGLGETQUERYOBJECTUI64VPROC)load( "glGetQueryObjectui64v" );
	glad_glVertexAttribDivisor = (PFNGLVERTEXATTRIBDIVISORPROC)load( "glVertexAttribDivisor" );
	glad_glVertexAttribP1ui = (PFNGLVERTEXATTRIBP1UIPROC)load( "glVertexAttribP1ui" );
	glad_glVertexAttribP1uiv = (PFNGLVERTEXATTRIBP1UIVPROC)load( "glVertexAttribP1uiv" );
	glad_glVertexAttribP2ui = (PFNGLVERTEXATTRIBP2UIPROC)load( "glVertexAttribP2ui" );
	glad_glVertexAttribP2uiv = (PFNGLVERTEXATTRIBP2UIVPROC)load( "glVertexAttribP2uiv" );
	glad_glVertexAttribP3ui = (PFNGLVERTEXATTRIBP3UIPROC)load( "glVertexAttribP3ui" );
	glad_glVertexAttribP3uiv = (PFNGLVERTEXATTRIBP3UIVPROC)load( "glVertexAttribP3uiv" );
	glad_glVertexAttribP4ui = (PFNGLVERTEXATTRIBP4UIPROC)load( "glVertexAttribP4ui" );
	glad_glVertexAttribP4uiv = (PFNGLVERTEXATTRIBP4UIVPROC)load( "glVertexAttribP4uiv" );
	glad_glVertexP2ui = (PFNGLVERTEXP2UIPROC)load( "glVertexP2ui" );
	glad_glVertexP2uiv = (PFNGLVERTEXP2UIVPROC)load( "glVertexP2uiv" );
	glad_glVertexP3ui = (PFNGLVERTEXP3UIPROC)load( "glVertexP3ui" );
	glad_glVertexP3uiv = (PFNGLVERTEXP3UIVPROC)load( "glVertexP3uiv" );
	glad_glVertexP4ui = (PFNGLVERTEXP4UIPROC)load( "glVertexP4ui" );
	glad_glVertexP4uiv = (PFNGLVERTEXP4UIVPROC)load( "glVertexP4uiv" );
	glad_glTexCoordP1ui = (PFNGLTEXCOORDP1UIPROC)load( "glTexCoordP1ui" );
	glad_glTexCoordP1uiv = (PFNGLTEXCOORDP1UIVPROC)load( "glTexCoordP1uiv" );
	glad_glTexCoordP2ui = (PFNGLTEXCOORDP2UIPROC)load( "glTexCoordP2ui" );
	glad_glTexCoordP2uiv = (PFNGLTEXCOORDP2UIVPROC)load( "glTexCoordP2uiv" );
	glad_glTexCoordP3ui = (PFNGLTEXCOORDP3UIPROC)load( "glTexCoordP3ui" );
	glad_glTexCoordP3uiv = (PFNGLTEXCOORDP3UIVPROC)load( "glTexCoordP3uiv" );
	glad_glTexCoordP4ui = (PFNGLTEXCOORDP4UIPROC)load( "glTexCoordP4ui" );
	glad_glTexCoordP4uiv = (PFNGLTEXCOORDP4UIVPROC)load( "glTexCoordP4uiv" );
	glad_glMultiTexCoordP1ui = (PFNGLMULTITEXCOORDP1UIPROC)load( "glMultiTexCoordP1ui" );
	glad_glMultiTexCoordP1uiv = (PFNGLMULTITEXCOORDP1UIVPROC)load( "glMultiTexCoordP1uiv" );
	glad_glMultiTexCoordP2ui = (PFNGLMULTITEXCOORDP2UIPROC)load( "glMultiTexCoordP2ui" );
	glad_glMultiTexCoordP2uiv = (PFNGLMULTITEXCOORDP2UIVPROC)load( "glMultiTexCoordP2uiv" );
	glad_glMultiTexCoordP3ui = (PFNGLMULTITEXCOORDP3UIPROC)load( "glMultiTexCoordP3ui" );
	glad_glMultiTexCoordP3uiv = (PFNGLMULTITEXCOORDP3UIVPROC)load( "glMultiTexCoordP3uiv" );
	glad_glMultiTexCoordP4ui = (PFNGLMULTITEXCOORDP4UIPROC)load( "glMultiTexCoordP4ui" );
	glad_glMultiTexCoordP4uiv = (PFNGLMULTITEXCOORDP4UIVPROC)load( "glMultiTexCoordP4uiv" );
	glad_glNormalP3ui = (PFNGLNORMALP3UIPROC)load( "glNormalP3ui" );
	glad_glNormalP3uiv = (PFNGLNORMALP3UIVPROC)load( "glNormalP3uiv" );
	glad_glColorP3ui = (PFNGLCOLORP3UIPROC)load( "glColorP3ui" );
	glad_glColorP3uiv = (PFNGLCOLORP3UIVPROC)load( "glColorP3uiv" );
	glad_glColorP4ui = (PFNGLCOLORP4UIPROC)load( "glColorP4ui" );
	glad_glColorP4uiv = (PFNGLCOLORP4UIVPROC)load( "glColorP4uiv" );
	glad_glSecondaryColorP3ui = (PFNGLSECONDARYCOLORP3UIPROC)load( "glSecondaryColorP3ui" );
	glad_glSecondaryColorP3uiv = (PFNGLSECONDARYCOLORP3UIVPROC)load( "glSecondaryColorP3uiv" );
}
static void load_GL_VERSION_4_0( GLADloadproc load )
{
	if (!GLAD_GL_VERSION_4_0) return;
	glad_glMinSampleShading = (PFNGLMINSAMPLESHADINGPROC)load( "glMinSampleShading" );
	glad_glBlendEquationi = (PFNGLBLENDEQUATIONIPROC)load( "glBlendEquationi" );
	glad_glBlendEquationSeparatei = (PFNGLBLENDEQUATIONSEPARATEIPROC)load( "glBlendEquationSeparatei" );
	glad_glBlendFunci = (PFNGLBLENDFUNCIPROC)load( "glBlendFunci" );
	glad_glBlendFuncSeparatei = (PFNGLBLENDFUNCSEPARATEIPROC)load( "glBlendFuncSeparatei" );
	glad_glDrawArraysIndirect = (PFNGLDRAWARRAYSINDIRECTPROC)load( "glDrawArraysIndirect" );
	glad_glDrawElementsIndirect = (PFNGLDRAWELEMENTSINDIRECTPROC)load( "glDrawElementsIndirect" );
	glad_glUniform1d = (PFNGLUNIFORM1DPROC)load( "glUniform1d" );
	glad_glUniform2d = (PFNGLUNIFORM2DPROC)load( "glUniform2d" );
	glad_glUniform3d = (PFNGLUNIFORM3DPROC)load( "glUniform3d" );
	glad_glUniform4d = (PFNGLUNIFORM4DPROC)load( "glUniform4d" );
	glad_glUniform1dv = (PFNGLUNIFORM1DVPROC)load( "glUniform1dv" );
	glad_glUniform2dv = (PFNGLUNIFORM2DVPROC)load( "glUniform2dv" );
	glad_glUniform3dv = (PFNGLUNIFORM3DVPROC)load( "glUniform3dv" );
	glad_glUniform4dv = (PFNGLUNIFORM4DVPROC)load( "glUniform4dv" );
	glad_glUniformMatrix2dv = (PFNGLUNIFORMMATRIX2DVPROC)load( "glUniformMatrix2dv" );
	glad_glUniformMatrix3dv = (PFNGLUNIFORMMATRIX3DVPROC)load( "glUniformMatrix3dv" );
	glad_glUniformMatrix4dv = (PFNGLUNIFORMMATRIX4DVPROC)load( "glUniformMatrix4dv" );
	glad_glUniformMatrix2x3dv = (PFNGLUNIFORMMATRIX2X3DVPROC)load( "glUniformMatrix2x3dv" );
	glad_glUniformMatrix2x4dv = (PFNGLUNIFORMMATRIX2X4DVPROC)load( "glUniformMatrix2x4dv" );
	glad_glUniformMatrix3x2dv = (PFNGLUNIFORMMATRIX3X2DVPROC)load( "glUniformMatrix3x2dv" );
	glad_glUniformMatrix3x4dv = (PFNGLUNIFORMMATRIX3X4DVPROC)load( "glUniformMatrix3x4dv" );
	glad_glUniformMatrix4x2dv = (PFNGLUNIFORMMATRIX4X2DVPROC)load( "glUniformMatrix4x2dv" );
	glad_glUniformMatrix4x3dv = (PFNGLUNIFORMMATRIX4X3DVPROC)load( "glUniformMatrix4x3dv" );
	glad_glGetUniformdv = (PFNGLGETUNIFORMDVPROC)load( "glGetUniformdv" );
	glad_glGetSubroutineUniformLocation = (PFNGLGETSUBROUTINEUNIFORMLOCATIONPROC)load( "glGetSubroutineUniformLocation" );
	glad_glGetSubroutineIndex = (PFNGLGETSUBROUTINEINDEXPROC)load( "glGetSubroutineIndex" );
	glad_glGetActiveSubroutineUniformiv = (PFNGLGETACTIVESUBROUTINEUNIFORMIVPROC)load( "glGetActiveSubroutineUniformiv" );
	glad_glGetActiveSubroutineUniformName = (PFNGLGETACTIVESUBROUTINEUNIFORMNAMEPROC)load( "glGetActiveSubroutineUniformName" );
	glad_glGetActiveSubroutineName = (PFNGLGETACTIVESUBROUTINENAMEPROC)load( "glGetActiveSubroutineName" );
	glad_glUniformSubroutinesuiv = (PFNGLUNIFORMSUBROUTINESUIVPROC)load( "glUniformSubroutinesuiv" );
	glad_glGetUniformSubroutineuiv = (PFNGLGETUNIFORMSUBROUTINEUIVPROC)load( "glGetUniformSubroutineuiv" );
	glad_glGetProgramStageiv = (PFNGLGETPROGRAMSTAGEIVPROC)load( "glGetProgramStageiv" );
	glad_glPatchParameteri = (PFNGLPATCHPARAMETERIPROC)load( "glPatchParameteri" );
	glad_glPatchParameterfv = (PFNGLPATCHPARAMETERFVPROC)load( "glPatchParameterfv" );
	glad_glBindTransformFeedback = (PFNGLBINDTRANSFORMFEEDBACKPROC)load( "glBindTransformFeedback" );
	glad_glDeleteTransformFeedbacks = (PFNGLDELETETRANSFORMFEEDBACKSPROC)load( "glDeleteTransformFeedbacks" );
	glad_glGenTransformFeedbacks = (PFNGLGENTRANSFORMFEEDBACKSPROC)load( "glGenTransformFeedbacks" );
	glad_glIsTransformFeedback = (PFNGLISTRANSFORMFEEDBACKPROC)load( "glIsTransformFeedback" );
	glad_glPauseTransformFeedback = (PFNGLPAUSETRANSFORMFEEDBACKPROC)load( "glPauseTransformFeedback" );
	glad_glResumeTransformFeedback = (PFNGLRESUMETRANSFORMFEEDBACKPROC)load( "glResumeTransformFeedback" );
	glad_glDrawTransformFeedback = (PFNGLDRAWTRANSFORMFEEDBACKPROC)load( "glDrawTransformFeedback" );
	glad_glDrawTransformFeedbackStream = (PFNGLDRAWTRANSFORMFEEDBACKSTREAMPROC)load( "glDrawTransformFeedbackStream" );
	glad_glBeginQueryIndexed = (PFNGLBEGINQUERYINDEXEDPROC)load( "glBeginQueryIndexed" );
	glad_glEndQueryIndexed = (PFNGLENDQUERYINDEXEDPROC)load( "glEndQueryIndexed" );
	glad_glGetQueryIndexediv = (PFNGLGETQUERYINDEXEDIVPROC)load( "glGetQueryIndexediv" );
}
static void load_GL_VERSION_4_1( GLADloadproc load )
{
	if (!GLAD_GL_VERSION_4_1) return;
	glad_glReleaseShaderCompiler = (PFNGLRELEASESHADERCOMPILERPROC)load( "glReleaseShaderCompiler" );
	glad_glShaderBinary = (PFNGLSHADERBINARYPROC)load( "glShaderBinary" );
	glad_glGetShaderPrecisionFormat = (PFNGLGETSHADERPRECISIONFORMATPROC)load( "glGetShaderPrecisionFormat" );
	glad_glDepthRangef = (PFNGLDEPTHRANGEFPROC)load( "glDepthRangef" );
	glad_glClearDepthf = (PFNGLCLEARDEPTHFPROC)load( "glClearDepthf" );
	glad_glGetProgramBinary = (PFNGLGETPROGRAMBINARYPROC)load( "glGetProgramBinary" );
	glad_glProgramBinary = (PFNGLPROGRAMBINARYPROC)load( "glProgramBinary" );
	glad_glProgramParameteri = (PFNGLPROGRAMPARAMETERIPROC)load( "glProgramParameteri" );
	glad_glUseProgramStages = (PFNGLUSEPROGRAMSTAGESPROC)load( "glUseProgramStages" );
	glad_glActiveShaderProgram = (PFNGLACTIVESHADERPROGRAMPROC)load( "glActiveShaderProgram" );
	glad_glCreateShaderProgramv = (PFNGLCREATESHADERPROGRAMVPROC)load( "glCreateShaderProgramv" );
	glad_glBindProgramPipeline = (PFNGLBINDPROGRAMPIPELINEPROC)load( "glBindProgramPipeline" );
	glad_glDeleteProgramPipelines = (PFNGLDELETEPROGRAMPIPELINESPROC)load( "glDeleteProgramPipelines" );
	glad_glGenProgramPipelines = (PFNGLGENPROGRAMPIPELINESPROC)load( "glGenProgramPipelines" );
	glad_glIsProgramPipeline = (PFNGLISPROGRAMPIPELINEPROC)load( "glIsProgramPipeline" );
	glad_glGetProgramPipelineiv = (PFNGLGETPROGRAMPIPELINEIVPROC)load( "glGetProgramPipelineiv" );
	glad_glProgramParameteri = (PFNGLPROGRAMPARAMETERIPROC)load( "glProgramParameteri" );
	glad_glProgramUniform1i = (PFNGLPROGRAMUNIFORM1IPROC)load( "glProgramUniform1i" );
	glad_glProgramUniform1iv = (PFNGLPROGRAMUNIFORM1IVPROC)load( "glProgramUniform1iv" );
	glad_glProgramUniform1f = (PFNGLPROGRAMUNIFORM1FPROC)load( "glProgramUniform1f" );
	glad_glProgramUniform1fv = (PFNGLPROGRAMUNIFORM1FVPROC)load( "glProgramUniform1fv" );
	glad_glProgramUniform1d = (PFNGLPROGRAMUNIFORM1DPROC)load( "glProgramUniform1d" );
	glad_glProgramUniform1dv = (PFNGLPROGRAMUNIFORM1DVPROC)load( "glProgramUniform1dv" );
	glad_glProgramUniform1ui = (PFNGLPROGRAMUNIFORM1UIPROC)load( "glProgramUniform1ui" );
	glad_glProgramUniform1uiv = (PFNGLPROGRAMUNIFORM1UIVPROC)load( "glProgramUniform1uiv" );
	glad_glProgramUniform2i = (PFNGLPROGRAMUNIFORM2IPROC)load( "glProgramUniform2i" );
	glad_glProgramUniform2iv = (PFNGLPROGRAMUNIFORM2IVPROC)load( "glProgramUniform2iv" );
	glad_glProgramUniform2f = (PFNGLPROGRAMUNIFORM2FPROC)load( "glProgramUniform2f" );
	glad_glProgramUniform2fv = (PFNGLPROGRAMUNIFORM2FVPROC)load( "glProgramUniform2fv" );
	glad_glProgramUniform2d = (PFNGLPROGRAMUNIFORM2DPROC)load( "glProgramUniform2d" );
	glad_glProgramUniform2dv = (PFNGLPROGRAMUNIFORM2DVPROC)load( "glProgramUniform2dv" );
	glad_glProgramUniform2ui = (PFNGLPROGRAMUNIFORM2UIPROC)load( "glProgramUniform2ui" );
	glad_glProgramUniform2uiv = (PFNGLPROGRAMUNIFORM2UIVPROC)load( "glProgramUniform2uiv" );
	glad_glProgramUniform3i = (PFNGLPROGRAMUNIFORM3IPROC)load( "glProgramUniform3i" );
	glad_glProgramUniform3iv = (PFNGLPROGRAMUNIFORM3IVPROC)load( "glProgramUniform3iv" );
	glad_glProgramUniform3f = (PFNGLPROGRAMUNIFORM3FPROC)load( "glProgramUniform3f" );
	glad_glProgramUniform3fv = (PFNGLPROGRAMUNIFORM3FVPROC)load( "glProgramUniform3fv" );
	glad_glProgramUniform3d = (PFNGLPROGRAMUNIFORM3DPROC)load( "glProgramUniform3d" );
	glad_glProgramUniform3dv = (PFNGLPROGRAMUNIFORM3DVPROC)load( "glProgramUniform3dv" );
	glad_glProgramUniform3ui = (PFNGLPROGRAMUNIFORM3UIPROC)load( "glProgramUniform3ui" );
	glad_glProgramUniform3uiv = (PFNGLPROGRAMUNIFORM3UIVPROC)load( "glProgramUniform3uiv" );
	glad_glProgramUniform4i = (PFNGLPROGRAMUNIFORM4IPROC)load( "glProgramUniform4i" );
	glad_glProgramUniform4iv = (PFNGLPROGRAMUNIFORM4IVPROC)load( "glProgramUniform4iv" );
	glad_glProgramUniform4f = (PFNGLPROGRAMUNIFORM4FPROC)load( "glProgramUniform4f" );
	glad_glProgramUniform4fv = (PFNGLPROGRAMUNIFORM4FVPROC)load( "glProgramUniform4fv" );
	glad_glProgramUniform4d = (PFNGLPROGRAMUNIFORM4DPROC)load( "glProgramUniform4d" );
	glad_glProgramUniform4dv = (PFNGLPROGRAMUNIFORM4DVPROC)load( "glProgramUniform4dv" );
	glad_glProgramUniform4ui = (PFNGLPROGRAMUNIFORM4UIPROC)load( "glProgramUniform4ui" );
	glad_glProgramUniform4uiv = (PFNGLPROGRAMUNIFORM4UIVPROC)load( "glProgramUniform4uiv" );
	glad_glProgramUniformMatrix2fv = (PFNGLPROGRAMUNIFORMMATRIX2FVPROC)load( "glProgramUniformMatrix2fv" );
	glad_glProgramUniformMatrix3fv = (PFNGLPROGRAMUNIFORMMATRIX3FVPROC)load( "glProgramUniformMatrix3fv" );
	glad_glProgramUniformMatrix4fv = (PFNGLPROGRAMUNIFORMMATRIX4FVPROC)load( "glProgramUniformMatrix4fv" );
	glad_glProgramUniformMatrix2dv = (PFNGLPROGRAMUNIFORMMATRIX2DVPROC)load( "glProgramUniformMatrix2dv" );
	glad_glProgramUniformMatrix3dv = (PFNGLPROGRAMUNIFORMMATRIX3DVPROC)load( "glProgramUniformMatrix3dv" );
	glad_glProgramUniformMatrix4dv = (PFNGLPROGRAMUNIFORMMATRIX4DVPROC)load( "glProgramUniformMatrix4dv" );
	glad_glProgramUniformMatrix2x3fv = (PFNGLPROGRAMUNIFORMMATRIX2X3FVPROC)load( "glProgramUniformMatrix2x3fv" );
	glad_glProgramUniformMatrix3x2fv = (PFNGLPROGRAMUNIFORMMATRIX3X2FVPROC)load( "glProgramUniformMatrix3x2fv" );
	glad_glProgramUniformMatrix2x4fv = (PFNGLPROGRAMUNIFORMMATRIX2X4FVPROC)load( "glProgramUniformMatrix2x4fv" );
	glad_glProgramUniformMatrix4x2fv = (PFNGLPROGRAMUNIFORMMATRIX4X2FVPROC)load( "glProgramUniformMatrix4x2fv" );
	glad_glProgramUniformMatrix3x4fv = (PFNGLPROGRAMUNIFORMMATRIX3X4FVPROC)load( "glProgramUniformMatrix3x4fv" );
	glad_glProgramUniformMatrix4x3fv = (PFNGLPROGRAMUNIFORMMATRIX4X3FVPROC)load( "glProgramUniformMatrix4x3fv" );
	glad_glProgramUniformMatrix2x3dv = (PFNGLPROGRAMUNIFORMMATRIX2X3DVPROC)load( "glProgramUniformMatrix2x3dv" );
	glad_glProgramUniformMatrix3x2dv = (PFNGLPROGRAMUNIFORMMATRIX3X2DVPROC)load( "glProgramUniformMatrix3x2dv" );
	glad_glProgramUniformMatrix2x4dv = (PFNGLPROGRAMUNIFORMMATRIX2X4DVPROC)load( "glProgramUniformMatrix2x4dv" );
	glad_glProgramUniformMatrix4x2dv = (PFNGLPROGRAMUNIFORMMATRIX4X2DVPROC)load( "glProgramUniformMatrix4x2dv" );
	glad_glProgramUniformMatrix3x4dv = (PFNGLPROGRAMUNIFORMMATRIX3X4DVPROC)load( "glProgramUniformMatrix3x4dv" );
	glad_glProgramUniformMatrix4x3dv = (PFNGLPROGRAMUNIFORMMATRIX4X3DVPROC)load( "glProgramUniformMatrix4x3dv" );
	glad_glValidateProgramPipeline = (PFNGLVALIDATEPROGRAMPIPELINEPROC)load( "glValidateProgramPipeline" );
	glad_glGetProgramPipelineInfoLog = (PFNGLGETPROGRAMPIPELINEINFOLOGPROC)load( "glGetProgramPipelineInfoLog" );
	glad_glVertexAttribL1d = (PFNGLVERTEXATTRIBL1DPROC)load( "glVertexAttribL1d" );
	glad_glVertexAttribL2d = (PFNGLVERTEXATTRIBL2DPROC)load( "glVertexAttribL2d" );
	glad_glVertexAttribL3d = (PFNGLVERTEXATTRIBL3DPROC)load( "glVertexAttribL3d" );
	glad_glVertexAttribL4d = (PFNGLVERTEXATTRIBL4DPROC)load( "glVertexAttribL4d" );
	glad_glVertexAttribL1dv = (PFNGLVERTEXATTRIBL1DVPROC)load( "glVertexAttribL1dv" );
	glad_glVertexAttribL2dv = (PFNGLVERTEXATTRIBL2DVPROC)load( "glVertexAttribL2dv" );
	glad_glVertexAttribL3dv = (PFNGLVERTEXATTRIBL3DVPROC)load( "glVertexAttribL3dv" );
	glad_glVertexAttribL4dv = (PFNGLVERTEXATTRIBL4DVPROC)load( "glVertexAttribL4dv" );
	glad_glVertexAttribLPointer = (PFNGLVERTEXATTRIBLPOINTERPROC)load( "glVertexAttribLPointer" );
	glad_glGetVertexAttribLdv = (PFNGLGETVERTEXATTRIBLDVPROC)load( "glGetVertexAttribLdv" );
	glad_glViewportArrayv = (PFNGLVIEWPORTARRAYVPROC)load( "glViewportArrayv" );
	glad_glViewportIndexedf = (PFNGLVIEWPORTINDEXEDFPROC)load( "glViewportIndexedf" );
	glad_glViewportIndexedfv = (PFNGLVIEWPORTINDEXEDFVPROC)load( "glViewportIndexedfv" );
	glad_glScissorArrayv = (PFNGLSCISSORARRAYVPROC)load( "glScissorArrayv" );
	glad_glScissorIndexed = (PFNGLSCISSORINDEXEDPROC)load( "glScissorIndexed" );
	glad_glScissorIndexedv = (PFNGLSCISSORINDEXEDVPROC)load( "glScissorIndexedv" );
	glad_glDepthRangeArrayv = (PFNGLDEPTHRANGEARRAYVPROC)load( "glDepthRangeArrayv" );
	glad_glDepthRangeIndexed = (PFNGLDEPTHRANGEINDEXEDPROC)load( "glDepthRangeIndexed" );
	glad_glGetFloati_v = (PFNGLGETFLOATI_VPROC)load( "glGetFloati_v" );
	glad_glGetDoublei_v = (PFNGLGETDOUBLEI_VPROC)load( "glGetDoublei_v" );
}
static void load_GL_VERSION_4_2( GLADloadproc load )
{
	if (!GLAD_GL_VERSION_4_2) return;
	glad_glDrawArraysInstancedBaseInstance = (PFNGLDRAWARRAYSINSTANCEDBASEINSTANCEPROC)load( "glDrawArraysInstancedBaseInstance" );
	glad_glDrawElementsInstancedBaseInstance = (PFNGLDRAWELEMENTSINSTANCEDBASEINSTANCEPROC)load( "glDrawElementsInstancedBaseInstance" );
	glad_glDrawElementsInstancedBaseVertexBaseInstance = (PFNGLDRAWELEMENTSINSTANCEDBASEVERTEXBASEINSTANCEPROC)load( "glDrawElementsInstancedBaseVertexBaseInstance" );
	glad_glGetInternalformativ = (PFNGLGETINTERNALFORMATIVPROC)load( "glGetInternalformativ" );
	glad_glGetActiveAtomicCounterBufferiv = (PFNGLGETACTIVEATOMICCOUNTERBUFFERIVPROC)load( "glGetActiveAtomicCounterBufferiv" );
	glad_glBindImageTexture = (PFNGLBINDIMAGETEXTUREPROC)load( "glBindImageTexture" );
	glad_glMemoryBarrier = (PFNGLMEMORYBARRIERPROC)load( "glMemoryBarrier" );
	glad_glTexStorage1D = (PFNGLTEXSTORAGE1DPROC)load( "glTexStorage1D" );
	glad_glTexStorage2D = (PFNGLTEXSTORAGE2DPROC)load( "glTexStorage2D" );
	glad_glTexStorage3D = (PFNGLTEXSTORAGE3DPROC)load( "glTexStorage3D" );
	glad_glDrawTransformFeedbackInstanced = (PFNGLDRAWTRANSFORMFEEDBACKINSTANCEDPROC)load( "glDrawTransformFeedbackInstanced" );
	glad_glDrawTransformFeedbackStreamInstanced = (PFNGLDRAWTRANSFORMFEEDBACKSTREAMINSTANCEDPROC)load( "glDrawTransformFeedbackStreamInstanced" );
}
static void load_GL_VERSION_4_3( GLADloadproc load )
{
	if (!GLAD_GL_VERSION_4_3) return;
	glad_glClearBufferData = (PFNGLCLEARBUFFERDATAPROC)load( "glClearBufferData" );
	glad_glClearBufferSubData = (PFNGLCLEARBUFFERSUBDATAPROC)load( "glClearBufferSubData" );
	glad_glDispatchCompute = (PFNGLDISPATCHCOMPUTEPROC)load( "glDispatchCompute" );
	glad_glDispatchComputeIndirect = (PFNGLDISPATCHCOMPUTEINDIRECTPROC)load( "glDispatchComputeIndirect" );
	glad_glCopyImageSubData = (PFNGLCOPYIMAGESUBDATAPROC)load( "glCopyImageSubData" );
	glad_glFramebufferParameteri = (PFNGLFRAMEBUFFERPARAMETERIPROC)load( "glFramebufferParameteri" );
	glad_glGetFramebufferParameteriv = (PFNGLGETFRAMEBUFFERPARAMETERIVPROC)load( "glGetFramebufferParameteriv" );
	glad_glGetInternalformati64v = (PFNGLGETINTERNALFORMATI64VPROC)load( "glGetInternalformati64v" );
	glad_glInvalidateTexSubImage = (PFNGLINVALIDATETEXSUBIMAGEPROC)load( "glInvalidateTexSubImage" );
	glad_glInvalidateTexImage = (PFNGLINVALIDATETEXIMAGEPROC)load( "glInvalidateTexImage" );
	glad_glInvalidateBufferSubData = (PFNGLINVALIDATEBUFFERSUBDATAPROC)load( "glInvalidateBufferSubData" );
	glad_glInvalidateBufferData = (PFNGLINVALIDATEBUFFERDATAPROC)load( "glInvalidateBufferData" );
	glad_glInvalidateFramebuffer = (PFNGLINVALIDATEFRAMEBUFFERPROC)load( "glInvalidateFramebuffer" );
	glad_glInvalidateSubFramebuffer = (PFNGLINVALIDATESUBFRAMEBUFFERPROC)load( "glInvalidateSubFramebuffer" );
	glad_glMultiDrawArraysIndirect = (PFNGLMULTIDRAWARRAYSINDIRECTPROC)load( "glMultiDrawArraysIndirect" );
	glad_glMultiDrawElementsIndirect = (PFNGLMULTIDRAWELEMENTSINDIRECTPROC)load( "glMultiDrawElementsIndirect" );
	glad_glGetProgramInterfaceiv = (PFNGLGETPROGRAMINTERFACEIVPROC)load( "glGetProgramInterfaceiv" );
	glad_glGetProgramResourceIndex = (PFNGLGETPROGRAMRESOURCEINDEXPROC)load( "glGetProgramResourceIndex" );
	glad_glGetProgramResourceName = (PFNGLGETPROGRAMRESOURCENAMEPROC)load( "glGetProgramResourceName" );
	glad_glGetProgramResourceiv = (PFNGLGETPROGRAMRESOURCEIVPROC)load( "glGetProgramResourceiv" );
	glad_glGetProgramResourceLocation = (PFNGLGETPROGRAMRESOURCELOCATIONPROC)load( "glGetProgramResourceLocation" );
	glad_glGetProgramResourceLocationIndex = (PFNGLGETPROGRAMRESOURCELOCATIONINDEXPROC)load( "glGetProgramResourceLocationIndex" );
	glad_glShaderStorageBlockBinding = (PFNGLSHADERSTORAGEBLOCKBINDINGPROC)load( "glShaderStorageBlockBinding" );
	glad_glTexBufferRange = (PFNGLTEXBUFFERRANGEPROC)load( "glTexBufferRange" );
	glad_glTexStorage2DMultisample = (PFNGLTEXSTORAGE2DMULTISAMPLEPROC)load( "glTexStorage2DMultisample" );
	glad_glTexStorage3DMultisample = (PFNGLTEXSTORAGE3DMULTISAMPLEPROC)load( "glTexStorage3DMultisample" );
	glad_glTextureView = (PFNGLTEXTUREVIEWPROC)load( "glTextureView" );
	glad_glBindVertexBuffer = (PFNGLBINDVERTEXBUFFERPROC)load( "glBindVertexBuffer" );
	glad_glVertexAttribFormat = (PFNGLVERTEXATTRIBFORMATPROC)load( "glVertexAttribFormat" );
	glad_glVertexAttribIFormat = (PFNGLVERTEXATTRIBIFORMATPROC)load( "glVertexAttribIFormat" );
	glad_glVertexAttribLFormat = (PFNGLVERTEXATTRIBLFORMATPROC)load( "glVertexAttribLFormat" );
	glad_glVertexAttribBinding = (PFNGLVERTEXATTRIBBINDINGPROC)load( "glVertexAttribBinding" );
	glad_glVertexBindingDivisor = (PFNGLVERTEXBINDINGDIVISORPROC)load( "glVertexBindingDivisor" );
	glad_glDebugMessageControl = (PFNGLDEBUGMESSAGECONTROLPROC)load( "glDebugMessageControl" );
	glad_glDebugMessageInsert = (PFNGLDEBUGMESSAGEINSERTPROC)load( "glDebugMessageInsert" );
	glad_glDebugMessageCallback = (PFNGLDEBUGMESSAGECALLBACKPROC)load( "glDebugMessageCallback" );
	glad_glGetDebugMessageLog = (PFNGLGETDEBUGMESSAGELOGPROC)load( "glGetDebugMessageLog" );
	glad_glPushDebugGroup = (PFNGLPUSHDEBUGGROUPPROC)load( "glPushDebugGroup" );
	glad_glPopDebugGroup = (PFNGLPOPDEBUGGROUPPROC)load( "glPopDebugGroup" );
	glad_glObjectLabel = (PFNGLOBJECTLABELPROC)load( "glObjectLabel" );
	glad_glGetObjectLabel = (PFNGLGETOBJECTLABELPROC)load( "glGetObjectLabel" );
	glad_glObjectPtrLabel = (PFNGLOBJECTPTRLABELPROC)load( "glObjectPtrLabel" );
	glad_glGetObjectPtrLabel = (PFNGLGETOBJECTPTRLABELPROC)load( "glGetObjectPtrLabel" );
	glad_glGetPointerv = (PFNGLGETPOINTERVPROC)load( "glGetPointerv" );
}
static void load_GL_VERSION_4_4( GLADloadproc load )
{
	if (!GLAD_GL_VERSION_4_4) return;
	glad_glBufferStorage = (PFNGLBUFFERSTORAGEPROC)load( "glBufferStorage" );
	glad_glClearTexImage = (PFNGLCLEARTEXIMAGEPROC)load( "glClearTexImage" );
	glad_glClearTexSubImage = (PFNGLCLEARTEXSUBIMAGEPROC)load( "glClearTexSubImage" );
	glad_glBindBuffersBase = (PFNGLBINDBUFFERSBASEPROC)load( "glBindBuffersBase" );
	glad_glBindBuffersRange = (PFNGLBINDBUFFERSRANGEPROC)load( "glBindBuffersRange" );
	glad_glBindTextures = (PFNGLBINDTEXTURESPROC)load( "glBindTextures" );
	glad_glBindSamplers = (PFNGLBINDSAMPLERSPROC)load( "glBindSamplers" );
	glad_glBindImageTextures = (PFNGLBINDIMAGETEXTURESPROC)load( "glBindImageTextures" );
	glad_glBindVertexBuffers = (PFNGLBINDVERTEXBUFFERSPROC)load( "glBindVertexBuffers" );
}
static void load_GL_VERSION_4_5( GLADloadproc load )
{
	if (!GLAD_GL_VERSION_4_5) return;
	glad_glClipControl = (PFNGLCLIPCONTROLPROC)load( "glClipControl" );
	glad_glCreateTransformFeedbacks = (PFNGLCREATETRANSFORMFEEDBACKSPROC)load( "glCreateTransformFeedbacks" );
	glad_glTransformFeedbackBufferBase = (PFNGLTRANSFORMFEEDBACKBUFFERBASEPROC)load( "glTransformFeedbackBufferBase" );
	glad_glTransformFeedbackBufferRange = (PFNGLTRANSFORMFEEDBACKBUFFERRANGEPROC)load( "glTransformFeedbackBufferRange" );
	glad_glGetTransformFeedbackiv = (PFNGLGETTRANSFORMFEEDBACKIVPROC)load( "glGetTransformFeedbackiv" );
	glad_glGetTransformFeedbacki_v = (PFNGLGETTRANSFORMFEEDBACKI_VPROC)load( "glGetTransformFeedbacki_v" );
	glad_glGetTransformFeedbacki64_v = (PFNGLGETTRANSFORMFEEDBACKI64_VPROC)load( "glGetTransformFeedbacki64_v" );
	glad_glCreateBuffers = (PFNGLCREATEBUFFERSPROC)load( "glCreateBuffers" );
	glad_glNamedBufferStorage = (PFNGLNAMEDBUFFERSTORAGEPROC)load( "glNamedBufferStorage" );
	glad_glNamedBufferData = (PFNGLNAMEDBUFFERDATAPROC)load( "glNamedBufferData" );
	glad_glNamedBufferSubData = (PFNGLNAMEDBUFFERSUBDATAPROC)load( "glNamedBufferSubData" );
	glad_glCopyNamedBufferSubData = (PFNGLCOPYNAMEDBUFFERSUBDATAPROC)load( "glCopyNamedBufferSubData" );
	glad_glClearNamedBufferData = (PFNGLCLEARNAMEDBUFFERDATAPROC)load( "glClearNamedBufferData" );
	glad_glClearNamedBufferSubData = (PFNGLCLEARNAMEDBUFFERSUBDATAPROC)load( "glClearNamedBufferSubData" );
	glad_glMapNamedBuffer = (PFNGLMAPNAMEDBUFFERPROC)load( "glMapNamedBuffer" );
	glad_glMapNamedBufferRange = (PFNGLMAPNAMEDBUFFERRANGEPROC)load( "glMapNamedBufferRange" );
	glad_glUnmapNamedBuffer = (PFNGLUNMAPNAMEDBUFFERPROC)load( "glUnmapNamedBuffer" );
	glad_glFlushMappedNamedBufferRange = (PFNGLFLUSHMAPPEDNAMEDBUFFERRANGEPROC)load( "glFlushMappedNamedBufferRange" );
	glad_glGetNamedBufferParameteriv = (PFNGLGETNAMEDBUFFERPARAMETERIVPROC)load( "glGetNamedBufferParameteriv" );
	glad_glGetNamedBufferParameteri64v = (PFNGLGETNAMEDBUFFERPARAMETERI64VPROC)load( "glGetNamedBufferParameteri64v" );
	glad_glGetNamedBufferPointerv = (PFNGLGETNAMEDBUFFERPOINTERVPROC)load( "glGetNamedBufferPointerv" );
	glad_glGetNamedBufferSubData = (PFNGLGETNAMEDBUFFERSUBDATAPROC)load( "glGetNamedBufferSubData" );
	glad_glCreateFramebuffers = (PFNGLCREATEFRAMEBUFFERSPROC)load( "glCreateFramebuffers" );
	glad_glNamedFramebufferRenderbuffer = (PFNGLNAMEDFRAMEBUFFERRENDERBUFFERPROC)load( "glNamedFramebufferRenderbuffer" );
	glad_glNamedFramebufferParameteri = (PFNGLNAMEDFRAMEBUFFERPARAMETERIPROC)load( "glNamedFramebufferParameteri" );
	glad_glNamedFramebufferTexture = (PFNGLNAMEDFRAMEBUFFERTEXTUREPROC)load( "glNamedFramebufferTexture" );
	glad_glNamedFramebufferTextureLayer = (PFNGLNAMEDFRAMEBUFFERTEXTURELAYERPROC)load( "glNamedFramebufferTextureLayer" );
	glad_glNamedFramebufferDrawBuffer = (PFNGLNAMEDFRAMEBUFFERDRAWBUFFERPROC)load( "glNamedFramebufferDrawBuffer" );
	glad_glNamedFramebufferDrawBuffers = (PFNGLNAMEDFRAMEBUFFERDRAWBUFFERSPROC)load( "glNamedFramebufferDrawBuffers" );
	glad_glNamedFramebufferReadBuffer = (PFNGLNAMEDFRAMEBUFFERREADBUFFERPROC)load( "glNamedFramebufferReadBuffer" );
	glad_glInvalidateNamedFramebufferData = (PFNGLINVALIDATENAMEDFRAMEBUFFERDATAPROC)load( "glInvalidateNamedFramebufferData" );
	glad_glInvalidateNamedFramebufferSubData = (PFNGLINVALIDATENAMEDFRAMEBUFFERSUBDATAPROC)load( "glInvalidateNamedFramebufferSubData" );
	glad_glClearNamedFramebufferiv = (PFNGLCLEARNAMEDFRAMEBUFFERIVPROC)load( "glClearNamedFramebufferiv" );
	glad_glClearNamedFramebufferuiv = (PFNGLCLEARNAMEDFRAMEBUFFERUIVPROC)load( "glClearNamedFramebufferuiv" );
	glad_glClearNamedFramebufferfv = (PFNGLCLEARNAMEDFRAMEBUFFERFVPROC)load( "glClearNamedFramebufferfv" );
	glad_glClearNamedFramebufferfi = (PFNGLCLEARNAMEDFRAMEBUFFERFIPROC)load( "glClearNamedFramebufferfi" );
	glad_glBlitNamedFramebuffer = (PFNGLBLITNAMEDFRAMEBUFFERPROC)load( "glBlitNamedFramebuffer" );
	glad_glCheckNamedFramebufferStatus = (PFNGLCHECKNAMEDFRAMEBUFFERSTATUSPROC)load( "glCheckNamedFramebufferStatus" );
	glad_glGetNamedFramebufferParameteriv = (PFNGLGETNAMEDFRAMEBUFFERPARAMETERIVPROC)load( "glGetNamedFramebufferParameteriv" );
	glad_glGetNamedFramebufferAttachmentParameteriv = (PFNGLGETNAMEDFRAMEBUFFERATTACHMENTPARAMETERIVPROC)load( "glGetNamedFramebufferAttachmentParameteriv" );
	glad_glCreateRenderbuffers = (PFNGLCREATERENDERBUFFERSPROC)load( "glCreateRenderbuffers" );
	glad_glNamedRenderbufferStorage = (PFNGLNAMEDRENDERBUFFERSTORAGEPROC)load( "glNamedRenderbufferStorage" );
	glad_glNamedRenderbufferStorageMultisample = (PFNGLNAMEDRENDERBUFFERSTORAGEMULTISAMPLEPROC)load( "glNamedRenderbufferStorageMultisample" );
	glad_glGetNamedRenderbufferParameteriv = (PFNGLGETNAMEDRENDERBUFFERPARAMETERIVPROC)load( "glGetNamedRenderbufferParameteriv" );
	glad_glCreateTextures = (PFNGLCREATETEXTURESPROC)load( "glCreateTextures" );
	glad_glTextureBuffer = (PFNGLTEXTUREBUFFERPROC)load( "glTextureBuffer" );
	glad_glTextureBufferRange = (PFNGLTEXTUREBUFFERRANGEPROC)load( "glTextureBufferRange" );
	glad_glTextureStorage1D = (PFNGLTEXTURESTORAGE1DPROC)load( "glTextureStorage1D" );
	glad_glTextureStorage2D = (PFNGLTEXTURESTORAGE2DPROC)load( "glTextureStorage2D" );
	glad_glTextureStorage3D = (PFNGLTEXTURESTORAGE3DPROC)load( "glTextureStorage3D" );
	glad_glTextureStorage2DMultisample = (PFNGLTEXTURESTORAGE2DMULTISAMPLEPROC)load( "glTextureStorage2DMultisample" );
	glad_glTextureStorage3DMultisample = (PFNGLTEXTURESTORAGE3DMULTISAMPLEPROC)load( "glTextureStorage3DMultisample" );
	glad_glTextureSubImage1D = (PFNGLTEXTURESUBIMAGE1DPROC)load( "glTextureSubImage1D" );
	glad_glTextureSubImage2D = (PFNGLTEXTURESUBIMAGE2DPROC)load( "glTextureSubImage2D" );
	glad_glTextureSubImage3D = (PFNGLTEXTURESUBIMAGE3DPROC)load( "glTextureSubImage3D" );
	glad_glCompressedTextureSubImage1D = (PFNGLCOMPRESSEDTEXTURESUBIMAGE1DPROC)load( "glCompressedTextureSubImage1D" );
	glad_glCompressedTextureSubImage2D = (PFNGLCOMPRESSEDTEXTURESUBIMAGE2DPROC)load( "glCompressedTextureSubImage2D" );
	glad_glCompressedTextureSubImage3D = (PFNGLCOMPRESSEDTEXTURESUBIMAGE3DPROC)load( "glCompressedTextureSubImage3D" );
	glad_glCopyTextureSubImage1D = (PFNGLCOPYTEXTURESUBIMAGE1DPROC)load( "glCopyTextureSubImage1D" );
	glad_glCopyTextureSubImage2D = (PFNGLCOPYTEXTURESUBIMAGE2DPROC)load( "glCopyTextureSubImage2D" );
	glad_glCopyTextureSubImage3D = (PFNGLCOPYTEXTURESUBIMAGE3DPROC)load( "glCopyTextureSubImage3D" );
	glad_glTextureParameterf = (PFNGLTEXTUREPARAMETERFPROC)load( "glTextureParameterf" );
	glad_glTextureParameterfv = (PFNGLTEXTUREPARAMETERFVPROC)load( "glTextureParameterfv" );
	glad_glTextureParameteri = (PFNGLTEXTUREPARAMETERIPROC)load( "glTextureParameteri" );
	glad_glTextureParameterIiv = (PFNGLTEXTUREPARAMETERIIVPROC)load( "glTextureParameterIiv" );
	glad_glTextureParameterIuiv = (PFNGLTEXTUREPARAMETERIUIVPROC)load( "glTextureParameterIuiv" );
	glad_glTextureParameteriv = (PFNGLTEXTUREPARAMETERIVPROC)load( "glTextureParameteriv" );
	glad_glGenerateTextureMipmap = (PFNGLGENERATETEXTUREMIPMAPPROC)load( "glGenerateTextureMipmap" );
	glad_glBindTextureUnit = (PFNGLBINDTEXTUREUNITPROC)load( "glBindTextureUnit" );
	glad_glGetTextureImage = (PFNGLGETTEXTUREIMAGEPROC)load( "glGetTextureImage" );
	glad_glGetCompressedTextureImage = (PFNGLGETCOMPRESSEDTEXTUREIMAGEPROC)load( "glGetCompressedTextureImage" );
	glad_glGetTextureLevelParameterfv = (PFNGLGETTEXTURELEVELPARAMETERFVPROC)load( "glGetTextureLevelParameterfv" );
	glad_glGetTextureLevelParameteriv = (PFNGLGETTEXTURELEVELPARAMETERIVPROC)load( "glGetTextureLevelParameteriv" );
	glad_glGetTextureParameterfv = (PFNGLGETTEXTUREPARAMETERFVPROC)load( "glGetTextureParameterfv" );
	glad_glGetTextureParameterIiv = (PFNGLGETTEXTUREPARAMETERIIVPROC)load( "glGetTextureParameterIiv" );
	glad_glGetTextureParameterIuiv = (PFNGLGETTEXTUREPARAMETERIUIVPROC)load( "glGetTextureParameterIuiv" );
	glad_glGetTextureParameteriv = (PFNGLGETTEXTUREPARAMETERIVPROC)load( "glGetTextureParameteriv" );
	glad_glCreateVertexArrays = (PFNGLCREATEVERTEXARRAYSPROC)load( "glCreateVertexArrays" );
	glad_glDisableVertexArrayAttrib = (PFNGLDISABLEVERTEXARRAYATTRIBPROC)load( "glDisableVertexArrayAttrib" );
	glad_glEnableVertexArrayAttrib = (PFNGLENABLEVERTEXARRAYATTRIBPROC)load( "glEnableVertexArrayAttrib" );
	glad_glVertexArrayElementBuffer = (PFNGLVERTEXARRAYELEMENTBUFFERPROC)load( "glVertexArrayElementBuffer" );
	glad_glVertexArrayVertexBuffer = (PFNGLVERTEXARRAYVERTEXBUFFERPROC)load( "glVertexArrayVertexBuffer" );
	glad_glVertexArrayVertexBuffers = (PFNGLVERTEXARRAYVERTEXBUFFERSPROC)load( "glVertexArrayVertexBuffers" );
	glad_glVertexArrayAttribBinding = (PFNGLVERTEXARRAYATTRIBBINDINGPROC)load( "glVertexArrayAttribBinding" );
	glad_glVertexArrayAttribFormat = (PFNGLVERTEXARRAYATTRIBFORMATPROC)load( "glVertexArrayAttribFormat" );
	glad_glVertexArrayAttribIFormat = (PFNGLVERTEXARRAYATTRIBIFORMATPROC)load( "glVertexArrayAttribIFormat" );
	glad_glVertexArrayAttribLFormat = (PFNGLVERTEXARRAYATTRIBLFORMATPROC)load( "glVertexArrayAttribLFormat" );
	glad_glVertexArrayBindingDivisor = (PFNGLVERTEXARRAYBINDINGDIVISORPROC)load( "glVertexArrayBindingDivisor" );
	glad_glGetVertexArrayiv = (PFNGLGETVERTEXARRAYIVPROC)load( "glGetVertexArrayiv" );
	glad_glGetVertexArrayIndexediv = (PFNGLGETVERTEXARRAYINDEXEDIVPROC)load( "glGetVertexArrayIndexediv" );
	glad_glGetVertexArrayIndexed64iv = (PFNGLGETVERTEXARRAYINDEXED64IVPROC)load( "glGetVertexArrayIndexed64iv" );
	glad_glCreateSamplers = (PFNGLCREATESAMPLERSPROC)load( "glCreateSamplers" );
	glad_glCreateProgramPipelines = (PFNGLCREATEPROGRAMPIPELINESPROC)load( "glCreateProgramPipelines" );
	glad_glCreateQueries = (PFNGLCREATEQUERIESPROC)load( "glCreateQueries" );
	glad_glGetQueryBufferObjecti64v = (PFNGLGETQUERYBUFFEROBJECTI64VPROC)load( "glGetQueryBufferObjecti64v" );
	glad_glGetQueryBufferObjectiv = (PFNGLGETQUERYBUFFEROBJECTIVPROC)load( "glGetQueryBufferObjectiv" );
	glad_glGetQueryBufferObjectui64v = (PFNGLGETQUERYBUFFEROBJECTUI64VPROC)load( "glGetQueryBufferObjectui64v" );
	glad_glGetQueryBufferObjectuiv = (PFNGLGETQUERYBUFFEROBJECTUIVPROC)load( "glGetQueryBufferObjectuiv" );
	glad_glMemoryBarrierByRegion = (PFNGLMEMORYBARRIERBYREGIONPROC)load( "glMemoryBarrierByRegion" );
	glad_glGetTextureSubImage = (PFNGLGETTEXTURESUBIMAGEPROC)load( "glGetTextureSubImage" );
	glad_glGetCompressedTextureSubImage = (PFNGLGETCOMPRESSEDTEXTURESUBIMAGEPROC)load( "glGetCompressedTextureSubImage" );
	glad_glGetGraphicsResetStatus = (PFNGLGETGRAPHICSRESETSTATUSPROC)load( "glGetGraphicsResetStatus" );
	glad_glGetnCompressedTexImage = (PFNGLGETNCOMPRESSEDTEXIMAGEPROC)load( "glGetnCompressedTexImage" );
	glad_glGetnTexImage = (PFNGLGETNTEXIMAGEPROC)load( "glGetnTexImage" );
	glad_glGetnUniformdv = (PFNGLGETNUNIFORMDVPROC)load( "glGetnUniformdv" );
	glad_glGetnUniformfv = (PFNGLGETNUNIFORMFVPROC)load( "glGetnUniformfv" );
	glad_glGetnUniformiv = (PFNGLGETNUNIFORMIVPROC)load( "glGetnUniformiv" );
	glad_glGetnUniformuiv = (PFNGLGETNUNIFORMUIVPROC)load( "glGetnUniformuiv" );
	glad_glReadnPixels = (PFNGLREADNPIXELSPROC)load( "glReadnPixels" );
	glad_glGetnMapdv = (PFNGLGETNMAPDVPROC)load( "glGetnMapdv" );
	glad_glGetnMapfv = (PFNGLGETNMAPFVPROC)load( "glGetnMapfv" );
	glad_glGetnMapiv = (PFNGLGETNMAPIVPROC)load( "glGetnMapiv" );
	glad_glGetnPixelMapfv = (PFNGLGETNPIXELMAPFVPROC)load( "glGetnPixelMapfv" );
	glad_glGetnPixelMapuiv = (PFNGLGETNPIXELMAPUIVPROC)load( "glGetnPixelMapuiv" );
	glad_glGetnPixelMapusv = (PFNGLGETNPIXELMAPUSVPROC)load( "glGetnPixelMapusv" );
	glad_glGetnPolygonStipple = (PFNGLGETNPOLYGONSTIPPLEPROC)load( "glGetnPolygonStipple" );
	glad_glGetnColorTable = (PFNGLGETNCOLORTABLEPROC)load( "glGetnColorTable" );
	glad_glGetnConvolutionFilter = (PFNGLGETNCONVOLUTIONFILTERPROC)load( "glGetnConvolutionFilter" );
	glad_glGetnSeparableFilter = (PFNGLGETNSEPARABLEFILTERPROC)load( "glGetnSeparableFilter" );
	glad_glGetnHistogram = (PFNGLGETNHISTOGRAMPROC)load( "glGetnHistogram" );
	glad_glGetnMinmax = (PFNGLGETNMINMAXPROC)load( "glGetnMinmax" );
	glad_glTextureBarrier = (PFNGLTEXTUREBARRIERPROC)load( "glTextureBarrier" );
}
static void load_GL_VERSION_4_6( GLADloadproc load )
{
	if (!GLAD_GL_VERSION_4_6) return;
	glad_glSpecializeShader = (PFNGLSPECIALIZESHADERPROC)load( "glSpecializeShader" );
	glad_glMultiDrawArraysIndirectCount = (PFNGLMULTIDRAWARRAYSINDIRECTCOUNTPROC)load( "glMultiDrawArraysIndirectCount" );
	glad_glMultiDrawElementsIndirectCount = (PFNGLMULTIDRAWELEMENTSINDIRECTCOUNTPROC)load( "glMultiDrawElementsIndirectCount" );
	glad_glPolygonOffsetClamp = (PFNGLPOLYGONOFFSETCLAMPPROC)load( "glPolygonOffsetClamp" );
}
static void load_GL_EXT_memory_object( GLADloadproc load )
{
	if (!GLAD_GL_EXT_memory_object) return;
	glad_glGetUnsignedBytevEXT = (PFNGLGETUNSIGNEDBYTEVEXTPROC)load( "glGetUnsignedBytevEXT" );
	glad_glGetUnsignedBytei_vEXT = (PFNGLGETUNSIGNEDBYTEI_VEXTPROC)load( "glGetUnsignedBytei_vEXT" );
	glad_glDeleteMemoryObjectsEXT = (PFNGLDELETEMEMORYOBJECTSEXTPROC)load( "glDeleteMemoryObjectsEXT" );
	glad_glIsMemoryObjectEXT = (PFNGLISMEMORYOBJECTEXTPROC)load( "glIsMemoryObjectEXT" );
	glad_glCreateMemoryObjectsEXT = (PFNGLCREATEMEMORYOBJECTSEXTPROC)load( "glCreateMemoryObjectsEXT" );
	glad_glMemoryObjectParameterivEXT = (PFNGLMEMORYOBJECTPARAMETERIVEXTPROC)load( "glMemoryObjectParameterivEXT" );
	glad_glGetMemoryObjectParameterivEXT = (PFNGLGETMEMORYOBJECTPARAMETERIVEXTPROC)load( "glGetMemoryObjectParameterivEXT" );
	glad_glTexStorageMem2DEXT = (PFNGLTEXSTORAGEMEM2DEXTPROC)load( "glTexStorageMem2DEXT" );
	glad_glTexStorageMem2DMultisampleEXT = (PFNGLTEXSTORAGEMEM2DMULTISAMPLEEXTPROC)load( "glTexStorageMem2DMultisampleEXT" );
	glad_glTexStorageMem3DEXT = (PFNGLTEXSTORAGEMEM3DEXTPROC)load( "glTexStorageMem3DEXT" );
	glad_glTexStorageMem3DMultisampleEXT = (PFNGLTEXSTORAGEMEM3DMULTISAMPLEEXTPROC)load( "glTexStorageMem3DMultisampleEXT" );
	glad_glBufferStorageMemEXT = (PFNGLBUFFERSTORAGEMEMEXTPROC)load( "glBufferStorageMemEXT" );
	glad_glTextureStorageMem2DEXT = (PFNGLTEXTURESTORAGEMEM2DEXTPROC)load( "glTextureStorageMem2DEXT" );
	glad_glTextureStorageMem2DMultisampleEXT = (PFNGLTEXTURESTORAGEMEM2DMULTISAMPLEEXTPROC)load( "glTextureStorageMem2DMultisampleEXT" );
	glad_glTextureStorageMem3DEXT = (PFNGLTEXTURESTORAGEMEM3DEXTPROC)load( "glTextureStorageMem3DEXT" );
	glad_glTextureStorageMem3DMultisampleEXT = (PFNGLTEXTURESTORAGEMEM3DMULTISAMPLEEXTPROC)load( "glTextureStorageMem3DMultisampleEXT" );
	glad_glNamedBufferStorageMemEXT = (PFNGLNAMEDBUFFERSTORAGEMEMEXTPROC)load( "glNamedBufferStorageMemEXT" );
	glad_glTexStorageMem1DEXT = (PFNGLTEXSTORAGEMEM1DEXTPROC)load( "glTexStorageMem1DEXT" );
	glad_glTextureStorageMem1DEXT = (PFNGLTEXTURESTORAGEMEM1DEXTPROC)load( "glTextureStorageMem1DEXT" );
}
static void load_GL_EXT_memory_object_fd( GLADloadproc load )
{
	if (!GLAD_GL_EXT_memory_object_fd) return;
	glad_glImportMemoryFdEXT = (PFNGLIMPORTMEMORYFDEXTPROC)load( "glImportMemoryFdEXT" );
}
static void load_GL_EXT_memory_object_win32( GLADloadproc load )
{
	if (!GLAD_GL_EXT_memory_object_win32) return;
	glad_glImportMemoryWin32HandleEXT = (PFNGLIMPORTMEMORYWIN32HANDLEEXTPROC)load( "glImportMemoryWin32HandleEXT" );
	glad_glImportMemoryWin32NameEXT = (PFNGLIMPORTMEMORYWIN32NAMEEXTPROC)load( "glImportMemoryWin32NameEXT" );
}
static int find_extensionsGL( void )
{
	if (!get_exts()) return 0;
	GLAD_GL_EXT_memory_object = has_ext( "GL_EXT_memory_object" );
	GLAD_GL_EXT_memory_object_fd = has_ext( "GL_EXT_memory_object_fd" );
	GLAD_GL_EXT_memory_object_win32 = has_ext( "GL_EXT_memory_object_win32" );
	free_exts();
	return 1;
}

static void find_coreGL( void )
{

	/* Thank you @elmindreda
	 * https://github.com/elmindreda/greg/blob/master/templates/greg.c.in#L176
	 * https://github.com/glfw/glfw/blob/master/src/context.c#L36
	 */
	int i, major, minor;

	const char* version;
	const char* prefixes[] = {
		"OpenGL ES-CM ",
		"OpenGL ES-CL ",
		"OpenGL ES ",
		NULL
	};

	version = (const char*)glGetString( GL_VERSION );
	if (!version) return;

	for (i = 0; prefixes[i]; i++)
	{
		const size_t length = strlen( prefixes[i] );
		if (strncmp( version, prefixes[i], length ) == 0)
		{
			version += length;
			break;
		}
	}

	/* PR #18 */
#ifdef _MSC_VER
	sscanf_s( version, "%d.%d", &major, &minor );
#else
	sscanf( version, "%d.%d", &major, &minor );
#endif

	GLVersion.major = major; GLVersion.minor = minor;
	max_loaded_major = major; max_loaded_minor = minor;
	GLAD_GL_VERSION_1_0 = (major == 1 && minor >= 0) || major > 1;
	GLAD_GL_VERSION_1_1 = (major == 1 && minor >= 1) || major > 1;
	GLAD_GL_VERSION_1_2 = (major == 1 && minor >= 2) || major > 1;
	GLAD_GL_VERSION_1_3 = (major == 1 && minor >= 3) || major > 1;
	GLAD_GL_VERSION_1_4 = (major == 1 && minor >= 4) || major > 1;
	GLAD_GL_VERSION_1_5 = (major == 1 && minor >= 5) || major > 1;
	GLAD_GL_VERSION_2_0 = (major == 2 && minor >= 0) || major > 2;
	GLAD_GL_VERSION_2_1 = (major == 2 && minor >= 1) || major > 2;
	GLAD_GL_VERSION_3_0 = (major == 3 && minor >= 0) || major > 3;
	GLAD_GL_VERSION_3_1 = (major == 3 && minor >= 1) || major > 3;
	GLAD_GL_VERSION_3_2 = (major == 3 && minor >= 2) || major > 3;
	GLAD_GL_VERSION_3_3 = (major == 3 && minor >= 3) || major > 3;
	GLAD_GL_VERSION_4_0 = (major == 4 && minor >= 0) || major > 4;
	GLAD_GL_VERSION_4_1 = (major == 4 && minor >= 1) || major > 4;
	GLAD_GL_VERSION_4_2 = (major == 4 && minor >= 2) || major > 4;
	GLAD_GL_VERSION_4_3 = (major == 4 && minor >= 3) || major > 4;
	GLAD_GL_VERSION_4_4 = (major == 4 && minor >= 4) || major > 4;
	GLAD_GL_VERSION_4_5 = (major == 4 && minor >= 5) || major > 4;
	GLAD_GL_VERSION_4_6 = (major == 4 && minor >= 6) || major > 4;
	if (GLVersion.major > 4 || (GLVersion.major >= 4 && GLVersion.minor >= 6))
	{
		max_loaded_major = 4;
		max_loaded_minor = 6;
	}
}

int gladLoadGLLoader( GLADloadproc load )
{
	GLVersion.major = 0; GLVersion.minor = 0;
	glGetString = (PFNGLGETSTRINGPROC)load( "glGetString" );
	if (glGetString == NULL) return 0;
	if (glGetString( GL_VERSION ) == NULL) return 0;
	find_coreGL();
	load_GL_VERSION_1_0( load );
	load_GL_VERSION_1_1( load );
	load_GL_VERSION_1_2( load );
	load_GL_VERSION_1_3( load );
	load_GL_VERSION_1_4( load );
	load_GL_VERSION_1_5( load );
	load_GL_VERSION_2_0( load );
	load_GL_VERSION_2_1( load );
	load_GL_VERSION_3_0( load );
	load_GL_VERSION_3_1( load );
	load_GL_VERSION_3_2( load );
	load_GL_VERSION_3_3( load );
	load_GL_VERSION_4_0( load );
	load_GL_VERSION_4_1( load );
	load_GL_VERSION_4_2( load );
	load_GL_VERSION_4_3( load );
	load_GL_VERSION_4_4( load );
	load_GL_VERSION_4_5( load );
	load_GL_VERSION_4_6( load );

	if (!find_extensionsGL()) return 0;
	load_GL_EXT_memory_object( load );
	load_GL_EXT_memory_object_fd( load );
	load_GL_EXT_memory_object_win32( load );
	return GLVersion.major != 0 || GLVersion.minor != 0;
}

// EOF