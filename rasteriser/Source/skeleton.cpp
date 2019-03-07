#include <iostream>
#include <glm/glm.hpp>
#include <SDL.h>
#include "SDLauxiliary.h"
#include "TestModelH.h"
#include <stdint.h>
#include <glm/gtx/string_cast.hpp>

using namespace std;
using glm::vec3;
using glm::mat3;
using glm::vec4;
using glm::mat4;
using glm::vec2;
using glm::ivec2;

SDL_Event event;

#define SCREEN_WIDTH 256
#define SCREEN_HEIGHT 256
#define FULLSCREEN_MODE false

vector<Triangle> triangles;
vec4 cameraPos( 0, 0, -3.001,1 );
mat4 R;
float yaw = 0; // Yaw angle controlling camera rotation around y-axis

/* ----------------------------------------------------------------------------*/
/* FUNCTIONS                                                                   */

bool Update();
void Draw(screen* screen);
void VertexShader( const vec4& v, ivec2& p );
void DrawLineSDL( screen* screen, ivec2 a, ivec2 b, vec3 color );
void DrawPolygonEdges( screen* screen, const vector<vec4>& vertices );
void ComputePolygonRows(const vector<ivec2>& vertexPixels, vector<ivec2>& leftPixels, vector<ivec2>& rightPixels );
void Interpolate( ivec2 a, ivec2 b, vector<ivec2>& result );
void TransformationMatrix(mat4& M);

int main( int argc, char* argv[] )
{

  screen *screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT, FULLSCREEN_MODE );
  LoadTestModel(triangles);
  // Initialise R
  R = mat4(vec4(cos(yaw),0.0,sin(yaw),0.0), vec4(0.0,1.0,0.0,0.0), vec4(-sin(yaw),0.0,cos(yaw),0.0), vec4(0.0,0.0,0.0,1.0));

  while ( Update())
    {
      Draw(screen);
      SDL_Renderframe(screen);
    }

  SDL_SaveImage( screen, "screenshot.bmp" );

  KillSDL(screen);
  return 0;
}

/*Place your drawing here*/
void Draw(screen* screen)
{
  /* Clear buffer */
  memset(screen->buffer, 0, screen->height*screen->width*sizeof(uint32_t));

/*
  vec3 testcolor(1,0,0);
  ivec2 a(0,0);
  ivec2 b(SCREEN_WIDTH/2, SCREEN_HEIGHT/2);
  DrawLineSDL(screen, a, b, testcolor);
*/

  for( uint32_t i=0; i<triangles.size(); ++i ) {
    vector<vec4> vertices(3);

    vertices[0] = triangles[i].v0;
    vertices[1] = triangles[i].v1;
    vertices[2] = triangles[i].v2;

/*
    for(int v=0; v<3; ++v) {
      ivec2 projPos;
      VertexShader( vertices[v], projPos );
      vec3 color(1,1,1);
      PutPixelSDL( screen, projPos.x, projPos.y, color );
    }
*/

    DrawPolygonEdges(screen, vertices);
  }
}

void VertexShader( const vec4& v, ivec2& p ) {
  float focalLength = SCREEN_HEIGHT;
  mat4 M;
  TransformationMatrix(M);
  // cout << "["<< glm::to_string(v) << std::endl;
  // cout << glm::to_string(M) << std::endl;
  // vec4 vmap = M * v;
  // vec4 vprime = vmap / vmap.w;
  vec4 vprime = v - cameraPos;
  // cout << glm::to_string(vprime) << "]" << std::endl;
  int xprime = floor(focalLength * vprime.x / vprime.z + (SCREEN_WIDTH/2));
  int yprime = floor(focalLength * vprime.y / vprime.z + (SCREEN_HEIGHT/2));
  p.x = xprime;
  p.y = yprime;
}

void DrawLineSDL( screen* screen, ivec2 a, ivec2 b, vec3 color ) {
  ivec2 delta = glm::abs(a - b);
  int pixels = glm::max(delta.x, delta.y) + 1;
  vector<ivec2> line(pixels);
  Interpolate(a, b, line);
  for (int i = 0; i < pixels; ++i) {
    PutPixelSDL(screen, line[i].x, line[i].y, color);
  }
}

void DrawPolygonEdges( screen* screen, const vector<vec4>& vertices ) {
  int V = vertices.size();

  // Transform each vertex from 3D world position to 2D image position:
  vector<ivec2> projectedVertices( V );
  for( int i=0; i<V; ++i ) {
    VertexShader( vertices[i], projectedVertices[i] );
  }

  // Loop over all vertices and draw the edge from it to the next vertex:
  for( int i=0; i<V; ++i ) {
    int j = (i+1)%V; // The next vertex
    vec3 color( 1, 1, 1 );
    DrawLineSDL( screen, projectedVertices[i], projectedVertices[j], color );
  }
}

void ComputePolygonRows(const vector<ivec2>& vertexPixels, vector<ivec2>& leftPixels, vector<ivec2>& rightPixels ) {
  // 1. Find max and min y-value of the polygon
  // and compute the number of rows it occupies.

  // 2. Resize leftPixels and rightPixels
  // so that they have an element for each row.

  // 3. Initialize the x-coordinates in leftPixels
  // to some really large value and the x-coordinates
  // in rightPixels to some really small value.

  // 4. Loop through all edges of the polygon and use
  // linear interpolation to find the x-coordinate for
  // each row it occupies. Update the corresponding
  // values in rightPixels and leftPixels.
}

/*Place updates of parameters here*/
bool Update()
{
  static int t = SDL_GetTicks();
  /* Compute frame time */
  int t2 = SDL_GetTicks();
  float dt = float(t2-t);
  t = t2;

  SDL_Event e;
  while(SDL_PollEvent(&e))
    {
      if (e.type == SDL_QUIT)
	{
	  return false;
	}
      else
	if (e.type == SDL_KEYDOWN)
	  {
	    int key_code = e.key.keysym.sym;
	    switch(key_code)
	      {
	      case SDLK_UP:
		      /* Move camera forward */
          cameraPos = vec4(cameraPos.x, cameraPos.y, cameraPos.z + 0.1, 1.0);
		      break;
	      case SDLK_DOWN:
		      /* Move camera backwards */
          cameraPos = vec4(cameraPos.x, cameraPos.y, cameraPos.z - 0.1, 1.0);
		      break;
	      case SDLK_LEFT:
		      /* Move camera left */
          // cameraPos = vec4(cameraPos.x - 0.1, cameraPos.y, cameraPos.z, 1.0);
          yaw += 0.1;
          R = mat4(vec4(cos(yaw),0.0,sin(yaw),0.0), vec4(0.0,1.0,0.0,0.0), vec4(-sin(yaw),0.0,cos(yaw),0.0), vec4(0.0,0.0,0.0,1.0));
		      break;
	      case SDLK_RIGHT:
      		/* Move camera right */
          yaw -= 0.1;
          R = mat4(vec4(cos(yaw),0.0,sin(yaw),0.0), vec4(0.0,1.0,0.0,0.0), vec4(-sin(yaw),0.0,cos(yaw),0.0), vec4(0.0,0.0,0.0,1.0));
      		break;
	      case SDLK_ESCAPE:
      		/* Move camera quit */
      		return false;
	      }
	  }
    }
  return true;
}

void Interpolate( ivec2 a, ivec2 b, vector<ivec2>& result ) {
  int N = result.size();
  vec2 step = vec2(b-a) / float(max(N-1,1));
  vec2 current( a );
  for( int i=0; i<N; ++i ) {
    result[i] = round(current);
    current += step;
  }
}

void TransformationMatrix(mat4& M) {
  mat4 tr(vec4(1.0,0.0,0.0,cameraPos.x), vec4(0.0,1.0,0.0,cameraPos.y), vec4(0.0,0.0,1.0,cameraPos.z), vec4(0.0,0.0,0.0,1.0));
  mat4 negtr(vec4(1.0,0.0,0.0,-1*cameraPos.x), vec4(0.0,1.0,0.0,-1*cameraPos.y), vec4(0.0,0.0,1.0,-1*cameraPos.z), vec4(0.0,0.0,0.0,1.0));
  M = tr * R;
}
