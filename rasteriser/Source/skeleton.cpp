#include <iostream>
#include <glm/glm.hpp>
#include <SDL.h>
#include "SDLauxiliary.h"
#include "TestModelH.h"
#include <stdint.h>

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

/* ----------------------------------------------------------------------------*/
/* FUNCTIONS                                                                   */

bool Update();
void Draw(screen* screen);
void VertexShader( const vec4& v, ivec2& p );
void DrawLineSDL( screen* screen, ivec2 a, ivec2 b, vec3 color );
void DrawPolygonEdges( screen* screen, const vector<vec4>& vertices );
void Interpolate( ivec2 a, ivec2 b, vector<ivec2>& result );

int main( int argc, char* argv[] )
{

  screen *screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT, FULLSCREEN_MODE );
  LoadTestModel(triangles);

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
  vec4 vprime = v - cameraPos;
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
		break;
	      case SDLK_DOWN:
		/* Move camera backwards */
		break;
	      case SDLK_LEFT:
		/* Move camera left */
		break;
	      case SDLK_RIGHT:
		/* Move camera right */
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
