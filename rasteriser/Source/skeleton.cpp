#include <iostream>
#include <glm/glm.hpp>
#include <SDL.h>
#include "SDLauxiliary.h"
#include "TestModelH.h"
#include <stdint.h>
#include <glm/gtx/string_cast.hpp>
#include <limits>

using namespace std;
using glm::vec3;
using glm::mat3;
using glm::vec4;
using glm::mat4;
using glm::vec2;
using glm::ivec2;

SDL_Event event;

#define SCREEN_WIDTH 512
#define SCREEN_HEIGHT 512
#define FULLSCREEN_MODE false

struct Pixel {
  int x;
  int y;
  float zinv;
};

vector<Triangle> triangles;
vec4 cameraPos( 0, 0, -3.001,1 );
mat4 R;
float yaw = 0; // Yaw angle controlling camera rotation around y-axis
float depthBuffer[SCREEN_HEIGHT][SCREEN_WIDTH];

/* ----------------------------------------------------------------------------*/
/* FUNCTIONS                                                                   */

bool Update();
void Draw(screen* screen);
void VertexShader( const vec4& v, Pixel& p );
void DrawLineSDL( screen* screen, ivec2 a, ivec2 b, vec3 color );
void DrawPolygonEdges( screen* screen, const vector<vec4>& vertices );

void ComputePolygonRows(const vector<Pixel>& vertexPixels, vector<Pixel>& leftPixels, vector<Pixel>& rightPixels );
void DrawRows(screen* screen, const vector<Pixel>& leftPixels, const vector<Pixel>& rightPixels, vec3 color );
void DrawPolygon(screen* screen, const vector<vec4>& vertices, vec3 currentColor );

void Interpolate( ivec2 a, ivec2 b, vector<ivec2>& result );
void InterpolateDepth( Pixel a, Pixel b, vector<Pixel>& result );
void TransformationMatrix(mat4& M);
void getEdges( ivec2 a, ivec2 b, vector<ivec2>& line);

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

  for( int y=0; y<SCREEN_HEIGHT; ++y )
    for( int x=0; x<SCREEN_WIDTH; ++x )
      depthBuffer[y][x] = 0;

  for( uint32_t i=0; i<triangles.size(); ++i ) {
    vec3 currentColor = triangles[i].color;

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

    // DrawPolygonEdges(screen, vertices);
    DrawPolygon(screen, vertices, currentColor);
  }
}

void VertexShader( const vec4& v, Pixel& p ) {
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
  p.zinv = 1/vprime.z;
}

void getEdges( Pixel a, Pixel b, vector<Pixel>& line) {
  int deltaX = abs(a.x - b.x);
  int deltaY = abs(a.y - b.y);
  int pixels = max(deltaX, deltaY) + 1;
  line.resize(pixels);
  InterpolateDepth(a, b, line);
}

void DrawLineSDL( screen* screen, ivec2 a, ivec2 b, vec3 color ) {
  /*
  vector<ivec2> line;
  getEdges(a, b, line);
  for (int i = 0; i < line.size(); ++i) {
    PutPixelSDL(screen, line[i].x, line[i].y, color);
  }
  */
}

void DrawPolygonEdges( screen* screen, const vector<vec4>& vertices ) {
  /*
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
  */
}

void ComputePolygonRows(const vector<Pixel>& vertexPixels, vector<Pixel>& leftPixels, vector<Pixel>& rightPixels ) {
  int v = vertexPixels.size();
  // 1. Find max and min y-value of the polygon
  // and compute the number of rows it occupies.
  float min = +numeric_limits<float>::max();
  float max = -numeric_limits<float>::max();
  for (int i=0; i < v; ++i) {
    if (vertexPixels[i].y < min) {
      min = vertexPixels[i].y;
    }
    if (vertexPixels[i].y > max) {
      max = vertexPixels[i].y;
    }
  }

  int rows = max - min + 1;

  // 2. Resize leftPixels and rightPixels
  // so that they have an element for each row.
  leftPixels.resize(rows);
  rightPixels.resize(rows);

  // 3. Initialize the x-coordinates in leftPixels
  // to some really large value and the x-coordinates
  // in rightPixels to some really small value.
  for (int i = 0; i < rows; ++i) {
    leftPixels[i].x  = +numeric_limits<int>::max();
    rightPixels[i].x = -numeric_limits<int>::max();
  }

  // 4. Loop through all edges of the polygon and use
  // linear interpolation to find the x-coordinate for
  // each row it occupies. Update the corresponding
  // values in rightPixels and leftPixels.
  for( int i=0; i<v; ++i ) {
    int j = (i+1)%v; // The next vertex
    vector<Pixel> edge;
    getEdges(vertexPixels[i], vertexPixels[j], edge);
    for ( int p = 0; p < edge.size(); ++p) {
      int row = edge[p].y - min;
      if (leftPixels[row].x > edge[p].x) {
        leftPixels[row].x = edge[p].x;
        leftPixels[row].y = edge[p].y;
        leftPixels[row].zinv = edge[p].zinv;
      }
      if (rightPixels[row].x < edge[p].x) {
        rightPixels[row].x = edge[p].x;
        rightPixels[row].y = edge[p].y;
        rightPixels[row].zinv = edge[p].zinv;
      }
    }

  }
}

void DrawRows(screen* screen, const vector<Pixel>& leftPixels, const vector<Pixel>& rightPixels, vec3 color ) {
  for (int row = 0; row < leftPixels.size(); ++row) {
    int delta = abs(rightPixels[row].x - leftPixels[row].x);
    int currentY = leftPixels[row].y;
    float deltazinv = (leftPixels[row].zinv - rightPixels[row].zinv) / float(max(delta,1));
    float currentzinv = leftPixels[row].zinv;
    for (int col = 0; col <= delta; ++col) {
      int currentX = leftPixels[row].x + col;
      if (currentzinv > depthBuffer[currentY][currentX]) {
        PutPixelSDL(screen, currentX, currentY, color);
        depthBuffer[currentY][currentX] = currentzinv;
      }
      currentzinv += deltazinv;
    }
  }
}

void DrawPolygon(screen* screen, const vector<vec4>& vertices, vec3 currentColor ) {
  int V = vertices.size();

  vector<Pixel> vertexPixels( V );
  for( int i=0; i<V; ++i ) VertexShader( vertices[i], vertexPixels[i] );

    vector<Pixel> leftPixels;
    vector<Pixel> rightPixels;
    ComputePolygonRows( vertexPixels, leftPixels, rightPixels );
    DrawRows(screen, leftPixels, rightPixels, currentColor);
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

void InterpolateDepth( Pixel a, Pixel b, vector<Pixel>& result ) {
  int N = result.size();
  vec2 step = vec2(b.x-a.x, b.y-a.y) / float(max(N-1,1));
  vec2 current = vec2(a.x, a.y);
  float stepzinv = (b.zinv - a.zinv) / float(max(N-1,1));
  float currentzinv = a.zinv;
  for( int i=0; i<N; ++i ) {
    result[i].x = round(current.x);
    result[i].y = round(current.y);
    current += step;
    result[i].zinv = currentzinv;
    currentzinv += stepzinv;
  }
}

void TransformationMatrix(mat4& M) {
  mat4 tr(vec4(1.0,0.0,0.0,cameraPos.x), vec4(0.0,1.0,0.0,cameraPos.y), vec4(0.0,0.0,1.0,cameraPos.z), vec4(0.0,0.0,0.0,1.0));
  mat4 negtr(vec4(1.0,0.0,0.0,-1*cameraPos.x), vec4(0.0,1.0,0.0,-1*cameraPos.y), vec4(0.0,0.0,1.0,-1*cameraPos.z), vec4(0.0,0.0,0.0,1.0));
  M = tr * R;
}
