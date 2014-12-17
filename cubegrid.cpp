#include "math.h"
#include "cubegrid.h"
#ifdef __MAC__
#   include <OpenGL/gl3.h>
#   include <GLUT/glut.h>
#else
#include <GL/glew.h>
#   include <GL/glut.h>
#endif
#include <cstddef>
#include <vector>

bool CubeGrid::CreateMemory()
{
	vertices=new CubeGridVtx[(maxGridSize+1)*(maxGridSize+1)*(maxGridSize+1)];
	if(!vertices)
	{
		return false;
	}

	cubes = new CubeGridCube[maxGridSize*maxGridSize*maxGridSize];
	if(!cubes)
	{
		return false;
	}

	return true;
}
		

bool CubeGrid::Init(int gridSize)
{
	
	//VERTICES
	numVertices=(gridSize+1)*(gridSize+1)*(gridSize+1);
	
	int currentVertex=0;

	for(int i=0; i<gridSize+1; i++)
	{
		for(int j=0; j<gridSize+1; j++)
		{
			for(int k=0; k<gridSize+1; k++)
			{
				vertices[currentVertex].pos = Cvec3((i*20.0)/(gridSize)-10.0, (j*20.0)/(gridSize)-10.0, (k*20.0)/(gridSize)-10.0);
				currentVertex++;
			}
		}
	}

	//CUBES
	numCubes=(gridSize)*(gridSize)*(gridSize);

	int currentCube=0;

	for(int i=0; i<gridSize; i++)
	{
		for(int j=0; j<gridSize; j++)
		{
			for(int k=0; k<gridSize; k++)
			{
				cubes[currentCube].vertices[0] = &vertices[(i*(gridSize+1)+j)*(gridSize+1)+k];
				cubes[currentCube].vertices[1] = &vertices[(i*(gridSize+1)+j)*(gridSize+1)+k+1];
				cubes[currentCube].vertices[2] = &vertices[(i*(gridSize+1)+(j+1))*(gridSize+1)+k+1];
				cubes[currentCube].vertices[3] = &vertices[(i*(gridSize+1)+(j+1))*(gridSize+1)+k];
				cubes[currentCube].vertices[4] = &vertices[((i+1)*(gridSize+1)+j)*(gridSize+1)+k];
				cubes[currentCube].vertices[5] = &vertices[((i+1)*(gridSize+1)+j)*(gridSize+1)+k+1];
				cubes[currentCube].vertices[6] = &vertices[((i+1)*(gridSize+1)+(j+1))*(gridSize+1)+k+1];
				cubes[currentCube].vertices[7] = &vertices[((i+1)*(gridSize+1)+(j+1))*(gridSize+1)+k];

				currentCube++;
			}
		}
	}

	return true;
}

std::vector<SurfaceVtx> CubeGrid::GetSurface(double threshold)
{
	numFacesDrawn=0;

	static SurfaceVtx edgeVertices[12];
	std::vector<SurfaceVtx> verts;

	//loop through cubes
	for(int i=0; i<numCubes; i++)
	{
		//calculate which vertices are inside the surface
		unsigned char cubeIndex=0;

		if(cubes[i].vertices[0]->value < threshold)
			cubeIndex |= 1;
		if(cubes[i].vertices[1]->value < threshold)
			cubeIndex |= 2;
		if(cubes[i].vertices[2]->value < threshold)
			cubeIndex |= 4;
		if(cubes[i].vertices[3]->value < threshold)
			cubeIndex |= 8;
		if(cubes[i].vertices[4]->value < threshold)
			cubeIndex |= 16;
		if(cubes[i].vertices[5]->value < threshold)
			cubeIndex |= 32;
		if(cubes[i].vertices[6]->value < threshold)
			cubeIndex |= 64;
		if(cubes[i].vertices[7]->value < threshold)
			cubeIndex |= 128;

		//look this value up in the edge table to see which edges to interpolate along
		int usedEdges=edgeTable[cubeIndex];

		//if the cube is entirely within/outside surface, no faces			
		if(usedEdges==0 || usedEdges==255)
			continue;

		//update these edges
		for(int currentEdge=0; currentEdge<12; currentEdge++)
		{
			if(usedEdges & 1<<currentEdge)
			{
				CubeGridVtx * v1 = cubes[i].vertices[verticesAtEndsOfEdges[currentEdge*2  ]];
				CubeGridVtx * v2 = cubes[i].vertices[verticesAtEndsOfEdges[currentEdge*2+1]];
			
				double delta = (threshold - v1->value)/(v2->value - v1->value);
				//edgeVertices[currentEdge].pos=v1->pos + delta*(v2->pos - v1->pos);
				edgeVertices[currentEdge].pos[0]=v1->pos[0] + delta*(v2->pos[0] - v1->pos[0]);
				edgeVertices[currentEdge].pos[1]=v1->pos[1] + delta*(v2->pos[1] - v1->pos[1]);
				edgeVertices[currentEdge].pos[2]=v1->pos[2] + delta*(v2->pos[2] - v1->pos[2]);
				//edgeVertices[currentEdge].normal=v1->normal + delta*(v2->normal - v1->normal);
				edgeVertices[currentEdge].normal[0]=v1->normal[0] + delta*(v2->normal[0] - v1->normal[0]);
				edgeVertices[currentEdge].normal[1]=v1->normal[1] + delta*(v2->normal[1] - v1->normal[1]);
				edgeVertices[currentEdge].normal[2]=v1->normal[2] + delta*(v2->normal[2] - v1->normal[2]);
			}
		}

		
		//send the vertices
		for(int k=0; triTable[cubeIndex][k]!=-1; k+=3)
		{
			verts.push_back(edgeVertices[triTable[cubeIndex][k]]);
			verts.push_back(edgeVertices[triTable[cubeIndex][k+2]]);
			verts.push_back(edgeVertices[triTable[cubeIndex][k+1]]);
							
			numFacesDrawn++;
		}
	}
	return verts;
}

void CubeGrid::FreeMemory()
{
	if(vertices)
		delete [] vertices;
	vertices=NULL;
	numVertices=0;

	if(cubes)
		delete [] cubes;
	cubes=NULL;
	numCubes=0;
}