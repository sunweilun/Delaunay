#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/time.h>
#include "macros.h"
#include "globalVariables.h"

#if DISPLAY
#include "glDisplay.h"
#endif

extern Mesh mesh;

int main(int argc, char** argv)
{
    char path[1024];
    char inputPath[1024];
    char outputPath[1024];
    outputPath[0] = inputPath[0] = 0;
    bool timing = false;
    bool disp = false;
    int seed = 0;
    int c;
    mesh.setRandomizeFlag(false);
    while((c = getopt(argc, argv, "wcdr::ti:o:")) != -1)
    {
        switch(c)
        {
        case 'd':
            disp = true;
            break;
        case 'i':
            sprintf(inputPath, "%s", optarg);
            break;
        case 'o':
            sprintf(outputPath, "%s", optarg);
            break;
        case 't':
            timing = true;
            break;
        case 'r':
            mesh.setRandomizeFlag(true);
            if(optarg == NULL)
            {
                struct timeval t;
                gettimeofday(&t, NULL);
                mesh.setSeed(unsigned(t.tv_sec*1e6+t.tv_usec));
            }
            else
            {
                seed = atoi(optarg);
                mesh.setSeed(unsigned(seed));
            }
            break;
        case 'w':
            mesh.setMethod(WALKING);
            break;
        case 'c':
            mesh.setMethod(CONFLICT_GRAPH);
            break;
        }
    }
	if(strlen(inputPath) == 0)
	{
		printf("Invalid input path. To know how to use it, please refer to the document.\n");
		return 0;
	}
    std::vector<vec3> v = mesh.loadNodes(inputPath);
    struct timeval ts, te;
    gettimeofday(&ts, NULL);
    mesh.delaunay(v);
    gettimeofday(&te, NULL);
    double dt = (te.tv_sec-ts.tv_sec)*1e6;
    dt += (te.tv_usec-ts.tv_usec);
    if(timing)
        printf("time = %0.3fs\n", dt/(1e6));
    if(strlen(outputPath) > 0)
    {
        sprintf(path, "%s.node", outputPath);
        mesh.outputNodes(path);
        sprintf(path, "%s.ele", outputPath);
        mesh.outputElements(path);
    }
	if(disp)
	{
#if DISPLAY    
        glDisplay(argc, argv);
#endif
	}
    return 0;
}
