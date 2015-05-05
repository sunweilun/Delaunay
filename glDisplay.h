#ifndef GLDISPLAY_H_INCLUDED
#define GLDISPLAY_H_INCLUDED

#define GLM_FORCE_RADIANS
#include "glm/gtc/matrix_transform.hpp"
#include "glm/glm.hpp"
#include <GL/freeglut.h>
#include <limits>
#include "Mesh.h"
#include "globalVariables.h"

int width = 1024;
int height = 768;

extern Mesh mesh;

vec3 translation(0, 0, 0);
real scale = 1;
glm::ivec2 mousePos;
int mouseButtonState = -1;

GLuint listIndex;

GLuint pickingListIndex;

glm::mat4 modelMat;

/* Handler for window-repaint event. Call back when the window first appears and
   whenever the window needs to be re-painted. */
void display() {
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f); // Set background color to black and opaque
    glClear(GL_COLOR_BUFFER_BIT);         // Clear the color buffer

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0, width, 0, height);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glTranslatef(width/2.0f, height/2.0f, 0);
    glScalef(scale, scale, scale);
    glTranslatef(translation.x, translation.y, 0);
    glMultMatrixf((GLfloat*)&modelMat);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glColor4f(1, 1, 1, 0.5);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glCallList(listIndex);
    glDisable(GL_BLEND);

    glColor3f(0.0f, 1.0f, 0.0f);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glCallList(listIndex);

    glutSwapBuffers();
    glFlush();  // Render now
}

unsigned pick()
{
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f); // Set background color to black and opaque
    glClear(GL_COLOR_BUFFER_BIT);         // Clear the color buffer

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0, width, 0, height);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glTranslatef(width/2.0f, height/2.0f, 0);
    glScalef(scale, scale, scale);
    glTranslatef(translation.x, translation.y, 0);
    glMultMatrixf((GLfloat*)&modelMat);

    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glCallList(pickingListIndex);

    glutSwapBuffers();
    glFlush();  // Render now

    unsigned char rgba[4];
    glReadPixels(mousePos.x, height - 1 - mousePos.y, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, rgba);
    return (rgba[0] << 16) + (rgba[1] << 8) + rgba[2];
}

void reshape(int w, int h)
{
    glViewport(0, 0, width, height);
    width = w;
    height = h;
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0, width, 0, height);
    glutPostRedisplay();
}

void mouse(int button, int state, int x, int y)
{
    if(button == GLUT_LEFT_BUTTON)
        mousePos = glm::ivec2(x, y);
    if(button == GLUT_RIGHT_BUTTON)
    {
        mousePos = glm::ivec2(x, y);
        if(state == GLUT_DOWN)
            printf("%u\n", pick());
    }
    if(button == 3)
        scale *= 1.1;
    if(button == 4)
        scale *= 0.9;
    mouseButtonState = state == GLUT_DOWN ? button : -1;
    glutPostRedisplay();
}

void motion(int x, int y)
{
    if(mouseButtonState == GLUT_LEFT)
        translation += vec3(x - mousePos.x, mousePos.y - y, 0) / scale;
    mousePos = glm::ivec2(x, y);
    glutPostRedisplay();
}

void prepareList()
{
    listIndex = glGenLists(1);
    pickingListIndex = glGenLists(1);
    std::vector<vec3> vList = mesh.getTriangles();
    vec3 minCoord(std::numeric_limits<real>::infinity());
    vec3 maxCoord(-std::numeric_limits<real>::infinity());

    glNewList(pickingListIndex, GL_COMPILE);
    glBegin(GL_TRIANGLES);
    for(unsigned i=0; i<vList.size(); i++)
    {
        unsigned idx = i / 3;
        glColor4ub((idx>>16)&0xff, (idx>>8)&0xff, (idx)&0xff, 255);
        minCoord = glm::min(minCoord, vList[i]);
        maxCoord = glm::max(maxCoord, vList[i]);
        glm::vec3 v;
        v.x = vList[i].x;
        v.y = vList[i].y;
        glVertex3fv((GLfloat*)&v);
    }

    glEnd();
    glEndList();

    glNewList(listIndex, GL_COMPILE);
    glBegin(GL_TRIANGLES);
    for(auto it=vList.begin(); it!=vList.end(); it++)
    {
        minCoord = glm::min(minCoord, *it);
        maxCoord = glm::max(maxCoord, *it);
        glm::vec3 v;
        v.x = it->x;
        v.y = it->y;
        glVertex3fv((GLfloat*)&v);
    }
    glEnd();
    glPointSize(5);
    glBegin(GL_POINTS);
    for(unsigned i=0; i<vList.size(); i++)
    {
        glColor4f(1, 0, 0, 1);
        minCoord = glm::min(minCoord, vList[i]);
        maxCoord = glm::max(maxCoord, vList[i]);
        glm::vec3 v;
        v.x = vList[i].x;
        v.y = vList[i].y;
        glVertex3fv((GLfloat*)&v);
    }
    glEnd();
    glEndList();
    vec3 sVec = maxCoord - minCoord;
    real minSize = std::min(width, height);
    real maxScale = std::max(sVec.x, sVec.y);
    modelMat = glm::scale(modelMat, glm::vec3(minSize / maxScale)*0.9f);
    vec3 trans = -minCoord-sVec*0.5;
    modelMat = glm::translate(modelMat, glm::vec3(trans.x, trans.y, 0));
}

void glDisplay(int argc, char** argv)
{
    glutInit(&argc, argv);                 // Initialize GLUT
    glutCreateWindow("Triangulation"); // Create a window with the given title
    glutInitWindowPosition(50, 50); // Position the window's initial top-left corner
    glutInitWindowSize(width, height);   // Set the window's initial width & height
    glutReshapeWindow(width, height);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
    glutDisplayFunc(display); // Register display callback handler for window re-paint
    glutReshapeFunc(reshape);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    prepareList();
    glutMainLoop();           // Enter the infinitely event-processing loop
}

#endif // GLDISPLAY_H_INCLUDED
