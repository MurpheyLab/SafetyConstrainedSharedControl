
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
//#define _USE_MATH_DEFINES
#define STB_IMAGE_IMPLEMENTATION

#include <glut.h> 
#include <math.h>
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <vector>
#include "stb/stb_image.h" 
#include <string.h>
#include <cstring>

// Change path for your computer
const char* tablepath = "C:\\Users\\kyrad\\Documents\\GitHub\\SafetyConstrainedSharedControl\\Textures\\wood_2.png";

//using namespace std;

class Renderer {

public:
  float t; 
private:
  GLuint texID_table;

public:
  // constructor
  Renderer() : t(0.0), texID_table(0) {}

  // destructor
  ~Renderer() {
    if(texID_table !=0) {
      glDeleteTextures( 1, &texID_table);
    }
  }

public:
  void init() {
    glEnable(GL_DEPTH_TEST);
    glEnable(GLenum(GL_POINT_SIZE));
    glEnable(GLenum(GL_POINT_SMOOTH));

    // assign ID# to each loaded texture
    texID_table = loadTexture(tablepath);
  }

  void DrawTable() 
  {
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    // set camera
    //gluLookAt(0.3, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0); // water zoom in for testing water level
    //gluLookAt(0.65, 0.0, 0.35, -0.015, 0.0, 0.0, 0.0, 0.0, 1.0);
    gluLookAt(10.2, 0.0, 5.1, -0.015, 0.0, 0.0, 0.0, 0.0, 1.0);

    glEnable(GL_TEXTURE_2D);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glBindTexture(GL_TEXTURE_2D, texID_table);
    glColor3f(1.0f,1.0f,0.0f);
    glBegin(GL_POLYGON);
    glTexCoord2f(0.0f,0.0f); glVertex3f(-8.5f, 3.0f, 0.0f);
    glTexCoord2f(1.0f,0.0f); glVertex3f(3.0f,3.0f, 0.0f);
    glTexCoord2f(1.0f,1.0f); glVertex3f( 3.0f,-3.0f, 0.0f);
    glTexCoord2f(0.0f,1.0f); glVertex3f( -8.5f, -3.0f, 0.0f);
    //glTexCoord2f(0.0f, 0.0f); glVertex3f(-0.85f, 0.3f, 0.0f);
    //glTexCoord2f(1.0f, 0.0f); glVertex3f(0.3f, 0.3f, 0.0f);
    //glTexCoord2f(1.0f, 1.0f); glVertex3f(0.3f, -0.3f, 0.0f);
    //glTexCoord2f(0.0f, 1.0f); glVertex3f(-0.85f, -0.3f, 0.0f);
    glEnd();

    glDisable(GL_TEXTURE_2D);
    glDisable(GL_BLEND);
  }

private:
  // returns a valid textureID on success, otherwise 0
  GLuint loadTexture(const char* filename) {

    int width, height, numComponents;
    int level = 0;
    int border = 0;

    // loads image in the correct orientation
    stbi_set_flip_vertically_on_load(1);

    // load image data
    unsigned char* image = stbi_load(filename, &width, &height, &numComponents, STBI_rgb_alpha);

    if (image == NULL)
    {
      cout << "Cannot load texture" << endl;
    }
    
    // data is aligned in byte order
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

    //request textureID
    GLuint textureID;
    glGenTextures(1, &textureID);

    // bind texture
    glBindTexture(GL_TEXTURE_2D, textureID);

    //define how to filter the texture (important but ignore for now)
    glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

    if(numComponents == 3)
    { 
      glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, image);
    }
    else if (numComponents == 4) 
    {
      glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, image);
    }
    
    //texture colors should replace the original color values
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE); //GL_MODULATE

    // specify the 2D texture map
    glTexImage2D(GL_TEXTURE_2D, level, GL_RGBA, width, height, border, GL_RGBA, GL_UNSIGNED_BYTE, image);

    glBindTexture(GL_TEXTURE_2D, 0);

    stbi_image_free(image);

    // return unique texture identifier
    return textureID;
  }
};
