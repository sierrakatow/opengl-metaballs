#include <fstream>
#include <vector>
#include <string>
#include <iostream>
#include <stdexcept>

#include "glsupport.h"

using namespace std;

void checkGlErrors() {
  const GLenum errCode = glGetError();

  if (errCode != GL_NO_ERROR) {
    string error("GL Error: ");
    error += reinterpret_cast<const char*>(gluErrorString(errCode));
    cerr << error << endl;
    throw runtime_error(error);
  }
}

// Dump text file into a character vector, throws exception on error
static void readTextFile(const char *fn, vector<char>& data) {
  // Sets ios::binary bit to prevent end of line translation, so that the
  // number of bytes we read equals file size
  ifstream ifs(fn, ios::binary);
  if (!ifs)
    throw runtime_error(string("Cannot open file ") + fn);

  // Sets bits to report IO error using exception
  ifs.exceptions(ios::eofbit | ios::failbit | ios::badbit);
  ifs.seekg(0, ios::end);
  size_t len = ifs.tellg();

  data.resize(len);

  ifs.seekg(0, ios::beg);
  ifs.read(&data[0], len);
}

// GL 3 core needs separate functions for shader and program info logs

// Print info regarding a GL program
static void printProgramInfoLog(GLuint obj, const string& filename) {
  GLsizei infoLogLength, charsWritten;
  glGetProgramiv(obj, GL_INFO_LOG_LENGTH, &infoLogLength);
  if (infoLogLength > 0) {
    string infoLog(infoLogLength, ' ');
    glGetProgramInfoLog(obj, infoLogLength, &charsWritten, &infoLog[0]);
    std::cerr << "##### Log [" << filename << "]:\n" << infoLog << endl;
  }
}

// Print info regarding a GL shader
static void printShaderInfoLog(GLuint obj, const string& filename) {
  GLsizei infoLogLength, charsWritten;
  glGetShaderiv(obj, GL_INFO_LOG_LENGTH, &infoLogLength);
  if (infoLogLength > 0) {
    string infoLog(infoLogLength, ' ');
    glGetShaderInfoLog(obj, infoLogLength, &charsWritten, &infoLog[0]);
    std::cerr << "##### Log [" << filename << "]:\n" << infoLog << endl;
  }
}

static void compileShader(GLuint shaderHandle, int sourceLength, const char *source, const char *filenameHint) {
  const char *ptrs[] = {source};
  const GLint lens[] = {sourceLength};
  glShaderSource(shaderHandle, 1, ptrs, lens);   // load the shader sources

  glCompileShader(shaderHandle);

  printShaderInfoLog(shaderHandle, filenameHint);

  GLint compiled = 0;
  glGetShaderiv(shaderHandle, GL_COMPILE_STATUS, &compiled);
  if (!compiled)
    throw runtime_error("fails to compile GL shader");
}

void readAndCompileSingleShaderFromMemory(GLuint shaderHandle,
                                          int sourceLength, const char *source) {
  compileShader(shaderHandle, sourceLength, source, "<in-memory source>");
}

void readAndCompileSingleShader(GLuint shaderHandle, const char *fn) {
  vector<char> source;
  readTextFile(fn, source);
  compileShader(shaderHandle, source.size(), &source[0], fn);
}

void linkShader(GLuint programHandle, GLuint vs, GLuint fs) {
  glAttachShader(programHandle, vs);
  glAttachShader(programHandle, fs);

  glLinkProgram(programHandle);

  glDetachShader(programHandle, vs);
  glDetachShader(programHandle, fs);

  GLint linked = 0;
  glGetProgramiv(programHandle, GL_LINK_STATUS, &linked);
  printProgramInfoLog(programHandle, "linking");

  if (!linked)
    throw runtime_error("fails to link shaders");
}


void readAndCompileShader(GLuint programHandle, const char * vertexShaderFileName, const char * fragmentShaderFileName) {
  GlShader vs(GL_VERTEX_SHADER);
  GlShader fs(GL_FRAGMENT_SHADER);

  readAndCompileSingleShader(vs, vertexShaderFileName);
  readAndCompileSingleShader(fs, fragmentShaderFileName);

  linkShader(programHandle, vs, fs);
}


void readAndCompileShaderFromMemory(GLuint programHandle,
                                    int vsSourceLength, const char *vsSource,
                                    int fsSourceLength, const char *fsSource) {
  GlShader vs(GL_VERTEX_SHADER);
  GlShader fs(GL_FRAGMENT_SHADER);

  readAndCompileSingleShaderFromMemory(vs, vsSourceLength, vsSource);
  readAndCompileSingleShaderFromMemory(fs, fsSourceLength, fsSource);

  linkShader(programHandle, vs, fs);
}
