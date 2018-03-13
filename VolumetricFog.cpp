/*
 * Copyright (C) 2017  Florian GOLESTIN
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "VolumetricFog.h"

#include <osg/Program>
#include <osg/Shader>

namespace Soleil {

  // The vertex shader will be used after the main scene is rendered to texture.
  // It simply transfers the positions and texture coordinates.
  static const char* vertSource = {"void main(void)\n"
                                   "{\n"
                                   "gl_Position = ftransform();\n"
                                   "gl_TexCoord[0] = gl_MultiTexCoord0;\n"
                                   "}\n"};
  static const char* fragSource = {
    "uniform sampler2D sceneTex;\n"
    "uniform sampler2D bufferA;\n"
    "uniform sampler2D bufferB;\n"
    "void main(void)\n"
    "{\n"
    // Get noise value from texture, changing by frame time
    // "float factor = osg_FrameTime * 100.0;\n"
    // "vec2 uv = vec2(0.4*sin(factor), 0.4*cos(factor));\n"
    // "vec3 n = texture2D(noiseTex, (gl_TexCoord[0].st*3.5) + uv).rgb;\n"
    "float alpha = texture2D(bufferA, gl_TexCoord[0].st).r - "
    "texture2D(bufferB, gl_TexCoord[0].st).r;"
    "if (alpha < 0.0) alpha = 0.0;" // TODO:
    // Get scene and compute greyscale intensity
    "vec4 c = mix(texture2D(sceneTex, gl_TexCoord[0].st), vec4(0.776, "
    "0.839, 0.851, 1.0), alpha); \n "
    "gl_FragColor = c;\n"
    "}\n"};

  osg::ref_ptr<osg::Program> CreateVolumetricFogProgram()
  {
    osg::ref_ptr<osg::Program> program = new osg::Program;
    program->addShader(new osg::Shader(osg::Shader::VERTEX, vertSource));
    program->addShader(new osg::Shader(osg::Shader::FRAGMENT, fragSource));
    return program;
  }

} // Soleil
