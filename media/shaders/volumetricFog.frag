
#version 120

uniform sampler2D sceneTex;
uniform sampler2D bufferA;
uniform sampler2D bufferB;
uniform vec4  fogColor;
uniform float density;
uniform bool  doSquared;

void
main(void)
{
  float z = texture2D(bufferA, gl_TexCoord[0].st).r -
    texture2D(bufferB, gl_TexCoord[0].st).r;
  z = max(0.0, z);
  

  float exponent = (z * density);
  if (doSquared) {
    exponent = exponent * exponent;
  }

  float fogFactor = exp(-exponent);

  vec4 c = mix(fogColor, texture2D(sceneTex, gl_TexCoord[0].st), fogFactor);
  gl_FragColor = c;
}
