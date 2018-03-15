
//#version 120 core

uniform sampler2D sceneTex;
uniform sampler2D bufferA;
uniform sampler2D bufferB;
uniform vec4  fogColor;
uniform float density;
uniform bool  doSquared;

void
main(void)
{
  float z = abs(texture2D(bufferA, gl_TexCoord[0].st).r -
                texture2D(bufferB, gl_TexCoord[0].st).r);

  float exponent = (z * density);
  if (doSquared) {
    exponent = exponent * exponent;
  }

  float fogFactor = exp(-exponent);

  vec4 c = mix(fogColor, texture2D(sceneTex, gl_TexCoord[0].st), fogFactor);
  gl_FragColor = c;
}
