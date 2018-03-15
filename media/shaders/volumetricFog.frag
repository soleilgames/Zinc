
//#version 120 core

uniform sampler2D sceneTex;
uniform sampler2D bufferA;
uniform sampler2D bufferB;
vec4              fogColor = vec4(0.776, 0.839, 0.851, 1.0);
// float fogMin = 10.0;
// float fogMax = 20.0;
float density = 05.0;

void
main(void)
{
  float z = abs(texture2D(bufferA, gl_TexCoord[0].st).r -
                texture2D(bufferB, gl_TexCoord[0].st).r);
  //if (z < 0.0) z = 0.0; // TODO:

  // float de = 0.025 * smoothstep(0.0, 6.0, 10.0 - fogMin);
  // float di = 0.045 * smoothstep(0.0, 40.0, 20.0 - fogMax);

  // float extinction = exp(-z * de);
  // float inscattering = exp(-z * di);
  
  //vec4 c       = mix(texture2D(sceneTex, gl_TexCoord[0].st), fogColor, z);
  // vec4 c = texture2D(sceneTex, gl_TexCoord[0].st) * extinction + fogColor * (1.0 - inscattering);

  float fogFactor = exp(-(z * density)*(z * density));
  
  vec4 c       = mix(fogColor, texture2D(sceneTex, gl_TexCoord[0].st), fogFactor);
  gl_FragColor = c;
}
