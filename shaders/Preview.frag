#version 110

uniform vec4 color1, color2;
varying vec3 vBC;
varying float shading;
uniform sampler2D ourTexture;

vec3 smoothstep3f(vec3 edge0, vec3 edge1, vec3 x) {
  vec3 t;
  t = clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0);
  return t * t * (3.0 - 2.0 * t);
}

float edgeFactor() {
  const float th = 1.414; // total thickness of half-edge (per triangle) including fade, (must be >= fade)
  const float fade = 1.0; // thickness of fade (antialiasing) in screen pixels
  vec3 d = fwidth(vec3(1.0)-vBC);
  vec3 a3 = smoothstep((th-fade)*d, th*d, vBC);
  return min(min(a3.x, a3.y), a3.z);
}

void main(void) {
  vec2 resolution = vec2(694,471);
  vec2 fragCoords = gl_FragCoord.xy; // / gl_FragCoord.w); // fragPos is MVP * worldPosition
  fragCoords = fragCoords / resolution;
  vec2 screenCoords = fragCoords; // Convert from [-1, 1] to [0, 1] to sample UV coordinates
  screenCoords = fragCoords;
  //gl_FragColor = vec4(screenCoords.xy, 0, 1); texture2D(ourTexture, screenCoords);
  vec4 d = texture2D(ourTexture, screenCoords);
  if( (abs(d.x-gl_FragCoord.z) < 0.0001) && ((d.x - gl_FragCoord.z) >= -0.001))
  {
    gl_FragColor = mix(color2, vec4(color1.rgb * shading, color1.a), edgeFactor());
    return;
  }
  discard;
}
