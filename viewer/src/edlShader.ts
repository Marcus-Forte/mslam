// Full-screen quad: PlaneGeometry(2, 2) is already in clip space, so no matrices.
export const edlVertexShader = /* glsl */ `
varying vec2 vUv;
void main() {
  vUv = uv;
  gl_Position = vec4(position.xy, 0.0, 1.0);
}
`;

export const edlFragmentShader = /* glsl */ `
uniform sampler2D tColor;
uniform sampler2D tDepth;
uniform vec2 uScreenSize;
uniform float uStrength;
uniform float uRadius;
uniform float uNear;
uniform float uFar;

varying vec2 vUv;

// Unit offsets of the 8 neighbours, every 45 degrees.
const vec2 OFFSETS[8] = vec2[8](
  vec2( 1.0,         0.0),
  vec2( 0.70710678,  0.70710678),
  vec2( 0.0,         1.0),
  vec2(-0.70710678,  0.70710678),
  vec2(-1.0,         0.0),
  vec2(-0.70710678, -0.70710678),
  vec2( 0.0,        -1.0),
  vec2( 0.70710678, -0.70710678)
);

float linearizeDepth(float d) {
  return uNear * uFar / (uFar - d * (uFar - uNear));
}

void main() {
  vec4 color = texture2D(tColor, vUv);
  float rawDepth = texture2D(tDepth, vUv).r;

  // Nothing was drawn here (depth buffer still at the far plane): no shading.
  if (rawDepth >= 1.0) {
    gl_FragColor = color;
    return;
  }

  float logDepth = log2(linearizeDepth(rawDepth));
  vec2 step = uRadius / uScreenSize;

  float response = 0.0;
  for (int i = 0; i < 8; i++) {
    float nd = linearizeDepth(texture2D(tDepth, vUv + OFFSETS[i] * step).r);
    response += max(0.0, logDepth - log2(nd));
  }
  response /= 8.0;

  float shade = exp(-response * 300.0 * uStrength);
  gl_FragColor = vec4(color.rgb * shade, color.a);
}
`;
