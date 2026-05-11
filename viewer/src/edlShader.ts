export const edlVertexShader = /* glsl */ `
varying vec2 vUv;
void main() {
  vUv = uv;
  gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
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

float linearizeDepth(float d) {
  return uNear * uFar / (uFar - d * (uFar - uNear));
}

float edlResponse(vec2 tc, float depth) {
  float response = 0.0;
  // 8 neighbours
  for (int i = 0; i < 8; i++) {
    float angle = float(i) * 0.785398; // 2π/8
    vec2 offset = vec2(cos(angle), sin(angle)) * uRadius / uScreenSize;
    float nd = linearizeDepth(texture2D(tDepth, tc + offset).r);
    response += max(0.0, log2(depth) - log2(nd));
  }
  return response / 8.0;
}

void main() {
  vec4 color = texture2D(tColor, vUv);
  float depth = linearizeDepth(texture2D(tDepth, vUv).r);
  float response = edlResponse(vUv, depth);
  float shade = exp(-response * 300.0 * uStrength);
  gl_FragColor = vec4(color.rgb * shade, color.a);
}
`;
