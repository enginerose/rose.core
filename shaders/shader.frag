#version 330 core
in vec3 vWorldNormal;
in vec2 vUv;

uniform sampler2D uBaseColor;
uniform bool uHasTexture;

out vec4 FragColor;

// Static directional light (sun) — world space, points toward the light source
const vec3  kSunDir  = vec3(0.4, 1.0, 0.3);
const float kAmbient = 0.25;
const float kDiffuse = 0.75;

void main() {
    vec3  n        = normalize(vWorldNormal);
    float diffuse  = max(dot(n, normalize(kSunDir)), 0.0) * kDiffuse;
    float lighting = kAmbient + diffuse;

    vec3 baseColor = uHasTexture
        ? texture(uBaseColor, vUv).rgb
        : vec3(0.8, 0.8, 0.8);

    FragColor = vec4(baseColor * lighting, 1.0);
}
