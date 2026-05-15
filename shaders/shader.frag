#version 450
layout(location = 0) in vec3 vWorldNormal;
layout(location = 1) in vec2 vUv;

layout(set = 0, binding = 0) uniform sampler2D uBaseColor;

layout(location = 0) out vec4 FragColor;

const vec3  kSunDir  = vec3(0.4, 1.0, 0.3);
const float kAmbient = 0.25;
const float kDiffuse = 0.75;

void main() {
    vec3  n = normalize(vWorldNormal);
    float diffuse = max(dot(n, normalize(kSunDir)), 0.0) * kDiffuse;
    float lighting = kAmbient + diffuse;
    vec3 baseColor = texture(uBaseColor, vUv).rgb;

    FragColor = vec4(baseColor * lighting, 1.0);
}
